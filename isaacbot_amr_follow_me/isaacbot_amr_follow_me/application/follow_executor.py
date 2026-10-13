# Copyright 2026 Mechatronics Academy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Follow-me as one Nav 2 FollowObject goal: start it, watch it, stop it.

The Following server does the following itself (distance, heading). This use case decides when
a goal may be sent and when it must be cancelled (domain/follow_rules.py), and turns the server's
feedback and result into a status line for the drive UI, the VDA 5050 adapter and the logs:

    IDLE | STARTING | FOLLOWING | SEARCHING | WAITING | STOPPING, as "PHASE: detail"

The server cannot wait for a lost person: it searches (or not) and then ends the goal. With
wait_when_lost, a goal that ended because the person was lost leaves the isaacbot standing in
WAITING, with no time limit, and a new goal goes out as soon as the person is tracked again.
Stop, leaving AUTOMATIC or a mission starting end the wait like they end following.
A goal the server ended on its own failure (a late TF under load ends it at once: max_retries
is 0) is resumed the same way; max_server_failures of those within server_failure_window end
follow-me, so a server that keeps failing is not retried forever.

Not thread-safe: the node calls it from one executor thread.
"""

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

from ..domain.follow_rules import abort_reason, Conditions, start_refusal
from .ports import FollowActionPort, Log


class Phase(str, Enum):
    IDLE = 'IDLE'
    STARTING = 'STARTING'    # goal sent, not accepted yet
    FOLLOWING = 'FOLLOWING'
    SEARCHING = 'SEARCHING'  # the server lost the person and is retrying / rotating to look
    WAITING = 'WAITING'      # person lost, no goal out: the isaacbot stands until they are back
    STOPPING = 'STOPPING'    # cancel sent, waiting for the result


RUNNING_PHASES = (Phase.STARTING, Phase.FOLLOWING, Phase.SEARCHING, Phase.WAITING)

# The server's errors (result detail "NAME: message") that mean it lost the person.
_LOST_ERRORS = ('FAILED_TO_DETECT_OBJECT',)
_LOST_MESSAGES = ('rotating to object', 'rotate to object')  # its search rotation gave up

# FollowObject feedback state -> (phase, detail)
_FEEDBACK = {
    'INITIAL_PERCEPTION': (Phase.FOLLOWING, 'waiting for the person'),
    'CONTROLLING': (Phase.FOLLOWING, 'following'),
    'STOPPING': (Phase.FOLLOWING, 'at following distance'),
    'RETRY': (Phase.SEARCHING, 'person lost; searching'),
}


@dataclass(frozen=True)
class ExecutorConfig:
    # A tracked_person older than this counts as no target.
    target_timeout: float = 1.0
    # Person lost: stand and wait for them (no time limit) instead of ending follow-me.
    wait_when_lost: bool = True
    # With wait_when_lost, server failures (not a lost person) are resumed too, up to this many
    # within server_failure_window seconds; the one that reaches it ends follow-me.
    max_server_failures: int = 3
    server_failure_window: float = 30.0


class FollowExecutor:

    def __init__(self, config: ExecutorConfig, action: FollowActionPort, log: Log):
        self._config = config
        self._action = action
        self._log = log
        self._phase = Phase.IDLE
        self._detail = 'not started'
        self._drive_mode: Optional[int] = None
        self._mission_state: Optional[int] = None
        self._target_state: Optional[int] = None
        self._target_at: Optional[float] = None
        self._now = 0.0
        self._failures: List[float] = []

    # --- inputs ---------------------------------------------------------------------------------

    def on_drive_mode(self, mode: int) -> None:
        self._drive_mode = mode

    def on_mission_state(self, state: int) -> None:
        self._mission_state = state

    def on_target(self, state: int, now: float) -> None:
        self._target_state = state
        self._target_at = now
        self._now = now

    # --- commands -------------------------------------------------------------------------------

    def start(self, now: float) -> Tuple[bool, str]:
        self._now = now
        if self._phase in RUNNING_PHASES:
            return False, 'already following'
        if self._phase == Phase.STOPPING:
            return False, 'still stopping the previous follow; try again'
        refusal = start_refusal(self._conditions(now))
        if refusal:
            self._log.warning(f'Follow-me not started: {refusal}')
            return False, refusal
        self._failures = []
        self._action.send_goal()
        self._set(Phase.STARTING, 'sending the FollowObject goal')
        return True, 'follow-me starting'

    def stop(self, reason: str = 'stopped on request') -> Tuple[bool, str]:
        if self._phase == Phase.STOPPING:
            return True, 'already stopping'
        if self._phase not in RUNNING_PHASES:
            return False, 'not following'
        if self._phase == Phase.WAITING:  # no goal out: nothing to cancel
            self._set(Phase.IDLE, reason)
            return True, reason
        self._action.cancel()
        self._set(Phase.STOPPING, reason)
        return True, reason

    def tick(self, now: float) -> None:
        self._now = now
        if self._phase not in RUNNING_PHASES:
            return
        conditions = self._conditions(now)
        reason = abort_reason(conditions)
        if reason:
            self._log.warning(f'Follow-me stopped: {reason}')
            self.stop(reason)
        elif self._phase == Phase.WAITING and start_refusal(conditions) is None:
            self._log.info('Follow-me: person tracked; resuming')
            self._action.send_goal()
            self._set(Phase.STARTING, 'person tracked; resuming')

    # --- action callbacks -----------------------------------------------------------------------

    def on_goal_accepted(self) -> None:
        if self._phase == Phase.STARTING:
            self._set(Phase.FOLLOWING, 'waiting for the person')

    def on_goal_rejected(self, why: str) -> None:
        if self._phase in (Phase.STARTING, Phase.STOPPING):
            self._set(Phase.IDLE, f'the Following server refused the goal: {why}')

    def on_feedback(self, state_name: str) -> None:
        if self._phase not in (Phase.FOLLOWING, Phase.SEARCHING):
            return
        phase, detail = _FEEDBACK.get(state_name, (Phase.FOLLOWING, state_name.lower()))
        self._set(phase, detail)

    def on_result(self, outcome: str, detail: str = '') -> None:
        """outcome: 'succeeded', 'canceled' or 'aborted'; detail: the server's error, if any."""
        if self._phase == Phase.STOPPING:
            self._set(Phase.IDLE, self._detail)  # keep why it was stopped
            return
        if self._phase in (Phase.IDLE, Phase.WAITING):
            return
        if outcome == 'aborted' and self._config.wait_when_lost:
            if _lost(detail):
                self._log.warning(f'Follow-me: person lost ({detail}); waiting for them')
                self._set(Phase.WAITING, 'person lost; stopped, waiting for them to come back')
                return
            if self._server_failure_tolerated():
                self._log.warning(f'Follow-me: the Following server failed ({detail}); resuming')
                self._set(Phase.WAITING, f'the Following server failed ({detail}); resuming')
                return
        if outcome == 'succeeded':
            text = 'following ended (max duration)'
        elif outcome == 'canceled':
            text = 'following cancelled by someone else'
        else:
            text = f'following ended: {detail}' if detail else 'following ended'
        self._log.warning(f'Follow-me: {text}')
        self._set(Phase.IDLE, text)

    # --- outputs --------------------------------------------------------------------------------

    @property
    def phase(self) -> Phase:
        return self._phase

    def status(self) -> str:
        return f'{self._phase.value}: {self._detail}'

    # --- internals ------------------------------------------------------------------------------

    def _conditions(self, now: float) -> Conditions:
        fresh = (self._target_at is not None
                 and now - self._target_at <= self._config.target_timeout)
        return Conditions(
            drive_mode=self._drive_mode,
            mission_state=self._mission_state,
            target_state=self._target_state,
            target_fresh=fresh,
            server_ready=self._action.ready())

    def _server_failure_tolerated(self) -> bool:
        window = self._config.server_failure_window
        self._failures = [t for t in self._failures if self._now - t < window] + [self._now]
        return len(self._failures) < self._config.max_server_failures

    def _set(self, phase: Phase, detail: str) -> None:
        if (phase, detail) != (self._phase, self._detail):
            self._log.info(f'follow_me: {phase.value}: {detail}')
        self._phase = phase
        self._detail = detail


def _lost(detail: str) -> bool:
    """Tell whether the server ended the goal because it lost the person (not its own failure)."""
    return (detail.startswith(_LOST_ERRORS)
            or any(m in detail.lower() for m in _LOST_MESSAGES))
