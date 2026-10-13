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
from isaacbot_amr_follow_me.application.follow_executor import (
    ExecutorConfig, FollowExecutor, Phase)
from isaacbot_amr_follow_me.application.ports import FollowActionPort, Log
from isaacbot_amr_follow_me.domain.follow_rules import DriveMode, MissionState, TargetState


class FakeAction(FollowActionPort):

    def __init__(self):
        self.up = True
        self.goals = 0
        self.cancels = 0

    def ready(self):
        return self.up

    def send_goal(self):
        self.goals += 1

    def cancel(self):
        self.cancels += 1


class FakeLog(Log):

    def __init__(self):
        self.lines = []

    def info(self, message):
        self.lines.append(message)

    def warning(self, message):
        self.lines.append(message)


def ready_executor():
    action = FakeAction()
    ex = FollowExecutor(ExecutorConfig(target_timeout=1.0), action, FakeLog())
    ex.on_drive_mode(DriveMode.AUTOMATIC)
    ex.on_mission_state(MissionState.IDLE)
    ex.on_target(TargetState.TRACKING, 10.0)
    return ex, action


def following():
    ex, action = ready_executor()
    assert ex.start(10.1)[0]
    ex.on_goal_accepted()
    return ex, action


def test_start_sends_one_goal_and_reports_progress():
    ex, action = ready_executor()
    ok, _ = ex.start(10.1)
    assert ok and action.goals == 1 and ex.phase == Phase.STARTING
    ex.on_goal_accepted()
    assert ex.status() == 'FOLLOWING: waiting for the person'
    ex.on_feedback('CONTROLLING')
    assert ex.status() == 'FOLLOWING: following'
    ex.on_feedback('RETRY')
    assert ex.phase == Phase.SEARCHING
    ex.on_feedback('CONTROLLING')
    assert ex.phase == Phase.FOLLOWING
    assert not ex.start(11.0)[0]


def test_start_refused_without_automatic_or_with_a_stale_target():
    ex, action = ready_executor()
    ex.on_drive_mode(DriveMode.ASSISTED)
    ok, why = ex.start(10.1)
    assert not ok and 'Automatic' in why
    ex.on_drive_mode(DriveMode.AUTOMATIC)
    ok, why = ex.start(12.0)  # last target 2 s ago
    assert not ok and 'no person' in why
    assert action.goals == 0 and ex.phase == Phase.IDLE


def test_stop_cancels_and_result_returns_to_idle_keeping_the_reason():
    ex, action = following()
    ok, _ = ex.stop()
    assert ok and action.cancels == 1 and ex.phase == Phase.STOPPING
    assert ex.stop()[0]  # already stopping: no second cancel
    assert action.cancels == 1
    ex.on_result('canceled')
    assert ex.status() == 'IDLE: stopped on request'
    assert not ex.stop()[0]


def test_stop_before_acceptance_still_cancels():
    ex, action = ready_executor()
    ex.start(10.1)
    ex.stop()
    assert action.cancels == 1 and ex.phase == Phase.STOPPING
    ex.on_goal_rejected('rejected')
    assert ex.phase == Phase.IDLE


def test_leaving_automatic_or_a_mission_starting_cancels():
    for event in ('mode', 'mission'):
        ex, action = following()
        if event == 'mode':
            ex.on_drive_mode(DriveMode.ASSISTED)
        else:
            ex.on_mission_state(MissionState.RUNNING)
        ex.tick(11.0)
        assert action.cancels == 1 and ex.phase == Phase.STOPPING
        ex.on_result('canceled')
        assert ex.phase == Phase.IDLE
        assert ('ASSISTED' if event == 'mode' else 'mission') in ex.status()


def lost():
    """Following, then the server ends the goal because the person is gone."""
    ex, action = following()
    ex.on_target(TargetState.LOST, 11.0)
    ex.tick(12.0)
    assert action.cancels == 0 and ex.phase == Phase.FOLLOWING  # the server's call, not ours
    ex.on_result('aborted', 'FAILED_TO_DETECT_OBJECT: Failed object detection')
    return ex, action


def test_a_lost_person_is_waited_for_without_a_time_limit():
    ex, action = lost()
    assert ex.phase == Phase.WAITING and 'waiting' in ex.status()
    ex.tick(1000.0)  # no time limit, no goal while nobody is tracked
    assert ex.phase == Phase.WAITING and action.goals == 1
    assert not ex.start(1000.0)[0]  # already following (waiting)


def test_waiting_resumes_when_the_person_is_tracked_again():
    ex, action = lost()
    ex.on_target(TargetState.COASTING, 30.0)
    ex.tick(30.1)
    assert ex.phase == Phase.WAITING and action.goals == 1
    ex.on_target(TargetState.TRACKING, 31.0)
    ex.tick(31.1)
    assert action.goals == 2 and ex.status() == 'STARTING: person tracked; resuming'
    ex.on_goal_accepted()
    assert ex.phase == Phase.FOLLOWING


def test_stop_mode_change_or_mission_end_the_wait_without_a_cancel():
    ex, action = lost()
    assert ex.stop() == (True, 'stopped on request')
    assert ex.phase == Phase.IDLE and action.cancels == 0
    for event in ('mode', 'mission'):
        ex, action = lost()
        if event == 'mode':
            ex.on_drive_mode(DriveMode.ASSISTED)
        else:
            ex.on_mission_state(MissionState.RUNNING)
        ex.on_target(TargetState.TRACKING, 20.0)
        ex.tick(20.1)
        assert ex.phase == Phase.IDLE and action.cancels == 0 and action.goals == 1


def test_a_search_rotation_that_gave_up_is_a_lost_person_too():
    ex, _ = following()
    ex.on_result('aborted', 'FAILED_TO_CONTROL: Timed out rotating to object')
    assert ex.phase == Phase.WAITING


def test_a_server_failure_is_resumed_while_the_person_is_tracked():
    ex, action = following()
    ex.on_result('aborted', 'FAILED_TO_CONTROL: Failed to get control')  # a late TF, once
    assert ex.phase == Phase.WAITING and 'FAILED_TO_CONTROL' in ex.status()
    ex.on_target(TargetState.TRACKING, 10.2)
    ex.tick(10.3)
    assert action.goals == 2 and ex.phase == Phase.STARTING


def test_repeated_server_failures_end_following():
    ex, action = following()
    for t in (11.0, 12.0):  # two within the window are resumed
        ex.tick(t)
        ex.on_result('aborted', 'FAILED_TO_CONTROL: Failed to get control')
        assert ex.phase == Phase.WAITING
        ex.on_target(TargetState.TRACKING, t + 0.1)
        ex.tick(t + 0.2)
        ex.on_goal_accepted()
    ex.on_result('aborted', 'FAILED_TO_CONTROL: Failed to get control')  # the third ends it
    assert ex.phase == Phase.IDLE and 'FAILED_TO_CONTROL' in ex.status()
    assert action.goals == 3


def test_server_failures_outside_the_window_do_not_add_up():
    ex, action = following()
    for t in (11.0, 50.0, 90.0, 130.0):
        ex.on_target(TargetState.TRACKING, t)
        ex.tick(t)
        ex.on_result('aborted', 'FAILED_TO_CONTROL: Failed to get control')
        assert ex.phase == Phase.WAITING
        ex.tick(t + 0.1)
        ex.on_goal_accepted()
    assert action.goals == 5


def test_wait_off_ends_following_on_any_abort():
    action = FakeAction()
    ex = FollowExecutor(ExecutorConfig(wait_when_lost=False), action, FakeLog())
    ex.on_drive_mode(DriveMode.AUTOMATIC)
    ex.on_mission_state(MissionState.IDLE)
    ex.on_target(TargetState.TRACKING, 10.0)
    ex.start(10.1)
    ex.on_goal_accepted()
    ex.on_result('aborted', 'FAILED_TO_DETECT_OBJECT: Failed object detection')
    assert ex.phase == Phase.IDLE and 'FAILED_TO_DETECT_OBJECT' in ex.status()
    ex.on_target(TargetState.TRACKING, 20.0)
    assert ex.start(20.1)[0]
    ex.on_goal_accepted()
    ex.on_result('aborted', 'FAILED_TO_CONTROL: Failed to get control')
    assert ex.phase == Phase.IDLE and 'FAILED_TO_CONTROL' in ex.status()


def test_rejected_goal_and_foreign_cancel_are_reported():
    ex, _ = ready_executor()
    ex.start(10.1)
    ex.on_goal_rejected('server busy')
    assert ex.phase == Phase.IDLE and 'server busy' in ex.status()
    ex, _ = following()
    ex.on_result('canceled')
    assert ex.phase == Phase.IDLE and 'someone else' in ex.status()


def test_status_line_format():
    ex, _ = ready_executor()
    assert ex.status() == 'IDLE: not started'
