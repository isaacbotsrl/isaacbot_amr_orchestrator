# Copyright 2026 IsaacBot
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

"""Which range sensor can localize right now? Pure logic: timestamps are passed in."""

import enum
from dataclasses import dataclass
from typing import Optional


class ScanSource(enum.Enum):
    """The scan SLAM and AMCL read (localization/scan), or none: odometry only."""

    NONE = 0
    LIDAR = 1   # the RS16's projected LaserScan
    CAMERA = 2  # the RealSense depth image as a LaserScan (narrower, shorter range)


def select_source(lidar_ok: bool, camera_ok: bool) -> ScanSource:
    """The lidar whenever it works: 360 deg and 20 m beat the camera's ~87 deg and ~6 m."""
    if lidar_ok:
        return ScanSource.LIDAR
    if camera_ok:
        return ScanSource.CAMERA
    return ScanSource.NONE


@dataclass(frozen=True)
class SensorThresholds:
    timeout: float = 2.0       # s without a message before the sensor counts as lost
    recover_time: float = 3.0  # s of steady messages before a lost sensor counts again


class SensorHealth:
    """Healthy/lost for one sensor, with hysteresis so a flapping driver cannot make the
    localization stack switch back and forth.

    Starts lost. A sensor becomes healthy once messages have kept arriving (no gap longer
    than `timeout`) for `recover_time`, and is lost again as soon as one gap exceeds
    `timeout` or its driver reports a fault.
    """

    def __init__(self, thresholds: SensorThresholds = SensorThresholds()):
        self._t = thresholds
        self._healthy = False
        self._last: Optional[float] = None
        self._streak_since: Optional[float] = None
        self._fault = False

    def on_message(self, now: float) -> None:
        if self._last is None or now - self._last > self._t.timeout:
            self._streak_since = now  # first message after a gap: the steady run starts here
        self._last = now

    def set_fault(self, fault: bool) -> None:
        """The driver's own verdict (diagnostics ERROR/STALE); overrides a steady topic."""
        self._fault = fault

    def evaluate(self, now: float) -> bool:
        fresh = self._last is not None and now - self._last <= self._t.timeout
        if not fresh or self._fault:
            self._healthy = False
            if not fresh:
                self._streak_since = None
        elif not self._healthy and self._streak_since is not None \
                and now - self._streak_since >= self._t.recover_time - 1e-9:
            self._healthy = True
        return self._healthy
