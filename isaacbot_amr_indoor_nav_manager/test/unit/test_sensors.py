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

from isaacbot_amr_indoor_nav_manager.domain.sensors import (
    ScanSource,
    select_source,
    SensorHealth,
    SensorThresholds,
)

T = SensorThresholds(timeout=2.0, recover_time=3.0)


def feed(health, start, end, period=0.1):
    t = start
    while t <= end + 1e-9:
        health.on_message(t)
        t += period


def test_lidar_wins_then_camera_then_nothing():
    assert select_source(True, True) == ScanSource.LIDAR
    assert select_source(True, False) == ScanSource.LIDAR
    assert select_source(False, True) == ScanSource.CAMERA
    assert select_source(False, False) == ScanSource.NONE


def test_a_sensor_never_heard_from_is_lost():
    assert not SensorHealth(T).evaluate(100.0)


def test_healthy_only_after_a_steady_run():
    h = SensorHealth(T)
    feed(h, 0.0, 2.9)
    assert not h.evaluate(2.9)
    feed(h, 3.0, 3.0)
    assert h.evaluate(3.0)


def test_lost_after_a_gap_longer_than_the_timeout():
    h = SensorHealth(T)
    feed(h, 0.0, 5.0)
    assert h.evaluate(5.0)
    assert h.evaluate(6.9)      # within the timeout: still fine
    assert not h.evaluate(7.1)


def test_recovering_needs_a_new_steady_run():
    h = SensorHealth(T)
    feed(h, 0.0, 5.0)
    assert not h.evaluate(8.0)
    feed(h, 10.0, 12.0)
    assert not h.evaluate(12.0)
    feed(h, 12.1, 13.0)
    assert h.evaluate(13.0)


def test_a_driver_fault_overrides_a_steady_topic():
    h = SensorHealth(T)
    feed(h, 0.0, 5.0)
    h.set_fault(True)
    assert not h.evaluate(5.0)
    h.set_fault(False)
    assert h.evaluate(5.0)  # the run of messages never broke
