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

import math

import pytest

from isaacbot_amr_indoor_nav_manager.domain.model import Pose2D
from isaacbot_amr_indoor_nav_manager.infrastructure.ros_sensors import map_to_odom


def compose(a, b):
    c, s = math.cos(a.theta), math.sin(a.theta)
    return Pose2D(a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, a.theta + b.theta)


@pytest.mark.parametrize('in_map,in_odom', [
    (Pose2D(0, 0, 0), Pose2D(0, 0, 0)),
    (Pose2D(2, 3, 1), Pose2D(0, 0, 0)),
    (Pose2D(2, 3, 1), Pose2D(-4, 1.5, -2.5)),
    (Pose2D(-1, 7, 3), Pose2D(10, -2, 0.7)),
])
def test_the_held_map_to_odom_puts_the_isaacbot_at_its_map_pose(in_map, in_odom):
    result = compose(map_to_odom(in_map, in_odom), in_odom)
    assert result.x == pytest.approx(in_map.x)
    assert result.y == pytest.approx(in_map.y)
    assert math.cos(result.theta - in_map.theta) == pytest.approx(1.0)
