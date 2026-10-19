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

"""Sensor side of the automatic localization: which scan feeds SLAM/AMCL, and map -> odom
while none does."""

import collections
import math
import threading
from typing import Callable, Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import tf2_ros

from ..domain.model import Pose2D
from ..domain.ports import MapOdomHold
from ..domain.sensors import ScanSource, select_source, SensorHealth, SensorThresholds
from .ros_adapters import yaw_from_quaternion

# diagnostic_msgs levels; `level` is a byte, which rclpy hands over as bytes.
_FAULT_LEVELS = (DiagnosticStatus.ERROR, DiagnosticStatus.STALE)


def _level(status) -> int:
    level = status.level
    return level if isinstance(level, int) else ord(level)


class RosSensorMonitor:
    """Watches the lidar and camera scans and relays the best working one to SLAM/AMCL.

    SLAM and AMCL read one topic (`out_topic`), so a switch between lidar and camera restarts
    nothing. The lidar also counts as lost while its driver reports ERROR/STALE (the rule the
    IsLidarHealthy BT condition uses). `on_source` is called on every change of source, from
    the executor: keep it short.
    """

    def __init__(self, node: Node, lidar_topic: str, camera_topic: str, out_topic: str,
                 lidar_diagnostic: str, thresholds: SensorThresholds,
                 on_source: Callable[[ScanSource], None], callback_group=None,
                 period: float = 0.25):
        self._node = node
        self._lidar = SensorHealth(thresholds)
        self._camera = SensorHealth(thresholds)
        self._lidar_diagnostic = lidar_diagnostic
        self._on_source = on_source
        self._source = ScanSource.NONE
        self._ready = threading.Event()
        # One mutually exclusive group for all of it: no locks around the health state.
        self._pub = node.create_publisher(LaserScan, out_topic, qos_profile_sensor_data)
        node.create_subscription(LaserScan, lidar_topic, self._on_lidar,
                                 qos_profile_sensor_data, callback_group=callback_group)
        node.create_subscription(LaserScan, camera_topic, self._on_camera,
                                 qos_profile_sensor_data, callback_group=callback_group)
        node.create_subscription(DiagnosticArray, 'diagnostics', self._on_diagnostics,
                                 QoSProfile(depth=10), callback_group=callback_group)
        node.create_timer(period, self._evaluate, callback_group=callback_group)

    @property
    def source(self) -> ScanSource:
        return self._source

    def wait_for_sensor(self, timeout: float) -> bool:
        """Block until a sensor works or `timeout` passes (startup grace)."""
        return self._ready.wait(timeout)

    def _now(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9

    def _on_lidar(self, msg: LaserScan) -> None:
        self._lidar.on_message(self._now())
        if self._source == ScanSource.LIDAR:
            self._pub.publish(msg)

    def _on_camera(self, msg: LaserScan) -> None:
        self._camera.on_message(self._now())
        if self._source == ScanSource.CAMERA:
            self._pub.publish(msg)

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name == self._lidar_diagnostic:
                self._lidar.set_fault(_level(status) in _FAULT_LEVELS)

    def _evaluate(self) -> None:
        now = self._now()
        source = select_source(self._lidar.evaluate(now), self._camera.evaluate(now))
        if source == self._source:
            return
        self._node.get_logger().info(f'Localization scan: {self._source.name} -> {source.name}')
        self._source = source
        if source != ScanSource.NONE:
            self._ready.set()
        self._on_source(source)


def map_to_odom(robot_in_map: Pose2D, robot_in_odom: Pose2D) -> Pose2D:
    """The map -> odom that puts the isaacbot at `robot_in_map`.

    T_map_odom = T_map_base * T_odom_base^-1, in the plane.
    """
    theta = robot_in_map.theta - robot_in_odom.theta
    c, s = math.cos(theta), math.sin(theta)
    x = robot_in_map.x - (c * robot_in_odom.x - s * robot_in_odom.y)
    y = robot_in_map.y - (s * robot_in_odom.x + c * robot_in_odom.y)
    return Pose2D(x, y, math.atan2(math.sin(theta), math.cos(theta)))


class RosMapOdomHolder(MapOdomHold):
    """Re-publishes a fixed map -> odom on /tf with fresh stamps while SLAM/AMCL cannot.

    /tf, not /tf_static: a static transform cannot be withdrawn, and AMCL or slam_toolbox must
    take over again. release() keeps publishing until a map -> odom that this holder did not
    send shows up on /tf (or `yield_timeout` passes), so Nav 2 never sees a gap while AMCL or
    slam_toolbox comes up. The /tf subscription exists only while releasing (tf is busy), and
    is created and destroyed on the timer, never inside its own callback.
    """

    def __init__(self, node: Node, buffer: tf2_ros.Buffer, map_frame: str, odom_frame: str,
                 base_frame: str, rate: float = 20.0, yield_timeout: float = 120.0,
                 callback_group=None):
        self._node = node
        self._buffer = buffer
        self._map, self._odom, self._base = map_frame, odom_frame, base_frame
        self._yield_timeout = yield_timeout
        self._callback_group = callback_group
        self._broadcaster = tf2_ros.TransformBroadcaster(node)
        self._lock = threading.Lock()
        self._held = False
        self._transform: Optional[TransformStamped] = None
        self._pending_pose: Optional[Pose2D] = None  # waits for odom -> base to resolve
        self._sent = collections.deque(maxlen=200)    # our own stamps, to tell them apart
        self._release_deadline: Optional[float] = None
        self._tf_sub = None
        self._watch = False  # whether /tf should be watched; the timer makes it so
        node.create_timer(1.0 / rate, self._tick, callback_group=callback_group)

    # --- MapOdomHold -------------------------------------------------------------------
    def hold(self, robot_in_map: Optional[Pose2D] = None) -> None:
        with self._lock:
            self._held = True
            self._release_deadline = None
            self._pending_pose = robot_in_map
            self._transform = None if robot_in_map is not None else self._current_or_identity()
            self._watch = False
        self._node.get_logger().info(
            'Holding map -> odom' + (f' with the isaacbot at ({robot_in_map.x:.2f}, '
                                     f'{robot_in_map.y:.2f}, {robot_in_map.theta:.2f})'
                                     if robot_in_map is not None else ''))

    def release(self) -> None:
        with self._lock:
            if not self._held or self._release_deadline is not None:
                return
            self._release_deadline = self._seconds() + self._yield_timeout
            self._watch = True

    def holding(self) -> bool:
        return self._held

    # --- internals ---------------------------------------------------------------------
    def _seconds(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9

    def _lookup(self, target: str, source: str):
        try:
            return self._buffer.lookup_transform(target, source, RclpyTime(),
                                                 timeout=Duration(seconds=0.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def _current_or_identity(self) -> TransformStamped:
        t = self._lookup(self._map, self._odom)
        if t is None:
            t = TransformStamped()
            t.transform.rotation.w = 1.0
        t.header.frame_id, t.child_frame_id = self._map, self._odom
        return t

    def _from_pose(self, robot_in_map: Pose2D) -> Optional[TransformStamped]:
        odom_base = self._lookup(self._odom, self._base)
        if odom_base is None:
            return None
        tr = odom_base.transform.translation
        p = map_to_odom(robot_in_map,
                        Pose2D(tr.x, tr.y, yaw_from_quaternion(odom_base.transform.rotation)))
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = self._map, self._odom
        t.transform.translation.x, t.transform.translation.y = p.x, p.y
        t.transform.rotation.z = math.sin(p.theta / 2.0)
        t.transform.rotation.w = math.cos(p.theta / 2.0)
        return t

    def _tick(self) -> None:
        with self._lock:
            if self._release_deadline is not None and self._seconds() > self._release_deadline:
                self._held = False
                self._release_deadline = None
                self._watch = False
                self._node.get_logger().warning(
                    'Nothing took over map -> odom after release; stopped holding it')
            watch, held = self._watch, self._held
            if held and self._transform is None and self._pending_pose is not None:
                self._transform = self._from_pose(self._pending_pose)
            t = self._transform
        if watch and self._tf_sub is None:
            self._tf_sub = self._node.create_subscription(
                TFMessage, '/tf', self._on_tf, QoSProfile(depth=100),
                callback_group=self._callback_group)
        elif not watch and self._tf_sub is not None:
            self._node.destroy_subscription(self._tf_sub)
            self._tf_sub = None
        if not held or t is None:
            return  # odometry not up yet
        t.header.stamp = self._node.get_clock().now().to_msg()
        self._sent.append((t.header.stamp.sec, t.header.stamp.nanosec))
        self._broadcaster.sendTransform(t)

    def _on_tf(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            if t.header.frame_id.lstrip('/') != self._map or \
                    t.child_frame_id.lstrip('/') != self._odom:
                continue
            if (t.header.stamp.sec, t.header.stamp.nanosec) in self._sent:
                continue  # our own
            with self._lock:
                if self._release_deadline is None:
                    return
                self._held = False
                self._release_deadline = None
                self._watch = False
            self._node.get_logger().info(
                'SLAM/AMCL publish map -> odom again; stopped holding it')
            return
