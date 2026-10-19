<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="icons/isaacbot-lockup-white.svg">
    <img src="icons/isaacbot-lockup-black-bold.svg" alt="IsaacBot" width="240">
  </picture>
</p>

# isaacbot_amr_orchestrator

IsaacBot's Ardennes autonomy stack, running on the orchestrator computer.

- [`isaacbot_amr_navigation`](isaacbot_amr_navigation/README.md) - Nav 2 configuration, the `IsMotionLocked`
  and `IsLidarHealthy` behavior-tree conditions and the SLAM map autosaver.
- [`isaacbot_amr_mission_manager`](isaacbot_amr_mission_manager/README.md) - behavior-tree mission
  supervision on top of Nav 2.
- [`isaacbot_amr_indoor_nav_manager`](isaacbot_amr_indoor_nav_manager/README.md) - indoor map library,
  places and automatic SLAM ↔ AMCL ↔ odometry switching (always started by
  `isaacbot_amr_navigation`'s bringup) for the isaacbot_amr_drive_interface web UI.
- [`isaacbot_amr_drive_mode`](isaacbot_amr_drive_mode/README.md) - the driving modes (Manual, Assisted with
  lidar slow-down/stop, Automatic): routes the web UI's joystick and Nav 2's commands to the
  platform according to the mode.
- [`isaacbot_amr_follow_me`](isaacbot_amr_follow_me/README.md) - follow-me: the person `isaacbot_amr_perception`
  tracks, followed through Nav 2's Following server; `follow_me/start` / `follow_me/stop`.
- [`isaacbot_amr_autonomy`](isaacbot_amr_autonomy/README.md) - metapackage grouping them.

## Quick start

### Create workspace

```bash
mkdir -p ~/ros2_ws/ardennes
cd ~/ros2_ws/ardennes
git clone -b master https://github.com/isaacbotsrl/isaacbot_amr_ros.git src/isaacbot_amr_ros
git clone -b master https://github.com/isaacbotsrl/isaacbot_amr_orchestrator.git src/isaacbot_amr_orchestrator
```

`isaacbot_amr_ros` is required: `isaacbot_amr_mission_manager` builds against `isaacbot_amr_utils`.

### Setup environment variables

```bash
# Every $ROS_DISTRO below is expanded before ROS is sourced, so set it explicitly.
export ROS_DISTRO=lyrical

# The namespace the isaacbot runs under. Both computers must agree, otherwise Nav 2 publishes
# /nav_cmd_vel_stamped while the isaacbot's mux listens on /isaacbot/nav_cmd_vel_stamped.
export ISAACBOT_AMR_SYSTEM_NAMESPACE=isaacbot
```

### Build

```bash
sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to isaacbot_amr_autonomy

source install/setup.bash
```

### Running

#### Simulated isaacbot:

`sim_nav_indoor.sh` starts everything, in order and each part only once the
previous one is up: a local Zenoh router, `isaacbot_amr_gazebo` (simulator, URDF, RViz, ros2_control,
EKF, twist_mux + motion_lock), `isaacbot_amr_navigation`, `isaacbot_amr_drive_mode` and `isaacbot_amr_mission_manager`.
It then switches the drive mode to AUTOMATIC, so Nav 2 goals (RViz "Nav2 Goal") drive the isaacbot.
One Ctrl+C stops everything, last-started first.

```bash
src/isaacbot_amr_orchestrator/scripts/sim/sim_nav_indoor.sh   # maps in ~/isaacbot_amr_sim_maps/indoor
```

Localization is automatic (`isaacbot_amr_indoor_nav_manager`): SLAM while no saved map is
selected, AMCL on the selected map, and odometry (map -> odom held) while no lidar or camera
scan arrives. The Gazebo world has no `camera/scan`, so only the lidar is watched there.

- Logs: one file per part under `~/.ros/isaacbot_amr_sim/nav-<time>/`; on failure the script prints
  the tail of the part that died.
- Every process runs as a Zenoh **client** of the local router (`tcp/localhost:7447`), whatever
  `ZENOH_CONFIG_OVERRIDE` says, so the simulated `/isaacbot` never joins the real isaacbot's router.
  Other terminals need the same:
  `export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/localhost:7447"]'`.
- Outside AUTOMATIC a Nav 2 goal plans but the isaacbot does not move: Nav 2's commands reach the
  platform only through `isaacbot_amr_drive_mode`, and only in AUTOMATIC, which needs
  `isaacbot_amr_mission_manager` running.
- The rotation shim turns at 0.7 rad/s in simulation instead of the isaacbot's 1.5: the simulated
  wheels have no static friction to break, and at 1.5 the isaacbot rocked back and forth in place
  before driving off (`ISAACBOT_AMR_SIM_SHIM_TURN_RATE` overrides it).
- Options (environment): `ISAACBOT_AMR_SIM_RVIZ`, `ISAACBOT_AMR_SIM_HEADLESS`, `ISAACBOT_AMR_SIM_MAPS_DIR`,
  `ISAACBOT_AMR_SIM_LOG_DIR`, `ISAACBOT_AMR_SIM_TIMEOUT`, `ISAACBOT_AMR_SIM_SHIM_TURN_RATE` - see
  `scripts/sim/sim_nav_common.sh`.

#### Real isaacbot:

```bash
# on the isaacbot: platform + sensor payload
ros2 launch isaacbot_amr_bringup isaacbot_amr_bringup.launch.py
ros2 launch isaacbot_amr_sensors_bringup isaacbot_amr_sensors.launch.py use_lidar:=true

# on the isaacbot or on the orchestrator computer: navigation
ros2 launch isaacbot_amr_navigation bringup.launch.py use_sim_time:=False
```

#### Mission manager:

```bash
# goals are in <namespace>/map, Nav 2's global frame
ros2 launch isaacbot_amr_mission_manager isaacbot_amr_mission_manager.launch.py \
  namespace:=isaacbot
```

### Testing

```bash
colcon test --packages-select isaacbot_amr_navigation isaacbot_amr_mission_manager
colcon test-result --all
```

Launch arguments, parameters, localization and troubleshooting are documented in the
package READMEs linked above.

## Related repositories

A complete isaacbot is three repositories, one per container:

- [`isaacbot_amr_ros`](https://github.com/isaacbotsrl/isaacbot_amr_ros) - the platform
  (`ardennes-platform`).
- [`isaacbot_amr_sensors`](https://github.com/isaacbotsrl/isaacbot_amr_sensors) - the sensor payload,
  GNSS and lidar drivers (`ardennes-sensors`).
- [`isaacbot_amr_orchestrator`](https://github.com/isaacbotsrl/isaacbot_amr_orchestrator) - this one, the
  autonomy stack (`ardennes-orchestrator`).
