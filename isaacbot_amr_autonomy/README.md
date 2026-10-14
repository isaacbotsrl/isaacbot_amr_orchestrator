# isaacbot_amr_autonomy

Metapackage for the autonomy stack that runs on the Ardennes **orchestrator computer**. It
builds nothing itself; it exists so the whole stack can be built and installed with one
`--packages-up-to`.

| Package | Description |
|---|---|
| [`isaacbot_amr_navigation`](../isaacbot_amr_navigation/README.md) | Nav 2 configuration, the `IsMotionLocked` and `IsLidarHealthy` BT condition plugins and the SLAM map autosaver. |
| [`isaacbot_amr_mission_manager`](../isaacbot_amr_mission_manager/README.md) | Behavior-tree-driven mission supervision, dispatching Nav 2 actions. |
| [`isaacbot_amr_follow_me`](../isaacbot_amr_follow_me/README.md) | Follow-me: a person tracked by `isaacbot_amr_perception` followed through Nav 2's Following server, started and stopped by `follow_me/start` / `follow_me/stop`. |

```bash
cd ~/ros2_ws/ardennes
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to isaacbot_amr_autonomy
source install/setup.bash
```

`autonomy_deps.repos` is the extension point for external sources, in the same spirit as
`isaacbot_amr_ros`'s `hardware_deps.repos` / `simulation_deps.repos`. Every `nav2_*` package,
`nav2_smac_planner` included, comes from apt through rosdep. The file pins nothing today;
importing it is still part of the orchestrator image build, so a new entry needs no Dockerfile
change:

```bash
vcs import src < src/isaacbot_amr_orchestrator/isaacbot_amr_autonomy/autonomy_deps.repos
```

Note `isaacbot_amr_navigation` is deliberately **not** part of `isaacbot_amr_ros`'s `isaacbot_amr_metapackage`:
the isaacbot computer and the orchestrator computer build different things.
