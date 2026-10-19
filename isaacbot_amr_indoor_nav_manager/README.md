# isaacbot_amr_indoor_nav_manager

Indoor map library, places, and automatic SLAM ↔ AMCL ↔ odometry switching for ardennes. It is
the robot side of the Clearpath IndoorNav-style **Facility** and **Places** features in
[isaacbot_amr_drive_interface](https://github.com/isaacbotsrl/isaacbot_amr_drive_interface).

`isaacbot_amr_navigation`'s `bringup.launch.py` always starts it (the orchestrator runs it with
`ISAACBOT_AMR_ORCH_NAVIGATION=true`):
- Nav 2 always runs with `global_frame=<ns>/map`.
- This node owns whatever publishes `map → odom`, running
  `isaacbot_amr_navigation/indoor_localization.launch.py` as a child process group. Which
  sub-stack runs is decided by the sensors and the selected map (`/maps/active`), not by a
  launch argument:

| Mode | When | Child | Publishes `map → odom` |
|------|------|-------|------------------------|
| `MAPPING` | lidar and/or camera working, no map selected | slam_toolbox + map_saver (`slam_launch.py`) | slam_toolbox |
| `LOCALIZATION` | lidar and/or camera working, a map selected | map_server + AMCL on that map (`localization.launch.py`, uncomposed) | AMCL |
| `ODOMETRY` | neither working | whatever was running keeps running, without a scan | this node (holds the last `map → odom`) |

A switch between modes stops and reaps the old group (SIGINT → SIGTERM → SIGKILL on the process
group) before starting the new one, so the two never publish `map → odom` at the same time.
`nav2_container` is never touched. The group runs as Zenoh clients (`zenoh_client_mode`), so
stopping it cannot stall the other containers' processes through direct peer links.

Loading another saved map while already localizing restarts nothing: the manager calls
map_server's `load_map` (which republishes `map`; AMCL has `first_map_only: false`) and
re-seeds AMCL. If that fails, it falls back to restarting the group.

## Interfaces (all relative to the isaacbot namespace)

| Kind | Name | Type | Notes |
|------|------|------|-------|
| service | `start_mapping` | `std_srvs/Trigger` | Start a new SLAM session. Refused in `ODOMETRY` |
| service | `save_map` | `isaacbot_amr_msgs/SaveMap` | Save the SLAM map as `/maps/<name>/map.{yaml,pgm}` (MAPPING, or a mapping session paused in `ODOMETRY`) |
| service | `load_map` | `isaacbot_amr_msgs/LoadMap` | Switch to AMCL on a saved map, optionally seeding the pose. Refused in `ODOMETRY` |
| service | `delete_map` | `isaacbot_amr_msgs/DeleteMap` | Not the map in use |
| service | `save_place` | `isaacbot_amr_msgs/SavePlace` | Create (empty id) or update a place on the active map |
| service | `delete_place` | `isaacbot_amr_msgs/DeletePlace` | |
| topic (latched) | `localization_state` | `isaacbot_amr_msgs/LocalizationState` | `MAPPING` / `LOCALIZATION` / `ODOMETRY` / `SWITCHING` / `UNAVAILABLE`, plus `scan_source` (`SCAN_NONE` / `SCAN_LIDAR` / `SCAN_CAMERA`) |
| topic (latched) | `maps` | `isaacbot_amr_msgs/MapList` | Stored maps plus the active one |
| topic (latched) | `places` | `isaacbot_amr_msgs/PlaceList` | Places of the active map |
| sub | `scan` | `sensor_msgs/LaserScan` | Lidar (`lidar_scan_topic`) |
| sub | `camera/scan` | `sensor_msgs/LaserScan` | RealSense depth as a scan (`camera_scan_topic`, from `isaacbot_amr_realsense`) |
| sub | `diagnostics` | `diagnostic_msgs/DiagnosticArray` | Lidar health (`lidar_diagnostic`) |
| pub | `localization/scan` | `sensor_msgs/LaserScan` | The preferred working scan, lidar first (`localization_scan_topic`); AMCL's and slam_toolbox's `scan_topic` |
| pub | `/tf` | `tf2_msgs/TFMessage` | `map → odom`, only while holding it in `ODOMETRY` (20 Hz) |
| client | `map_saver/save_map` | `nav2_msgs/SaveMap` | Provided by slam_launch.py |

`start_mapping` and `load_map` answer within about 2 s, even though a switch can take longer.
Follow `localization_state` for the outcome.

## Behaviour

- **Sensors.** Each of the lidar (`scan`) and the camera (`camera/scan`) counts as lost after
  2 s without a message (`sensor_timeout`) and as healthy again after 3 s of steady messages
  (`sensor_recover_time`). The lidar also counts as lost while its diagnostic
  `isaacbot_amr_rs16_lidar_node: Lidar status` is ERROR or STALE. The manager relays the preferred
  working scan, the lidar first, onto `localization/scan`; the costmaps and the collision monitor
  still read `scan` directly.
- **Startup.** The manager waits up to `startup_sensor_grace` (10 s) for a working sensor. With
  one, it resumes on the last loaded map (`/maps/active`), or starts mapping when no map is
  selected. If that map has a remembered pose (`/maps/<name>/last_pose.yaml`), AMCL starts there
  with a **wider spread**:
  - σ 1.5 m / 45° (`restore_sigma_xy` / `restore_sigma_yaw`) instead of AMCL's built-in
    ~0.5 m / 15°;
  - so it still converges if the isaacbot was nudged or the pose is slightly stale;
  - the manager publishes this `initialpose` once AMCL is up, meaning it has a subscriber and the
    map → base_link TF resolves;
  - with no remembered pose it uses the map origin.

  With no sensor after the grace period it starts in `ODOMETRY`, placing the isaacbot at the active
  map's `last_pose.yaml` (identity with no map).
- **Odometry only (`ODOMETRY`).** When neither the lidar nor the camera works, SLAM or AMCL keep
  running (an unsaved SLAM map is not lost) but get no scan, so the manager holds `map → odom`
  itself: it re-publishes the last `map → odom` on `/tf` at 20 Hz with fresh stamps, and the isaacbot
  dead-reckons on odometry. `start_mapping` and `load_map` are refused; `save_map` still works in
  a paused mapping session.
- **A sensor comes back.** If the stack kept running, the manager releases the hold and, when
  localizing, re-seeds AMCL at the dead-reckoned pose with the widened spread. After a cold start
  without sensors it starts AMCL seeded at the odometry pose (widened), or SLAM. The hold is only
  dropped once SLAM or AMCL publish `map → odom` themselves, or after 120 s.
- **When the pose is saved:**
  - the moment the isaacbot comes to rest, from `odometry/wheels`: below 0.03 m/s and 0.05 rad/s for
    0.5 s after moving;
  - on shutdown;
  - before switching to another map or to mapping;
  - every 5 s while localized, as a backstop.
- **Find me.** The drive interface's **Find me** button calls AMCL's
  `reinitialize_global_localization` directly (particles over the whole map), then
  `request_nomotion_update` three times. It's a last resort: in repetitive areas AMCL can settle on
  a look-alike spot, so use Set pose when you know where the isaacbot is.
- **Save, then load.** Loading a map right after saving it from the running SLAM session seeds
  AMCL with SLAM's current pose, so the isaacbot stays localized.
- **Places.** Places live per map in `/maps/<name>/places.yaml`. Names are unique per map
  (case-insensitive). Places can only be edited while a map is loaded.
- **Map names.** `[A-Za-z0-9_-]{1,64}`, used directly as directory names. An existing name is
  refused (delete it first), as VDA 5050 does with `DUPLICATE_MAP`.
- **Crashes.** If the child stack exits on its own, the state becomes `UNAVAILABLE`. It is not
  restarted automatically.

## Layout (Clean Architecture)

```
isaacbot_amr_indoor_nav_manager/
├── domain/          # Place, PlaceBook, MapRecord, LocalizationState, ports - no ROS
├── application/     # IndoorNavService: every use case, over the ports
├── infrastructure/  # FileMapRepository, LaunchLocalizationController, ROS adapters, node
└── presentation/    # main()
```

`scripts/check_domain_purity.sh` (a CTest) fails the build if `domain/` or `application/`
imports ROS or an outer layer.

## Parameters

| Name | Default | |
|------|---------|--|
| `localization_params_file` | *(required)* | bringup's namespaced `isaacbot_amr_nav_params.yaml` |
| `maps_dir` | `/maps` | The `isaacbot-maps` volume in ardennes-orchestrator |
| `save_map_timeout` | `5.0` | s, map_saver |
| `load_map_timeout` | `5.0` | s, map_server `load_map` (in-place map switch) |
| `zenoh_client_mode` | `true` | Run the child launch as Zenoh clients (rmw_zenoh only) |
| `pose_record_period` | `5.0` | s, backstop for the stop-triggered save |
| `motion_topic` | `odometry/wheels` | Speed source for detecting a stop |
| `stop_linear_threshold` / `stop_angular_threshold` / `stop_hold_time` | `0.03` / `0.05` / `0.5` | m/s, rad/s, s |
| `restore_sigma_xy` / `restore_sigma_yaw` | `1.5` / `0.785` | m / rad, AMCL spread around a remembered pose |
| `auto_start` | `true` | Resume / start mapping on startup |
| `lidar_scan_topic` / `camera_scan_topic` | `scan` / `camera/scan` | Scans the manager watches and relays |
| `localization_scan_topic` | `localization/scan` | Relay output, read by AMCL and slam_toolbox |
| `lidar_diagnostic` | `isaacbot_amr_rs16_lidar_node: Lidar status` | Diagnostic status whose ERROR/STALE marks the lidar lost |
| `sensor_timeout` / `sensor_recover_time` | `2.0` / `3.0` | s, sensor lost after a gap / healthy after steady messages |
| `startup_sensor_grace` | `10.0` | s to wait for a sensor before starting in `ODOMETRY` |
| `launch_package`, `launch_file` | `isaacbot_amr_navigation`, `indoor_localization.launch.py` | |

## Tests

```bash
colcon test --packages-select isaacbot_amr_indoor_nav_manager
```

The tests are pytest with fakes for every port: domain rules, every use case, the file
repository on a temp dir, the launch command line, and process-group start/stop. They need no
ROS graph.
