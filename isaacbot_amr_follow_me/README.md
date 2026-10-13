# isaacbot_amr_follow_me

Follow-me for Ardennes. A person tracker in `isaacbot_amr_perception` (fmoc, `isaacbot_amr_perception_fmoc`)
publishes `tracked_person`; the **follow_me** node here runs following as one goal of Nav 2's
**Following server** (`opennav_following`, `nav2_msgs/action/FollowObject`, in
`isaacbot_amr_navigation`). The server keeps a set distance from the person, turning to face them, and
rotates to search when it loses them. Its velocity goes into `cmd_vel_nav`, so the velocity
smoother, the collision monitor, drive mode (AUTOMATIC only) and the motion lock guard it like
any Nav 2 motion. Nothing here publishes `cmd_vel`.

```
 rover-a1-sensors                          rover-a1-orchestrator
 camera/depth/points ─┐
 odom, TF ────────────┤ fmoc ──tracked_person──▶ follow_me ──follow_me/target_pose──┐
                        (isaacbot_amr_perception)       start/stop, rules   PoseStamped     │
                                                 FollowObject goal ─▶ following_server (Nav 2)
                                                                     └▶ cmd_vel_nav → smoother
                                                                        → collision monitor → drive mode
```

It runs in `rover-a1-orchestrator`, next to Nav 2, the mission manager and drive mode it
coordinates with. `ISAACBOT_AMR_SYSTEM_FOLLOW_ME_ENABLE=true` starts both halves: fmoc in `rover-a1-sensors`,
this node in the orchestrator (`rover_docker/README.md`, "Follow-me").

Clean Architecture: `domain/` (start/stop rules) and `application/` (the follow executor and its
ports) import no ROS (`scripts/check_domain_purity.sh` checks it); `infrastructure/` holds the
rclpy lifecycle node.

## Starting and stopping

`follow_me/start` and `follow_me/stop` (`std_srvs/Trigger`) are the one control interface; three
things call them:

- the **drive UI** (Navigate tab, *Follow me* card), through foxglove_bridge;
- **VDA 5050** master control, with the custom instant actions `startFollowing` /
  `stopFollowing` (`isaacbot_amr_vda5050`): the fleet decides *that* the isaacbot follows, the isaacbot decides
  *how*;
- anything else on the ROS graph (CLI, Foxglove, ros-mcp).

**Start** is refused, with the reason, unless all hold:
- a person is tracked (`tracked_person` TRACKING or COASTING, fresher than `target_timeout`):
  stand about 1.5 m in front of the isaacbot while it stands still;
- drive mode is AUTOMATIC;
- no mission is RUNNING or HELD (missions and the Following server share `cmd_vel_nav`);
- the Following server is up.

**Following stops** on `follow_me/stop`, when drive mode leaves AUTOMATIC (the joystick or RC
taking over does that), and when a mission starts (a fleet order or GoTo wins). A lost person
does not stop it: the isaacbot stands and waits for them (WAITING, below). The collision monitor,
the motion lock and the E-Stop stop motion independently of all of that.

`follow_me/status` (`std_msgs/String`, latched) reads `PHASE: detail`, PHASE one of `IDLE`,
`STARTING`, `FOLLOWING`, `SEARCHING`, `WAITING`, `STOPPING`.

## How following works

- **Follow.** The node republishes the tracked position as `follow_me/target_pose`, the pose
  topic its `FollowObject` goal names. The Following server (`following_server:` in
  `isaacbot_amr_navigation`'s `isaacbot_amr_nav_params.yaml`) drives to `desired_distance` (1.2 m) from it with
  the graceful control law, always facing it, at up to 0.5 m/s and 1.0 rad/s.
- **Lost and found.** fmoc keeps a briefly hidden person as COASTING (forwarded). With no fresh
  pose for `detection_timeout` (1 s) the server ends the goal and stops the isaacbot; it does not
  search by rotating (`search_by_rotating: false`, `max_retries: 0`). follow_me then goes to
  WAITING: the isaacbot stands, with no time limit, until fmoc tracks the person again, and a new
  goal goes out by itself. fmoc picks them up again near where they were last seen (within
  1.5 m, for 10 s), and after that in the acquire zone (1.5 m in front of the isaacbot). Stop,
  leaving AUTOMATIC or a mission end the wait. `wait_when_lost: false` ends follow-me instead.
- **Server failures.** With `max_retries: 0` the server also ends the goal on its own failures,
  such as one late TF under load ("Failed to get control"). follow_me resumes those like a lost
  person (WAITING, then a new goal while the person is tracked); `max_server_failures` (3) of
  them within `server_failure_window` (30 s) end follow-me, so a failing server is not retried
  forever.
- **Another tracker** can replace fmoc: anything publishing `isaacbot_amr_msgs/TrackedPerson` on
  `tracked_person`, stamped with the sensor time, in a frame TF can transform to odom.

## Running

```bash
ros2 launch isaacbot_amr_follow_me isaacbot_amr_follow_me.launch.py namespace:=isaacbot
ros2 service call /isaacbot/follow_me/start std_srvs/srv/Trigger
```

| Argument | Default |
|----------|---------|
| `namespace` | `$ISAACBOT_AMR_SYSTEM_NAMESPACE` |
| `params_file` | `config/follow_me.yaml` (`/**/follow_me:`) |
| `use_sim_time` | `False` |
| `log_level` | `info` |

### Simulation

Needs the Gazebo depth camera (`ISAACBOT_AMR_SYSTEM_USE_CAMERA=true`), Nav 2 with the Following server
(`ros-lyrical-opennav-following`), the mission manager and drive mode in AUTOMATIC:

```bash
export ISAACBOT_AMR_SYSTEM_USE_CAMERA=true
ros2 launch isaacbot_amr_gazebo simulation.launch.py \
  gz_world:=$(ros2 pkg prefix isaacbot_amr_world)/share/isaacbot_amr_world/world/follow_me_world.sdf
# Nav 2 (isaacbot_amr_navigation bringup) + drive mode + mission manager, then drive mode AUTOMATIC
ros2 launch isaacbot_amr_perception_bringup isaacbot_amr_perception.launch.py namespace:=isaacbot \
  use_person_tracking:=true use_sim_time:=true
ros2 launch isaacbot_amr_follow_me isaacbot_amr_follow_me.launch.py namespace:=isaacbot use_sim_time:=true
ros2 service call /isaacbot/follow_me/start std_srvs/srv/Trigger
```

The world's person (Fuel's walking actor) waits 25 s in the acquire zone, then walks a loop round
the east half of the world. The first start downloads its mesh from Fuel, which can stall the world
long enough for the controller spawners' 5 s switch timeout to shut the simulation down: start it
again (the mesh is cached in `~/.gz/fuel`).

### Tests

```bash
colcon test --packages-select isaacbot_amr_follow_me && colcon test-result --verbose
```

Rules and the executor against a fake action, plus layer purity; no ROS graph needed.

## What has been verified

- **2026-10-08, Gazebo end to end after the move** (fmoc from `isaacbot_amr_perception_bringup`
  `use_person_tracking:=true`, this package's launch, the world from `isaacbot_amr_world`), started with
  `follow_me/start`: the isaacbot followed the person through both 90° corners, a median 2.47 m
  behind, facing them (bearing above 43° in 1 of 225 samples), until the collision monitor
  stopped it next to a box (Known limits); the person walked out of view and the server searched,
  then gave up (`FAILED_TO_CONTROL`). The VDA 5050 path was not re-run (no MQTT broker at hand);
  only its error text changed.
- **2026-10-08, Gazebo end to end** (`follow_me_world.sdf`, headless, Nav 2 with the Following
  server, mission manager, drive mode, the VDA 5050 connector), before the move into this repo:
  - `startFollowing` from a fake master control: `FINISHED`, "Following started."; with the isaacbot
    not in Automatic: `FAILED`, reason in the `actionFailed` error.
  - The isaacbot followed the walking person continuously for about 13 m, through both 90° corners of
    the route, facing them throughout (bearing above 43° in 1 of 267 samples). It stayed a median
    2.4 m behind (the server's 0.5 m/s against the actor's 0.6 m/s).
  - A person briefly out of view: SEARCHING, then FOLLOWING again.
  - Switching to Assisted stopped following ("drive mode changed to ASSISTED").
  - The collision monitor slowed and stopped the follower's commands near a box; the isaacbot then
    waited there (see Known limits).

## Known limits

- **Forward only.** The Following server never reverses; a person who walks up to the isaacbot stops
  it (desired distance, then the collision monitor), they don't push it back.
- **No path planning.** The Following server drives straight at the person. With an obstacle
  between them, the collision monitor slows and stops the isaacbot, and it waits there until the
  person comes back into a clear line; walk around obstacles with room to spare. Nav 2's other
  dynamic-following approach (a BT re-planning ComputePathToPose + FollowPath to the moving
  goal) would route around them, at the cost of the server's simplicity.
- **No following during a mission.** Missions and the server share `cmd_vel_nav`.
- **Narrow view.** The D435i sees 87° (its colour image, which the person detector uses, 69°);
  the server turns toward the person continuously, which covers normal walking turns, but someone
  who steps sideways out of view quickly is lost. The isaacbot then stands and waits: it does not
  turn to look for them, so they have to come back into its view.
- **Anyone person-sized** is a person, and the camera mount is still an ASSUMPTION in the URDF
  (see `isaacbot_amr_perception`'s README).
