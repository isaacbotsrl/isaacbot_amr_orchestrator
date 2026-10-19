#!/usr/bin/env python3

# Copyright 2025 IsaacBot
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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString, RewrittenYaml

def generate_launch_description():
    isaacbot_amr_navigation = FindPackageShare("isaacbot_amr_navigation")
    launch_dir = PathJoinSubstitution([isaacbot_amr_navigation, "launch"])

    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    maps_dir = LaunchConfiguration("maps_dir")
    namespace = LaunchConfiguration("namespace")
    observation_topic = LaunchConfiguration("observation_topic")
    camera_depth_topic = LaunchConfiguration("camera_depth_topic")
    # Local costmap sources: the lidar scan always, the D435i depth cloud only when asked for
    # (use_camera, ISAACBOT_AMR_ORCH_NAV_USE_CAMERA). Not ISAACBOT_AMR_SYSTEM_USE_CAMERA: that only starts the
    # camera (ardennes-sensors), which follow-me and perception use without Nav 2 relying on it.
    use_camera = LaunchConfiguration("use_camera")
    observation_sources = PythonExpression(
        [
            "'laserscan depth_camera' if '",
            use_camera,
            "'.strip().lower() in ('true', '1', 'yes', 'on') else 'laserscan'",
        ]
    )
    params_file = LaunchConfiguration("params_file")
    robot_model = LaunchConfiguration("robot_model")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically startup the nav2 stack.",
    )
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level.",
        choices=["debug", "info", "warning", "error"],
    )
    declare_maps_dir_arg = DeclareLaunchArgument(
        "maps_dir",
        default_value="/maps",
        description=(
            "Where isaacbot_amr_indoor_nav_manager keeps its maps, places and last pose "
            "(the isaacbot-maps volume in ardennes-orchestrator)."
        ),
    )
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable(
            "ISAACBOT_AMR_SYSTEM_NAMESPACE",
            default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        ),
        description="Add namespace to all launched nodes.",
    )
    declare_observation_topic_arg = DeclareLaunchArgument(
        "observation_topic",
        default_value="scan",
        description=(
            "Lidar LaserScan topic for the costmaps' stvl_layer and collision_monitor. "
            "isaacbot_amr_rs16_lidar publishes it as 'scan'. AMCL and slam_toolbox read "
            "localization/scan, which isaacbot_amr_indoor_nav_manager relays from the lidar "
            "or the camera."
        ),
    )
    declare_use_camera_arg = DeclareLaunchArgument(
        "use_camera",
        default_value=EnvironmentVariable("ISAACBOT_AMR_ORCH_NAV_USE_CAMERA", default_value="false"),
        description=(
            "Add the RealSense depth cloud as a local-costmap observation source "
            "(ISAACBOT_AMR_ORCH_NAV_USE_CAMERA). The camera itself is ISAACBOT_AMR_SYSTEM_USE_CAMERA, in "
            "ardennes-sensors."
        ),
    )
    declare_camera_depth_topic_arg = DeclareLaunchArgument(
        "camera_depth_topic",
        default_value="camera/depth/points",
        description="PointCloud2 topic of the depth camera (isaacbot_amr_perception_bringup).",
    )
    declare_params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [isaacbot_amr_navigation, "config", "isaacbot_amr_nav_params.yaml"]
        ),
        description="Path to the parameters file to use for all nav2 related nodes",
    )

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable(name="ROBOT_MODEL_NAME", default_value="ardennes"),
        description="Specify robot model",
        choices=["ardennes"],
    )
    declare_use_composition_arg = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup.",
    )
    declare_use_respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true.",
    )

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time}

    namespace_ext = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])
    # Nav 2's global frame is always <namespace>/map. isaacbot_amr_indoor_nav_manager owns
    # map -> odom through the sub-stack it runs (slam_toolbox, or map_server + AMCL), and holds
    # it itself while no lidar or camera works. The local costmap is a rolling window and
    # always stays on <namespace>/odom -- it is deliberately not tokenised.
    global_frame = PythonExpression(["'", namespace_ext, "' + 'map'"])

    bb_padding = 0.04
    # Outer tyre edges from the CAD, matching isaacbot_amr_description's wheel_01.yaml:
    # wheelbase / 2 + tyre_radius and wheel_separation / 2 + wheel_width / 2 + tyre_y_offset
    # (the tyre sits 1.71 mm inboard of the wheel joint).
    half_length = 0.503 / 2 + 0.1699  # 0.4214
    half_width = 0.617 / 2 + 0.108 / 2 - 0.00171  # 0.3608
    robot_bounding_box = {
        "ardennes": {
            "min_x": round(-half_length - bb_padding, 4),
            "min_y": round(-half_width - bb_padding, 4),
            "max_x": round(half_length + bb_padding, 4),
            "max_y": round(half_width + bb_padding, 4),
        },
    }

    def override_params_file(robot_model_name):
        bounding_box = robot_bounding_box[robot_model_name]
        params = ReplaceString(
            source_file=params_file,
            replacements={
                "<namespace>/": namespace_ext,
                "<min_x>": str(bounding_box["min_x"]),
                "<max_x>": str(bounding_box["max_x"]),
                "<min_y>": str(bounding_box["min_y"]),
                "<max_y>": str(bounding_box["max_y"]),
                "<global_frame>": global_frame,
                "<observation_topic>": observation_topic,
                "<observation_sources>": observation_sources,
                "<camera_depth_topic>": camera_depth_topic,
            },
            condition=IfCondition(
                PythonExpression(["'", robot_model, f"' == '{robot_model_name}'"])
            ),
        )

        return params

    params_file = override_params_file("ardennes")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace),
            Node(
                condition=IfCondition(use_composition),
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[configured_params, {"autostart": autostart}],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([launch_dir, "isaacbot_amr_nav.launch.py"])
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                }.items(),
            ),
            # The manager picks and runs localization (SLAM, AMCL, or odometry when no lidar or
            # camera works). It gets the namespaced params file so the sub-stack it launches
            # sees the same <namespace>/ substitutions as Nav 2. It is not a declared dependency
            # of isaacbot_amr_navigation (it depends on this package's launch files);
            # isaacbot_amr_autonomy pulls both in.
            Node(
                package="isaacbot_amr_indoor_nav_manager",
                executable="indoor_nav_manager_node",
                name="indoor_nav_manager",
                parameters=[
                    {
                        "localization_params_file": params_file,
                        "maps_dir": maps_dir,
                        "use_sim_time": use_sim_time,
                        "log_level": log_level,
                    }
                ],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
        ]
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            declare_autostart_arg,
            declare_log_level_arg,
            declare_maps_dir_arg,
            declare_namespace_arg,
            declare_observation_topic_arg,
            declare_use_camera_arg,
            declare_camera_depth_topic_arg,
            declare_params_file_arg,
            declare_robot_model_arg,
            declare_use_composition_arg,
            declare_use_respawn_arg,
            declare_use_sim_time_arg,
            bringup_cmd_group,
        ]
    )
