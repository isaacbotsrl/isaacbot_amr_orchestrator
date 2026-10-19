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

"""Introspection tests for localization routing.

These build the LaunchDescriptions and inspect them; nothing is spawned, so the test
needs no ROS graph and no nav2 binaries.

There is one localization mode: bringup always starts isaacbot_amr_indoor_nav_manager, which
runs slam_toolbox or map_server + AMCL (indoor_localization.launch.py) and holds map -> odom
itself while no lidar or camera works. What is locked down here:

1. No launch file still takes `localization_source`: a leftover argument would accept a value
   and silently ignore it.
2. localization.launch.py starts AMCL and has lifecycle_manager manage it after map_server,
   otherwise AMCL is launched but never configured and never publishes map -> odom.
3. bringup starts the manager and Nav 2, and none of the fixed-mode localization.
"""

import importlib.util
import os

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.utilities import evaluate_parameters

_NAV_SHARE = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_MISSION_LAUNCH = os.path.join(
    _NAV_SHARE, "..", "isaacbot_amr_mission_manager", "launch", "isaacbot_amr_mission_manager.launch.py"
)


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _nav_launch(name):
    return _load(os.path.join(_NAV_SHARE, "launch", name), name.replace(".", "_"))


def _perform(context, value):
    """Resolve a name that may be a plain str or a list of substitutions.

    Node stores `name=` verbatim when it is a literal; ComposableNode normalizes it into
    a substitution list. Both appear here.
    """
    if isinstance(value, str):
        return value
    return "".join(context.perform_substitution(s) for s in value)


def _declared(launch_description):
    return [e.name for e in launch_description.entities if isinstance(e, DeclareLaunchArgument)]


def _localization_actions(use_composition):
    """Resolve localization.launch.py and report what it would start."""
    module = _nav_launch("localization.launch.py")
    launch_description = module.generate_launch_description()

    context = LaunchContext()
    # Seeded before the declarations are visited: DeclareLaunchArgument only supplies a
    # default when the configuration is unset, and `map` deliberately has no default.
    context.launch_configurations.update(
        {
            "namespace": "isaacbot",
            "map": "/maps/map.yaml",
            "use_composition": use_composition,
            "initial_pose_x": "1.5",
            "initial_pose_y": "-2.25",
            "initial_pose_yaw": "0.75",
        }
    )
    for entity in launch_description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(context)

    opaque = next(
        e for e in launch_description.entities if isinstance(e, OpaqueFunction)
    )

    plain, composable, node_names, amcl_overrides = [], [], None, None
    for action in opaque.execute(context):
        if isinstance(action, GroupAction):
            for node in action._GroupAction__actions:
                if not isinstance(node, Node):
                    continue
                plain.append(_perform(context, node._Node__node_name))
                # launch_ros stores parameters YAML-encoded; evaluate_parameters decodes
                # them, so this is exactly what the node would receive.
                dicts = [p for p in (node._Node__parameters or []) if isinstance(p, dict)]
                for params in evaluate_parameters(context, dicts):
                    if "node_names" in params:
                        node_names = list(params["node_names"])
                    if "initial_pose.x" in params:
                        amcl_overrides = dict(params)
        elif isinstance(action, LoadComposableNodes):
            composable = [
                _perform(context, d.node_name)
                for d in action._LoadComposableNodes__composable_node_descriptions
            ]

    return plain, composable, node_names, amcl_overrides


def test_no_launch_file_takes_localization_source():
    per_file = {
        "bringup": _nav_launch("bringup.launch.py").generate_launch_description(),
        "localization": _nav_launch("localization.launch.py").generate_launch_description(),
        "indoor_localization": _nav_launch(
            "indoor_localization.launch.py").generate_launch_description(),
        "mission_manager": _load(_MISSION_LAUNCH, "mission_launch").generate_launch_description(),
    }
    for name, launch_description in per_file.items():
        assert "localization_source" not in _declared(launch_description), name


@pytest.mark.parametrize("use_composition", ["True", "False"])
def test_localization_launch_starts_map_server_and_amcl(use_composition):
    plain, composable, node_names, _ = _localization_actions(use_composition)
    started = composable if use_composition == "True" else plain
    assert "map_server" in started and "amcl" in started, started
    # Order matters: lifecycle_manager transitions in list order, and AMCL blocks
    # waiting for the map, so map_server has to be activated first.
    assert node_names == ["map_server", "amcl"], node_names


def test_initial_pose_overrides_are_floats():
    _, _, _, overrides = _localization_actions("False")
    assert overrides == {
        "initial_pose.x": 1.5,
        "initial_pose.y": -2.25,
        "initial_pose.yaw": 0.75,
    }, overrides
    # AMCL declares these as doubles; launch substitutions would hand it strings.
    assert all(isinstance(v, float) for v in overrides.values()), overrides


def _included_file(action):
    """File name of an IncludeLaunchDescription, from its (unresolved) location."""
    import re

    match = re.search(r"([\w.]+\.py)['\")]*$", str(action.launch_description_source.location))
    return match.group(1) if match else str(action.launch_description_source.location)


def _bringup_actions():
    """Which of bringup's grouped actions run."""
    from launch.actions import IncludeLaunchDescription

    launch_description = _nav_launch("bringup.launch.py").generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update({
        "namespace": "isaacbot",
        "use_composition": "True",
    })
    running = []
    for entity in launch_description.entities:
        if not isinstance(entity, GroupAction):
            continue
        for action in entity._GroupAction__actions:
            condition = action.condition
            if condition is not None and not condition.evaluate(context):
                continue
            if isinstance(action, IncludeLaunchDescription):
                running.append(_included_file(action))
            elif isinstance(action, Node):
                running.append(_perform(context, action.node_executable))
    return running


def test_bringup_runs_nav2_and_the_manager_only():
    running = _bringup_actions()
    assert "isaacbot_amr_nav.launch.py" in running, running
    assert "indoor_nav_manager_node" in running, running
    # The manager starts these itself; bringup starting them too would make two owners of
    # map -> odom.
    for absent in ("slam_launch.py", "localization.launch.py", "map_autosaver_node"):
        assert absent not in running, (absent, running)


def _indoor_includes(mode):
    """Included files for a mode, with their literal (non-substitution) arguments."""
    from launch.actions import IncludeLaunchDescription
    from launch.substitutions import TextSubstitution

    launch_description = _nav_launch("indoor_localization.launch.py").generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update({"mode": mode})
    group = [e for e in launch_description.entities if isinstance(e, GroupAction)][0]
    included = {}
    for action in group._GroupAction__actions:
        if isinstance(action, IncludeLaunchDescription) and action.condition.evaluate(context):
            literal = {}
            for key, value in action.launch_arguments:
                # Literals are str or [TextSubstitution]; everything else is a LaunchConfiguration.
                parts = value if isinstance(value, (list, tuple)) else [value]
                if all(isinstance(v, (str, TextSubstitution)) for v in parts):
                    literal[_perform(context, key)] = "".join(
                        v if isinstance(v, str) else _perform(context, [v]) for v in parts)
            included[_included_file(action)] = literal
    return included


def test_indoor_localization_swaps_slam_and_amcl():
    assert list(_indoor_includes("mapping")) == ["slam_launch.py"]
    localization = _indoor_includes("localization")
    assert list(localization) == ["localization.launch.py"]
    args = localization["localization.launch.py"]
    # Out of nav2_container so stopping this launch never touches Nav 2.
    assert args["use_composition"] == "False"
    assert "localization_source" not in args
