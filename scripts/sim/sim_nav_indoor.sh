#!/usr/bin/env bash
# Copyright 2026 IsaacBot
# Licensed under the Apache License, Version 2.0.
#
# Simulated Ardennes + orchestrator. isaacbot_amr_indoor_nav_manager picks localization on its own:
# slam_toolbox while no saved map is selected, map_server + AMCL on the selected one, and
# odometry (map -> odom held) while no lidar or camera scan arrives. Kill the simulated lidar's
# bridge to watch the odometry fallback.
# Its map library lives in $ISAACBOT_AMR_SIM_MAPS_DIR/indoor (default ~/isaacbot_amr_sim_maps/indoor),
# standing in for the isaacbot's /maps volume. Ctrl+C stops everything.

source "$(dirname "${BASH_SOURCE[0]}")/sim_nav_common.sh"
mkdir -p "$MAPS_DIR/indoor" || die "cannot create $MAPS_DIR/indoor"
run_stack maps_dir:="$MAPS_DIR/indoor"
