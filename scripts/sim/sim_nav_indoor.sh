#!/usr/bin/env bash
# Copyright 2026 Mechatronics Academy
# Licensed under the Apache License, Version 2.0.
#
# Simulated Ardennes + orchestrator, isaacbot_amr_indoor_nav_manager switching between slam_toolbox
# (mapping) and map_server + AMCL (saved maps) at runtime (localization_source:=indoor).
# Its map library lives in $ISAACBOT_AMR_SIM_MAPS_DIR/indoor (default ~/isaacbot_amr_sim_maps/indoor),
# standing in for the isaacbot's /maps volume. Ctrl+C stops everything.

source "$(dirname "${BASH_SOURCE[0]}")/sim_nav_common.sh"
mkdir -p "$MAPS_DIR/indoor" || die "cannot create $MAPS_DIR/indoor"
run_stack indoor maps_dir:="$MAPS_DIR/indoor"
