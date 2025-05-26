#!/bin/bash

# Print commands and their arguments as they are executed.
set -x

SCRIPT_DIR=$(dirname "$(realpath "$0")")
FLEET_MGMT_DIR="$SCRIPT_DIR/../../.."

$SCRIPT_DIR/stop_simulated_vehicle.sh
$FLEET_MGMT_DIR/simulated_fleet/docker_/build_run_catmux.py --catmux_session_file demo.yaml --catmux_params "rosbag=baustelle_1,vehicle_name=shuttle_anna"