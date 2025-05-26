#!/bin/bash

# Print commands and their arguments as they are executed.
set -x

SCRIPT_DIR=$(dirname "$(realpath "$0")")
FLEET_MGMT_DIR="$SCRIPT_DIR/../../.."

$SCRIPT_DIR/stop_control_center.sh
$FLEET_MGMT_DIR/center/docker_/build_run_catmux.py --catmux_session_file demo.yaml