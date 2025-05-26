#!/bin/bash

# Print commands and their arguments as they are executed.
set -x

SCRIPT_DIR=$(dirname "$(realpath "$0")")
FLEET_MGMT_DIR="$SCRIPT_DIR/../.."

$FLEET_MGMT_DIR/ros_communication_devcontainer/docker_/stop.py