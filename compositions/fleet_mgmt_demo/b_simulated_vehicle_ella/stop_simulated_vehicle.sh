#!/bin/bash

# Print commands and their arguments as they are executed.
set -x

SCRIPT_DIR=$(dirname "$(realpath "$0")")
FLEET_MGMT_DIR="$SCRIPT_DIR/../../.."

$FLEET_MGMT_DIR/simulated_fleet/docker_/stop.py