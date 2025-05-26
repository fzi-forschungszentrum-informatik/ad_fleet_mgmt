#!/bin/bash

# Print commands and their arguments as they are executed.
set -x

catmux -L composition kill-session

SCRIPT_DIR=$(dirname "$(realpath "$0")")
$SCRIPT_DIR/stop_control_center.sh
$SCRIPT_DIR/stop_com.sh