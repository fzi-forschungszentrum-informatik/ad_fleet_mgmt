#!/bin/bash

# Print commands and their arguments as they are executed.
set -x

SESSION_DIR=/session/actors/machine_a/demo_control_center
$(dirname "$(realpath "$0")")/../../shared/restart_com_for_session.py --session-dir $SESSION_DIR
