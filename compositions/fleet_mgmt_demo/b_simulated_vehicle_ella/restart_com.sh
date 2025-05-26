#!/bin/bash

# Print commands and their arguments as they are executed.
set -x

SESSION_DIR=/session/actors/machine_b/demo_simulated_vehicle_ella
$(dirname "$(realpath "$0")")/../../shared/restart_com_for_session.py --session-dir $SESSION_DIR
