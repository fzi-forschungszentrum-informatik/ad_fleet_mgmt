#!/bin/bash

# Print commands and their arguments as they are executed.
set -x

COMPOSITION_DIR=$(dirname "$(realpath "$0")")
$COMPOSITION_DIR/stop_composition.sh
catmux_create_session $COMPOSITION_DIR/composition.yaml -L composition -n comp__ --overwrite composition_dir=$COMPOSITION_DIR