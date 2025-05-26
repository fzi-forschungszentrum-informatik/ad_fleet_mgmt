#!/bin/bash

# Print commands and their arguments as they are executed.
set -x

script_path=$(realpath "$0")
COMPOSITION_DIR=$(dirname "$script_path")
DEMO_DIR=$(dirname "$COMPOSITION_DIR")
DEMO_NAME=$(basename "$DEMO_DIR")

LINK_DIR=~/$DEMO_NAME
COMPOSITION_DIR=$(dirname "$(realpath "$0")")

if [ ! -e "$LINK_DIR" ]; then
    ln -sf "$COMPOSITION_DIR" "$LINK_DIR"
    echo "Link created."
else
    echo "Link already exists. Doing nothing."
fi