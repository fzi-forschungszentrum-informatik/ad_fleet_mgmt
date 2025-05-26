#!/bin/bash

# Print commands and their arguments as they are executed.
set -x

DS=foxglove-websocket
DS_URL=ws://localhost:8765/

LAYOUT=lay_0dZQuP9z4eeetAyH # vehicle_map

foxglove-studio "foxglove://open?ds=$DS&ds.url=$DS_URL&layoutId=$LAYOUT"
