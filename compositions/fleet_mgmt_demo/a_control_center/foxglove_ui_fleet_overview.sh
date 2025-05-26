#!/bin/bash

# Print commands and their arguments as they are executed.
set -x

DS=foxglove-websocket
DS_URL=ws://localhost:8765/

LAYOUT=lay_0dZfoW3X5JI5qV1W # os_fleet_overview_demo

foxglove-studio "foxglove://open?ds=$DS&ds.url=$DS_URL&layoutId=$LAYOUT"
