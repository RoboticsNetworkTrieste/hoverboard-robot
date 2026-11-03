#!/usr/bin/env bash
# Saves the map being built by start_mapping.sh, which must still be running.
# Usage: save_map.sh <map name>
set -euo pipefail
source "$(dirname "$0")/common.sh"

if [[ $# -ne 1 || ! "$1" =~ ^[A-Za-z0-9_-]+$ ]]; then
    echo "Usage: $0 <map name>   (letters, digits, - and _ only)" >&2
    exit 1
fi
if [[ -e "$MAPS_DIR/$1.yaml" ]]; then
    echo "Map '$1' already exists in $MAPS_DIR: choose another name or delete it first." >&2
    exit 1
fi

mkdir -p "$MAPS_DIR"
"$RUN" ros2 run nav2_map_server map_saver_cli -f "$CONTAINER_MAPS_DIR/$1"
echo "Saved $MAPS_DIR/$1.yaml and .pgm. Use it with: scripts/start_navigation.sh $1"
