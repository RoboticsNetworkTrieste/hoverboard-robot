#!/usr/bin/env bash
# Starts the whole stack on a saved map.
# Usage: start_navigation.sh <map name> [esp_host]   (default: USB tethering)
set -euo pipefail
source "$(dirname "$0")/common.sh"

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <map name> [esp_host]" >&2
    exit 1
fi
if [[ ! -f "$MAPS_DIR/$1.yaml" ]]; then
    echo "Map '$1' not found in $MAPS_DIR. Saved maps:" >&2
    find "$MAPS_DIR" -maxdepth 1 -name '*.yaml' -printf '  %f\n' 2> /dev/null | sed 's/\.yaml$//' >&2
    exit 1
fi

run_stack stack_navigation.launch.py "map:=$CONTAINER_MAPS_DIR/$1.yaml" "esp_host:=${2:-$DEFAULT_ESP_HOST}"
