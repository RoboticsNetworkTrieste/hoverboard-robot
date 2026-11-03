#!/usr/bin/env bash
# Starts the whole stack and builds a new map. Ctrl-C stops it: save the map first with save_map.sh.
# Usage: start_mapping.sh [esp_host]   (default: USB tethering)
set -euo pipefail
source "$(dirname "$0")/common.sh"

run_stack stack_mapping.launch.py "esp_host:=${1:-$DEFAULT_ESP_HOST}"
