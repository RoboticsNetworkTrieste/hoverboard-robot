#!/usr/bin/env bash
# Keyboard driving on /cmd_vel, with the stack already running. The robot stops about 0.5 s after the last key press.
set -euo pipefail
source "$(dirname "$0")/common.sh"

exec "$RUN" ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.6
