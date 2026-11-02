#!/usr/bin/env bash
# Manual driving session: bridge, LIDAR driver and RViz in the background, keyboard teleop in this terminal.
# Usage: teleop_debug.sh [esp_host]   (default 192.168.7.1: USB; WiFi station 192.168.1.50, WiFi AP 192.168.4.1)
# Logs: tools/docker-humble/run.sh tail -f /ws/log/teleop_debug/bridge.log
set -euo pipefail

ESP_HOST="${1:-192.168.7.1}"

if [[ ! -t 0 ]]; then
    echo "teleop_debug.sh needs an interactive terminal for the keyboard" >&2
    exit 1
fi
xhost +local: > /dev/null

# Everything runs in one container: the LIDAR driver needs the pseudo terminal in the bridge's /tmp.
exec "$(dirname "$0")/run.sh" bash -c '
LOG_DIR=/ws/log/teleop_debug
LIDAR_PTY=/tmp/ttyLIDAR
mkdir -p "$LOG_DIR"

echo "Building the workspace..."
cd /ws
if ! colcon build --symlink-install > "$LOG_DIR/build.log" 2>&1; then
    cat "$LOG_DIR/build.log"
    exit 1
fi
source /ws/install/setup.bash

pids=()
start() {
    local name=$1
    shift
    "$@" > "$LOG_DIR/$name.log" 2>&1 < /dev/null &
    pids+=($!)
}
stop() {
    # SIGTERM: background jobs of a non-interactive shell ignore SIGINT
    kill -TERM "${pids[@]}" 2> /dev/null
    wait "${pids[@]}" 2> /dev/null
}
trap stop EXIT

echo "Starting the bridge towards $1..."
start bridge ros2 launch hoverboard_bridge bridge.launch.py esp_host:="$1"
for _ in {1..50}; do
    [ -e "$LIDAR_PTY" ] && break
    sleep 0.2
done
if [ ! -e "$LIDAR_PTY" ]; then
    cat "$LOG_DIR/bridge.log"
    echo "The bridge did not create $LIDAR_PTY" >&2
    exit 1
fi

run_lidar() {
    # The driver exits when it gets no data for 3 s
    while true; do
        ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node --ros-args \
            -p product_name:=LDLiDAR_LD19 -p topic_name:=scan -p frame_id:=laser_link \
            -p port_name:="$LIDAR_PTY" -p port_baudrate:=230400 -p laser_scan_dir:=true
        sleep 2
    done
}
start lidar run_lidar
start laser_tf ros2 run tf2_ros static_transform_publisher \
    --z 0.20 --frame-id base_link --child-frame-id laser_link
start rviz rviz2 -d /ws/src/hoverboard_bridge/config/teleop_debug.rviz

echo "Logs: tools/docker-humble/run.sh tail -f $LOG_DIR/bridge.log"
echo "The robot stops by itself about 0.5 s after the last key press. Ctrl-C closes everything."
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.6
' teleop_debug "$ESP_HOST"
