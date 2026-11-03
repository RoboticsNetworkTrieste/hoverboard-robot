# Shared by the scripts in this folder, not meant to be run directly.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN="$REPO_ROOT/tools/docker-humble/run.sh"
MAPS_DIR="$REPO_ROOT/ros2/hoverboard_bringup/maps"
CONTAINER_MAPS_DIR=/ws/src/hoverboard_bringup/maps   # Same folder, seen from the container
DEFAULT_ESP_HOST=192.168.7.1                         # USB tethering

# Builds the workspace, then runs a hoverboard_bringup launch file with the given arguments.
# RViz starts when a display is available; RVIZ=false disables it.
run_stack() {
    local rviz="${RVIZ:-true}"
    if [[ -z "${DISPLAY:-}" ]]; then
        rviz=false
    fi
    if [[ "$rviz" == true ]]; then
        xhost +local: > /dev/null
    fi
    exec "$RUN" bash -c \
        'cd /ws && colcon build --symlink-install && source install/setup.bash && exec ros2 launch hoverboard_bringup "$@"' \
        stack "$@" "rviz:=$rviz"
}
