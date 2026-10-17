# Sources ROS2 Humble, the LIDAR driver underlay and the workspace overlay (if built).
source /opt/ros/humble/setup.bash
source /opt/ldlidar_ws/install/setup.bash
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi
