"""Everything that runs next to the ESP link: bridge, robot description and LIDAR driver."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('hoverboard_bringup')
    bridge_launch = os.path.join(get_package_share_directory('hoverboard_bridge'), 'launch', 'bridge.launch.py')
    lidar_params = os.path.join(bringup, 'config', 'lidar.yaml')
    with open(os.path.join(bringup, 'urdf', 'hoverboard.urdf')) as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('esp_host', default_value='192.168.1.50',
                              description='WiFi STA static IP (192.168.1.50), WiFi AP 192.168.4.1, USB 192.168.7.1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch),
            launch_arguments={'esp_host': LaunchConfiguration('esp_host')}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        # Exits while the bridge has not created the pseudo terminal yet, so it is respawned
        Node(package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node', name='ldlidar',
             parameters=[lidar_params], respawn=True, respawn_delay=2.0),
        Node(package='hoverboard_bridge', executable='scan_resampler', parameters=[lidar_params]),
    ])
