import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('hoverboard_bridge'), 'config', 'bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('esp_host', default_value='192.168.1.50',
                              description='WiFi STA static IP (192.168.1.50), WiFi AP 192.168.4.1, USB 192.168.7.1'),
        Node(
            package='hoverboard_bridge',
            executable='bridge_node',
            name='hoverboard_bridge',
            output='screen',
            parameters=[params, {'esp_host': LaunchConfiguration('esp_host')}],
        ),
    ])
