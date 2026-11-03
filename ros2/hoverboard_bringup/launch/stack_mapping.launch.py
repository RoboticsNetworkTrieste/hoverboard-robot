"""Whole stack, building a new map: robot, slam_toolbox, Nav2 and RViz."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('hoverboard_bringup'), 'launch')
    rviz_config = os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('esp_host', default_value='192.168.7.1', description='ESP address, USB 192.168.7.1'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot.launch.py')),
            launch_arguments={'esp_host': LaunchConfiguration('esp_host')}.items()),
        # No map argument: nav.launch.py runs slam_toolbox
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav.launch.py'))),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
