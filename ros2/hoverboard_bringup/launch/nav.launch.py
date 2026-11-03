"""Nav2 with slam_toolbox (no map argument) or with AMCL on a saved map."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    bringup = get_package_share_directory('hoverboard_bringup')
    nav2_launch = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    map_file = LaunchConfiguration('map')
    slam_mode = PythonExpression(["'", map_file, "' == ''"])
    common = {
        'params_file': os.path.join(bringup, 'config', 'nav2.yaml'),
        'use_sim_time': 'false',
        'autostart': 'true',
    }

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='',
                              description='Map yaml for AMCL localization; empty runs slam_toolbox instead'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'slam.launch.py')),
            condition=IfCondition(slam_mode)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch, 'localization_launch.py')),
            launch_arguments={**common, 'map': map_file}.items(),
            condition=UnlessCondition(slam_mode)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch, 'navigation_launch.py')),
            launch_arguments=common.items()),
    ])
