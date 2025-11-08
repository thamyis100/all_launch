#!/usr/bin/env python3
"""Launch robot nodes and Nav2 navigation stack with a chosen controller config."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # --- Launch arguments ---
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation time'))
    ld.add_action(DeclareLaunchArgument(
        'params_file',
        default_value='/home/mobimobi/thamyis/robot_TA_ws/src/all_launch/config/mppi.yaml',
        description='Full path to Nav2 params file'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # --- Include Nav2 bringup ---
    try:
        nav2_pkg = get_package_share_directory('nav2_bringup')
        nav2_launch = os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={'params_file': params_file,
                              'use_sim_time': use_sim_time}.items()))
    except Exception as e:
        print(f"[WARN] Could not include nav2_bringup: {e}")

    return ld
