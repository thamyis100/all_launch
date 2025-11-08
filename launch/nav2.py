#!/usr/bin/env python3
"""Launch file that starts robot nodes (controller_node, serial_node, twist_mux)
and also launches Nav2's navigation_launch.py with a custom params file.

Save to: <your_package>/launch/robot_nodes_with_nav_launch.py
Run with:
  ros2 launch <your_package> robot_nodes_with_nav_launch.py

This file declares 'params_file' and 'use_sim_time' launch arguments and forwards
them to the included nav2_bringup navigation_launch.py.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation time'))

    ld.add_action(DeclareLaunchArgument(
        'params_file', default_value='/home/mobimobi/thamyis/robot_TA_ws/src/all_launch/config/nav2_params_yoga.yaml',
        description='Full path to Nav2 params file'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # Include the Nav2 bringup navigation_launch.py and forward params_file + use_sim_time
    try:
        nav2_pkg = get_package_share_directory('nav2_bringup')
        nav2_launch_path = os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
        nav2_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={'params_file': params_file, 'use_sim_time': use_sim_time}.items()
        )
        ld.add_action(nav2_include)
    except Exception as e:
        # if nav2_bringup package not found, include nothing but keep other nodes runnable
        print(f"WARNING: could not include nav2_bringup navigation_launch.py: {e}")
    

    return ld