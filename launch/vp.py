#!/usr/bin/env python3
"""
Minimal launch file for Nav2 Velocity Smoother (VP Controller).

Usage:
  ros2 launch all_launch vp_controller_launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Default params file
    default_params = os.path.join(
        get_package_share_directory('all_launch'),
        'config',
        'vp_controller.yaml'
    )

    # Launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to vp_controller parameter file'
    )

    # Use_sim_time arg if needed
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )

    # Launch Node
    vp_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[LaunchConfiguration('params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/cmd_vel_raw', '/cmd_vel_nav_raw'),  # Input from Nav2 controller
            ('/cmd_vel_smoothed', '/cmd_vel'),     # Output to robot
        ]
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        vp_node
    ])
