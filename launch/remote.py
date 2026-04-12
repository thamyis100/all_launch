#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    all_launch_share = get_package_share_directory('all_launch')
    preferred_rviz_config = '/home/thamyis/SLAM_ws/person.rviz'
    rviz_config = (
        preferred_rviz_config
        if os.path.exists(preferred_rviz_config)
        else os.path.join(all_launch_share, 'rviz', 'person.rviz')
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        # VS Code snap can leak runtime env vars that crash rviz2 with core20 glibc symbols.
        additional_env={
            'GTK_EXE_PREFIX': '',
            'GTK_PATH': '',
            'GIO_MODULE_DIR': '',
            'GSETTINGS_SCHEMA_DIR': '',
            'GTK_IM_MODULE_FILE': '',
            'LOCPATH': '',
            'XDG_DATA_HOME': os.path.expanduser('~/.local/share'),
            'XDG_DATA_DIRS': '/usr/local/share:/usr/share:/var/lib/snapd/desktop',
        },
    )

    dock_pose_bridge_node = Node(
        package='all_launch',
        executable='dock_pose_bridge.py',
        name='dock_pose_bridge',
        output='screen',
        parameters=[{
            'global_frame': 'map',
            'base_frame': 'base_link',
            'odom_topic': '/odom',
            'cmd_vel_topic': '/cmd_vel_nav',
            'scan_topic': '/scan',
            'goal_topic': '/goal_pose',
            'dock_pose_topic': '/dock_pose',
            'undock_trigger_topic': '/undock_trigger',
            'dock_type': 'simple_charging_dock',
            'enable_visualization': False,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_node,
        dock_pose_bridge_node,
    ])