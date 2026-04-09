#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    all_launch_share = get_package_share_directory('all_launch')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )


    controller_switcher = Node(
        package='all_launch',
        executable='controller_switcher.py',
        name='controller_switcher',
        output='screen',
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_frame_to_base_link',
        arguments=['-0.24', '0', '0', '0', '0', '0', 'laser_frame', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(all_launch_share, 'config', 'mapper_params_online_async.yaml'),
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(all_launch_share, 'rviz', 'person.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    mppi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                all_launch_share,
                'launch',
                'mppi.py',
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    robot_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot'),
                'launch',
                'sim_bringup.launch.py',
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    return LaunchDescription([
        use_sim_time_arg,
        static_tf_node,
        slam_node,
        rviz_node,
        controller_switcher,
        mppi,
        robot_sim,
    ])
