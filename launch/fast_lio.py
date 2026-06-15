#!/usr/bin/env python3
"""Combined launch file to run multiple launch files (NO RViz)."""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation time'
    ))
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Static transform: map -> odom (fixed relationship) ---
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    ld.add_action(map_to_odom_tf)
    
    # --- Static transform: laser_frame -> base_link (lidar mounting offset)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_frame_to_base_link',
        arguments=['--x', '-0.35', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'laser_frame', '--child-frame-id', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    ld.add_action(static_tf_node)

    # pointcloud_to_laserscan
    try:
        pkg_p2l = get_package_share_directory('pointcloud_to_laserscan')
        p2l_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_p2l, 'launch', 'sample_pointcloud_to_laserscan_launch_copy.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )
        ld.add_action(p2l_launch)
    except Exception as e:
        print(f"WARNING: could not include pointcloud_to_laserscan launch: {e}")

    # point_lio (disable its RViz by passing rviz:=false)
    try:
        pkg_point_lio = get_package_share_directory('fast_lio')
        point_lio_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_point_lio, 'launch', 'mapping.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'rviz': 'true',
            }.items(),
        )
        ld.add_action(point_lio_launch)
    except Exception as e:
        print(f"WARNING: could not include point_lio launch: {e}")

    # slam_toolbox
    try:
        slam_params = '/home/mobimobi/Desktop/TayoAMR/src/all_launch/config/mapper_params_online_async.yaml'
        slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': use_sim_time}],
        )
        ld.add_action(slam_node)
    except Exception as e:
        print(f"WARNING: could not create slam_toolbox node: {e}")

    # NOTE: Removed the RViz2 node from this combined launch.

    return ld
