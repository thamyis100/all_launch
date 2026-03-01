#!/usr/bin/env python3
"""Combined launch file to run multiple launch files and RViz.

Saves/places: <your_ros2_package>/launch/ros2_combined_launch.py

What it starts:
 - pointcloud_to_laserscan sample_pointcloud_to_laserscan_launch.py (use_sim_time:=True)
 - point_lio mapping_mid360.launch.py (use_sim_time:=True)
 - slam_gmapping slam_gmapping.launch.py (use_sim_time:=True)
 - rviz2 with nav2_default_view.rviz

Usage example:
  ros2 launch <your_package> ros2_combined_launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node




def generate_launch_description():
    ld = LaunchDescription()
    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time'))
    use_sim_time = LaunchConfiguration('use_sim_time')

    # # --- Static transform: laser_frame -> base_link
    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='laser_frame_to_base_link',
    #     arguments=['-0.24', '0', '0', '0', '0', '0', 'laser_frame', 'base_link'],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen'
    # )
    # ld.add_action(static_tf_node)

    # slam_toolbox: launch node directly with custom params and tf remaps
    try:
        pkg_gm = get_package_share_directory('slam_toolbox')
        # absolute path to your params file (use your path)
        slam_params = '/home/thamyis/SLAM_ws/src/all_launch/config/mapper_params_online_async.yaml'

        slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',   # matches "ros2 run slam_toolbox async_slam_toolbox_node"
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': use_sim_time}],
            # remappings=[
            #     ('/tf', '/tf_slam'),
            #     ('/tf_static', '/tf_static_slam'),
            # ],
        )
        ld.add_action(slam_node)
    except Exception as e:
        print(f"WARNING: could not create slam_toolbox node: {e}")

    # RViz2 node running with the Nav2 default view
    rviz_config = '/home/thamyis/SLAM_ws/person.rviz'
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    ld.add_action(rviz_node)

    return ld
