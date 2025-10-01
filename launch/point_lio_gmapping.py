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

Note: This assumes the three packages (pointcloud_to_laserscan, point_lio, slam_gmapping)
provide the referenced launch files under their "launch" directories. Adjust paths or
launch file names if your installation differs.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # pointcloud_to_laserscan: sample_pointcloud_to_laserscan_launch.py
    try:
        pkg_p2l = get_package_share_directory('pointcloud_to_laserscan')
        p2l_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_p2l, 'launch', 'sample_pointcloud_to_laserscan_launch.py')
            ),
            launch_arguments={'use_sim_time': 'True'}.items(),
        )
        ld.add_action(p2l_launch)
    except Exception as e:
        # if package / launch not found, still continue so user can see error on screen
        print(f"WARNING: could not include pointcloud_to_laserscan launch: {e}")

    # point_lio: mapping_mid360.launch.py
    try:
        pkg_point_lio = get_package_share_directory('point_lio')
        point_lio_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_point_lio, 'launch', 'mapping_mid360.launch.py')
            ),
            launch_arguments={'use_sim_time': 'True'}.items(),
        )
        ld.add_action(point_lio_launch)
    except Exception as e:
        print(f"WARNING: could not include point_lio launch: {e}")

    # slam_gmapping: slam_gmapping.launch.py
    try:
        pkg_gm = get_package_share_directory('slam_gmapping')
        gmapping_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gm, 'launch', 'slam_gmapping.launch.py')
            ),
            # pass use_sim_time=True as well to unify clocks
            launch_arguments={'use_sim_time': 'True'}.items(),
        )
        ld.add_action(gmapping_launch)
    except Exception as e:
        print(f"WARNING: could not include slam_gmapping launch: {e}")

    # RViz2 node running with the Nav2 default view
    rviz_config = '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )
    ld.add_action(rviz_node)

    return ld
