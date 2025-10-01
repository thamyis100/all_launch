#!/usr/bin/env python3
"""Launch file: amcl_with_map_and_nav_launch.py

Starts:
 - nav2_map_server (map_server) with a YAML map
 - nav2_amcl (amcl) with params file
 - nav2_lifecycle_manager to autostart map_server and amcl
 - Includes pointcloud_to_laserscan sample launch (use_sim_time)
 - Starts RViz2 with nav2_default_view.rviz
 - Includes point_lio mapping_mid360.launch.py (use_sim_time)
 - Includes nav2_bringup navigation_launch.py (use_sim_time)

Save into a package's launch/ directory and run:
  ros2 launch <your_package> amcl_with_map_and_nav_launch.py map_yaml:=/home/you/maps/my_map.yaml amcl_params:=/path/to/amcl_params.yaml use_sim_time:=True

This file attempts to resolve package share directories for included launches; if a launch file name or package path differs on your system, edit the paths accordingly.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(DeclareLaunchArgument('map_yaml', default_value='home/thamyis/SLAM_ws/src/all_launch/maps/fixed_map.yaml',
                                        description='Full path to map yaml file'))
    ld.add_action(DeclareLaunchArgument('amcl_params', default_value='home/thamyis/SLAM_ws/src/all_launch/config/amcl_config.yaml',
                                        description='Full path to amcl params file'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True',
                                        description='Use simulation time'))

    map_yaml = LaunchConfiguration('map_yaml')
    amcl_params = LaunchConfiguration('amcl_params')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Map server (nav2_map_server)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}],
    )

    # AMCL (nav2_amcl)
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, LaunchConfiguration('amcl_params')],
    )

    # Lifecycle manager to autostart map_server and amcl
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    ld.add_action(map_server)
    ld.add_action(amcl)
    ld.add_action(lifecycle_mgr)


    # Start RViz2 with the nav2 default view
    rviz_config = '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    ld.add_action(rviz_node)

    # Include nav2_bringup navigation_launch.py
    try:
        pkg_nav2 = get_package_share_directory('nav2_bringup')
        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )
        ld.add_action(nav2_launch)
    except Exception as e:
        ld.add_action(LogInfo(msg=f"[WARN] could not include nav2_bringup navigation_launch: {e}"))

    return ld
