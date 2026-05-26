#!/usr/bin/env python3
"""Launch Point-LIO + RTAB-Map 3D SLAM (PointCloud2 input).

What it starts:
 - point_lio mapping_mid360.launch.py
 - rtabmap_slam rtabmap with 3D mapping params
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time'))
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Static TF: laser_frame -> base_link
    # Point-LIO publishes odom -> laser_frame by default. Nav2 expects base_link.
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_frame_to_base_link',
        arguments=['-0.24', '0', '0', '0', '3.1415926535', '0', 'laser_frame', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    ))

    # point_lio: launch node directly so we can disable Point-LIO's map->odom TF
    point_lio_config = os.path.join(
        get_package_share_directory('point_lio'), 'config', 'mid360.yaml'
    )
    point_lio_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            point_lio_config,
            {
                'use_sim_time': use_sim_time,
                # match mapping_mid360.launch.py behavior
                'use_imu_as_input': False,
                'prop_at_freq_of_imu': True,
                'check_satu': True,
                'init_map_size': 10,
                'point_filter_num': 3,
                'space_down_sample': True,
                'filter_size_surf': 0.5,
                'filter_size_map': 0.5,
                'cube_side_length': 1000.0,
                'runtime_pos_log_enable': False,
                'pcd_save_enable': True,

                # IMPORTANT: let RTAB-Map publish the real map->odom TF
                'publish_map_to_odom_tf': False,
            }
        ],
    )
    ld.add_action(point_lio_node)

    # rtabmap_slam: 3D mapping with PointCloud2 input
    rtabmap_params = '/home/thamyis/SLAM_ws/src/all_launch/config/rtabmap_pointlio_3d.yaml'
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('scan_cloud', '/cloud_registered'),
            ('odom', '/odom'),
        ],
    )
    ld.add_action(rtabmap_node)


    return ld
