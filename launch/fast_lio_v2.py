#!/usr/bin/env python3
"""Ikunio framework launch file: uses lio_interface + sensor_scan_generation."""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # --- Arguments ---
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation time'
    ))
    ld.add_action(DeclareLaunchArgument(
        'rviz', default_value='false', description='Launch RViz'
    ))
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('rviz')

    # --- Static TF: map -> odom (placeholder, will be overwritten by relocalization) ---
    # This static transform ensures the tree is never broken.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_placeholder',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    ld.add_action(map_to_odom)

    # --- Static TF: base_link -> base_footprint (Nav2 requires base_footprint) ---
    # Your robot currently has base_link and laser_frame. Create base_footprint as a child of base_link.
    base_link_to_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_footprint',
        arguments=['-0.35', '0', '0', '0', '0', '0', 'laser_frame', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    ld.add_action(base_link_to_footprint)

    # --- Static TF: laser_frame -> base_link (your LiDAR mounting offset) ---
    static_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_frame_to_base_link',
        arguments=['-0.35', '0', '0', '0', '0', '0', 'laser_frame', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    ld.add_action(static_laser)

    # Add chassis frame (required by Ikunio's sensor_scan_generation)
    base_footprint_to_chassis = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_chassis',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'chassis'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    ld.add_action(base_footprint_to_chassis)

    # ==================== IKUNIO BRIDGE 1: lio_interface ====================
    # Converts FAST-LIO's 3D odometry into clean 2D TF: odom -> base_footprint
    try:
        pkg_lio_int = get_package_share_directory('lio_interface')
        lio_interface_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_lio_int, 'launch', 'fastlio_lio_interface_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'odom_topic': '/odom',          # FAST-LIO's default odometry topic
                'odom_frame': 'odom',
                'base_frame': 'base_link',     # Nav2 expects base_footprint
                'child_frame': 'base_link',
            }.items(),
        )
        ld.add_action(lio_interface_launch)
        print("✅ lio_interface added: publishes odom -> base_footprint (2D)")
    except Exception as e:
        print(f"❌ lio_interface missing: {e}")

    # ==================== IKUNIO BRIDGE 2: sensor_scan_generation ====================
    # Publishes /odom topic and /registered_scan (cleaned point cloud)
    try:
        pkg_senscan = get_package_share_directory('sensor_scan_generation')
        senscan_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_senscan, 'launch', 'sensor_scan_generation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'input_cloud_topic': '/cloud_registered',   # From FAST-LIO
                'output_cloud_topic': '/registered_scan',
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
            }.items(),
        )
        ld.add_action(senscan_launch)
        print("✅ sensor_scan_generation added: publishes /odom and /registered_scan")
    except Exception as e:
        print(f"❌ sensor_scan_generation missing: {e}")

    # ==================== pointcloud_to_laserscan (using /registered_scan) ====================
    p2l_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'laser_frame',
            'transform_tolerance': 0.01,
            'min_height': -0.1,      # Adjust to your LiDAR's height above ground
            'max_height': 0.1,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00872665,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 20.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        remappings=[
            ('cloud_registered', 'lidar_frame_pcd'),  # From sensor_scan_generation
            ('scan', '/scan'),
        ],
        output='screen'
    )
    ld.add_action(p2l_node)
    print("✅ pointcloud_to_laserscan: converting /registered_scan -> /scan")

    # ==================== FAST-LIO (your existing LIO) ====================
    try:
        pkg_flio = get_package_share_directory('fast_lio')
        flio_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_flio, 'launch', 'mapping.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'rviz': launch_rviz,
                'config': 'mid360.yaml',      # Change to your config file
            }.items(),
        )
        ld.add_action(flio_launch)
        print("✅ FAST-LIO started")
    except Exception as e:
        print(f"❌ FAST-LIO not found: {e}")

    # ==================== SLAM Toolbox ====================
    slam_params = '/home/mobimobi/Desktop/TayoAMR/src/all_launch/config/mapper_params_online_async.yaml'
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time}],
        remappings=[('scan', '/scan')]
    )
    ld.add_action(slam_node)
    print("✅ SLAM Toolbox started")

    return ld