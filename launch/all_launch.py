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
    camera_source = LaunchConfiguration('camera_source')
    all_launch_share = get_package_share_directory('all_launch')
    preferred_rviz_config = '/home/thamyis/SLAM_ws/person.rviz'
    rviz_config = (
        preferred_rviz_config
        if os.path.exists(preferred_rviz_config)
        else os.path.join(all_launch_share, 'rviz', 'person.rviz')
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time',
    )
    camera_source_arg = DeclareLaunchArgument(
        'camera_source',
        default_value='virtual',
        description="Camera source passed to mecanum_bot sim: 'virtual' or 'webcam'",
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

    mppi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                all_launch_share,
                'launch',
                'mppi.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # Keep legacy all_launch webcam node off; camera source is controlled by mecanum_bot.
            'use_logitech_camera': 'false',
        }.items(),
    )

    robot_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot'),
                'launch',
                'sim_bringup.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'camera_source': camera_source,
        }.items(),
    )

    return LaunchDescription([
        use_sim_time_arg,
        camera_source_arg,
        static_tf_node,
        slam_node,
        
        controller_switcher,
        mppi,
        robot_sim,
        rviz_node,
    ])
