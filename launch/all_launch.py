#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_launch = LaunchConfiguration('robot_launch')
    point_lio_launch = LaunchConfiguration('point_lio_launch')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Use simulation time (True for rosbag --clock)'
    ))

    ld.add_action(DeclareLaunchArgument(
        'robot_launch', default_value='robot.py',
        description='Launch filename inside all_launch/launch'
    ))

    ld.add_action(DeclareLaunchArgument(
        'point_lio_launch', default_value='point_lio.py',
        description='Launch filename inside all_launch/launch'
    ))

    # Build launch paths safely (NO os.path.join with LaunchConfiguration)
    robot_launch_path = PathJoinSubstitution([
        FindPackageShare('all_launch'), 'launch', robot_launch
    ])

    point_lio_launch_path = PathJoinSubstitution([
        FindPackageShare('all_launch'), 'launch', point_lio_launch
    ])

    livox_launch_path = PathJoinSubstitution([
        FindPackageShare('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360_launch.py'
    ])

    # all_launch robot
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    ))

    # livox_ros_driver2
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    ))

    # all_launch point_lio
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(point_lio_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    ))

    # person pose bridge (change these 2 lines if your names differ)
    ld.add_action(Node(
        package='person_pose_bridge',
        executable='person_pose_bridge_node',
        name='person_pose_bridge_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    ))

    return ld
