#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_launch = LaunchConfiguration('robot_launch')
    point_lio_launch = LaunchConfiguration('point_lio_launch')
    enable_camera_tf = LaunchConfiguration('enable_camera_tf')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

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

    ld.add_action(DeclareLaunchArgument(
        'enable_camera_tf', default_value='true',
        description='Publish static TF for camera_link and camera_optical_frame'
    ))

    ld.add_action(DeclareLaunchArgument(
        'camera_x', default_value='0.00',
        description='base_link -> camera_link X offset (m)'
    ))

    ld.add_action(DeclareLaunchArgument(
        'camera_y', default_value='0.00',
        description='base_link -> camera_link Y offset (m)'
    ))

    ld.add_action(DeclareLaunchArgument(
        'camera_z', default_value='0.25',
        description='base_link -> camera_link Z offset (m)'
    ))

    ld.add_action(DeclareLaunchArgument(
        'camera_roll', default_value='0.0',
        description='base_link -> camera_link roll (rad)'
    ))

    ld.add_action(DeclareLaunchArgument(
        'camera_pitch', default_value='3.14159265359',
        description='base_link -> camera_link pitch (rad)'
    ))

    ld.add_action(DeclareLaunchArgument(
        'camera_yaw', default_value='3.14159265359',
        description='base_link -> camera_link yaw (rad)'
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

    # camera static TFs for camera visualization and tag localization chain
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_link',
        arguments=[
            camera_x, camera_y, camera_z,
            camera_roll, camera_pitch, camera_yaw,
            'base_link', 'camera_link'
        ],
        condition=IfCondition(enable_camera_tf),
        output='screen'
    ))

    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_camera_optical_frame',
        arguments=[
            '0', '0', '0',
            '-1.57079632679', '0', '-1.57079632679',
            'camera_link', 'camera_optical_frame'
        ],
        condition=IfCondition(enable_camera_tf),
        output='screen'
    ))

    # person pose bridge (change these 2 lines if your names differ)
    ld.add_action(Node(
        package='person_pose_bridge',
        executable='person_pose_bridge_node',
        name='person_pose_bridge_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    ))

    # Run raw python script from all_launch/scripts using python3
    controller_switch_script = PathJoinSubstitution([
        FindPackageShare('all_launch'),
        'scripts',
        'controller_switcher.py'
    ])

    ld.add_action(ExecuteProcess(
        cmd=['python3', controller_switch_script],
        output='screen'
    ))

    return ld
