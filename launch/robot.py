#!/usr/bin/env python3
"""Launch file to start three nodes from the `robot` package:
 - controller_node
 - serial_node
 - twist_mux

Save to: <your_package>/launch/robot_nodes_launch.py
Run with:
  ros2 launch <your_package> robot_nodes_launch.py

This launch declares a `use_sim_time` launch argument (default: false) and
passes it to each node as a parameter. Adjust parameters, remappings, namespaces,
or add respawn/launch configurations as needed.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use simulation time'))

    use_sim_time = LaunchConfiguration('use_sim_time')

    # controller_node
    controller = Node(
        package='robot',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # serial_node
    serial = Node(
        package='robot',
        executable='serial_node',
        name='serial_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # twist_mux
    twist_mux = Node(
        package='robot',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    ld.add_action(controller)
    ld.add_action(serial)
    ld.add_action(twist_mux)

    return ld
