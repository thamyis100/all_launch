#!/usr/bin/env python3
"""
Nav2 navigation stack with cmd_vel topic renames:

- cmd_vel_nav  -> cmd_vel_nav_raw   (Nav2 raw output from controller)
- cmd_vel      -> cmd_vel_nav       (final output after smoothing/monitoring)

Also: collision_monitor is optional (OFF by default) to avoid YAML errors when not configured.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_collision_monitor = LaunchConfiguration("use_collision_monitor")

    # Desired topics
    CMD_VEL_RAW = "cmd_vel_nav_raw"
    CMD_VEL_FINAL = "cmd_vel_nav"

    common_remaps = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

    # Always-managed nodes
    lifecycle_nodes_base = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use simulation clock if true",
        ),
        DeclareLaunchArgument(
            "autostart", default_value="true",
            description="Automatically startup the nav2 stack",
        ),
        DeclareLaunchArgument(
            "use_collision_monitor", default_value="false",
            description="Launch collision_monitor (requires parameters in YAML)",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value="/home/thamyis/SLAM_ws/src/all_launch/config/mppi_new.yaml",
            description="Full path to the Nav2 parameters file",
        ),

        GroupAction(actions=[
            SetParameter(name="use_sim_time", value=use_sim_time),

            # Controller output -> RAW
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[params_file],
                remappings=common_remaps + [
                    ("cmd_vel", CMD_VEL_RAW),
                    ("/cmd_vel", "/" + CMD_VEL_RAW),
                ],
            ),

            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[params_file],
                remappings=common_remaps,
            ),

            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[params_file],
                remappings=common_remaps,
            ),

            # Behaviors can also emit cmd_vel -> RAW
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[params_file],
                remappings=common_remaps + [
                    ("cmd_vel", CMD_VEL_RAW),
                    ("/cmd_vel", "/" + CMD_VEL_RAW),
                ],
            ),

            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[params_file],
                remappings=common_remaps,
            ),

            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[params_file],
                remappings=common_remaps,
            ),

            # Velocity smoother: input RAW, output FINAL
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[params_file],
                remappings=common_remaps + [
                    # input
                    ("cmd_vel", CMD_VEL_RAW),
                    ("/cmd_vel", "/" + CMD_VEL_RAW),

                    # output (cover both common names across versions)
                    ("smoothed_cmd_vel", CMD_VEL_FINAL),
                    ("/smoothed_cmd_vel", "/" + CMD_VEL_FINAL),
                    ("cmd_vel_smoothed", CMD_VEL_FINAL),
                    ("/cmd_vel_smoothed", "/" + CMD_VEL_FINAL),
                ],
            ),

            # Optional collision monitor (ONLY enable if YAML has observation_sources etc)
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=[params_file],
                remappings=common_remaps + [
                    # keep all outputs on FINAL if it publishes cmd_vel
                    ("cmd_vel", CMD_VEL_FINAL),
                    ("/cmd_vel", "/" + CMD_VEL_FINAL),

                    # if it expects smoothed input, point it to FINAL
                    ("smoothed_cmd_vel", CMD_VEL_FINAL),
                    ("cmd_vel_smoothed", CMD_VEL_FINAL),
                ],
                condition=IfCondition(use_collision_monitor),
            ),

            # Lifecycle manager for base nodes
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{
                    "autostart": autostart,
                    "node_names": lifecycle_nodes_base,
                }],
            ),

            # Lifecycle manager for collision_monitor only (when enabled)
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_collision_monitor",
                output="screen",
                parameters=[{
                    "autostart": autostart,
                    "node_names": ["collision_monitor"],
                }],
                condition=IfCondition(use_collision_monitor),
            ),
        ]),
    ])
