#!/usr/bin/env python3
"""
Nav2 navigation stack with cmd_vel topic renames:

- cmd_vel_nav  -> cmd_vel_nav_raw   (Nav2 raw output from controller)
- cmd_vel      -> cmd_vel_nav       (final output after smoothing/monitoring)

Also: collision_monitor is optional (OFF by default) to avoid YAML errors when not configured.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    all_launch_share = get_package_share_directory("all_launch")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_collision_monitor = LaunchConfiguration("use_collision_monitor")
    use_docking = LaunchConfiguration("use_docking")
    use_logitech_camera = LaunchConfiguration("use_logitech_camera")
    camera_device = LaunchConfiguration("camera_device")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    camera_fps = LaunchConfiguration("camera_fps")
    camera_hfov_deg = LaunchConfiguration("camera_hfov_deg")
    apriltag_params_file = LaunchConfiguration("apriltag_params_file")

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
            default_value=os.path.join(all_launch_share, "config", "mppi.yaml"),
            description="Full path to the Nav2 parameters file",
        ),
        DeclareLaunchArgument(
            "use_docking", default_value="true",
            description="Launch docking server and AprilTag docking helpers",
        ),
        DeclareLaunchArgument(
            "use_logitech_camera", default_value="true",
            description="Launch Logitech webcam publisher node",
        ),
        DeclareLaunchArgument(
            "camera_device", default_value="/dev/video0",
            description="Video device path for Logitech webcam",
        ),
        DeclareLaunchArgument(
            "camera_width", default_value="1280",
            description="Requested webcam image width",
        ),
        DeclareLaunchArgument(
            "camera_height", default_value="720",
            description="Requested webcam image height",
        ),
        DeclareLaunchArgument(
            "camera_fps", default_value="30.0",
            description="Requested webcam FPS",
        ),
        DeclareLaunchArgument(
            "camera_hfov_deg", default_value="78.0",
            description="Approximate horizontal FOV in degrees for camera_info",
        ),
        DeclareLaunchArgument(
            "apriltag_params_file",
            default_value=os.path.join(all_launch_share, "config", "apriltag_36h11.yaml"),
            description="Full path to the apriltag_ros parameter file",
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

            Node(
                package="opennav_docking",
                executable="opennav_docking",
                name="docking_server",
                output="screen",
                parameters=[params_file],
                remappings=common_remaps + [
                    ("cmd_vel", CMD_VEL_RAW),
                    ("/cmd_vel", "/" + CMD_VEL_RAW),
                ],
                condition=IfCondition(use_docking),
            ),

            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_docking",
                output="screen",
                parameters=[{
                    "autostart": autostart,
                    "node_names": ["docking_server"],
                }],
                condition=IfCondition(use_docking),
            ),

            Node(
                package="all_launch",
                executable="logitech_camera_publisher.py",
                name="logitech_camera_publisher",
                output="screen",
                parameters=[{
                    "device": camera_device,
                    "image_topic": "/camera/image_raw",
                    "camera_info_topic": "/camera/camera_info",
                    "frame_id": "camera_optical_frame",
                    "width": camera_width,
                    "height": camera_height,
                    "fps": camera_fps,
                    "hfov_deg": camera_hfov_deg,
                }],
                condition=IfCondition(use_logitech_camera),
            ),

            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                name="apriltag",
                output="screen",
                parameters=[apriltag_params_file],
                remappings=common_remaps + [
                    # Use directly-published camera topics for robust image/info pairing.
                    ("image_rect", "/camera/image_raw"),
                    ("camera_info", "/camera/camera_info"),
                ],
                condition=IfCondition(use_docking),
            ),

            Node(
                package="all_launch",
                executable="apriltag_dock_pose_publisher.py",
                name="apriltag_dock_pose_publisher",
                output="screen",
                parameters=[{
                    "fixed_frame": "odom",
                    "tag_frame": "dock_tag",
                    "output_topic": "/detected_dock_pose",
                    "max_pose_age": 2.5,
                }],
                condition=IfCondition(use_docking),
            ),

            Node(
                package="all_launch",
                executable="dock_pose_bridge.py",
                name="dock_pose_bridge",
                output="screen",
                parameters=[{
                    "global_frame": "map",
                    "base_frame": "base_link",
                    "odom_topic": "/odom",
                    "cmd_vel_topic": "/" + CMD_VEL_FINAL,
                    "scan_topic": "/scan",
                    "goal_topic": "/goal_pose",
                    "dock_pose_topic": "/dock_pose",
                    "undock_trigger_topic": "/undock_trigger",
                    "dock_type": "simple_charging_dock",
                    "enable_visualization": False,
                }],
            ),
        ]),
    ])
