#!/usr/bin/env python3
"""Bridge RViz docking commands to opennav docking actions with status topics."""

import math
import sys
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker

try:
    from opennav_docking_msgs.action import DockRobot, UndockRobot
    DOCKING_AVAILABLE = True
except ImportError:
    DOCKING_AVAILABLE = False
    print("[WARN] opennav_docking_msgs not available. Dock bridge disabled.")


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class DockBridge(Node):
    def __init__(self) -> None:
        super().__init__("dock_pose_bridge")

        # Keep parameter names compatible with existing launch files.
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("enable_visualization", False)

        self.declare_parameter("dock_type", "simple_charging_dock")
        self.declare_parameter("dock_pose_topic", "/dock_pose")
        self.declare_parameter("undock_trigger_topic", "/undock_trigger")
        self.declare_parameter("docking_status_topic", "/docking/status_text")
        self.declare_parameter("docking_status_marker_topic", "/docking/status_marker")
        self.declare_parameter("detected_dock_pose_source_topic", "/detected_dock_pose_source")
        self.declare_parameter("detected_dock_pose_topic", "/detected_dock_pose")
        self.declare_parameter("prefer_detected_dock_pose", True)
        self.declare_parameter("max_detected_dock_pose_age", 3.0)
        self.declare_parameter("allow_rejected_fallback_source", False)

        self.global_frame = self.get_parameter("global_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.dock_type = self.get_parameter("dock_type").get_parameter_value().string_value
        self.dock_pose_topic = self.get_parameter("dock_pose_topic").get_parameter_value().string_value
        self.undock_trigger_topic = self.get_parameter("undock_trigger_topic").get_parameter_value().string_value
        self.prefer_detected_dock_pose = self.get_parameter(
            "prefer_detected_dock_pose"
        ).get_parameter_value().bool_value
        self.max_detected_dock_pose_age = self.get_parameter(
            "max_detected_dock_pose_age"
        ).get_parameter_value().double_value
        self.allow_rejected_fallback_source = self.get_parameter(
            "allow_rejected_fallback_source"
        ).get_parameter_value().bool_value

        detected_source_topic = self.get_parameter(
            "detected_dock_pose_source_topic"
        ).get_parameter_value().string_value
        detected_pose_topic = self.get_parameter(
            "detected_dock_pose_topic"
        ).get_parameter_value().string_value

        docking_status_topic = self.get_parameter(
            "docking_status_topic"
        ).get_parameter_value().string_value
        docking_status_marker_topic = self.get_parameter(
            "docking_status_marker_topic"
        ).get_parameter_value().string_value

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = QoSReliabilityPolicy.RELIABLE
        status_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, docking_status_topic, status_qos)
        self.status_marker_pub = self.create_publisher(Marker, docking_status_marker_topic, status_qos)

        self.detected_dock_pose_source = "idle"
        self.latest_detected_dock_pose: Optional[PoseStamped] = None
        self.current_phase = "IDLE"
        self.active_mode: Optional[str] = None

        self.create_subscription(String, detected_source_topic, self.on_detected_dock_pose_source, 10)
        self.create_subscription(PoseStamped, detected_pose_topic, self.on_detected_dock_pose, 10)

        if not DOCKING_AVAILABLE:
            self._dock_ac = None
            self._undock_ac = None
            self.get_logger().error(
                "opennav_docking_msgs not found! Install with: sudo apt install ros-humble-opennav-docking-msgs"
            )
            return

        self._dock_ac = ActionClient(self, DockRobot, "/dock_robot")
        self._undock_ac = ActionClient(self, UndockRobot, "/undock_robot")
        self._dock_goal_handle = None
        self._undock_goal_handle = None

        self.create_subscription(PoseStamped, self.dock_pose_topic, self.on_dock_pose, 10)
        self.create_subscription(Empty, self.undock_trigger_topic, self.on_undock_trigger, 10)

        self.get_logger().info(
            f"[DockBridge] {self.dock_pose_topic} -> /dock_robot | "
            f"{self.undock_trigger_topic} -> /undock_robot | dock_type={self.dock_type}"
        )
        self.publish_docking_status("Docking: IDLE", "Awaiting /dock_pose", 0.8, 0.8, 0.8)

    def now(self) -> rclpy.time.Time:
        return self.get_clock().now()

    def _is_detected_pose_fresh(self, pose: Optional[PoseStamped]) -> bool:
        if pose is None:
            return False
        if pose.header.stamp.sec == 0 and pose.header.stamp.nanosec == 0:
            return True
        stamp = rclpy.time.Time.from_msg(pose.header.stamp)
        age = (self.now() - stamp).nanoseconds * 1e-9
        return age <= self.max_detected_dock_pose_age

    def dock_feedback_state_name(self, state: int) -> str:
        state_map = {
            0: "NONE",
            1: "NAV_TO_STAGING",
            2: "INITIAL_PERCEPTION",
            3: "CONTROLLING",
            4: "WAIT_FOR_CHARGE",
            5: "RETRY",
        }
        return state_map.get(int(state), f"STATE_{state}")

    def publish_docking_status(self, title: str, detail: str, red: float, green: float, blue: float) -> None:
        text = title
        if detail:
            text += f"\n{detail}"

        self.status_pub.publish(String(data=text))

        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.now().to_msg()
        marker.ns = "docking_status"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.frame_locked = True
        marker.pose.position.z = 0.45
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.12
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        marker.text = text
        self.status_marker_pub.publish(marker)

    def on_detected_dock_pose_source(self, msg: String) -> None:
        self.detected_dock_pose_source = msg.data or "unknown"
        if self.active_mode in ("DOCK", "UNDOCK"):
            self.publish_docking_status(
                f"{self.active_mode}: {self.current_phase}",
                f"Source: {self.detected_dock_pose_source}",
                1.0,
                1.0,
                0.2,
            )

    def on_detected_dock_pose(self, msg: PoseStamped) -> None:
        self.latest_detected_dock_pose = msg

    def on_dock_pose(self, msg: PoseStamped) -> None:
        if not DOCKING_AVAILABLE or self._dock_ac is None:
            self.get_logger().error("Docking not available.")
            return

        if self.active_mode is not None:
            self.get_logger().warn("Already docking/undocking; ignoring new /dock_pose.", throttle_duration_sec=2.0)
            return

        if (
            not self.allow_rejected_fallback_source
            and self.detected_dock_pose_source == "fallback_rejected_tag_tf"
        ):
            self.get_logger().error(
                "Refusing dock command: source is fallback_rejected_tag_tf. Update /dock_pose or relax consistency checks."
            )
            self.publish_docking_status(
                "DOCK: BLOCKED (UNSAFE SOURCE)",
                "Source=fallback_rejected_tag_tf",
                1.0,
                0.3,
                0.3,
            )
            return

        goal_pose_msg = msg
        if (
            self.prefer_detected_dock_pose
            and self.detected_dock_pose_source == "apriltag_tf"
            and self._is_detected_pose_fresh(self.latest_detected_dock_pose)
        ):
            goal_pose_msg = self.latest_detected_dock_pose
            self.get_logger().info("Using fresh /detected_dock_pose for dock goal.")

        if msg.header.frame_id and msg.header.frame_id != self.global_frame:
            self.get_logger().warn(
                f"Dock pose frame '{msg.header.frame_id}' != global_frame '{self.global_frame}'."
            )

        q = goal_pose_msg.pose.orientation
        self.get_logger().info(
            f"Dock request: x={goal_pose_msg.pose.position.x:.3f} "
            f"y={goal_pose_msg.pose.position.y:.3f} "
            f"yaw={math.degrees(quat_to_yaw(q.x, q.y, q.z, q.w)):.1f}deg"
        )

        if not self._dock_ac.wait_for_server(timeout_sec=5.0):
            self.publish_docking_status("DOCK: SERVER MISSING", "/dock_robot unavailable", 1.0, 0.2, 0.2)
            return

        goal = DockRobot.Goal()
        goal.use_dock_id = False
        goal.dock_type = self.dock_type
        goal.dock_pose = goal_pose_msg

        self.active_mode = "DOCK"
        self.current_phase = "REQUESTED"
        self.publish_docking_status("DOCK: REQUESTED", f"Source: {self.detected_dock_pose_source}", 0.2, 0.8, 1.0)

        send_future = self._dock_ac.send_goal_async(goal, feedback_callback=self.on_dock_feedback)
        send_future.add_done_callback(self._on_dock_goal_response)

    def _on_dock_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.active_mode = None
            self.publish_docking_status("DOCK: REJECTED", "Action server rejected request", 1.0, 0.2, 0.2)
            return

        self._dock_goal_handle = goal_handle
        self.publish_docking_status("DOCK: ACCEPTED", f"Source: {self.detected_dock_pose_source}", 0.2, 0.8, 1.0)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_dock_action_result)

    def _on_dock_action_result(self, future) -> None:
        self._dock_goal_handle = None
        self._handle_action_result(future, "DOCK")

    def on_dock_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.current_phase = self.dock_feedback_state_name(int(feedback.state))
        docking_time = float(feedback.docking_time.sec) + float(feedback.docking_time.nanosec) * 1e-9
        self.publish_docking_status(
            f"DOCK: {self.current_phase}",
            f"t={docking_time:.1f}s retries={int(feedback.num_retries)} source={self.detected_dock_pose_source}",
            1.0,
            1.0,
            0.2,
        )

    def on_undock_trigger(self, _msg: Empty) -> None:
        if not DOCKING_AVAILABLE or self._undock_ac is None:
            self.get_logger().error("Docking not available.")
            return

        if self.active_mode is not None:
            if self.active_mode == "DOCK" and self._dock_goal_handle is not None:
                self.current_phase = "CANCEL_REQUESTED"
                self.publish_docking_status("DOCK: CANCEL REQUESTED", "Stopping docking and returning to IDLE", 1.0, 0.7, 0.2)
                cancel_future = self._dock_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._on_dock_cancel_response)
            else:
                self.get_logger().warn("Already docking/undocking; ignoring undock trigger.")
            return

        if not self._undock_ac.wait_for_server(timeout_sec=5.0):
            self.publish_docking_status("UNDOCK: SERVER MISSING", "/undock_robot unavailable", 1.0, 0.2, 0.2)
            return

        goal = UndockRobot.Goal()
        goal.dock_type = self.dock_type

        self.active_mode = "UNDOCK"
        self.current_phase = "REQUESTED"
        self.publish_docking_status("UNDOCK: REQUESTED", "Waiting for /undock_robot", 0.2, 0.8, 1.0)

        send_future = self._undock_ac.send_goal_async(goal)
        send_future.add_done_callback(self._on_undock_goal_response)

    def _on_undock_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.active_mode = None
            self.publish_docking_status("UNDOCK: REJECTED", "Action server rejected request", 1.0, 0.2, 0.2)
            return

        self._undock_goal_handle = goal_handle
        self.publish_docking_status("UNDOCK: ACCEPTED", "", 0.2, 0.8, 1.0)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_undock_action_result)

    def _on_undock_action_result(self, future) -> None:
        self._undock_goal_handle = None
        self._handle_action_result(future, "UNDOCK")

    def _on_dock_cancel_response(self, future) -> None:
        try:
            cancel_response = future.result()
            canceled = len(cancel_response.goals_canceling) > 0
        except Exception as exc:
            self.get_logger().error(f"Dock cancel failed: {exc}")
            canceled = False

        if canceled:
            self.publish_docking_status("Docking: IDLE", "Canceled by Undock button", 0.8, 0.8, 0.8)
        else:
            self.publish_docking_status("DOCK: CANCEL FAILED", "Action server did not cancel goal", 1.0, 0.3, 0.3)

        self.active_mode = None
        self.current_phase = "IDLE"
        self._dock_goal_handle = None

    def _handle_action_result(self, future, action_name: str) -> None:
        result_wrapper = future.result()
        result = result_wrapper.result
        status = int(result_wrapper.status)

        success = bool(getattr(result, "success", status == GoalStatus.STATUS_SUCCEEDED))
        error_code = int(getattr(result, "error_code", 0))
        num_retries = int(getattr(result, "num_retries", 0))

        if success:
            self.publish_docking_status(
                f"{action_name}: SUCCESS",
                f"error={error_code} retries={num_retries}",
                0.2,
                1.0,
                0.2,
            )
        else:
            self.publish_docking_status(
                f"{action_name}: FAILED",
                f"status={status} error={error_code} retries={num_retries}",
                1.0,
                0.2,
                0.2,
            )

        self.active_mode = None
        self.current_phase = "IDLE"


def main() -> None:
    rclpy.init(args=sys.argv)
    node = DockBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
