#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
import tf2_geometry_msgs  # noqa: F401
from tf2_ros import Buffer, TransformException, TransformListener


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class AprilTagDockPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_dock_pose_publisher")

        self.declare_parameter("fixed_frame", "odom")
        self.declare_parameter("tag_frame", "dock_tag")
        self.declare_parameter("output_topic", "/detected_dock_pose")
        self.declare_parameter("source_topic", "/detected_dock_pose_source")
        self.declare_parameter("fallback_dock_pose_topic", "/dock_pose")
        self.declare_parameter("use_fallback_dock_pose", True)
        self.declare_parameter("publish_rate", 15.0)
        self.declare_parameter("max_pose_age", 0.5)
        self.declare_parameter("fallback_transform_use_latest", True)
        # Set <= 0 to disable position-delta rejection against fallback pose.
        self.declare_parameter("max_detection_position_delta", -1.0)
        self.declare_parameter("max_detection_yaw_delta", -1.0)

        self.fixed_frame = self.get_parameter("fixed_frame").value
        self.tag_frame = self.get_parameter("tag_frame").value
        self.output_topic = self.get_parameter("output_topic").value
        self.source_topic = self.get_parameter("source_topic").value
        self.fallback_dock_pose_topic = self.get_parameter("fallback_dock_pose_topic").value
        self.use_fallback_dock_pose = bool(self.get_parameter("use_fallback_dock_pose").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.max_pose_age = float(self.get_parameter("max_pose_age").value)
        self.fallback_transform_use_latest = bool(
            self.get_parameter("fallback_transform_use_latest").value
        )
        self.max_detection_position_delta = float(
            self.get_parameter("max_detection_position_delta").value
        )
        self.max_detection_yaw_delta = float(
            self.get_parameter("max_detection_yaw_delta").value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.source_publisher = self.create_publisher(String, self.source_topic, 10)
        self.fallback_pose = None

        if self.use_fallback_dock_pose:
            self.create_subscription(
                PoseStamped,
                self.fallback_dock_pose_topic,
                self.on_fallback_dock_pose,
                10,
            )

        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(period, self.publish_pose)

        self.get_logger().info(
            f"Publishing {self.output_topic} from TF {self.fixed_frame} -> {self.tag_frame} "
            f"with max_pose_age={self.max_pose_age:.2f}s"
        )

    def publish_source(self, source: str) -> None:
        self.source_publisher.publish(String(data=source))

    def on_fallback_dock_pose(self, msg: PoseStamped) -> None:
        self.fallback_pose = msg
        self.get_logger().info(
            f"Updated fallback dock pose from {self.fallback_dock_pose_topic} "
            f"in frame {msg.header.frame_id or 'map'}",
            throttle_duration_sec=1.0,
        )

    def transform_pose(self, source: PoseStamped) -> PoseStamped | None:
        transformed_source = PoseStamped()
        transformed_source.header = source.header
        if not transformed_source.header.frame_id:
            transformed_source.header.frame_id = "map"

        # Fallback /dock_pose can be old; use latest TF to avoid repeated
        # extrapolation errors from stale source timestamps.
        if self.fallback_transform_use_latest:
            transformed_source.header.stamp = rclpy.time.Time().to_msg()

        transformed_source.pose = source.pose

        try:
            return self.tf_buffer.transform(
                transformed_source,
                self.fixed_frame,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Could not transform pose {transformed_source.header.frame_id} -> {self.fixed_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None

    def build_pose_from_transform(self, transform) -> PoseStamped:
        msg = PoseStamped()
        if transform.header.stamp.sec == 0 and transform.header.stamp.nanosec == 0:
            msg.header.stamp = self.get_clock().now().to_msg()
        else:
            msg.header.stamp = transform.header.stamp
        msg.header.frame_id = self.fixed_frame
        msg.pose.position.x = transform.transform.translation.x
        msg.pose.position.y = transform.transform.translation.y
        msg.pose.position.z = transform.transform.translation.z
        msg.pose.orientation = transform.transform.rotation
        return msg

    def is_detection_consistent(self, detected_pose: PoseStamped) -> bool:
        if not self.use_fallback_dock_pose or self.fallback_pose is None:
            return True

        if self.max_detection_position_delta <= 0.0 and self.max_detection_yaw_delta <= 0.0:
            return True

        fallback_in_fixed = self.transform_pose(self.fallback_pose)
        if fallback_in_fixed is None:
            return True

        dx = detected_pose.pose.position.x - fallback_in_fixed.pose.position.x
        dy = detected_pose.pose.position.y - fallback_in_fixed.pose.position.y
        dist = math.hypot(dx, dy)

        yaw_delta_deg = None
        if self.max_detection_yaw_delta > 0.0:
            detected_yaw = quat_to_yaw(
                detected_pose.pose.orientation.x,
                detected_pose.pose.orientation.y,
                detected_pose.pose.orientation.z,
                detected_pose.pose.orientation.w,
            )
            fallback_yaw = quat_to_yaw(
                fallback_in_fixed.pose.orientation.x,
                fallback_in_fixed.pose.orientation.y,
                fallback_in_fixed.pose.orientation.z,
                fallback_in_fixed.pose.orientation.w,
            )
            yaw_delta = abs(angle_wrap(detected_yaw - fallback_yaw))
            yaw_delta_deg = math.degrees(yaw_delta)
        else:
            yaw_delta = 0.0

        check_position = self.max_detection_position_delta > 0.0 and dist > self.max_detection_position_delta
        check_yaw = self.max_detection_yaw_delta > 0.0 and yaw_delta > self.max_detection_yaw_delta

        if check_position or check_yaw:
            detail = f"delta_from_fallback={dist:.2f}m"
            if yaw_delta_deg is not None:
                detail += f" yaw_delta={yaw_delta_deg:.1f}deg"
            self.get_logger().warn(
                f"Rejecting dock TF from {self.tag_frame}; "
                f"{detail}",
                throttle_duration_sec=1.0,
            )
            return False
        return True

    def publish_pose(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                self.tag_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Waiting for TF {self.fixed_frame} -> {self.tag_frame}: {exc}",
                throttle_duration_sec=5.0,
            )
            self.publish_fallback_pose("fallback_missing_tag_tf")
            return

        stamp_time = rclpy.time.Time.from_msg(transform.header.stamp)
        if transform.header.stamp.sec == 0 and transform.header.stamp.nanosec == 0:
            age = 0.0
        else:
            age = (self.get_clock().now() - stamp_time).nanoseconds * 1e-9
            if age < 0.0:
                age = 0.0
        if age > self.max_pose_age:
            self.get_logger().warn(
                f"Skipping stale dock pose from {self.tag_frame}; age={age:.2f}s",
                throttle_duration_sec=2.0,
            )
            self.publish_fallback_pose("fallback_stale_tag_tf")
            return

        msg = self.build_pose_from_transform(transform)
        if not self.is_detection_consistent(msg):
            self.publish_fallback_pose("fallback_rejected_tag_tf")
            return

        self.publisher.publish(msg)
        self.publish_source("apriltag_tf")

    def publish_fallback_pose(self, source_label: str) -> None:
        if not self.use_fallback_dock_pose or self.fallback_pose is None:
            return

        transformed = self.transform_pose(self.fallback_pose)
        if transformed is None:
            return

        transformed.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(transformed)
        self.publish_source(source_label)


def main() -> None:
    rclpy.init()
    node = AprilTagDockPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
