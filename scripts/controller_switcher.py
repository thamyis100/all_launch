#!/usr/bin/env python3
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav2_msgs.action import FollowPath

try:
    FollowPathFeedbackMsg = FollowPath.Impl.FeedbackMessage
except AttributeError:
    from nav2_msgs.action._follow_path import FollowPath_FeedbackMessage as FollowPathFeedbackMsg


def hypot2(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class ControllerSwitcher(Node):
    def __init__(self):
        super().__init__("controller_switcher")

        # thresholds
        self.first_m = float(os.getenv("FIRST_M", "2.0"))
        self.last_m = float(os.getenv("LAST_M", "2.0"))
        self.feedback_timeout_s = float(os.getenv("FEEDBACK_TIMEOUT_S", "5.0"))

        self.odom_topic = os.getenv("ODOM_TOPIC", "/odom")

        self.cur_xy = None
        self.prev_xy = None
        self.traveled = 0.0

        self.dist_to_goal = None
        self.last_feedback_time = None
        self.last_published = None

        # Latched publisher for ControllerSelector
        qos_latched = QoSProfile(depth=1)
        qos_latched.reliability = ReliabilityPolicy.RELIABLE
        qos_latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(String, "/controller_selector", qos_latched)

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)

        qos_fb = QoSProfile(depth=10)
        qos_fb.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(FollowPathFeedbackMsg, "/follow_path/_action/feedback", self.on_feedback, qos_fb)

        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz
        self.debug_timer = self.create_timer(1.0, self.debug)  # 1 Hz

        self.publish_if_changed("FollowPath")
        self.get_logger().info(f"Using ODOM_TOPIC={self.odom_topic} FIRST_M={self.first_m} LAST_M={self.last_m} FEEDBACK_TIMEOUT_S={self.feedback_timeout_s}")

    def on_odom(self, msg: Odometry):
        self.cur_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.prev_xy is not None and self.last_feedback_time is not None:
            self.traveled += hypot2(self.cur_xy, self.prev_xy)
        self.prev_xy = self.cur_xy

    def on_feedback(self, msg: FollowPathFeedbackMsg):
        self.last_feedback_time = self.get_clock().now()
        self.dist_to_goal = float(msg.feedback.distance_to_goal)

    def publish_if_changed(self, controller_id: str):
        if controller_id == self.last_published:
            return
        self.pub.publish(String(data=controller_id))
        self.last_published = controller_id
        self.get_logger().info(f"controller_selector -> {controller_id}")

    def tick(self):
        now = self.get_clock().now()

        # if feedback stale, consider idle and reset travel
        if self.last_feedback_time is None or (now - self.last_feedback_time).nanoseconds * 1e-9 > self.feedback_timeout_s:
            self.traveled = 0.0
            self.prev_xy = self.cur_xy
            self.dist_to_goal = None
            self.publish_if_changed("FollowPath")
            return

        if self.cur_xy is None or self.dist_to_goal is None:
            self.publish_if_changed("FollowPath")
            return

        # rule
        if self.dist_to_goal <= self.last_m:
            self.publish_if_changed("FollowPath")
        elif self.traveled < self.first_m:
            self.publish_if_changed("FollowPath")
        else:
            self.publish_if_changed("FollowPathDiff")

    def debug(self):
        self.get_logger().info(f"debug traveled={self.traveled:.2f}m dist_to_goal={self.dist_to_goal} odom_xy={self.cur_xy}")

def main():
    rclpy.init()
    node = ControllerSwitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()