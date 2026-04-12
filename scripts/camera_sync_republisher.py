#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CameraInfo, Image


class CameraSyncRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("camera_sync_republisher")

        self.declare_parameter("input_image_topic", "/camera/image_raw")
        self.declare_parameter("input_camera_info_topic", "/camera/camera_info")
        self.declare_parameter("output_image_topic", "/camera/image_apriltag")
        self.declare_parameter("output_camera_info_topic", "/camera/camera_info_apriltag")

        self.input_image_topic = str(self.get_parameter("input_image_topic").value)
        self.input_camera_info_topic = str(self.get_parameter("input_camera_info_topic").value)
        self.output_image_topic = str(self.get_parameter("output_image_topic").value)
        self.output_camera_info_topic = str(self.get_parameter("output_camera_info_topic").value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.last_camera_info = None
        self.image_pub = self.create_publisher(Image, self.output_image_topic, sensor_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, self.output_camera_info_topic, sensor_qos)

        self.create_subscription(Image, self.input_image_topic, self.on_image, sensor_qos)
        self.create_subscription(CameraInfo, self.input_camera_info_topic, self.on_camera_info, sensor_qos)

        self.get_logger().info(
            f"Republishing synchronized camera topics: "
            f"{self.input_image_topic} + {self.input_camera_info_topic} -> "
            f"{self.output_image_topic} + {self.output_camera_info_topic}"
        )

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.last_camera_info = msg

    def on_image(self, msg: Image) -> None:
        if self.last_camera_info is None:
            self.get_logger().warn("Waiting for first CameraInfo before republishing image", throttle_duration_sec=5.0)
            return

        image_out = Image()
        image_out.header = msg.header
        image_out.height = msg.height
        image_out.width = msg.width
        image_out.encoding = msg.encoding
        image_out.is_bigendian = msg.is_bigendian
        image_out.step = msg.step
        image_out.data = msg.data
        self.image_pub.publish(image_out)

        info_out = CameraInfo()
        info_out.header = msg.header
        info_out.height = self.last_camera_info.height
        info_out.width = self.last_camera_info.width
        info_out.distortion_model = self.last_camera_info.distortion_model
        info_out.d = list(self.last_camera_info.d)
        info_out.k = list(self.last_camera_info.k)
        info_out.r = list(self.last_camera_info.r)
        info_out.p = list(self.last_camera_info.p)
        info_out.binning_x = self.last_camera_info.binning_x
        info_out.binning_y = self.last_camera_info.binning_y
        info_out.roi = self.last_camera_info.roi
        self.camera_info_pub.publish(info_out)


def main() -> None:
    rclpy.init()
    node = CameraSyncRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
