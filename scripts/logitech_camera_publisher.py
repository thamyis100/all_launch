#!/usr/bin/env python3

import math
import glob

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class LogitechCameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__("logitech_camera_publisher")

        self.declare_parameter("device", "/dev/video2")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("hfov_deg", 78.0)
        self.declare_parameter("pixel_format", "MJPG")
        self.declare_parameter("buffer_size", 1)

        self.device = str(self.get_parameter("device").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = float(self.get_parameter("fps").value)
        self.hfov_deg = float(self.get_parameter("hfov_deg").value)
        self.pixel_format = str(self.get_parameter("pixel_format").value).upper()
        self.buffer_size = int(self.get_parameter("buffer_size").value)

        self.bridge = CvBridge()
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.image_pub = self.create_publisher(Image, self.image_topic, sensor_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, sensor_qos)

        self.cap, opened_device = self.open_camera(self.device)
        if self.cap is None:
            available = sorted(glob.glob("/dev/video*"))
            self.get_logger().error(
                f"Failed to open webcam device '{self.device}'. "
                f"Available devices: {available if available else 'none'}"
            )
            self.get_logger().error(
                "Try setting device explicitly, e.g. "
                "ros2 run all_launch logitech_camera_publisher.py --ros-args -p device:=/dev/video0"
            )
            raise RuntimeError(f"Cannot open any camera device (requested {self.device})")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        if self.buffer_size > 0:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, self.buffer_size)

        if self.pixel_format and len(self.pixel_format) == 4:
            fourcc = cv2.VideoWriter_fourcc(*self.pixel_format)
            self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)

        self.actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.actual_fps = float(self.cap.get(cv2.CAP_PROP_FPS))

        timer_hz = self.actual_fps if self.actual_fps > 1.0 else max(self.fps, 5.0)
        self.timer = self.create_timer(1.0 / timer_hz, self.publish_frame)

        self.get_logger().info(
            f"Publishing webcam {opened_device} -> {self.image_topic} + {self.camera_info_topic} "
            f"at {self.actual_width}x{self.actual_height} @{timer_hz:.1f}Hz "
            f"(fmt={self.pixel_format}, buffer={self.buffer_size})"
        )

    def open_camera(self, requested_device: str):
        candidates = [requested_device]
        for dev in sorted(glob.glob("/dev/video*")):
            if dev not in candidates:
                candidates.append(dev)

        for dev in candidates:
            # Prefer V4L2 for Linux webcams, then fall back to default backend.
            for backend in (cv2.CAP_V4L2, None):
                cap = cv2.VideoCapture(dev, backend) if backend is not None else cv2.VideoCapture(dev)
                if cap.isOpened():
                    return cap, dev
                cap.release()
        return None, None

    def build_camera_info(self, stamp) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.width = self.actual_width
        msg.height = self.actual_height
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        hfov_rad = math.radians(max(1.0, self.hfov_deg))
        fx = self.actual_width / (2.0 * math.tan(hfov_rad / 2.0))
        fy = fx
        cx = (self.actual_width - 1) / 2.0
        cy = (self.actual_height - 1) / 2.0

        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def publish_frame(self) -> None:
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Webcam frame read failed", throttle_duration_sec=2.0)
            return

        stamp = self.get_clock().now().to_msg()

        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.frame_id
        self.image_pub.publish(image_msg)

        camera_info_msg = self.build_camera_info(stamp)
        self.camera_info_pub.publish(camera_info_msg)

    def destroy_node(self):
        if hasattr(self, "cap") and self.cap is not None:
            self.cap.release()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = LogitechCameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
