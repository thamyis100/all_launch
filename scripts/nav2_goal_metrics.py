#!/usr/bin/env python3
import math
import sys
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

import tf2_ros


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    # yaw (z-axis rotation) from quaternion
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_wrap(a: float) -> float:
    # Wrap to [-pi, pi]
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def hypot2(x: float, y: float) -> float:
    return math.sqrt(x * x + y * y)


@dataclass
class RunMetrics:
    along_track: Optional[float] = None
    cross_track: Optional[float] = None 
    # Straight-line efficiency
    straight_line_dist: Optional[float] = None

    # Motion quality
    stop_go_count: int = 0
    last_speed: Optional[float] = None
    sign_flip_count: int = 0
    last_vx_sign: Optional[int] = None

    sum_v_sq: float = 0.0
    sum_w_sq: float = 0.0
    sample_count: int = 0

    # Clearance
    unsafe_clearance_time: float = 0.0
    clearance_threshold: float = 0.35
    last_scan_time: Optional[rclpy.time.Time] = None
    run_id: int = 0
    active: bool = False

    # Goal (in global frame)
    goal_x: float = 0.0
    goal_y: float = 0.0
    goal_yaw: float = 0.0

    # Timing
    t_start: Optional[rclpy.time.Time] = None
    t_end: Optional[rclpy.time.Time] = None

    # Path length (from odom positions)
    path_len_m: float = 0.0
    last_odom_x: Optional[float] = None
    last_odom_y: Optional[float] = None

    # Cmd stats
    max_cmd_v: float = 0.0   # magnitude (m/s)
    max_cmd_w: float = 0.0   # rad/s

    # Actual stats (from odom twist)
    max_act_v: float = 0.0
    max_act_w: float = 0.0

    # Scan clearance
    min_scan: float = float("inf")

    # Plan update counts
    global_plan_updates: int = 0
    local_plan_updates: int = 0

    # Feedback
    min_distance_remaining: float = float("inf")
    last_distance_remaining: Optional[float] = None
    recoveries: Optional[int] = None

    # Settling detection (optional)
    settle_window_s: float = 1.0
    tol_xy: float = 0.15
    tol_yaw: float = 0.15
    within_tol_since: Optional[rclpy.time.Time] = None
    settle_time_s: Optional[float] = None

    # Final pose error
    final_dx: Optional[float] = None
    final_dy: Optional[float] = None
    final_dist: Optional[float] = None
    final_dyaw: Optional[float] = None

    # Status
    result_status: Optional[int] = None


class Nav2GoalMetrics(Node):
    def __init__(self):
        super().__init__("nav2_goal_metrics")

        # --- Parameters ---
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("nav_action_name", "navigate_to_pose")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")

        self.declare_parameter("global_plan_topic", "/plan")       # nav_msgs/Path (if available)
        self.declare_parameter("local_plan_topic", "/trajectories")  # nav_msgs/Path (if available)

        self.declare_parameter("settle_window_s", 1.0)
        self.declare_parameter("tol_xy", 0.15)
        self.declare_parameter("tol_yaw", 0.15)

        self.goal_topic = self.get_parameter("goal_topic").value
        self.action_name = self.get_parameter("nav_action_name").value
        self.global_frame = self.get_parameter("global_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.odom_topic = self.get_parameter("odom_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value

        self.global_plan_topic = self.get_parameter("global_plan_topic").value
        self.local_plan_topic = self.get_parameter("local_plan_topic").value

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Action client
        self._ac = ActionClient(self, NavigateToPose, self.action_name)

        # Subscriptions
        self.create_subscription(PoseStamped, self.goal_topic, self.on_goal_pose, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)
        self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 50)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)

        # Set QoS to best effort for /scan
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        scan_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, scan_qos)
        self.create_subscription(Path, self.global_plan_topic, self.on_global_plan, 10)
        self.create_subscription(Path, self.local_plan_topic, self.on_local_plan, 10)

        self.metrics = RunMetrics()

        self.get_logger().info(
            f"Listening for goals on {self.goal_topic} -> sending to action '{self.action_name}'.\n"
            f"Odom:{self.odom_topic}  Cmd:{self.cmd_vel_topic}  Scan:{self.scan_topic}\n"
            f"TF: {self.global_frame} -> {self.base_frame}\n"
            f"Tip: if in Gazebo, run with --ros-args -p use_sim_time:=true"
        )

    def now(self) -> rclpy.time.Time:
        return self.get_clock().now()

    def lookup_robot_pose_map(self) -> Optional[tuple]:
        """Return (x, y, yaw) of base_frame in global_frame using TF."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time()
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
            return (x, y, yaw)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed ({self.global_frame}->{self.base_frame}): {e}")
            return None

    def reset_for_new_run(self, goal: PoseStamped):
        self.metrics = RunMetrics()
        self.metrics.run_id += 1  # will be overwritten by below if you prefer persistent counter
        # Better persistent counter:
        # keep a separate counter:
        pass

    def on_goal_pose(self, msg: PoseStamped):
        if self.metrics.active:
            self.get_logger().warn("Already navigating; ignoring new /goal_pose.")
            return

        # Expect goal is in global_frame (usually 'map' from RViz 2D Goal Pose)
        if msg.header.frame_id and msg.header.frame_id != self.global_frame:
            self.get_logger().warn(
                f"Goal frame '{msg.header.frame_id}' != global_frame '{self.global_frame}'. "
                "For simplicity, this script assumes goal_pose is already in global_frame."
            )

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        q = msg.pose.orientation
        goal_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        pose = self.lookup_robot_pose_map()
        if pose is not None:
            rx, ry, _ = pose
            self.metrics.straight_line_dist = math.sqrt(
                (self.metrics.goal_x - rx) ** 2 +
                (self.metrics.goal_y - ry) ** 2
            )

        # Start a new run
        self.metrics = RunMetrics(
            run_id=(self.metrics.run_id + 1),
            active=True,
            goal_x=goal_x,
            goal_y=goal_y,
            goal_yaw=goal_yaw,
            t_start=self.now(),
            settle_window_s=float(self.get_parameter("settle_window_s").value),
            tol_xy=float(self.get_parameter("tol_xy").value),
            tol_yaw=float(self.get_parameter("tol_yaw").value),
        )

        self.get_logger().info(
            f"\n=== RUN {self.metrics.run_id} START ===\n"
            f"Goal: x={goal_x:.3f} y={goal_y:.3f} yaw={math.degrees(goal_yaw):.1f}deg  frame={msg.header.frame_id or self.global_frame}"
        )

        # Wait for action server then send goal
        if not self._ac.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"Nav2 action server '{self.action_name}' not available.")
            self.metrics.active = False
            return

        goal = NavigateToPose.Goal()
        goal.pose = msg  # send the same PoseStamped

        send_future = self._ac.send_goal_async(goal, feedback_callback=self.on_feedback)
        send_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2.")
            self.metrics.active = False
            return

        self.get_logger().info("Goal accepted. Collecting metrics...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        # NavigateToPose feedback typically includes distance_remaining, navigation_time, number_of_recoveries, current_pose
        if hasattr(fb, "distance_remaining"):
            dr = float(fb.distance_remaining)
            self.metrics.last_distance_remaining = dr
            self.metrics.min_distance_remaining = min(self.metrics.min_distance_remaining, dr)

        if hasattr(fb, "number_of_recoveries"):
            self.metrics.recoveries = int(fb.number_of_recoveries)

        # Settling detection using feedback current_pose if available
        if hasattr(fb, "current_pose"):
            cp = fb.current_pose
            cx = cp.pose.position.x
            cy = cp.pose.position.y
            cq = cp.pose.orientation
            cyaw = quat_to_yaw(cq.x, cq.y, cq.z, cq.w)

            dx = self.metrics.goal_x - cx
            dy = self.metrics.goal_y - cy
            dist = hypot2(dx, dy)
            dyaw = abs(angle_wrap(self.metrics.goal_yaw - cyaw))

            now = self.now()
            within = (dist <= self.metrics.tol_xy) and (dyaw <= self.metrics.tol_yaw)

            if within:
                if self.metrics.within_tol_since is None:
                    self.metrics.within_tol_since = now
                else:
                    dt = (now - self.metrics.within_tol_since).nanoseconds * 1e-9
                    if self.metrics.settle_time_s is None and dt >= self.metrics.settle_window_s:
                        self.metrics.settle_time_s = (now - self.metrics.t_start).nanoseconds * 1e-9
            else:
                self.metrics.within_tol_since = None

    def on_odom(self, msg: Odometry):
        if not self.metrics.active:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.metrics.last_odom_x is not None:
            self.metrics.path_len_m += hypot2(x - self.metrics.last_odom_x, y - self.metrics.last_odom_y)

        self.metrics.last_odom_x = x
        self.metrics.last_odom_y = y

        # actual speed from odom twist
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        v = math.sqrt(vx * vx + vy * vy)
        # RMS accumulation
        self.metrics.sum_v_sq += v * v
        self.metrics.sum_w_sq += abs(wz) * abs(wz)
        self.metrics.sample_count += 1

        # Stop-go detection
        stop_threshold = 0.05
        if self.metrics.last_speed is not None:
            if self.metrics.last_speed > stop_threshold and v < stop_threshold:
                self.metrics.stop_go_count += 1
        self.metrics.last_speed = v

        # Velocity sign flip (forward/back oscillation)
        vx = msg.twist.twist.linear.x
        current_sign = 1 if vx > 0 else (-1 if vx < 0 else 0)
        if self.metrics.last_vx_sign is not None:
            if current_sign != 0 and current_sign != self.metrics.last_vx_sign:
                self.metrics.sign_flip_count += 1
        self.metrics.last_vx_sign = current_sign
        self.metrics.max_act_v = max(self.metrics.max_act_v, v)
        self.metrics.max_act_w = max(self.metrics.max_act_w, abs(wz))

    def on_cmd_vel(self, msg: Twist):
        if not self.metrics.active:
            return
        v = math.sqrt(msg.linear.x * msg.linear.x + msg.linear.y * msg.linear.y)
        w = abs(msg.angular.z)
        self.metrics.max_cmd_v = max(self.metrics.max_cmd_v, v)
        self.metrics.max_cmd_w = max(self.metrics.max_cmd_w, w) 

    def on_scan(self, msg: LaserScan):
        if not self.metrics.active:
            return

        now = self.now()

        # 1️⃣ Compute minimum valid scan distance first
        m = float("inf")
        rmin = msg.range_min
        rmax = msg.range_max

        for r in msg.ranges:
            if math.isfinite(r) and rmin <= r <= rmax:
                if r < m:
                    m = r

        # 2️⃣ Accumulate unsafe clearance time
        if self.metrics.last_scan_time is not None:
            dt = (now - self.metrics.last_scan_time).nanoseconds * 1e-9
            if m < self.metrics.clearance_threshold:
                self.metrics.unsafe_clearance_time += dt

        self.metrics.last_scan_time = now

        # 3️⃣ Track global minimum scan distance
        if m < self.metrics.min_scan:
            self.metrics.min_scan = m

    def on_global_plan(self, _msg: Path):
        if self.metrics.active:
            self.metrics.global_plan_updates += 1

    def on_local_plan(self, _msg: Path):
        if self.metrics.active:
            self.metrics.local_plan_updates += 1

    def on_result(self, future):
        res = future.result()
        self.metrics.t_end = self.now()
        self.metrics.result_status = int(res.status)

        # Final robot pose via TF (best)
        pose = self.lookup_robot_pose_map()
        if pose is not None:
            rx, ry, ryaw = pose

            dx = self.metrics.goal_x - rx
            dy = self.metrics.goal_y - ry

            # Transform error into dock frame
            dock_yaw = self.metrics.goal_yaw
            along_track =  math.cos(dock_yaw) * dx + math.sin(dock_yaw) * dy
            cross_track = -math.sin(dock_yaw) * dx + math.cos(dock_yaw) * dy

            dist = math.sqrt(dx*dx + dy*dy)
            dyaw = angle_wrap(self.metrics.goal_yaw - ryaw)

            self.metrics.final_dx = dx
            self.metrics.final_dy = dy
            self.metrics.final_dist = dist
            self.metrics.final_dyaw = dyaw

            self.metrics.along_track = along_track
            self.metrics.cross_track = cross_track

        self.metrics.active = False

        dt = (self.metrics.t_end - self.metrics.t_start).nanoseconds * 1e-9 if self.metrics.t_start else float("nan")
        status_str = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        }.get(self.metrics.result_status, f"STATUS_{self.metrics.result_status}")

        print("\n" + "=" * 60)
        print(f"RUN {self.metrics.run_id} RESULT: {status_str}")
        print(f"Duration            : {dt:.3f} s")
        print(f"Path length (odom)  : {self.metrics.path_len_m:.3f} m")
        print(f"Max cmd speed       : {self.metrics.max_cmd_v:.3f} m/s")
        print(f"Max cmd yaw rate    : {self.metrics.max_cmd_w:.3f} rad/s")
        print(f"Max actual speed    : {self.metrics.max_act_v:.3f} m/s")
        print(f"Max actual yaw rate : {self.metrics.max_act_w:.3f} rad/s")

        if math.isfinite(self.metrics.min_scan):
            print(f"Min lidar range     : {self.metrics.min_scan:.3f} m")
        else:
            print("Min lidar range     : (no valid /scan ranges)")

        if self.metrics.final_dist is not None:
            print("Final pose error (map):")
            print(f"  dx={self.metrics.final_dx:.3f} m  dy={self.metrics.final_dy:.3f} m  dist={self.metrics.final_dist:.3f} m")
            print(f"  dyaw={math.degrees(self.metrics.final_dyaw):.2f} deg")
        else:
            print("Final pose error     : (TF unavailable)")

        if self.metrics.settle_time_s is not None:
            print(f"Settle time          : {self.metrics.settle_time_s:.3f} s "
                  f"(within xy<= {self.metrics.tol_xy} m, yaw<= {math.degrees(self.metrics.tol_yaw):.1f} deg "
                  f"for {self.metrics.settle_window_s:.1f}s)")

        if self.metrics.last_distance_remaining is not None:
            print(f"Distance remaining   : last={self.metrics.last_distance_remaining:.3f} m  min={self.metrics.min_distance_remaining:.3f} m")

        if self.metrics.recoveries is not None:
            print(f"Recoveries (Nav2)    : {self.metrics.recoveries}")

        print(f"Global plan updates  : {self.metrics.global_plan_updates}  (topic: {self.global_plan_topic})")
        print(f"Local plan updates   : {self.metrics.local_plan_updates}  (topic: {self.local_plan_topic})")
        # Efficiency
        if self.metrics.straight_line_dist is not None and self.metrics.straight_line_dist > 0:
            ratio = self.metrics.path_len_m / self.metrics.straight_line_dist
            print(f"Straight-line ratio  : {ratio:.3f}")

        # RMS velocity
        if self.metrics.sample_count > 0:
            rms_v = math.sqrt(self.metrics.sum_v_sq / self.metrics.sample_count)
            rms_w = math.sqrt(self.metrics.sum_w_sq / self.metrics.sample_count)
            print(f"RMS linear velocity  : {rms_v:.3f} m/s")
            print(f"RMS angular velocity : {rms_w:.3f} rad/s")

        print(f"Stop-go count        : {self.metrics.stop_go_count}")
        print(f"Velocity sign flips  : {self.metrics.sign_flip_count}")

        print(f"Unsafe clearance time: {self.metrics.unsafe_clearance_time:.3f} s "
            f"(threshold {self.metrics.clearance_threshold} m)")

        if self.metrics.along_track is not None:
            print("Dock frame error:")
            print(f"  Along-track error  : {self.metrics.along_track:.3f} m")
            print(f"  Cross-track error  : {self.metrics.cross_track:.3f} m")
        print("=" * 60 + "\n")


def main():
    rclpy.init(args=sys.argv)
    node = Nav2GoalMetrics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()