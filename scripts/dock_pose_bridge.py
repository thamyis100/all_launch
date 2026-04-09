#!/usr/bin/env python3
"""
Nav2 Goal Metrics with Trajectory Visualization and Docking Support

Architecture (inheritance-based, zero duplication):
  BaseMetricsNode  — shared infra (TF, odom, cmd_vel, scan, visualizer, _finalize_run)
  Nav2GoalMetrics  — /goal_pose  → NavigateToPose action
  DockMetrics      — /dock_pose  → DockRobot action, /undock_trigger → UndockRobot action
"""
import math
import sys
import os
from dataclasses import dataclass, field
from typing import Optional, List, Dict
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatus
from std_msgs.msg import Empty
from nav2_msgs.action import NavigateToPose
import tf2_ros

try:
    from opennav_docking_msgs.action import DockRobot, UndockRobot
    DOCKING_AVAILABLE = True
except ImportError:
    DOCKING_AVAILABLE = False
    print("[WARN] opennav_docking_msgs not available. DockMetrics node disabled.")

# ============================================================================
# VISUALIZATION IMPORTS
# ============================================================================
try:
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend for server/headless use
    import matplotlib.pyplot as plt
    import numpy as np
    VISUALIZATION_AVAILABLE = True
except ImportError:
    VISUALIZATION_AVAILABLE = False
    print("[WARN] matplotlib not available. Visualization features disabled.")

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================
def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """yaw (z-axis rotation) from quaternion"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def angle_wrap(a: float) -> float:
    """Wrap to [-pi, pi]"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def hypot2(x: float, y: float) -> float:
    return math.sqrt(x * x + y * y)

# ============================================================================
# METRICS DATA CLASS
# ============================================================================
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
    # Settling detection
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
    # Run mode: "NAV" for navigation, "DOCK" for docking, "UNDOCK" for undocking
    run_mode: str = "NAV"
    # Trajectory visualization data
    trajectory_points: List[Dict[str, float]] = field(default_factory=list)
    scan_obstacles: List[Dict] = field(default_factory=list)
    start_x: Optional[float] = None
    start_y: Optional[float] = None
    start_yaw: Optional[float] = None
    image_path: Optional[str] = None

# ============================================================================
# TRAJECTORY VISUALIZATION CLASS
# ============================================================================
class TrajectoryVisualizer:
    """Generates static trajectory visualization images with obstacles"""

    def __init__(self, output_dir: str = "./trajectory_images"):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

    def generate_static_image(self, metrics: RunMetrics,
                              title: str = "Nav2 Navigation Trajectory",
                              show_heading: bool = True,
                              show_goal: bool = True,
                              show_obstacles: bool = True,
                              figsize: tuple = (14, 10),
                              dpi: int = 300) -> Optional[str]:
        """Generate static trajectory visualization image with obstacles"""

        if not VISUALIZATION_AVAILABLE:
            return None

        if len(metrics.trajectory_points) < 2:
            return None

        fig, ax = plt.subplots(1, 1, figsize=figsize)

        # 1) Draw obstacle points from laser scans
        if show_obstacles and len(metrics.scan_obstacles) > 0:
            all_obstacle_x = []
            all_obstacle_y = []
            for scan_data in metrics.scan_obstacles:
                obstacles = scan_data.get('obstacles', {'x': [], 'y': []})
                all_obstacle_x.extend(obstacles['x'])
                all_obstacle_y.extend(obstacles['y'])
            if all_obstacle_x:
                ax.scatter(all_obstacle_x, all_obstacle_y, c='gray', s=10,
                           alpha=0.5, label='Obstacles (Laser Scan)', zorder=1)

        # Extract trajectory data
        x_coords = [p['x'] for p in metrics.trajectory_points]
        y_coords = [p['y'] for p in metrics.trajectory_points]
        yaws = [p.get('yaw', 0) for p in metrics.trajectory_points]

        # 2) Draw trajectory path
        ax.plot(x_coords, y_coords, 'b-', linewidth=2, alpha=0.7, label='Trajectory', zorder=2)

        # 3) Draw start/end points
        if metrics.start_x is not None:
            ax.plot(metrics.start_x, metrics.start_y, 'go', markersize=12,
                    label='Start', markeredgecolor='darkgreen', markeredgewidth=2, zorder=3)
        elif len(x_coords) > 0:
            ax.plot(x_coords[0], y_coords[0], 'go', markersize=12,
                    label='Start', markeredgecolor='darkgreen', markeredgewidth=2, zorder=3)

        if len(x_coords) > 0:
            ax.plot(x_coords[-1], y_coords[-1], 'rs', markersize=12,
                    label='End', markeredgecolor='darkred', markeredgewidth=2, zorder=3)

        # 4) Draw goal pose
        if show_goal:
            ax.plot(metrics.goal_x, metrics.goal_y, 'm*', markersize=18,
                    label='Goal', markeredgecolor='purple', markeredgewidth=2, zorder=3)
            goal_dx = 0.6 * math.cos(metrics.goal_yaw)
            goal_dy = 0.6 * math.sin(metrics.goal_yaw)
            ax.arrow(metrics.goal_x, metrics.goal_y, goal_dx, goal_dy,
                     head_width=0.12, head_length=0.18, fc='purple', ec='purple',
                     alpha=0.9, linewidth=2.5, zorder=3)

        # 5) Draw heading arrows along trajectory
        if show_heading and len(yaws) > 0:
            skip_rate = max(1, len(metrics.trajectory_points) // 25)
            for i in range(0, len(metrics.trajectory_points), skip_rate):
                p = metrics.trajectory_points[i]
                self._draw_heading_arrow(ax, p['x'], p['y'], p.get('yaw', 0),
                                         length=0.3, color='orange', alpha=0.6, zorder=2)

        # 6) Formatting
        ax.set_xlabel('X Position (m)', fontsize=11)
        ax.set_ylabel('Y Position (m)', fontsize=11)
        ax.set_title(title, fontsize=13, fontweight='bold')
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.set_aspect('equal')

        if x_coords and y_coords:
            margin = 1.0
            ax.set_xlim(min(x_coords) - margin, max(x_coords) + margin)
            ax.set_ylim(min(y_coords) - margin, max(y_coords) + margin)

        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(handles, labels, loc='best', fontsize=9, framealpha=0.9)

        # Statistics overlay
        stats_text = self._generate_statistics_text(metrics)
        props = dict(boxstyle='round', facecolor='white', alpha=0.85, edgecolor='gray')
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, fontsize=9,
                verticalalignment='top', bbox=props, fontfamily='monospace', zorder=4)

        # Save image
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        status_str = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCESS",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }.get(metrics.result_status, "UNKNOWN")

        filename = f"trajectory_run{metrics.run_id}_{status_str}_{timestamp}.png"
        filepath = os.path.join(self.output_dir, filename)

        plt.tight_layout()
        plt.savefig(filepath, dpi=dpi, bbox_inches='tight', facecolor='white')
        plt.close()

        metrics.image_path = filepath
        return filepath

    def _draw_heading_arrow(self, ax, x, y, yaw,
                            length=0.3, color='orange', alpha=0.6, zorder=2):
        dx = length * math.cos(yaw)
        dy = length * math.sin(yaw)
        ax.arrow(x, y, dx, dy, head_width=0.06, head_length=0.09,
                 fc=color, ec=color, alpha=alpha, linewidth=1.5, zorder=zorder)

    def _generate_statistics_text(self, metrics: RunMetrics) -> str:
        if metrics.t_start and metrics.t_end:
            duration = (metrics.t_end - metrics.t_start).nanoseconds * 1e-9
        else:
            duration = 0

        status_str = {
            GoalStatus.STATUS_SUCCEEDED: "OK SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "X ABORTED",
            GoalStatus.STATUS_CANCELED: "O CANCELED",
        }.get(metrics.result_status, "? UNKNOWN")

        error_line = f"Error: {metrics.final_dist:.3f}m" if metrics.final_dist else "Error: N/A"
        stats = (
            f"{status_str} | {metrics.run_mode} Run #{metrics.run_id}\n"
            f"----------------------\n"
            f"Duration: {duration:.1f}s\n"
            f"Path: {metrics.path_len_m:.2f}m\n"
            f"{error_line}"
        )
        return stats

# ============================================================================
# BASE METRICS NODE — shared infrastructure for Nav and Dock
# ============================================================================
class BaseMetricsNode(Node):
    """
    Shared infrastructure: TF, odom/cmd_vel/scan/plan subscribers,
    trajectory recording, visualizer, and _finalize_run().
    Subclasses only need to add their action client + goal subscription.
    """

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)

        # --- Shared parameters ---
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("global_plan_topic", "/plan")
        self.declare_parameter("local_plan_topic", "/trajectories")
        self.declare_parameter("settle_window_s", 1.0)
        self.declare_parameter("tol_xy", 0.15)
        self.declare_parameter("tol_yaw", 0.15)
        self.declare_parameter("enable_visualization", True)
        self.declare_parameter("visualization_output_dir", "./trajectory_images")
        self.declare_parameter("trajectory_record_rate_hz", 10.0)

        self.global_frame = self.get_parameter("global_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.global_plan_topic = self.get_parameter("global_plan_topic").value
        self.local_plan_topic = self.get_parameter("local_plan_topic").value

        # Visualization
        self.enable_viz = self.get_parameter("enable_visualization").value
        self.viz_output_dir = self.get_parameter("visualization_output_dir").value
        self.trajectory_rate = self.get_parameter("trajectory_record_rate_hz").value

        if self.enable_viz and VISUALIZATION_AVAILABLE:
            self.visualizer = TrajectoryVisualizer(self.viz_output_dir)
            self.get_logger().info(f"Visualization enabled. Output: {self.viz_output_dir}")
        else:
            self.visualizer = None
            if self.enable_viz and not VISUALIZATION_AVAILABLE:
                self.get_logger().warn("Visualization requested but matplotlib not available")

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Shared subscriptions
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)
        self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 50)

        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, scan_qos)

        self.create_subscription(Path, self.global_plan_topic, self.on_global_plan, 10)
        self.create_subscription(Path, self.local_plan_topic, self.on_local_plan, 10)

        # Metrics state
        self.metrics = RunMetrics()

        # Trajectory recording timer
        if self.enable_viz and VISUALIZATION_AVAILABLE:
            self.trajectory_timer = self.create_timer(
                1.0 / self.trajectory_rate,
                self.record_trajectory_callback
            )

    # ----- Utilities ---------------------------------------------------------
    def now(self) -> rclpy.time.Time:
        return self.get_clock().now()

    def lookup_robot_pose_map(self) -> Optional[tuple]:
        """Return (x, y, yaw) of base_frame in global_frame via TF."""
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
            self.get_logger().warn(
                f"TF lookup failed ({self.global_frame}->{self.base_frame}): {e}",
                throttle_duration_sec=5.0
            )
            return None

    # ----- Run lifecycle helpers ---------------------------------------------
    def _start_run(self, goal_x: float, goal_y: float, goal_yaw: float,
                   run_mode: str = "NAV") -> None:
        """Create a fresh RunMetrics and record starting pose."""
        pose = self.lookup_robot_pose_map()
        straight_line = None
        if pose is not None:
            rx, ry, _ = pose
            straight_line = math.sqrt((goal_x - rx) ** 2 + (goal_y - ry) ** 2)

        self.metrics = RunMetrics(
            run_id=(self.metrics.run_id + 1),
            active=True,
            goal_x=goal_x,
            goal_y=goal_y,
            goal_yaw=goal_yaw,
            straight_line_dist=straight_line,
            t_start=self.now(),
            settle_window_s=float(self.get_parameter("settle_window_s").value),
            tol_xy=float(self.get_parameter("tol_xy").value),
            tol_yaw=float(self.get_parameter("tol_yaw").value),
            trajectory_points=[],
            scan_obstacles=[],
            run_mode=run_mode,
        )

        if pose is not None:
            self.metrics.start_x, self.metrics.start_y, self.metrics.start_yaw = pose

    def _check_settling(self, dist: float, dyaw: float) -> None:
        """Update settling-detection state (shared by nav + dock feedback)."""
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

    def _on_action_result(self, future) -> None:
        """Shared result callback — works for NavigateToPose, DockRobot, UndockRobot."""
        res = future.result()
        self.metrics.t_end = self.now()
        self.metrics.result_status = int(res.status)

        # For undock the "goal" is wherever we end up
        if self.metrics.run_mode == "UNDOCK":
            pose = self.lookup_robot_pose_map()
            if pose is not None:
                self.metrics.goal_x, self.metrics.goal_y, self.metrics.goal_yaw = pose

        self._finalize_run()

    def _finalize_run(self) -> None:
        """Compute errors, generate trajectory image, print stats."""

        # Final robot pose via TF
        pose = self.lookup_robot_pose_map()
        if pose is not None:
            rx, ry, ryaw = pose
            dx = self.metrics.goal_x - rx
            dy = self.metrics.goal_y - ry

            goal_yaw = self.metrics.goal_yaw
            along_track =  math.cos(goal_yaw) * dx + math.sin(goal_yaw) * dy
            cross_track = -math.sin(goal_yaw) * dx + math.cos(goal_yaw) * dy
            dist = math.sqrt(dx * dx + dy * dy)
            dyaw = angle_wrap(self.metrics.goal_yaw - ryaw)

            self.metrics.final_dx = dx
            self.metrics.final_dy = dy
            self.metrics.final_dist = dist
            self.metrics.final_dyaw = dyaw
            self.metrics.along_track = along_track
            self.metrics.cross_track = cross_track

        self.metrics.active = False

        # --- TRAJECTORY IMAGE ---
        if self.visualizer and len(self.metrics.trajectory_points) > 0:
            status_str = {
                GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                GoalStatus.STATUS_ABORTED: "ABORTED",
                GoalStatus.STATUS_CANCELED: "CANCELED",
            }.get(self.metrics.result_status, "UNKNOWN")

            mode_label = self.metrics.run_mode
            title = f"{mode_label} Run {self.metrics.run_id}: {status_str}"
            image_path = self.visualizer.generate_static_image(self.metrics, title=title)

            if image_path:
                self.get_logger().info(f"Trajectory image saved: {image_path}")
            else:
                self.get_logger().warn("Failed to generate trajectory image")

        # --- CONSOLE OUTPUT ---
        dt = (self.metrics.t_end - self.metrics.t_start).nanoseconds * 1e-9 if self.metrics.t_start else float("nan")
        status_str = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        }.get(self.metrics.result_status, f"STATUS_{self.metrics.result_status}")

        mode_label = self.metrics.run_mode
        print("\n" + "=" * 60)
        print(f"{mode_label} RUN {self.metrics.run_id} RESULT: {status_str}")
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

        if self.metrics.straight_line_dist is not None and self.metrics.straight_line_dist > 0:
            ratio = self.metrics.path_len_m / self.metrics.straight_line_dist
            print(f"Straight-line ratio  : {ratio:.3f}")

        if self.metrics.sample_count > 0:
            rms_v = math.sqrt(self.metrics.sum_v_sq / self.metrics.sample_count)
            rms_w = math.sqrt(self.metrics.sum_w_sq / self.metrics.sample_count)
            print(f"RMS linear velocity  : {rms_v:.3f} m/s")
            print(f"RMS angular velocity : {rms_w:.3f} rad/s")

        print(f"Stop-go count        : {self.metrics.stop_go_count}")
        print(f"Velocity sign flips  : {self.metrics.sign_flip_count}")
        print(f"Unsafe clearance time: {self.metrics.unsafe_clearance_time:.3f} s "
              f"(threshold {self.metrics.clearance_threshold} m)")

        if self.metrics.image_path:
            print(f"Trajectory image     : {self.metrics.image_path}")

        if self.metrics.along_track is not None:
            print("Goal frame error:")
            print(f"  Along-track error  : {self.metrics.along_track:.3f} m")
            print(f"  Cross-track error  : {self.metrics.cross_track:.3f} m")

        print("=" * 60 + "\n")

    # ----- Trajectory recording ----------------------------------------------
    def record_trajectory_callback(self):
        """Periodically record robot pose for trajectory visualization."""
        if not self.metrics.active or not self.visualizer:
            return

        pose = self.lookup_robot_pose_map()
        if pose is not None:
            x, y, yaw = pose
            self.metrics.trajectory_points.append({
                'x': x,
                'y': y,
                'yaw': yaw,
                'timestamp': self.now().nanoseconds * 1e-9
            })

            if self.metrics.start_x is None:
                self.metrics.start_x = x
                self.metrics.start_y = y
                self.metrics.start_yaw = yaw

    # ----- Shared topic callbacks --------------------------------------------
    def on_odom(self, msg: Odometry):
        if not self.metrics.active:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.metrics.last_odom_x is not None:
            self.metrics.path_len_m += hypot2(
                x - self.metrics.last_odom_x, y - self.metrics.last_odom_y
            )
        self.metrics.last_odom_x = x
        self.metrics.last_odom_y = y

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        v = math.sqrt(vx * vx + vy * vy)

        self.metrics.sum_v_sq += v * v
        self.metrics.sum_w_sq += abs(wz) * abs(wz)
        self.metrics.sample_count += 1

        stop_threshold = 0.05
        if self.metrics.last_speed is not None:
            if self.metrics.last_speed > stop_threshold and v < stop_threshold:
                self.metrics.stop_go_count += 1
        self.metrics.last_speed = v

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
        pose = self.lookup_robot_pose_map()
        if pose is None:
            return

        rx, ry, ryaw = pose

        m = float("inf")
        rmin = msg.range_min
        rmax = msg.range_max
        valid_ranges = []
        valid_angles = []

        for i, r in enumerate(msg.ranges):
            if math.isfinite(r) and rmin <= r <= rmax:
                if r < m:
                    m = r
                if len(valid_ranges) < 200:
                    angle = msg.angle_min + i * msg.angle_increment
                    valid_ranges.append(r)
                    valid_angles.append(angle)

        obstacle_points = {'x': [], 'y': []}
        for r, angle in zip(valid_ranges, valid_angles):
            world_angle = ryaw + angle
            ox = rx + r * math.cos(world_angle)
            oy = ry + r * math.sin(world_angle)
            obstacle_points['x'].append(ox)
            obstacle_points['y'].append(oy)

        if len(self.metrics.scan_obstacles) < 30:
            self.metrics.scan_obstacles.append({
                'robot_x': rx,
                'robot_y': ry,
                'robot_yaw': ryaw,
                'obstacles': obstacle_points,
                'timestamp': now.nanoseconds * 1e-9
            })

        if self.metrics.last_scan_time is not None:
            dt = (now - self.metrics.last_scan_time).nanoseconds * 1e-9
            if m < self.metrics.clearance_threshold:
                self.metrics.unsafe_clearance_time += dt
        self.metrics.last_scan_time = now

        if m < self.metrics.min_scan:
            self.metrics.min_scan = m

    def on_global_plan(self, _msg: Path):
        if self.metrics.active:
            self.metrics.global_plan_updates += 1

    def on_local_plan(self, _msg: Path):
        if self.metrics.active:
            self.metrics.local_plan_updates += 1

# ============================================================================
# NAV2 GOAL METRICS — /goal_pose -> NavigateToPose
# ============================================================================
class Nav2GoalMetrics(BaseMetricsNode):
    """Listens to /goal_pose, sends NavigateToPose, collects full metrics."""

    def __init__(self):
        super().__init__("nav2_goal_metrics")

        # Extra parameters
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("nav_action_name", "navigate_to_pose")

        self.goal_topic = self.get_parameter("goal_topic").value
        self.action_name = self.get_parameter("nav_action_name").value

        # Action client
        self._ac = ActionClient(self, NavigateToPose, self.action_name)

        # Subscription
        self.create_subscription(PoseStamped, self.goal_topic, self.on_goal_pose, 10)

        self.get_logger().info(
            f"[Nav] Listening on {self.goal_topic} -> action '{self.action_name}'  |  "
            f"Odom:{self.odom_topic}  Cmd:{self.cmd_vel_topic}  Scan:{self.scan_topic}  "
            f"TF: {self.global_frame} -> {self.base_frame}"
        )

    # ----- Goal handling -----------------------------------------------------
    def on_goal_pose(self, msg: PoseStamped):
        if self.metrics.active:
            self.get_logger().warn(
                "Already navigating; ignoring new /goal_pose.",
                throttle_duration_sec=2.0
            )
            return

        if msg.header.frame_id and msg.header.frame_id != self.global_frame:
            self.get_logger().warn(
                f"Goal frame '{msg.header.frame_id}' != global_frame '{self.global_frame}'."
            )

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        q = msg.pose.orientation
        goal_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        self._start_run(goal_x, goal_y, goal_yaw, run_mode="NAV")

        self.get_logger().info(
            f"\n=== NAV RUN {self.metrics.run_id} START ===\n"
            f"Goal: x={goal_x:.3f} y={goal_y:.3f} yaw={math.degrees(goal_yaw):.1f}deg  "
            f"frame={msg.header.frame_id or self.global_frame}"
        )

        if not self._ac.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"Nav2 action server '{self.action_name}' not available.")
            self.metrics.active = False
            return

        goal = NavigateToPose.Goal()
        goal.pose = msg
        send_future = self._ac.send_goal_async(goal, feedback_callback=self.on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2.")
            self.metrics.active = False
            return
        self.get_logger().info("Goal accepted. Collecting metrics...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_action_result)

    def on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        if hasattr(fb, "distance_remaining"):
            dr = float(fb.distance_remaining)
            self.metrics.last_distance_remaining = dr
            self.metrics.min_distance_remaining = min(self.metrics.min_distance_remaining, dr)
        if hasattr(fb, "number_of_recoveries"):
            self.metrics.recoveries = int(fb.number_of_recoveries)

        # Settling detection
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
            self._check_settling(dist, dyaw)

# ============================================================================
# DOCK METRICS — /dock_pose -> DockRobot, /undock_trigger -> UndockRobot
#   Full metrics + trajectory images, modeled after Nav2GoalMetrics.
# ============================================================================
class DockMetrics(BaseMetricsNode):
    """Listens to /dock_pose and /undock_trigger, collects full metrics."""

    def __init__(self):
        super().__init__("dock_metrics")

        if not DOCKING_AVAILABLE:
            self.get_logger().error(
                "opennav_docking_msgs not found! Install with:\n"
                "  sudo apt install ros-humble-opennav-docking-msgs"
            )
            self._dock_ac = None
            self._undock_ac = None
            return

        # Extra parameters
        self.declare_parameter("dock_type", "simple_charging_dock")
        self.declare_parameter("dock_pose_topic", "/dock_pose")
        self.declare_parameter("undock_trigger_topic", "/undock_trigger")

        self.dock_type = self.get_parameter("dock_type").get_parameter_value().string_value
        dock_pose_topic = self.get_parameter("dock_pose_topic").get_parameter_value().string_value
        undock_trigger_topic = self.get_parameter("undock_trigger_topic").get_parameter_value().string_value

        # Action clients
        self._dock_ac = ActionClient(self, DockRobot, "/dock_robot")
        self._undock_ac = ActionClient(self, UndockRobot, "/undock_robot")

        # Subscriptions
        self.create_subscription(PoseStamped, dock_pose_topic, self.on_dock_pose, 10)
        self.create_subscription(Empty, undock_trigger_topic, self.on_undock_trigger, 10)

        self.get_logger().info(
            f"[Dock] Listening on {dock_pose_topic} -> /dock_robot  |  "
            f"{undock_trigger_topic} -> /undock_robot  |  "
            f"dock_type='{self.dock_type}'  "
            f"TF: {self.global_frame} -> {self.base_frame}"
        )

    # =========================================================================
    # DOCK POSE HANDLER
    # =========================================================================
    def on_dock_pose(self, msg: PoseStamped):
        if not DOCKING_AVAILABLE or self._dock_ac is None:
            self.get_logger().error("Docking not available.")
            return

        if self.metrics.active:
            self.get_logger().warn(
                "Already docking/undocking; ignoring new /dock_pose.",
                throttle_duration_sec=2.0
            )
            return

        if msg.header.frame_id and msg.header.frame_id != self.global_frame:
            self.get_logger().warn(
                f"Dock pose frame '{msg.header.frame_id}' != global_frame '{self.global_frame}'."
            )

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        q = msg.pose.orientation
        goal_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        self._start_run(goal_x, goal_y, goal_yaw, run_mode="DOCK")

        self.get_logger().info(
            f"\n=== DOCK RUN {self.metrics.run_id} START ===\n"
            f"Dock pose: x={goal_x:.3f} y={goal_y:.3f} yaw={math.degrees(goal_yaw):.1f}deg  "
            f"frame={msg.header.frame_id or self.global_frame}"
        )

        if not self._dock_ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("/dock_robot action server not available!")
            self.metrics.active = False
            return

        goal = DockRobot.Goal()
        goal.use_dock_id = False
        goal.dock_type = self.dock_type
        goal.dock_pose = msg

        send_future = self._dock_ac.send_goal_async(
            goal, feedback_callback=self.on_dock_feedback
        )
        send_future.add_done_callback(self._on_dock_goal_response)

    def _on_dock_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Dock goal REJECTED")
            self.metrics.active = False
            return
        self.get_logger().info("Dock goal accepted. Collecting metrics...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_action_result)

    def on_dock_feedback(self, feedback_msg):
        """DockRobot feedback — compute distance from TF (no distance_remaining field)."""
        pose = self.lookup_robot_pose_map()
        if pose is not None:
            rx, ry, ryaw = pose
            dx = self.metrics.goal_x - rx
            dy = self.metrics.goal_y - ry
            dist = math.sqrt(dx * dx + dy * dy)
            self.metrics.last_distance_remaining = dist
            self.metrics.min_distance_remaining = min(
                self.metrics.min_distance_remaining, dist
            )
            dyaw = abs(angle_wrap(self.metrics.goal_yaw - ryaw))
            self._check_settling(dist, dyaw)

    # =========================================================================
    # UNDOCK TRIGGER HANDLER
    # =========================================================================
    def on_undock_trigger(self, msg: Empty):
        if not DOCKING_AVAILABLE or self._undock_ac is None:
            self.get_logger().error("Docking not available.")
            return

        if self.metrics.active:
            self.get_logger().warn("Already docking/undocking; ignoring undock trigger.")
            return

        pose = self.lookup_robot_pose_map()

        # For undock we don't really have a "goal" yet — set to current pose,
        # _on_action_result will update it to final pose.
        gx = pose[0] if pose else 0.0
        gy = pose[1] if pose else 0.0
        gyaw = pose[2] if pose else 0.0
        self._start_run(gx, gy, gyaw, run_mode="UNDOCK")

        self.get_logger().info(f"\n=== UNDOCK RUN {self.metrics.run_id} START ===")

        if not self._undock_ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("/undock_robot action server not available!")
            self.metrics.active = False
            return

        goal = UndockRobot.Goal()
        goal.dock_type = self.dock_type

        send_future = self._undock_ac.send_goal_async(goal)
        send_future.add_done_callback(self._on_undock_goal_response)

    def _on_undock_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Undock goal REJECTED")
            self.metrics.active = False
            return
        self.get_logger().info("Undock goal accepted. Collecting metrics...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_action_result)

# ============================================================================
# MAIN FUNCTION
# ============================================================================
def main():
    rclpy.init(args=sys.argv)

    from rclpy.executors import MultiThreadedExecutor

    nav_node = Nav2GoalMetrics()
    dock_node = DockMetrics() if DOCKING_AVAILABLE else None

    executor = MultiThreadedExecutor()
    executor.add_node(nav_node)
    if dock_node is not None:
        executor.add_node(dock_node)
    else:
        nav_node.get_logger().warn(
            "DockMetrics disabled (opennav_docking_msgs not found). "
            "Only /goal_pose navigation metrics available."
        )

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        nav_node.destroy_node()
        if dock_node is not None:
            dock_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
