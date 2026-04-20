#!/usr/bin/env python3
"""
Nav2 Goal Metrics with Trajectory Visualization
Collects navigation metrics and generates static trajectory images with obstacles
"""
import math
import sys
import os
import csv
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
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
import tf2_ros

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
# METRICS DATA CLASS (Enhanced with visualization data)
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
    # Path length (from recorded TF trajectory)
    path_len_tf_m: float = 0.0
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
    
    # =========================================================================
    # NEW: TRAJECTORY VISUALIZATION DATA
    # =========================================================================
    trajectory_points: List[Dict[str, float]] = field(default_factory=list)
    scan_obstacles: List[Dict] = field(default_factory=list)
    start_x: Optional[float] = None
    start_y: Optional[float] = None
    start_yaw: Optional[float] = None
    image_path: Optional[str] = None
    # Goal source + dock tf snapshot
    source_topic: str = ""
    dock_tf_x: Optional[float] = None
    dock_tf_y: Optional[float] = None
    dock_tf_yaw: Optional[float] = None

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
        
        # Create figure
        fig, ax = plt.subplots(1, 1, figsize=figsize)
        
        # 1️⃣ Draw obstacle points from laser scans
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
        
        # 2️⃣ Draw trajectory path (controller-colored when available)
        controller_tags = [p.get('controller', 'unknown') for p in metrics.trajectory_points]
        self._plot_controller_colored_trajectory(ax, x_coords, y_coords, controller_tags)
        
        # 3️⃣ Draw start point
        if metrics.start_x is not None:
            ax.plot(metrics.start_x, metrics.start_y, 'go', markersize=12,
                   label='Start', markeredgecolor='darkgreen', markeredgewidth=2, zorder=3)
        elif len(x_coords) > 0:
            ax.plot(x_coords[0], y_coords[0], 'go', markersize=12,
                   label='Start', markeredgecolor='darkgreen', markeredgewidth=2, zorder=3)
        
        # Draw final robot position
        if len(x_coords) > 0:
            ax.plot(x_coords[-1], y_coords[-1], 'rs', markersize=10,
                   label='Robot Final', markeredgecolor='darkred', markeredgewidth=2, zorder=3)
        
        # 4️⃣ Draw target pose (goal or dock tf)
        if show_goal:
            target_x = metrics.goal_x
            target_y = metrics.goal_y
            target_yaw = metrics.goal_yaw
            target_label = 'Goal'
            target_marker_color = 'purple'

            if metrics.dock_tf_x is not None and metrics.dock_tf_y is not None:
                target_x = metrics.dock_tf_x
                target_y = metrics.dock_tf_y
                target_yaw = metrics.dock_tf_yaw if metrics.dock_tf_yaw is not None else metrics.goal_yaw
                target_label = 'Dock TF'
                target_marker_color = 'magenta'

            ax.plot(target_x, target_y, 'm*', markersize=18,
                   label=target_label, markeredgecolor='purple', markeredgewidth=2, zorder=3)

            # Target orientation arrow
            target_dx = 0.6 * math.cos(target_yaw)
            target_dy = 0.6 * math.sin(target_yaw)
            ax.arrow(target_x, target_y, target_dx, target_dy,
                    head_width=0.12, head_length=0.18, fc=target_marker_color, ec=target_marker_color,
                    alpha=0.9, linewidth=2.5, zorder=3)
        
        # 5️⃣ Draw heading arrows along trajectory
        if show_heading and len(yaws) > 0:
            skip_rate = max(1, len(metrics.trajectory_points) // 25)
            for i in range(0, len(metrics.trajectory_points), skip_rate):
                p = metrics.trajectory_points[i]
                self._draw_heading_arrow(ax, p['x'], p['y'], p.get('yaw', 0),
                                        length=0.3, color='orange', alpha=0.6, zorder=2)
        
        # 6️⃣ Formatting
        ax.set_xlabel('X Position (m)', fontsize=11)
        ax.set_ylabel('Y Position (m)', fontsize=11)
        ax.set_title(title, fontsize=13, fontweight='bold')
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.set_aspect('equal')
        
        # Auto-scale with margin
        if x_coords and y_coords:
            margin = 1.0
            ax.set_xlim(min(x_coords) - margin, max(x_coords) + margin)
            ax.set_ylim(min(y_coords) - margin, max(y_coords) + margin)
        
        # Legend
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
    
    def _draw_heading_arrow(self, ax, x: float, y: float, yaw: float,
                           length: float = 0.3, color: str = 'orange',
                           alpha: float = 0.6, zorder: int = 2):
        """Draw a heading arrow at position"""
        dx = length * math.cos(yaw)
        dy = length * math.sin(yaw)
        ax.arrow(x, y, dx, dy, head_width=0.06, head_length=0.09,
                fc=color, ec=color, alpha=alpha, linewidth=1.5, zorder=zorder)

    def _plot_controller_colored_trajectory(self, ax, x_coords, y_coords, controller_tags):
        """Plot trajectory segments with colors based on controller source."""
        if len(x_coords) < 2:
            return

        color_map = {
            'mppi': ('royalblue', 'Trajectory (MPPI)'),
            'docking_server': ('crimson', 'Trajectory (Docking Server)'),
            'unknown': ('royalblue', 'Trajectory'),
        }

        shown_labels = set()
        for i in range(1, len(x_coords)):
            tag = controller_tags[i] if i < len(controller_tags) else 'unknown'
            color, label = color_map.get(tag, color_map['unknown'])
            plot_label = label if label not in shown_labels else None
            ax.plot(
                [x_coords[i - 1], x_coords[i]],
                [y_coords[i - 1], y_coords[i]],
                color=color,
                linewidth=2,
                alpha=0.8,
                label=plot_label,
                zorder=2,
            )
            shown_labels.add(label)
    
    def _generate_statistics_text(self, metrics: RunMetrics) -> str:
        """Generate statistics text for image overlay"""
        if metrics.t_start and metrics.t_end:
            duration = (metrics.t_end - metrics.t_start).nanoseconds * 1e-9
        else:
            duration = 0
        
        status_str = {
            GoalStatus.STATUS_SUCCEEDED: "✓ SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "✗ ABORTED",
            GoalStatus.STATUS_CANCELED: "○ CANCELED",
        }.get(metrics.result_status, "? UNKNOWN")
        
        path_for_overlay = metrics.path_len_m if metrics.path_len_m > 0.0 else metrics.path_len_tf_m
        error_line = f"Error: {metrics.final_dist:.3f}m" if metrics.final_dist is not None else "Error: N/A"
        stats = (
            f"{status_str} | Run #{metrics.run_id}\n"
            f"----------------------\n"
            f"Duration: {duration:.1f}s\n"
            f"Path: {path_for_overlay:.2f}m\n"
            f"{error_line}"
        )
        return stats

# ============================================================================
# MAIN NODE CLASS
# ============================================================================
class Nav2GoalMetrics(Node):
    def __init__(self):
        super().__init__("nav2_goal_metrics")
        
        # --- Parameters ---
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("dock_pose_topic", "/dock_pose")
        self.declare_parameter("goal_source", "both")  # goal_pose | dock_pose | both
        self.declare_parameter("dock_pose_listen_only", True)
        self.declare_parameter("nav_action_name", "navigate_to_pose")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("dock_tf_frame", "dock_frame")
        self.declare_parameter("enable_dock_tf_finish", True)
        self.declare_parameter("dock_xy_tolerance", 0.20)
        self.declare_parameter("dock_yaw_tolerance", 0.20)
        self.declare_parameter("dock_hold_time_s", 1.0)
        self.declare_parameter("dock_timeout_s", 120.0)
        self.declare_parameter("enable_docking_status_finish", True)
        self.declare_parameter("docking_status_topic", "/docking/status_text")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("global_plan_topic", "/plan")
        self.declare_parameter("local_plan_topic", "/trajectories")
        self.declare_parameter("settle_window_s", 1.0)
        self.declare_parameter("tol_xy", 0.15)
        self.declare_parameter("tol_yaw", 0.15)
        # =========================================================================
        # NEW: VISUALIZATION PARAMETERS
        # =========================================================================
        self.declare_parameter("enable_visualization", True)
        self.declare_parameter("visualization_output_dir", "./trajectory_images")
        self.declare_parameter("trajectory_record_rate_hz", 10.0)
        self.declare_parameter("enable_csv_logging", True)
        self.declare_parameter("csv_summary_path", "./trajectory_images/nav2_metrics_summary.csv")
        self.declare_parameter("write_trajectory_csv", False)
        self.declare_parameter("csv_trajectory_path", "./trajectory_images/nav2_metrics_trajectory.csv")
        
        self.goal_topic = self.get_parameter("goal_topic").value
        self.dock_pose_topic = self.get_parameter("dock_pose_topic").value
        self.goal_source = self.get_parameter("goal_source").value
        self.dock_pose_listen_only = bool(self.get_parameter("dock_pose_listen_only").value)
        self.action_name = self.get_parameter("nav_action_name").value
        self.global_frame = self.get_parameter("global_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.dock_tf_frame = self.get_parameter("dock_tf_frame").value
        self.enable_dock_tf_finish = bool(self.get_parameter("enable_dock_tf_finish").value)
        self.dock_xy_tolerance = float(self.get_parameter("dock_xy_tolerance").value)
        self.dock_yaw_tolerance = float(self.get_parameter("dock_yaw_tolerance").value)
        self.dock_hold_time_s = float(self.get_parameter("dock_hold_time_s").value)
        self.dock_timeout_s = float(self.get_parameter("dock_timeout_s").value)
        self.enable_docking_status_finish = bool(self.get_parameter("enable_docking_status_finish").value)
        self.docking_status_topic = self.get_parameter("docking_status_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.global_plan_topic = self.get_parameter("global_plan_topic").value
        self.local_plan_topic = self.get_parameter("local_plan_topic").value

        if self.goal_source not in ("goal_pose", "dock_pose", "both"):
            self.get_logger().warn(
                f"Invalid goal_source '{self.goal_source}', fallback to 'goal_pose'"
            )
            self.goal_source = "goal_pose"
        
        # =========================================================================
        # NEW: VISUALIZATION SETUP
        # =========================================================================
        self.enable_viz = self.get_parameter("enable_visualization").value
        self.viz_output_dir = self.get_parameter("visualization_output_dir").value
        self.trajectory_rate = self.get_parameter("trajectory_record_rate_hz").value
        self.enable_csv_logging = bool(self.get_parameter("enable_csv_logging").value)
        self.csv_summary_path = self.get_parameter("csv_summary_path").value
        self.write_trajectory_csv = bool(self.get_parameter("write_trajectory_csv").value)
        self.csv_trajectory_path = self.get_parameter("csv_trajectory_path").value
        
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
        
        # Action client
        self._ac = ActionClient(self, NavigateToPose, self.action_name)
        
        # Subscriptions
        if self.goal_source in ("goal_pose", "both"):
            self.create_subscription(PoseStamped, self.goal_topic, self.on_goal_pose, 10)
        if self.goal_source in ("dock_pose", "both"):
            self.create_subscription(PoseStamped, self.dock_pose_topic, self.on_dock_pose, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)
        self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 50)
        
        # =========================================================================
        # FIX: QoS for /scan - BEST_EFFORT + VOLATILE
        # =========================================================================
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, scan_qos)
        
        self.create_subscription(Path, self.global_plan_topic, self.on_global_plan, 10)
        self.create_subscription(Path, self.local_plan_topic, self.on_local_plan, 10)

        if self.enable_docking_status_finish:
            status_qos = QoSProfile(depth=1)
            status_qos.reliability = QoSReliabilityPolicy.RELIABLE
            status_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(String, self.docking_status_topic, self.on_docking_status, status_qos)
        
        self.metrics = RunMetrics()
        self.latest_docking_phase = "IDLE"

        if self.enable_csv_logging:
            self._initialize_csv_outputs()
        
        # =========================================================================
        # NEW: TRAJECTORY RECORDING TIMER
        # =========================================================================
        if self.enable_viz and VISUALIZATION_AVAILABLE:
            self.trajectory_timer = self.create_timer(
                1.0 / self.trajectory_rate,
                self.record_trajectory_callback
            )

        self.dock_monitor_timer = self.create_timer(0.1, self.monitor_dock_completion)
        
        self.get_logger().info(
            f"Goal source: {self.goal_source} | goal_topic: {self.goal_topic} | dock_pose_topic: {self.dock_pose_topic}\n"
            f"dock_pose_listen_only={self.dock_pose_listen_only}  dock_tf_frame={self.dock_tf_frame}\n"
            f"enable_docking_status_finish={self.enable_docking_status_finish}  docking_status_topic={self.docking_status_topic}\n"
            f"csv_logging={self.enable_csv_logging}  summary_csv={self.csv_summary_path}\n"
            f"If forwarding enabled, sending to action '{self.action_name}'.\n"
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

    def lookup_frame_pose_map(self, frame_name: str) -> Optional[tuple]:
        """Return (x, y, yaw) of frame_name in global_frame using TF."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, frame_name, rclpy.time.Time()
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
            return (x, y, yaw)
        except Exception:
            return None

    def _compute_tf_trajectory_path_length(self) -> float:
        """Compute path length from recorded trajectory points."""
        points = self.metrics.trajectory_points
        if len(points) < 2:
            return 0.0

        total = 0.0
        for i in range(1, len(points)):
            dx = points[i]['x'] - points[i - 1]['x']
            dy = points[i]['y'] - points[i - 1]['y']
            total += hypot2(dx, dy)
        return total

    def _target_pose_for_error(self) -> Optional[tuple]:
        """Return target pose for final error: prefer dock TF, fallback to requested target pose."""
        dock_tf = self.lookup_frame_pose_map(self.dock_tf_frame)
        if dock_tf is not None:
            return dock_tf
        if self.metrics.dock_tf_x is not None and self.metrics.dock_tf_y is not None:
            yaw = self.metrics.dock_tf_yaw if self.metrics.dock_tf_yaw is not None else self.metrics.goal_yaw
            return (self.metrics.dock_tf_x, self.metrics.dock_tf_y, yaw)
        return (self.metrics.goal_x, self.metrics.goal_y, self.metrics.goal_yaw)

    def _update_active_dock_target(self, msg: PoseStamped) -> None:
        """Update active dock run target from the newest /dock_pose message."""
        q = msg.pose.orientation
        self.metrics.goal_x = msg.pose.position.x
        self.metrics.goal_y = msg.pose.position.y
        self.metrics.goal_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        # Keep straight-line denominator aligned with latest target estimate.
        if self.metrics.start_x is not None and self.metrics.start_y is not None:
            self.metrics.straight_line_dist = hypot2(
                self.metrics.goal_x - self.metrics.start_x,
                self.metrics.goal_y - self.metrics.start_y,
            )

        self.get_logger().info(
            f"Updated active dock target: x={self.metrics.goal_x:.3f} y={self.metrics.goal_y:.3f} "
            f"yaw={math.degrees(self.metrics.goal_yaw):.1f}deg",
            throttle_duration_sec=2.0,
        )

    def _fill_final_error_from_last_trajectory(self, target_pose: Optional[tuple]) -> None:
        """Fallback final error calculation using last recorded trajectory sample."""
        if self.metrics.final_dist is not None:
            return
        if target_pose is None:
            return
        if len(self.metrics.trajectory_points) == 0:
            return

        last = self.metrics.trajectory_points[-1]
        lx = last['x']
        ly = last['y']
        lyaw = float(last.get('yaw', 0.0))
        tx, ty, tyaw = target_pose

        dx = tx - lx
        dy = ty - ly
        self.metrics.final_dx = dx
        self.metrics.final_dy = dy
        self.metrics.final_dist = hypot2(dx, dy)
        self.metrics.final_dyaw = angle_wrap(tyaw - lyaw)
        self.metrics.along_track = math.cos(tyaw) * dx + math.sin(tyaw) * dy
        self.metrics.cross_track = -math.sin(tyaw) * dx + math.cos(tyaw) * dy

    def _initialize_csv_outputs(self) -> None:
        """Create CSV output files with headers if they do not exist."""
        summary_dir = os.path.dirname(self.csv_summary_path)
        if summary_dir:
            os.makedirs(summary_dir, exist_ok=True)

        if (not os.path.exists(self.csv_summary_path)) or os.path.getsize(self.csv_summary_path) == 0:
            with open(self.csv_summary_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow([
                    "timestamp",
                    "run_id",
                    "status",
                    "source_topic",
                    "duration_s",
                    "path_len_odom_m",
                    "path_len_tf_m",
                    "straight_line_dist_m",
                    "straight_line_ratio",
                    "path_source",
                    "goal_x",
                    "goal_y",
                    "goal_yaw_deg",
                    "final_dx_m",
                    "final_dy_m",
                    "final_dist_m",
                    "final_dyaw_deg",
                    "along_track_m",
                    "cross_track_m",
                    "max_cmd_v_mps",
                    "max_cmd_w_rps",
                    "max_act_v_mps",
                    "max_act_w_rps",
                    "min_scan_m",
                    "global_plan_updates",
                    "local_plan_updates",
                    "stop_go_count",
                    "velocity_sign_flips",
                    "unsafe_clearance_time_s",
                    "image_path",
                ])

        if self.write_trajectory_csv:
            traj_dir = os.path.dirname(self.csv_trajectory_path)
            if traj_dir:
                os.makedirs(traj_dir, exist_ok=True)
            if (not os.path.exists(self.csv_trajectory_path)) or os.path.getsize(self.csv_trajectory_path) == 0:
                with open(self.csv_trajectory_path, "w", newline="", encoding="utf-8") as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        "run_id",
                        "source_topic",
                        "point_index",
                        "timestamp_s",
                        "x_m",
                        "y_m",
                        "yaw_rad",
                        "controller",
                    ])

    def _write_csv_outputs(self, status_str: str, dt: float, ratio: Optional[float], ratio_src: str) -> None:
        """Append run summary and trajectory samples to CSV outputs."""
        if not self.enable_csv_logging:
            return

        goal_yaw_deg = math.degrees(self.metrics.goal_yaw)
        final_dyaw_deg = math.degrees(self.metrics.final_dyaw) if self.metrics.final_dyaw is not None else ""

        with open(self.csv_summary_path, "a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                datetime.now().isoformat(timespec="seconds"),
                self.metrics.run_id,
                status_str,
                self.metrics.source_topic,
                f"{dt:.3f}",
                f"{self.metrics.path_len_m:.3f}",
                f"{self.metrics.path_len_tf_m:.3f}",
                "" if self.metrics.straight_line_dist is None else f"{self.metrics.straight_line_dist:.3f}",
                "" if ratio is None else f"{ratio:.3f}",
                ratio_src,
                f"{self.metrics.goal_x:.3f}",
                f"{self.metrics.goal_y:.3f}",
                f"{goal_yaw_deg:.2f}",
                "" if self.metrics.final_dx is None else f"{self.metrics.final_dx:.3f}",
                "" if self.metrics.final_dy is None else f"{self.metrics.final_dy:.3f}",
                "" if self.metrics.final_dist is None else f"{self.metrics.final_dist:.3f}",
                final_dyaw_deg,
                "" if self.metrics.along_track is None else f"{self.metrics.along_track:.3f}",
                "" if self.metrics.cross_track is None else f"{self.metrics.cross_track:.3f}",
                f"{self.metrics.max_cmd_v:.3f}",
                f"{self.metrics.max_cmd_w:.3f}",
                f"{self.metrics.max_act_v:.3f}",
                f"{self.metrics.max_act_w:.3f}",
                "" if not math.isfinite(self.metrics.min_scan) else f"{self.metrics.min_scan:.3f}",
                self.metrics.global_plan_updates,
                self.metrics.local_plan_updates,
                self.metrics.stop_go_count,
                self.metrics.sign_flip_count,
                f"{self.metrics.unsafe_clearance_time:.3f}",
                self.metrics.image_path or "",
            ])

        if self.write_trajectory_csv and len(self.metrics.trajectory_points) > 0:
            with open(self.csv_trajectory_path, "a", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                for idx, p in enumerate(self.metrics.trajectory_points):
                    writer.writerow([
                        self.metrics.run_id,
                        self.metrics.source_topic,
                        idx,
                        f"{float(p.get('timestamp', 0.0)):.3f}",
                        f"{float(p.get('x', 0.0)):.3f}",
                        f"{float(p.get('y', 0.0)):.3f}",
                        f"{float(p.get('yaw', 0.0)):.6f}",
                        p.get('controller', 'unknown'),
                    ])

    def start_run(self, msg: PoseStamped, source_topic: str) -> bool:
        """Initialize a new run from goal_pose or dock_pose input."""
        if self.metrics.active:
            if source_topic == "dock_pose" and self.metrics.source_topic == "dock_pose":
                self._update_active_dock_target(msg)
                return False
            self.get_logger().warn(
                f"Already navigating; ignoring new {source_topic}.",
                throttle_duration_sec=10.0,
            )
            return False

        if msg.header.frame_id and msg.header.frame_id != self.global_frame:
            self.get_logger().warn(
                f"Pose frame '{msg.header.frame_id}' != global_frame '{self.global_frame}'. "
                "For simplicity, this script assumes pose is already in global_frame."
            )

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        q = msg.pose.orientation
        goal_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        pose = self.lookup_robot_pose_map()
        straight_line = None
        if pose is not None:
            rx, ry, _ = pose
            straight_line = math.sqrt(
                (goal_x - rx) ** 2 +
                (goal_y - ry) ** 2
            )

        dock_pose = self.lookup_frame_pose_map(self.dock_tf_frame)

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
            source_topic=source_topic,
            dock_tf_x=(dock_pose[0] if dock_pose is not None else None),
            dock_tf_y=(dock_pose[1] if dock_pose is not None else None),
            dock_tf_yaw=(dock_pose[2] if dock_pose is not None else None),
        )

        if pose is not None:
            self.metrics.start_x, self.metrics.start_y, self.metrics.start_yaw = pose

        self.get_logger().info(
            f"\n=== RUN {self.metrics.run_id} START ({source_topic}) ===\n"
            f"Target: x={goal_x:.3f} y={goal_y:.3f} yaw={math.degrees(goal_yaw):.1f}deg  frame={msg.header.frame_id or self.global_frame}"
        )
        return True

    def on_dock_pose(self, msg: PoseStamped):
        if not self.start_run(msg, source_topic="dock_pose"):
            return
        if self.dock_pose_listen_only:
            self.get_logger().info(
                "Dock pose run active in listen-only mode; Nav2 command is handled by dock_pose_bridge.py"
            )
        else:
            self.get_logger().info(
                "Dock pose run active with forwarding mode disabled in this script implementation"
            )
    
    # =========================================================================
    # NEW: TRAJECTORY RECORDING CALLBACK
    # =========================================================================
    def record_trajectory_callback(self):
        """Periodically record robot pose for trajectory visualization"""
        if not self.metrics.active or not self.visualizer:
            return
        
        pose = self.lookup_robot_pose_map()
        if pose is not None:
            x, y, yaw = pose
            controller = self._current_controller_source()
            self.metrics.trajectory_points.append({
                'x': x,
                'y': y,
                'yaw': yaw,
                'controller': controller,
                'timestamp': self.now().nanoseconds * 1e-9
            })
            
            # Record start position
            if self.metrics.start_x is None:
                self.metrics.start_x = x
                self.metrics.start_y = y
                self.metrics.start_yaw = yaw
    
    def on_goal_pose(self, msg: PoseStamped):
        if not self.start_run(msg, source_topic="goal_pose"):
            return

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
        
        # Get current robot pose
        pose = self.lookup_robot_pose_map()
        if pose is None:
            return
        
        rx, ry, ryaw = pose
        
        # 1️⃣ Compute minimum valid scan distance
        m = float("inf")
        rmin = msg.range_min
        rmax = msg.range_max
        valid_ranges = []
        valid_angles = []
        
        for i, r in enumerate(msg.ranges):
            if math.isfinite(r) and rmin <= r <= rmax:
                if r < m:
                    m = r
                # Store valid scan points (limit to reduce memory)
                if len(valid_ranges) < 200:
                    angle = msg.angle_min + i * msg.angle_increment
                    valid_ranges.append(r)
                    valid_angles.append(angle)
        
        # 2️⃣ Convert scan points to world coordinates
        obstacle_points = {'x': [], 'y': []}
        for r, angle in zip(valid_ranges, valid_angles):
            # Transform from laser frame to world frame
            world_angle = ryaw + angle
            ox = rx + r * math.cos(world_angle)
            oy = ry + r * math.sin(world_angle)
            obstacle_points['x'].append(ox)
            obstacle_points['y'].append(oy)
        
        # 3️⃣ Store obstacle points (limit total storage)
        if len(self.metrics.scan_obstacles) < 30:
            self.metrics.scan_obstacles.append({
                'robot_x': rx,
                'robot_y': ry,
                'robot_yaw': ryaw,
                'obstacles': obstacle_points,
                'timestamp': now.nanoseconds * 1e-9
            })
        
        # 4️⃣ Accumulate unsafe clearance time
        if self.metrics.last_scan_time is not None:
            dt = (now - self.metrics.last_scan_time).nanoseconds * 1e-9
            if m < self.metrics.clearance_threshold:
                self.metrics.unsafe_clearance_time += dt
        self.metrics.last_scan_time = now
        
        # 5️⃣ Track global minimum scan distance
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

        dock_pose = self.lookup_frame_pose_map(self.dock_tf_frame)
        if dock_pose is not None:
            self.metrics.dock_tf_x = dock_pose[0]
            self.metrics.dock_tf_y = dock_pose[1]
            self.metrics.dock_tf_yaw = dock_pose[2]
        
        # Final robot pose via TF (best)
        pose = self.lookup_robot_pose_map()
        if pose is not None:
            rx, ry, ryaw = pose
            target_x = self.metrics.goal_x
            target_y = self.metrics.goal_y
            target_yaw = self.metrics.goal_yaw
            if dock_pose is not None:
                target_x, target_y, target_yaw = dock_pose

            dx = target_x - rx
            dy = target_y - ry
            
            # Transform error into dock frame
            dock_yaw = target_yaw
            along_track =  math.cos(dock_yaw) * dx + math.sin(dock_yaw) * dy
            cross_track = -math.sin(dock_yaw) * dx + math.cos(dock_yaw) * dy
            dist = math.sqrt(dx*dx + dy*dy)
            dyaw = angle_wrap(target_yaw - ryaw)
            
            self.metrics.final_dx = dx
            self.metrics.final_dy = dy
            self.metrics.final_dist = dist
            self.metrics.final_dyaw = dyaw
            self.metrics.along_track = along_track
            self.metrics.cross_track = cross_track
        
        self.metrics.active = False
        self.finalize_run_output()

    def monitor_dock_completion(self):
        """In listen-only dock mode, finish run based on tf distance/yaw thresholds."""
        if not self.metrics.active:
            return
        if self.metrics.source_topic != "dock_pose":
            return
        if not self.dock_pose_listen_only:
            return
        if not self.enable_dock_tf_finish:
            return

        robot_pose = self.lookup_robot_pose_map()
        dock_pose = self.lookup_frame_pose_map(self.dock_tf_frame)
        if robot_pose is None or dock_pose is None:
            return

        rx, ry, ryaw = robot_pose
        dx = dock_pose[0] - rx
        dy = dock_pose[1] - ry
        dist = hypot2(dx, dy)
        dyaw = abs(angle_wrap(dock_pose[2] - ryaw))

        now = self.now()
        within = (dist <= self.dock_xy_tolerance) and (dyaw <= self.dock_yaw_tolerance)
        if within:
            if self.metrics.within_tol_since is None:
                self.metrics.within_tol_since = now
            else:
                hold = (now - self.metrics.within_tol_since).nanoseconds * 1e-9
                if hold >= self.dock_hold_time_s:
                    self.metrics.t_end = now
                    self.metrics.result_status = GoalStatus.STATUS_SUCCEEDED
                    self.metrics.goal_x = dock_pose[0]
                    self.metrics.goal_y = dock_pose[1]
                    self.metrics.goal_yaw = dock_pose[2]
                    self.metrics.dock_tf_x = dock_pose[0]
                    self.metrics.dock_tf_y = dock_pose[1]
                    self.metrics.dock_tf_yaw = dock_pose[2]
                    self.metrics.final_dx = dx
                    self.metrics.final_dy = dy
                    self.metrics.final_dist = dist
                    self.metrics.final_dyaw = angle_wrap(dock_pose[2] - ryaw)
                    self.metrics.along_track = math.cos(dock_pose[2]) * dx + math.sin(dock_pose[2]) * dy
                    self.metrics.cross_track = -math.sin(dock_pose[2]) * dx + math.cos(dock_pose[2]) * dy
                    self.metrics.active = False
                    self.get_logger().info(
                        f"Dock listen-only completion: dist={dist:.3f}m yaw_err={math.degrees(dyaw):.2f}deg"
                    )
                    self.finalize_run_output()
                    return
        else:
            self.metrics.within_tol_since = None

        if self.metrics.t_start is not None and self.dock_timeout_s > 0.0:
            elapsed = (now - self.metrics.t_start).nanoseconds * 1e-9
            if elapsed >= self.dock_timeout_s:
                self.metrics.t_end = now
                self.metrics.result_status = GoalStatus.STATUS_ABORTED
                self.metrics.goal_x = dock_pose[0]
                self.metrics.goal_y = dock_pose[1]
                self.metrics.goal_yaw = dock_pose[2]
                self.metrics.dock_tf_x = dock_pose[0]
                self.metrics.dock_tf_y = dock_pose[1]
                self.metrics.dock_tf_yaw = dock_pose[2]
                self.metrics.final_dx = dx
                self.metrics.final_dy = dy
                self.metrics.final_dist = dist
                self.metrics.final_dyaw = angle_wrap(dock_pose[2] - ryaw)
                self.metrics.along_track = math.cos(dock_pose[2]) * dx + math.sin(dock_pose[2]) * dy
                self.metrics.cross_track = -math.sin(dock_pose[2]) * dx + math.cos(dock_pose[2]) * dy
                self.metrics.active = False
                self.get_logger().warn(
                    f"Dock listen-only timeout after {elapsed:.1f}s: dist={dist:.3f}m yaw_err={math.degrees(dyaw):.2f}deg"
                )
                self.finalize_run_output()

    def on_docking_status(self, msg: String):
        """Finish listen-only dock runs from dock_pose_bridge status topic."""
        self._update_docking_phase_from_status(msg.data)

        if not self.metrics.active:
            return
        if self.metrics.source_topic != "dock_pose":
            return
        if not self.dock_pose_listen_only:
            return

        status_text = (msg.data or "").upper()
        if "DOCK: SUCCESS" in status_text:
            self._finalize_dock_from_status(GoalStatus.STATUS_SUCCEEDED, "Dock status reports success")
            return

        if (
            ("DOCK: FAILED" in status_text)
            or ("DOCK: REJECTED" in status_text)
            or ("DOCK: SERVER MISSING" in status_text)
            or ("DOCK: CANCEL FAILED" in status_text)
        ):
            self._finalize_dock_from_status(GoalStatus.STATUS_ABORTED, f"Dock status reports failure: {msg.data}")

    def _update_docking_phase_from_status(self, status_text: str):
        """Extract DOCK phase token from status text published by dock_pose_bridge."""
        text = (status_text or "").upper()
        if not text:
            return
        first_line = text.splitlines()[0].strip()
        if first_line.startswith("DOCK:"):
            self.latest_docking_phase = first_line.replace("DOCK:", "", 1).strip()
        elif first_line.startswith("DOCKING:"):
            self.latest_docking_phase = first_line.replace("DOCKING:", "", 1).strip()

    def _current_controller_source(self) -> str:
        """Classify current controller for trajectory coloring."""
        if self.metrics.source_topic != "dock_pose":
            return "mppi"

        phase = (self.latest_docking_phase or "").upper()
        if phase in ("NAV_TO_STAGING", "REQUESTED", "ACCEPTED"):
            return "mppi"
        if phase in ("INITIAL_PERCEPTION", "CONTROLLING", "WAIT_FOR_CHARGE", "RETRY"):
            return "docking_server"
        return "unknown"

    def _finalize_dock_from_status(self, result_status: int, reason: str):
        """Finalize active dock_pose run from external docking status signal."""
        if not self.metrics.active:
            return

        now = self.now()
        self.metrics.t_end = now
        self.metrics.result_status = int(result_status)

        robot_pose = self.lookup_robot_pose_map()
        dock_pose = self.lookup_frame_pose_map(self.dock_tf_frame)

        if dock_pose is not None:
            self.metrics.goal_x = dock_pose[0]
            self.metrics.goal_y = dock_pose[1]
            self.metrics.goal_yaw = dock_pose[2]
            self.metrics.dock_tf_x = dock_pose[0]
            self.metrics.dock_tf_y = dock_pose[1]
            self.metrics.dock_tf_yaw = dock_pose[2]

        if robot_pose is not None and dock_pose is not None:
            rx, ry, ryaw = robot_pose
            dx = dock_pose[0] - rx
            dy = dock_pose[1] - ry
            self.metrics.final_dx = dx
            self.metrics.final_dy = dy
            self.metrics.final_dist = hypot2(dx, dy)
            self.metrics.final_dyaw = angle_wrap(dock_pose[2] - ryaw)
            self.metrics.along_track = math.cos(dock_pose[2]) * dx + math.sin(dock_pose[2]) * dy
            self.metrics.cross_track = -math.sin(dock_pose[2]) * dx + math.cos(dock_pose[2]) * dy

        self.metrics.active = False
        self.get_logger().info(reason)
        self.finalize_run_output()

    def finalize_run_output(self):
        """Generate image and print summary for action result and listen-only completion."""
        self.metrics.path_len_tf_m = self._compute_tf_trajectory_path_length()
        target_pose = self._target_pose_for_error()
        self._fill_final_error_from_last_trajectory(target_pose)

        if self.visualizer and len(self.metrics.trajectory_points) > 0:
            status_str = {
                GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                GoalStatus.STATUS_ABORTED: "ABORTED",
                GoalStatus.STATUS_CANCELED: "CANCELED",
            }.get(self.metrics.result_status, "UNKNOWN")

            title = f"Nav2 Run {self.metrics.run_id}: {status_str} ({self.metrics.source_topic})"
            image_path = self.visualizer.generate_static_image(self.metrics, title=title)

            if image_path:
                self.get_logger().info(f"Trajectory image saved: {image_path}")
            else:
                self.get_logger().warn("Failed to generate trajectory image")

        dt = (self.metrics.t_end - self.metrics.t_start).nanoseconds * 1e-9 if self.metrics.t_start else float("nan")
        status_str = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        }.get(self.metrics.result_status, f"STATUS_{self.metrics.result_status}")

        ratio = None
        ratio_src = ""
        if self.metrics.straight_line_dist is not None and self.metrics.straight_line_dist > 0:
            effective_path = self.metrics.path_len_m if self.metrics.path_len_m > 0.0 else self.metrics.path_len_tf_m
            ratio = effective_path / self.metrics.straight_line_dist
            ratio_src = "odom" if self.metrics.path_len_m > 0.0 else "tf"

        self._write_csv_outputs(status_str, dt, ratio, ratio_src)

        print("\n" + "=" * 60)
        print(f"RUN {self.metrics.run_id} RESULT: {status_str}")
        print(f"Source topic        : {self.metrics.source_topic}")
        print(f"Duration            : {dt:.3f} s")
        print(f"Path length (odom)  : {self.metrics.path_len_m:.3f} m")
        print(f"Path length (tf)    : {self.metrics.path_len_tf_m:.3f} m")
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

        if ratio is not None:
            print(f"Straight-line ratio  : {ratio:.3f}  (path source: {ratio_src})")

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

        if self.metrics.final_dist is not None and self.metrics.final_dyaw is not None:
            label = "Dock TF closeness" if (self.metrics.dock_tf_x is not None and self.metrics.dock_tf_y is not None) else "Target closeness"
            print(f"{label:<20}: dist={self.metrics.final_dist:.3f} m  yaw={math.degrees(self.metrics.final_dyaw):.2f} deg")

        if self.metrics.along_track is not None:
            print("Dock frame error:")
            print(f"  Along-track error  : {self.metrics.along_track:.3f} m")
            print(f"  Cross-track error  : {self.metrics.cross_track:.3f} m")

        print("=" * 60 + "\n")

# ============================================================================
# MAIN FUNCTION
# ============================================================================
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