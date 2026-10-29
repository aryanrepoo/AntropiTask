#!/usr/bin/env python3
"""
Antropi Robotics — Factory Telemetry HUD & Black-Box Incident Recorder

A passive observer node that subscribes to /odom, /scan, and /cmd_vel to
compute real-time safety metrics (minimum obstacle distance, estimated
Time-to-Collision, safety tier classification) and automatically logs
structured JSON incident reports when proximity breaches are detected.

This node does NOT publish or interfere with Nav2 — it is a pure read-only
diagnostics layer, mirroring how production Factory OS architectures
implement fleet observability and automated anomaly detection.

Usage:
    ros2 run antropi_amr_sim factory_telemetry_hud
"""

import json
import math
import os
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# =============================================================================
# Safety Configuration — Thresholds for factory floor operations
# =============================================================================
PROXIMITY_NOMINAL_M = 0.60      # > 0.60m  → NOMINAL (green)
PROXIMITY_CAUTION_M = 0.35      # 0.35–0.60m → CAUTION (yellow)
                                # < 0.35m  → BREACH (red)
TTC_WARNING_SEC = 3.0           # TTC below this triggers caution
TTC_CRITICAL_SEC = 1.5          # TTC below this triggers alert
MIN_VELOCITY_THRESHOLD = 0.05   # m/s — below this, TTC is undefined (robot ~stationary)

# Incident cooldown: avoid logging duplicate incidents within this window
INCIDENT_COOLDOWN_SEC = 3.0

# HUD refresh rate
HUD_REFRESH_HZ = 4.0

# =============================================================================
# ANSI Terminal Escape Codes (zero external dependencies)
# =============================================================================
RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"
RED = "\033[91m"
YELLOW = "\033[93m"
GREEN = "\033[92m"
CYAN = "\033[96m"
WHITE = "\033[97m"
BG_RED = "\033[41m"
BG_YELLOW = "\033[43m"
BG_GREEN = "\033[42m"
CLEAR_SCREEN = "\033[2J"
CURSOR_HOME = "\033[H"
HIDE_CURSOR = "\033[?25l"
SHOW_CURSOR = "\033[?25h"


# =============================================================================
# Factory Zone Classification (based on proximity to known station coords)
# =============================================================================
FACTORY_ZONES = [
    {"name": "Raw_Material_Dock",    "x": -2.0, "y": -0.5},
    {"name": "CNC_Mill_Cell_01",     "x":  0.5, "y":  1.8},
    {"name": "CNC_Lathe_Cell_02",    "x":  1.8, "y": -0.5},
    {"name": "CMM_Quality_Control",  "x":  0.4, "y": -1.8},
]


def classify_zone(x: float, y: float) -> str:
    """Return the name of the nearest factory zone based on Euclidean distance."""
    min_dist = float('inf')
    nearest = "Transit_Aisle"
    for zone in FACTORY_ZONES:
        dist = math.hypot(x - zone["x"], y - zone["y"])
        if dist < min_dist:
            min_dist = dist
            nearest = zone["name"]
    # If robot is far from all stations, it's in a transit aisle
    if min_dist > 1.0:
        return f"Transit_Aisle (near {nearest})"
    return nearest


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw (radians) from a quaternion using the atan2 formula."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class FactoryTelemetryHud(Node):
    """
    ROS 2 Node: Real-time AMR telemetry dashboard and black-box incident logger.

    Subscriptions:
        /odom  (nav_msgs/Odometry)   — Robot pose and odometry velocity
        /scan  (sensor_msgs/LaserScan) — 360° LiDAR range data
        /cmd_vel (Twist or TwistStamped) — Commanded velocities from Nav2

    Outputs:
        - Live ANSI terminal HUD (refreshed at HUD_REFRESH_HZ)
        - Structured JSONL incident log (written on proximity breaches)
    """

    def __init__(self):
        super().__init__('factory_telemetry_hud')

        # ------- State Variables -------
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw_deg = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.cmd_linear_vel = 0.0
        self.cmd_angular_vel = 0.0
        self.min_laser_range = float('inf')
        self.min_laser_angle_deg = 0.0
        self.ttc_sec = float('inf')
        self.safety_tier = "NOMINAL"
        self.scan_count = 0
        self.odom_count = 0
        self.incident_count = 0
        self.last_incident_time = 0.0
        self.start_time = time.monotonic()

        # ------- Incident Log Setup -------
        self.incident_dir = Path.home() / "antropi_ws" / "incidents"
        # Fall back to /tmp if home dir write fails
        try:
            self.incident_dir.mkdir(parents=True, exist_ok=True)
        except OSError:
            self.incident_dir = Path("/tmp") / "antropi_incidents"
            self.incident_dir.mkdir(parents=True, exist_ok=True)

        self.incident_log_path = self.incident_dir / "incident_log.jsonl"
        self.get_logger().info(
            f"Incident log path: {self.incident_log_path}"
        )

        # ------- QoS Profile for sensor topics -------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # ------- Subscriptions -------
        self.create_subscription(
            Odometry, '/odom', self._odom_callback, sensor_qos
        )
        self.create_subscription(
            LaserScan, '/scan', self._scan_callback, sensor_qos
        )
        # Try both Twist and TwistStamped — ROS 2 Jazzy transition
        self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, sensor_qos
        )

        # ------- HUD Refresh Timer -------
        refresh_period = 1.0 / HUD_REFRESH_HZ
        self.create_timer(refresh_period, self._refresh_hud)

        # Hide cursor for cleaner HUD rendering
        print(HIDE_CURSOR, end='', flush=True)

    def destroy_node(self):
        """Restore terminal state on shutdown."""
        print(SHOW_CURSOR, end='', flush=True)
        super().destroy_node()

    # =====================================================================
    # Subscriber Callbacks
    # =====================================================================

    def _odom_callback(self, msg: Odometry):
        """Extract robot pose and velocity from odometry."""
        self.odom_count += 1
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        yaw_rad = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.robot_yaw_deg = math.degrees(yaw_rad)

        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z

    def _scan_callback(self, msg: LaserScan):
        """Process 360° LiDAR scan to find minimum obstacle distance."""
        self.scan_count += 1

        min_range = float('inf')
        min_angle_rad = 0.0

        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max and r < min_range:
                min_range = r
                min_angle_rad = msg.angle_min + i * msg.angle_increment

        self.min_laser_range = min_range
        self.min_laser_angle_deg = math.degrees(min_angle_rad)

        # --- Compute Time-to-Collision ---
        # Project velocity onto the direction of minimum distance
        forward_vel = abs(self.linear_vel)
        if forward_vel > MIN_VELOCITY_THRESHOLD and min_range < 5.0:
            # Simple TTC: how long until we hit the closest obstacle
            # if we maintain current forward velocity
            self.ttc_sec = min_range / forward_vel
        else:
            self.ttc_sec = float('inf')

        # --- Classify Safety Tier ---
        prev_tier = self.safety_tier
        if min_range <= PROXIMITY_CAUTION_M:
            self.safety_tier = "PROXIMITY_BREACH"
        elif min_range <= PROXIMITY_NOMINAL_M:
            self.safety_tier = "CAUTION"
        else:
            self.safety_tier = "NOMINAL"

        # --- Incident Detection & Logging ---
        if self.safety_tier == "PROXIMITY_BREACH":
            now = time.monotonic()
            if (now - self.last_incident_time) > INCIDENT_COOLDOWN_SEC:
                self.last_incident_time = now
                self._log_incident(min_range)

    def _cmd_vel_callback(self, msg: Twist):
        """Capture commanded velocities from Nav2 controller."""
        self.cmd_linear_vel = msg.linear.x
        self.cmd_angular_vel = msg.angular.z

    # =====================================================================
    # Incident Logger (Black-Box Forensics)
    # =====================================================================

    def _log_incident(self, distance: float):
        """Write a structured JSON incident record to the JSONL log file."""
        self.incident_count += 1

        incident = {
            "incident_id": self.incident_count,
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "timestamp_monotonic": round(time.monotonic() - self.start_time, 3),
            "incident_type": "PROXIMITY_BREACH",
            "severity": "CRITICAL" if distance < 0.20 else "WARNING",
            "amr_state": {
                "position": {
                    "x": round(self.robot_x, 4),
                    "y": round(self.robot_y, 4),
                    "yaw_deg": round(self.robot_yaw_deg, 1)
                },
                "velocity": {
                    "linear_mps": round(self.linear_vel, 4),
                    "angular_radps": round(self.angular_vel, 4)
                },
                "cmd_velocity": {
                    "linear_mps": round(self.cmd_linear_vel, 4),
                    "angular_radps": round(self.cmd_angular_vel, 4)
                }
            },
            "obstacle_proximity": {
                "min_distance_m": round(distance, 4),
                "bearing_deg": round(self.min_laser_angle_deg, 1),
                "ttc_sec": round(self.ttc_sec, 2) if self.ttc_sec < 999 else None
            },
            "zone_classification": classify_zone(self.robot_x, self.robot_y),
            "safety_thresholds": {
                "breach_threshold_m": PROXIMITY_CAUTION_M,
                "caution_threshold_m": PROXIMITY_NOMINAL_M
            }
        }

        try:
            with open(self.incident_log_path, 'a') as f:
                f.write(json.dumps(incident) + '\n')
        except OSError as e:
            self.get_logger().error(f"Failed to write incident log: {e}")

    # =====================================================================
    # Terminal HUD Renderer
    # =====================================================================

    def _refresh_hud(self):
        """Redraw the full-screen ANSI terminal dashboard."""
        # Choose colors based on safety tier
        if self.safety_tier == "PROXIMITY_BREACH":
            tier_color = RED
            tier_bg = BG_RED
            tier_icon = "⚠"
            tier_label = "PROXIMITY BREACH — DYNAMIC REPLANNING"
        elif self.safety_tier == "CAUTION":
            tier_color = YELLOW
            tier_bg = BG_YELLOW
            tier_icon = "△"
            tier_label = "CAUTION — OBSTACLE APPROACHING"
        else:
            tier_color = GREEN
            tier_bg = BG_GREEN
            tier_icon = "●"
            tier_label = "NOMINAL — ALL CLEAR"

        # Format TTC
        if self.ttc_sec == float('inf') or self.ttc_sec > 99:
            ttc_str = "  ∞  "
            ttc_color = GREEN
        elif self.ttc_sec > TTC_WARNING_SEC:
            ttc_str = f"{self.ttc_sec:5.1f}s"
            ttc_color = GREEN
        elif self.ttc_sec > TTC_CRITICAL_SEC:
            ttc_str = f"{self.ttc_sec:5.1f}s"
            ttc_color = YELLOW
        else:
            ttc_str = f"{self.ttc_sec:5.1f}s"
            ttc_color = RED

        # Format min distance
        if self.min_laser_range == float('inf') or self.min_laser_range > 10:
            dist_str = " >10m "
            dist_color = GREEN
        else:
            dist_str = f"{self.min_laser_range:5.3f}m"
            if self.min_laser_range <= PROXIMITY_CAUTION_M:
                dist_color = RED
            elif self.min_laser_range <= PROXIMITY_NOMINAL_M:
                dist_color = YELLOW
            else:
                dist_color = GREEN

        zone = classify_zone(self.robot_x, self.robot_y)
        uptime = time.monotonic() - self.start_time
        uptime_str = f"{int(uptime // 60):02d}:{int(uptime % 60):02d}"

        # Build HUD frame
        W = 74  # width
        border = f"{CYAN}{'─' * W}{RESET}"
        hdr_border = f"{CYAN}{'═' * W}{RESET}"

        lines = [
            CURSOR_HOME + CLEAR_SCREEN,
            "",
            hdr_border,
            f"{CYAN}║{RESET}  {BOLD}{WHITE}ANTROPI FACTORY OS — AMR FLEET TELEMETRY & DIAGNOSTICS HUD{RESET}       {CYAN}║{RESET}",
            hdr_border,
            "",
            f"  {BOLD}SAFETY STATUS:{RESET}  {tier_bg}{BOLD}{WHITE} {tier_icon} {tier_label} {RESET}",
            "",
            border,
            f"  {BOLD}{WHITE}AMR POSE{RESET}",
            f"    Position:       x = {BOLD}{self.robot_x:+7.3f}{RESET} m   "
            f"y = {BOLD}{self.robot_y:+7.3f}{RESET} m   "
            f"yaw = {BOLD}{self.robot_yaw_deg:+6.1f}{RESET}°",
            f"    Zone:           {CYAN}{zone}{RESET}",
            "",
            border,
            f"  {BOLD}{WHITE}VELOCITY{RESET}",
            f"    Odometry:       linear = {BOLD}{self.linear_vel:+6.3f}{RESET} m/s   "
            f"angular = {BOLD}{self.angular_vel:+6.3f}{RESET} rad/s",
            f"    Commanded:      linear = {BOLD}{self.cmd_linear_vel:+6.3f}{RESET} m/s   "
            f"angular = {BOLD}{self.cmd_angular_vel:+6.3f}{RESET} rad/s",
            "",
            border,
            f"  {BOLD}{WHITE}PROXIMITY & SAFETY METRICS{RESET}",
            f"    Min LiDAR Dist: {dist_color}{BOLD}{dist_str}{RESET}  "
            f"@ bearing {self.min_laser_angle_deg:+6.1f}°",
            f"    Time-to-Collision (TTC): {ttc_color}{BOLD}{ttc_str}{RESET}",
            f"    Safety Tier:    {tier_color}{BOLD}{self.safety_tier}{RESET}",
            "",
            border,
            f"  {BOLD}{WHITE}BLACK-BOX INCIDENT RECORDER{RESET}",
            f"    Incidents Logged:  {BOLD}{self.incident_count}{RESET}",
            f"    Log File:          {DIM}{self.incident_log_path}{RESET}",
            "",
            border,
            f"  {DIM}Uptime: {uptime_str}  │  "
            f"Scan msgs: {self.scan_count}  │  "
            f"Odom msgs: {self.odom_count}  │  "
            f"Refresh: {HUD_REFRESH_HZ:.0f} Hz{RESET}",
            hdr_border,
        ]

        print('\n'.join(lines), flush=True)


def main():
    rclpy.init()
    node = FactoryTelemetryHud()

    print(f"\n{GREEN}[Factory Telemetry] Starting AMR diagnostics HUD...{RESET}")
    print(f"{DIM}[Factory Telemetry] Incident log: {node.incident_log_path}{RESET}\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(SHOW_CURSOR)  # Restore cursor
        print(f"\n{YELLOW}[Factory Telemetry] HUD shut down. "
              f"{node.incident_count} incident(s) logged to:{RESET}")
        print(f"  {node.incident_log_path}\n")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
