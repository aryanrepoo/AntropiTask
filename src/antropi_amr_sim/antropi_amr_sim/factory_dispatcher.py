#!/usr/bin/env python3
"""
Antropi Robotics — Autonomous CNC Factory Dispatcher
Demonstrates programmatic Nav2 orchestration using nav2_simple_commander.
Cycles a TurtleBot3 AMR through simulated CNC shop-floor stations while
monitoring navigation telemetry and obstacle recovery events.
"""

import math
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# ---------------------------------------------------------------------------
# CNC Shop-Floor Station Definitions (turtlebot3_world collision-free coords)
# ---------------------------------------------------------------------------
FACTORY_STATIONS = [
    {"name": "CNC_Mill_Cell_01",    "x":  0.5, "y":  1.2, "yaw":   0.0, "dwell_sec": 3.0},
    {"name": "CNC_Lathe_Cell_02",   "x":  1.2, "y": -0.5, "yaw": -90.0, "dwell_sec": 3.0},
    {"name": "CMM_Quality_Control", "x":  0.4, "y": -1.2, "yaw": 180.0, "dwell_sec": 3.0},
    {"name": "Raw_Material_Dock",   "x": -1.8, "y": -0.5, "yaw":   0.0, "dwell_sec": 2.0},
]

INITIAL_POSE = {"x": -2.0, "y": -0.5, "yaw_deg": 0.0}
STATION_NAV_TIMEOUT = 120.0  # seconds — abort if station unreachable


def create_pose(navigator: BasicNavigator, x: float, y: float, yaw_deg: float) -> PoseStamped:
    """Construct a PoseStamped in the 'map' frame from (x, y, yaw_degrees)."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    yaw_rad = math.radians(yaw_deg)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return pose


def run_factory_cycle(navigator: BasicNavigator):
    """Execute one full CNC production cycle across all factory stations."""
    successes = 0
    failures = 0

    for idx, station in enumerate(FACTORY_STATIONS, start=1):
        print("─" * 64)
        print(f"[DISPATCH {idx}/{len(FACTORY_STATIONS)}] Routing AMR → {station['name']} "
              f"at (x={station['x']}, y={station['y']}, yaw={station['yaw']}°)")
        print("─" * 64)

        goal_pose = create_pose(navigator, station["x"], station["y"], station["yaw"])
        navigator.goToPose(goal_pose)

        start_time = time.monotonic()
        last_log_time = 0.0

        while not navigator.isTaskComplete():
            # Per-station timeout guard
            elapsed = time.monotonic() - start_time
            if elapsed > STATION_NAV_TIMEOUT:
                print(f"  [TIMEOUT] {station['name']} unreachable after "
                      f"{STATION_NAV_TIMEOUT:.0f}s. Canceling.")
                navigator.cancelTask()
                break

            feedback = navigator.getFeedback()
            if feedback and (time.monotonic() - last_log_time) >= 2.0:
                print(
                    f"  → [TELEMETRY] Target: {station['name']:20s} │ "
                    f"Dist: {feedback.distance_remaining:5.2f} m │ "
                    f"ETA: {feedback.estimated_time_remaining.sec:3d}s │ "
                    f"Recoveries: {feedback.number_of_recoveries}"
                )
                last_log_time = time.monotonic()

        # Evaluate navigation result
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            successes += 1
            print(f"\n  ✓ [DOCKED] {station['name']}  —  Executing part transfer "
                  f"({station['dwell_sec']}s dwell)...")
            time.sleep(station["dwell_sec"])
        elif result == TaskResult.CANCELED:
            failures += 1
            print(f"\n  ✗ [CANCELED] Mission to {station['name']} was canceled.")
        elif result == TaskResult.FAILED:
            failures += 1
            print(f"\n  ✗ [FAILED] Could not reach {station['name']}. "
                  f"Advancing to next station.")

    return successes, failures


def main():
    rclpy.init()
    navigator = BasicNavigator()

    try:
        print("\n" + "═" * 64)
        print("    ANTROPI ROBOTICS — CNC FACTORY AMR DISPATCH SYSTEM")
        print("═" * 64)

        # 1. Set Initial AMCL Pose (matches turtlebot3_world spawn)
        print(f"\n[Factory OS] Setting initial pose at Raw_Material_Dock "
              f"({INITIAL_POSE['x']}, {INITIAL_POSE['y']})...")
        initial = create_pose(navigator, **INITIAL_POSE)
        navigator.setInitialPose(initial)

        # 2. Wait for Nav2 lifecycle servers to fully activate
        print("[Factory OS] Waiting for Nav2 lifecycle activation...")
        navigator.waitUntilNav2Active()
        print("[Factory OS] Nav2 Active — beginning production cycle.\n")

        # 3. Execute the factory dispatch loop
        successes, failures = run_factory_cycle(navigator)

        # 4. Summary
        print("\n" + "═" * 64)
        total = len(FACTORY_STATIONS)
        print(f"[Factory OS] Cycle complete: {successes}/{total} stations reached, "
              f"{failures} failure(s).")
        print("═" * 64 + "\n")

    except KeyboardInterrupt:
        print("\n\n[Factory OS] ⚠ EMERGENCY STOP — canceling active navigation goal...")
        navigator.cancelTask()

    finally:
        print("[Factory OS] Shutting down dispatcher.")
        rclpy.shutdown()


if __name__ == '__main__':
    main()
