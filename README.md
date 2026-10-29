# Antropi Robotics — Autonomous CNC Factory AMR Simulation

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-blue?logo=ros)](https://docs.ros.org/en/jazzy/)
[![Simulator](https://img.shields.io/badge/Simulator-Gazebo_Harmonic-orange)](https://gazebosim.org)
[![Nav2](https://img.shields.io/badge/Nav2-Navigation2-green)](https://docs.nav2.org/)
[![Robot](https://img.shields.io/badge/Platform-TurtleBot3_Waffle-yellow)](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
[![License](https://img.shields.io/badge/License-MIT-purple.svg)](LICENSE)

An end-to-end autonomous mobile robot (AMR) simulation tailored to **Antropi Robotics'** mission of building 24/7 dark CNC factories. Features a programmatic multi-station dispatch node, **real-time safety telemetry HUD with Time-to-Collision monitoring**, and **automated black-box incident logging** for factory floor anomaly forensics.

---

## 📹 Video Demo

> **[▶ Watch the 90-Second Demo](https://youtube.com/watch?v=REPLACE_WITH_YOUR_LINK)**

The demo shows:
1. **Automated AMCL localization** — no manual RViz2 clicks.
2. **4-station CNC production loop** — `Raw Material Dock` → `CNC Mill 01` → `CNC Lathe 02` → `CMM Inspection` → `Dock`.
3. **Live Telemetry HUD** — real-time minimum LiDAR distance, Time-to-Collision (TTC), and safety tier classification.
4. **Dynamic obstacle stress test** — a box dropped mid-transit triggers a `[PROXIMITY BREACH]` alert on the HUD and automatically writes a structured JSON incident report with the robot's exact coordinates.

---

## 🏗️ System Architecture

```text
                               +----------------------------+
                               |    factory_dispatcher.py   |
                               | (nav2_simple_commander API)|
                               +--------------+-------------+
                                              |
               /navigate_to_pose (ROS 2 Action: Goal / Feedback / Result)
                                              v
+-----------------------------------------------------------------------------------+
|                                ROS 2 Nav2 Stack                                   |
|  +--------------------+   +-----------------------+   +------------------------+  |
|  |    BT Navigator    |-->| Global Planner (NavFn)|-->| Local Controller (DWB) |  |
|  +--------------------+   +-----------------------+   +------------------------+  |
|                                       ^                           ^               |
|                           +-----------+-----------+   +-----------+-----------+   |
|                           |    Global Costmap     |   |     Local Costmap     |   |
|                           | (Static + Obstacle +  |   | (Obstacle + Inflation)|   |
|                           |  Inflation Layers)    |   +-----------+-----------+   |
|                           +-----------+-----------+               ^               |
+---------------------------------------|---------------------------|---------------+
                                        |                           |
                 /map (AMCL)            |                           | /scan (LiDAR)
                                        v                           v
+-----------------------------------------------------------------------------------+
|                        Gazebo Harmonic Physics Simulation                         |
|    TurtleBot3 Waffle  |  360° 2D LiDAR  |  Differential Drive                    |
|    ros_gz_bridge (Clock, Odom, TF, CmdVel sync)                                  |
+-----------------------------------------------------------------------------------+
                                        ^           ^           ^
                                        |           |           |
                              /odom ----+   /scan --+   /cmd_vel+
                                        |           |           |
                               +--------+-----------+-----------+--------+
                               |       factory_telemetry_hud.py          |
                               |  (Passive Observer — Read-Only)         |
                               |                                         |
                               |  • Min LiDAR Distance (d_min)           |
                               |  • Time-to-Collision (TTC = d/v)        |
                               |  • Safety Tier (NOMINAL/CAUTION/BREACH) |
                               |  • Black-Box JSON Incident Logger       |
                               +-----------------------------------------+
```

---

## 🏭 Simulated Factory Workstations

| Station | Workstation | Coordinates (x, y, yaw) | Dwell |
| :---: | :--- | :---: | :---: |
| 01 | `Raw_Material_Dock` (Spawn) | (-2.00, -0.50, 0°) | 2.0s |
| 02 | `CNC_Mill_Cell_01` | (0.50, 1.80, 0°) | 3.0s |
| 03 | `CNC_Lathe_Cell_02` | (1.80, -0.50, -90°) | 3.0s |
| 04 | `CMM_Quality_Control` | (0.40, -1.80, 180°) | 3.0s |

---

## 📊 Factory Telemetry HUD & Black-Box Incident Logger

A core differentiator of this submission: in a real 24/7 dark factory, nobody watches RViz2 at 3:00 AM. The **Factory Telemetry HUD** is a passive observer node that provides automated fleet diagnostics:

### Live Terminal Dashboard
```text
══════════════════════════════════════════════════════════════════════════
║  ANTROPI FACTORY OS — AMR FLEET TELEMETRY & DIAGNOSTICS HUD          ║
══════════════════════════════════════════════════════════════════════════

  SAFETY STATUS:   ⚠ PROXIMITY BREACH — DYNAMIC REPLANNING

──────────────────────────────────────────────────────────────────────────
  AMR POSE
    Position:       x =  +0.820 m   y =  +0.450 m   yaw =  -42.1°
    Zone:           Transit_Aisle (near CNC_Mill_Cell_01)

──────────────────────────────────────────────────────────────────────────
  PROXIMITY & SAFETY METRICS
    Min LiDAR Dist: 0.310m  @ bearing  +12.5°
    Time-to-Collision (TTC): 1.4s
    Safety Tier:    PROXIMITY_BREACH

──────────────────────────────────────────────────────────────────────────
  BLACK-BOX INCIDENT RECORDER
    Incidents Logged:  3
    Log File:          ~/antropi_ws/incidents/incident_log.jsonl
══════════════════════════════════════════════════════════════════════════
```

### Safety Tier Classification
| Tier | Condition | Meaning |
| :--- | :--- | :--- |
| 🟢 **NOMINAL** | Min LiDAR distance > 0.60m | All clear, normal operation |
| 🟡 **CAUTION** | 0.35m < distance ≤ 0.60m | Obstacle approaching, monitoring |
| 🔴 **PROXIMITY BREACH** | distance ≤ 0.35m | Near-miss detected, replanning triggered |

### Black-Box Incident Reports
When a proximity breach is detected, the HUD automatically writes a structured JSON record:
```json
{
  "incident_id": 1,
  "timestamp_utc": "2026-10-29T07:14:21.320000+00:00",
  "incident_type": "PROXIMITY_BREACH",
  "severity": "WARNING",
  "amr_state": {
    "position": {"x": 0.8200, "y": 0.4500, "yaw_deg": -42.1},
    "velocity": {"linear_mps": 0.2200, "angular_radps": 0.1500}
  },
  "obstacle_proximity": {
    "min_distance_m": 0.3100,
    "bearing_deg": 12.5,
    "ttc_sec": 1.41
  },
  "zone_classification": "Transit_Aisle (near CNC_Mill_Cell_01)"
}
```

These logs can be post-processed to identify **high-congestion factory zones** and optimize station placement or aisle layouts.

---

## 🚀 Quick Start (Native Ubuntu 24.04 / UTM VM)

### 1. Prerequisites

**OS:** Ubuntu 24.04 LTS (ARM64 or AMD64)  
**ROS 2:** Jazzy Jalisco

```bash
# Install required packages
sudo apt update && sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-simple-commander \
  ros-jazzy-turtlebot3 \
  ros-jazzy-turtlebot3-msgs \
  ros-jazzy-turtlebot3-simulations \
  python3-colcon-common-extensions

# Set TurtleBot3 model
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

> **Apple Silicon UTM Users:** If Gazebo shows a black viewport, add `export LIBGL_ALWAYS_SOFTWARE=1` to `~/.bashrc`.

### 2. Clone & Build

```bash
mkdir -p ~/antropi_ws/src && cd ~/antropi_ws/src
git clone https://github.com/boozards/AntropiTask.git

cd ~/antropi_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch Simulation + Nav2

**Terminal 1** — Spawns Gazebo, starts Nav2 (with tuned parameters), opens RViz2:
```bash
source ~/antropi_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch antropi_amr_sim factory_sim_bringup.launch.py
```

### 4. Run the Factory Dispatcher

**Terminal 2** — Begins the autonomous CNC material handling loop:
```bash
source ~/antropi_ws/install/setup.bash
ros2 run antropi_amr_sim factory_dispatcher
```

### 5. Run the Telemetry HUD (New!)

**Terminal 3** — Live safety dashboard and black-box incident recorder:
```bash
source ~/antropi_ws/install/setup.bash
ros2 run antropi_amr_sim factory_telemetry_hud
```

The HUD will:
- Display real-time AMR pose, velocity, minimum LiDAR distance, and TTC
- Flash **PROXIMITY BREACH** alerts when obstacles come within 0.35m
- Auto-log structured JSON incident reports to `~/antropi_ws/incidents/incident_log.jsonl`

Press **Ctrl+C** to exit — prints a summary of incidents logged.

---

## 🐳 Docker Quick Start (Reproducible Evaluation)

> **Note:** Docker GUI forwarding requires a Linux host with X11. On macOS, use the native UTM approach above.

```bash
git clone https://github.com/boozards/AntropiTask.git
cd antropi_amr_sim

xhost +local:root

# Terminal 1: Launch Gazebo + Nav2 + RViz2
docker compose up --build antropi_sim

# Terminal 2: Run the factory dispatcher
docker compose run dispatcher

# Terminal 3: Run the telemetry HUD
docker compose run telemetry_hud
```

---

## ⚙️ Nav2 Parameter Tuning & Engineering Decisions

All overrides are annotated with `[OVERRIDE]` comments in [`config/nav2_factory_params.yaml`](src/antropi_amr_sim/config/nav2_factory_params.yaml).

### Planner & Controller Selection
- **Global Planner → NavFn (A\*):** Sufficient for the open `turtlebot3_world` arena. SmacPlanner2D would be preferred for tighter multi-aisle layouts with kinematic feasibility constraints.
- **Local Controller → DWB:** Well-characterized at TurtleBot3 speeds (0.26 m/s). MPPI adds GPU overhead with marginal benefit at these velocities.

### Costmap Inflation Tuning
- **`cost_scaling_factor: 5.0`** (local) / **`4.0`** (global): Steeper exponential decay prevents inflation overlap in ~0.6m corridors.
- **`inflation_radius: 0.45m`** (local) / **`0.50m`** (global): Exceeds Waffle footprint (0.22m) for safety clearance.
- **`BaseObstacle.scale: 0.05`**: Increased from `0.02` to prioritize factory safety over path optimality.

### Frame Consistency
All nodes standardize on `base_footprint` (REP-105). TF chain: `map` → `odom` → `base_footprint` → `base_link`.

---

## 📁 Repository Structure

```text
antropi_amr_sim/
├── Dockerfile                      # ROS 2 Jazzy container definition
├── docker-compose.yml              # Three services: sim, dispatcher, telemetry_hud
├── README.md                       # This file
├── SUBMISSION_ANSWERS.md           # Assignment questionnaire responses
├── LICENSE                         # MIT License
├── .gitignore
├── media/                          # Demo recording
└── src/
    └── antropi_amr_sim/
        ├── package.xml             # ROS 2 package manifest
        ├── setup.py                # Build config & entrypoints
        ├── setup.cfg               # colcon install paths
        ├── resource/
        │   └── antropi_amr_sim     # ament resource index marker
        ├── launch/
        │   └── factory_sim_bringup.launch.py   # Unified Gazebo + Nav2 + RViz2 launch
        ├── config/
        │   └── nav2_factory_params.yaml        # Complete Nav2 params (annotated)
        └── antropi_amr_sim/
            ├── __init__.py
            ├── factory_dispatcher.py           # Autonomous dispatch (nav2_simple_commander)
            └── factory_telemetry_hud.py        # Live safety HUD + incident logger
```

---

## 🛡️ License
Distributed under the MIT License. See [LICENSE](LICENSE) for details.
