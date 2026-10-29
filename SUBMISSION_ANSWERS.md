# Antropi Robotics — Submission Questionnaire Answers

**Role:** Robotics Software Intern  
**Package:** `antropi_amr_sim`

---

### Q1: Describe your approach to setting up the simulation and Nav2 stack.

I built a modular ROS 2 Python package (`antropi_amr_sim`) that goes beyond manually clicking goals in RViz2. The package is structured around Antropi's core use case — autonomous material handling in a CNC factory:

**1. Simulation Environment:**
- Deployed **ROS 2 Jazzy** and **Gazebo Harmonic** on an ARM64 Ubuntu 24.04 VM (UTM on Apple Silicon), using the **TurtleBot3 Waffle** with its simulated 360° 2D LiDAR.
- Addressed Apple Silicon graphics by configuring `LIBGL_ALWAYS_SOFTWARE=1` for Mesa software rasterization fallback when VirtIO GPU acceleration produced Ogre2 viewport blackouts.

**2. Unified Launch Architecture:**
- Created `factory_sim_bringup.launch.py` that orchestrates three systems in a single command: Gazebo simulation (world + robot), Nav2 stack (AMCL, planners, controllers, lifecycle managers), and RViz2.
- Nav2 launch is wrapped in a `TimerAction(period=5.0)` delay so that the Gazebo `/clock` topic and `ros_gz_bridge` TF publishers are active before AMCL and costmap nodes attempt their first transform lookups. Without this delay, I consistently observed `Could not transform` errors during cold starts.
- The launch file resolves all paths via `ament_index_python.get_package_share_directory()` — no hardcoded filesystem paths.

**3. Autonomous Factory Dispatcher (`factory_dispatcher.py`):**
- Built on `nav2_simple_commander`'s `BasicNavigator`, which wraps the `/navigate_to_pose` ROS 2 Action interface.
- Programmatically sets the initial AMCL pose at a simulated Raw Material Dock, waits for all Nav2 lifecycle servers to transition to `active` via `waitUntilNav2Active()`, then dispatches the AMR through a 4-station production loop (`Raw_Material_Dock` → `CNC_Mill_Cell_01` → `CNC_Lathe_Cell_02` → `CMM_Quality_Control` → `Dock`).
- Each station navigation includes: live feedback polling (distance remaining, ETA, recovery count), a per-station 120-second timeout to prevent infinite recovery loops, and a simulated machine-cell dwell time.
- `KeyboardInterrupt` is caught to gracefully cancel the active Nav2 goal before `rclpy.shutdown()`.

**4. Custom Nav2 Parameter Tuning:**
- Forked the upstream `waffle.yaml` into `nav2_factory_params.yaml` and applied targeted overrides for the narrow-corridor geometry of `turtlebot3_world`. Every override is annotated with `[OVERRIDE]` tags and engineering justification in the YAML file itself.
- Standardized all nodes to `robot_base_frame: base_footprint` per REP-105.

**5. Fleet Observability & Black-Box Incident Recorder (`factory_telemetry_hud.py`):**
- Designed a decoupled, passive observer node that subscribes to `/odom`, `/scan`, and `/cmd_vel` without imposing computational overhead on the Nav2 action servers.
- Computes real-time minimum LiDAR range ($d_{\min}$), forward Time-to-Collision ($\text{TTC} = d_{\min} / v_x$), and classifies safety tiers (`NOMINAL`, `CAUTION`, `PROXIMITY_BREACH`).
- Automatically triggers a structured JSON incident report written to `incidents/incident_log.jsonl` whenever an obstacle breach occurs ($< 0.35\text{ m}$), capturing timestamp, AMR coordinates, velocity vectors, and zone classification for post-shift spatial congestion analysis.

---

### Q2: What challenges did you encounter, and how did you solve them?

**Challenge 1 — Cold-Start TF Race Condition:**
- *What happened:* When launching Gazebo and Nav2 simultaneously, the Nav2 `controller_server` and `amcl` nodes started requesting transforms (`map → odom → base_footprint`) before Gazebo's `ros_gz_bridge` had begun publishing `/clock` and `/tf`. This caused `tf2::ExtrapolationException` errors and costmap initialization failures.
- *Root cause:* Nav2 lifecycle nodes use `/clock` for all TF timestamp comparisons when `use_sim_time: True`. If `/clock` hasn't started, the nodes interpret all transforms as "in the future" and reject them.
- *Fix:* Wrapped the Nav2 `IncludeLaunchDescription` in a `TimerAction(period=5.0)` so Gazebo's physics loop and bridge are fully online before Nav2 attempts its first transform lookup.

**Challenge 2 — Narrow Corridor Costmap Inflation Overlap:**
- *What happened:* The cylindrical pillars in `turtlebot3_world` create ~0.6m passages. With default Nav2 inflation parameters (`inflation_radius: 1.0m`, `cost_scaling_factor: 3.0`), the inflated obstacle costs from opposing pillars overlapped completely across the corridor, producing a wall of lethal/inscribed cells. The NavFn global planner consistently returned "no valid path."
- *Root cause:* The exponential decay function $\text{cost} = 253 \times e^{-\text{factor} \times (\text{distance} - r_{\text{inscribed}})}$ with $\text{factor}=3.0$ decays too slowly — at 0.3m from the obstacle surface, the cost is still ~100 (out of 253), making the corridor center appear near-lethal from both sides.
- *Fix:* Increased `cost_scaling_factor` to `5.0` in the local costmap (steeper decay → cost drops to ~30 at 0.3m) and reduced `inflation_radius` to `0.45m` (still exceeds the Waffle's 0.22m footprint radius). The corridor center now has traversable cost values while the immediate obstacle vicinity remains lethal.

**Challenge 3 — BaseObstacle Critic Weight for Factory Safety:**
- *What happened:* During initial testing, the DWB controller occasionally chose trajectories that clipped obstacle edges by <5cm. While no collisions occurred (the costmap lethal band prevented that), the trajectories were uncomfortably close for a factory context where an AMR carrying CNC parts must maintain conservative clearance.
- *Fix:* Increased `BaseObstacle.scale` from `0.02` to `0.05`, which more heavily penalizes trajectory candidates that pass near obstacle cells. The robot now consistently selects paths centered in corridors at the cost of slightly longer route times (~0.5s per segment).

**Challenge 4 — ROS 2 Jazzy `TwistStamped` Transition:**
- *What happened:* I investigated the known ROS 2 Jazzy migration from `geometry_msgs/msg/Twist` to `geometry_msgs/msg/TwistStamped` on `/cmd_vel`. Using `ros2 topic info /cmd_vel -v`, I verified that in the current `turtlebot3_simulations` Jazzy release, both the Nav2 velocity smoother (publisher) and the `ros_gz_bridge` differential drive (subscriber) already agree on the same message type. No manual `enable_stamped_cmd_vel` override was needed for this specific package combination, but this is a version-sensitive configuration that should be re-verified after any `apt upgrade`.

**Challenge 5 — Dynamic Obstacle Injection & Telemetry Near-Miss Logging:**
- *What happened:* Demonstrating obstacle avoidance required verifying that the local costmap and trajectory controller react dynamically to unexpected changes, not just static map features.
- *Solution:* Dropped a cylinder primitive into the active trajectory between `CNC_Lathe_Cell_02` and `CMM_Quality_Control`. The DWB local planner successfully halted linear advance, turned into clear space, and replanned around the obstacle. Concurrently, the `factory_telemetry_hud` node captured the transient proximity drop ($d_{\min} = 0.31\text{ m}$), logged an incident to disk, and returned to `NOMINAL` status once clearance exceeded $0.60\text{ m}$.

---

### Q3: What were your key learnings from this exercise?

**1. Nav2 Lifecycle Orchestration Is Non-Trivial:**
Understanding the `unconfigured → inactive → active` state machine managed by `nav2_lifecycle_manager` was essential. I initially tried sending navigation goals before AMCL's particle filter had converged (the lifecycle was `active` but the localization wasn't stable). Adding a brief post-activation pause and verifying the AMCL particle cloud convergence in RViz2 before dispatching goals resolved intermittent "goal rejected" errors.

**2. Costmap Math Drives Real Engineering Decisions:**
Tuning inflation parameters isn't about finding "magic numbers" — it requires understanding the exponential decay function, the relationship between `robot_radius`, `inflation_radius`, and `cost_scaling_factor`, and how the global planner's cost threshold interacts with corridor geometry. This exercise made clear why production AMR teams invest heavily in costmap layer design.

**3. Signal Handling and Graceful Shutdown Matter:**
My first version of `factory_dispatcher.py` had no `KeyboardInterrupt` handler. When I pressed Ctrl+C during navigation, the Nav2 action server kept executing the last goal (the BT navigator continued driving the robot) because no cancel request was sent. In a real factory, an AMR that doesn't stop when the dispatch system crashes is a safety hazard. Wrapping the main loop in `try/except KeyboardInterrupt` with `navigator.cancelTask()` in the handler was a direct lesson in defensive robotics software design.

**4. Frame Consistency Across the Stack (REP-105):**
I initially had `base_footprint` in AMCL but `base_link` in costmaps and `bt_navigator`. While these frames are connected by an identity transform on the Waffle, the inconsistency caused subtle TF lookup warnings. Standardizing to `base_footprint` everywhere (per REP-105 for mobile bases) eliminated the warnings and made the TF tree cleaner when inspected via `ros2 run tf2_tools view_frames`.

**5. Observability and Incident Forensics in Dark Factories:**
In lights-out manufacturing environments running 24/7, manual RViz inspection is impossible. Building the passive telemetry HUD and black-box JSON incident recorder reinforced that robotics software engineering isn't just about motion planning—it's about observability, deterministic failure logging, and edge diagnostics that allow remote fleet managers to diagnose anomalies without re-running tests.
