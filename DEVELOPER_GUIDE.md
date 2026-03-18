# Ultimate Developer Guide — TRI (Wall-Following Reactive Robot)

## 1. Executive Summary

This repository implements a **ROS 2 Jazzy + Gazebo** simulation of an **Andino differential-drive robot** that must:
1. Spawn at a random position/heading near walls
2. Find and follow the left wall reactively
3. Enter an open circle (270° arc with a gap at the bottom)
4. Navigate to the center of the circle
5. Stop facing outward

**Current status:** The robot successfully follows walls and enters the circle. When >60% of lidar rays detect walls, it **stops completely** (termination). This is the wrong behavior — that 60% condition should **trigger a state transition** to center-seeking mode, not termination.

**Biggest gap:** No center-seeking logic exists. The 60% threshold currently stops the robot instead of switching to a centering phase. The "stop when all non-inf readings are approximately equal" logic is not yet implemented.

**Compliance concern:** The code uses `stop_confirm_count` (a frame counter) which constitutes **forbidden memory**. The PD controller's `prev_error` and `prev_time` are acceptable (derivative term internal state), but the confirmation counter must be redesigned.

---

## 2. Assignment Constraints That Shape the Design

### The No-Memory Rule

| Allowed | Forbidden |
|---------|-----------|
| Current lidar scan snapshot | History of past scans |
| PD/PID internal state (`prev_error`, `prev_time` for derivative) | Counters that accumulate across frames (e.g. `stop_confirm_count`) |
| Instantaneous geometric computations | Maps, SLAM, localization |
| Reactive state variable (current mode flag based on current sensor) | GPS, ground-truth pose from simulator |
| Wheel velocity commands from current readings | Storing past positions or trajectories |

### What "reactive state" means

A **single mode variable** (e.g., `WALL_FOLLOW` vs `CENTERING`) is acceptable **only if** the transition is determined entirely from the current sensor reading. The robot can know "I am currently in centering mode" but must decide that purely from whether the current lidar snapshot shows >60% valid readings. This is a reactive latch, not memory.

### Current Compliance Issues

> [!CAUTION]
> **`stop_confirm_count`** (line 51, `wall_follower_node.py`) is a **frame counter** that accumulates across callbacks. It counts how many consecutive scans exceed 60% valid readings before stopping. This is **temporal memory** and violates the assignment rules. It must be removed or replaced with a purely instantaneous check.

> [!NOTE]
> **`prev_error`** and **`prev_time`** (lines 48–49) are standard PD controller internals. These are generally accepted as "controller state" rather than "world model memory," since they exist only to compute a derivative term. Confirm with the instructor if unsure.

---

## 3. Repository Map

```
TRI/
├── README.md                          # Quick-start in Portuguese
├── andino/                            # Upstream Andino robot description (submodule/fork)
│   └── andino_description/
│       ├── urdf/
│       │   ├── andino.urdf.xacro      # Main robot URDF (includes sensors, wheels, caster)
│       │   └── include/
│       │       ├── common_sensors.urdf.xacro  # Lidar + camera link/joint macros
│       │       ├── common_macros.urdf.xacro   # Inertia helpers
│       │       └── andino_caster_macro.urdf.xacro
│       ├── config/andino/
│       │   └── sensors.yaml           # Lidar/camera mount offsets (dx=0.05, dy=0, dz=0.08)
│       └── meshes/sensors/            # STL files for rplidar, camera
│
├── andino_gz/                         # Gazebo simulation package
│   └── andino_gz/
│       ├── launch/
│       │   ├── assignment1.launch.py  # ★ MAIN ENTRY: random spawn + world selection
│       │   ├── andino_gz.launch.py    # Orchestrator: Gazebo + ROS bridge + RViz + Nav2
│       │   └── include/
│       │       ├── spawn_robot.launch.py    # URDF→Gazebo spawn + robot_state_publisher
│       │       └── gz_ros_bridge.launch.py  # Parametric ROS↔Gazebo topic bridge
│       ├── config/
│       │   └── bridge_config.yaml     # ★ Topic mapping: scan, cmd_vel, odom, tf, camera
│       ├── worlds/
│       │   └── assignment1.sdf        # ★ World: L-shaped wall + 270° open circle (18 segs)
│       └── urdf/
│           └── andino_gz.urdf.xacro   # ★ Gazebo overlays: DiffDrive, lidar (720 rays), camera
│
└── wall_follower/                     # ★ CUSTOM PACKAGE — the behavior you write
    ├── wall_follower/
    │   ├── __init__.py
    │   └── wall_follower_node.py      # ★★★ ALL CONTROL LOGIC LIVES HERE (200 lines)
    ├── launch/
    │   └── wall_follower.launch.py    # Node launch with parameter defaults
    ├── setup.py                       # ament_python package setup
    └── package.xml                    # Dependencies: rclpy, sensor_msgs, geometry_msgs
```

### Critical Path Files (must understand to modify behavior)

| File | Role | When it runs |
|------|------|-------------|
| `wall_follower/wall_follower/wall_follower_node.py` | All control logic | Every lidar scan (~10 Hz) |
| `wall_follower/launch/wall_follower.launch.py` | Parameter defaults | At launch |
| `andino_gz/andino_gz/launch/assignment1.launch.py` | Random spawn pose | At launch |
| `andino_gz/andino_gz/worlds/assignment1.sdf` | Physical environment | At Gazebo start |
| `andino_gz/andino_gz/urdf/andino_gz.urdf.xacro` | Sensor config (lidar specs) | At robot spawn |
| `andino_gz/andino_gz/config/bridge_config.yaml` | ROS↔Gazebo topic wiring | At bridge start |

---

## 4. System Architecture

### ROS 2 Node Graph

```
┌──────────────┐    /scan (LaserScan)     ┌──────────────────┐
│  Gazebo Sim  │ ──────────────────────── │  wall_follower   │
│  (gz-sim)    │                          │  node             │
│              │ ◄──────────────────────  │                  │
└──────────────┘    /cmd_vel (Twist)       └──────────────────┘
       │                                           │
       │  /clock, /odom, /tf, /joint_states        │ reads /scan
       ▼                                           │ publishes /cmd_vel
 ┌─────────────┐                                   │
 │ ros_gz_bridge│ (parameter_bridge)                │
 └─────────────┘                                   │
       │                                           │
 ┌─────────────┐                                   │
 │ robot_state │                                    │
 │ _publisher  │ publishes /tf, /tf_static          │
 └─────────────┘                                   │
```

### Key Topics

| ROS Topic | Gazebo Source | Type | Direction | Rate |
|-----------|-------------|------|-----------|------|
| `/scan` | `sensor_ray_front` | `sensor_msgs/LaserScan` | Gz→ROS | 10 Hz |
| `/cmd_vel` | DiffDrive plugin | `geometry_msgs/Twist` | ROS→Gz | ~10 Hz |
| `/odom` | OdometryPublisher | `nav_msgs/Odometry` | Gz→ROS | 20 Hz |
| `/tf` | Pose plugin | `tf2_msgs/TFMessage` | Gz→ROS | 20 Hz |
| `/clock` | Gazebo | `rosgraph_msgs/Clock` | Gz→ROS | 1000 Hz |

### Lidar Configuration (from `andino_gz.urdf.xacro`)

| Parameter | Value |
|-----------|-------|
| Samples | 720 (0.5° resolution) |
| FOV | Full 360° (−π to +π) |
| Min range | 0.20 m |
| Max range | 12.0 m |
| Noise | Gaussian, mean=0, σ=0.01 m |
| Update rate | 10 Hz |
| Frame | `rplidar_laser_link` |

> [!IMPORTANT]
> **The lidar is mounted rotated 180° (π radians)** in the URDF joint (`common_sensors.urdf.xacro`, line 45: `rpy="0 0 ${pi}"`). This means scan angle 0 points **backward** and ±π points **forward**. The code comments on lines 113–117 of `wall_follower_node.py` correctly document this.

### Diff-Drive Parameters (from `andino_gz.urdf.xacro`)

| Parameter | Value |
|-----------|-------|
| Wheel separation | 0.137 m |
| Wheel radius | 0.033 m |
| Left joint | `left_wheel_joint` |
| Right joint | `right_wheel_joint` |

### Launch Sequence

1. **Terminal 1:** `ros2 launch andino_gz assignment1.launch.py`
   - `assignment1.launch.py` → calls `random_spawn()` → injects pose into `sys.argv`
   - Includes `andino_gz.launch.py` with `world_name=assignment1.sdf`
   - `andino_gz.launch.py` → starts Gazebo, spawns robot, starts ROS bridge, starts RViz
2. **Terminal 2:** `ros2 launch wall_follower wall_follower.launch.py`
   - Starts `wall_follower_node` with PD parameters

### World Geometry (from `assignment1.sdf` and `assignment1.launch.py`)

| Element | Position (center) | Size/Radius |
|---------|-------------------|-------------|
| L-wall vertical arm | (−1.0, 2.982) | 0.5 × 5.5 m |
| L-wall horizontal arm | (0.175, 0.0) | 2.85 × 0.7 m |
| Circle center | (3.0, 2.0) | R_inner=2.0, R_outer=3.0 |
| Circle arc | 270° (gap at bottom, −135° to −45°) | 18 box segments |

---

## 5. How to Run the Project

### Prerequisites

- **ROS 2 Jazzy** (Ubuntu 24.04 recommended)
- **Gazebo Harmonic** (`gz-sim`)
- Packages: `ros-jazzy-ros-gz`, `ros-jazzy-nav2-common`, `ros-jazzy-nav2-bringup`
- Andino packages: `andino_description` (included in repo)

### Build

```bash
# Assuming workspace at ~/tri_ws with this repo cloned into src/
cd ~/tri_ws
colcon build
source install/setup.bash
```

### Run

**Terminal 1 — Gazebo + Robot:**
```bash
source ~/tri_ws/install/setup.bash
ros2 launch andino_gz assignment1.launch.py
```
Wait for Gazebo to fully load and the robot to appear. The console prints the random spawn pose.

**Terminal 2 — Wall Follower:**
```bash
source ~/tri_ws/install/setup.bash
ros2 launch wall_follower wall_follower.launch.py
```

### Sanity Checks

```bash
# Verify lidar data is flowing
ros2 topic echo /scan --once | head -20

# Verify cmd_vel is being published
ros2 topic hz /cmd_vel

# Check all active topics
ros2 topic list

# Verify the node is running
ros2 node list | grep wall_follower

# Monitor robot behavior in real-time
ros2 topic echo /cmd_vel
```

### Common Startup Failures

| Symptom | Cause | Fix |
|---------|-------|-----|
| `package 'andino_gz' not found` | Not sourced | Run `source install/setup.bash` |
| Gazebo starts but no robot | Spawn failed (collision) | Re-launch; spawn is random |
| `/scan` has no data | Bridge not running or Gazebo still loading | Wait 10s, check `ros2 topic hz /scan` |
| Robot doesn't move | `wall_follower` not launched or wrong topic | Check Terminal 2, verify `/cmd_vel` |
| `nav2_common` not found | Missing dependency | `sudo apt install ros-jazzy-nav2-common` |

---

## 6. Reactive Control Logic Explained

All logic lives in `wall_follower/wall_follower/wall_follower_node.py`, method `scan_callback()` (line 87).

### Execution Flow Per Scan

```
scan_callback(msg)
│
├─ If is_stopped → publish zero Twist, return
│
├─ Compute valid_reading_ratio (% non-inf rays)
│   ├─ If ratio ≥ 60% → increment stop_confirm_count ⚠️ MEMORY
│   │   └─ If count ≥ 15 → set is_stopped=True, publish zero, return
│   └─ Else → reset stop_confirm_count to 0 ⚠️ MEMORY
│
├─ Measure sectors:
│   ├─ left_dist:     min range in [−120°, −60°]  (robot's left)
│   ├─ left_fwd_dist: min range in [−160°, −120°] (left-forward)
│   ├─ front_dist:    min of [135°,179°] and [−179°,−135°] (robot's front)
│   └─ wall_dist = min(left_dist, left_fwd_dist × 1.1)
│
├─ Decision tree:
│   ├─ front_dist < 0.8m → FRONT OBSTACLE: stop, turn right (ω = −1.0)
│   ├─ wall_dist > 3.0m  → NO WALL: drive forward, gentle left turn (ω = +0.3)
│   └─ else → WALL FOLLOWING with PD:
│       ├─ error = wall_dist − desired_distance
│       ├─ d_error = (error − prev_error) / dt
│       ├─ angular_z = kp × error + kd × d_error, clamped to [−1, 1]
│       ├─ speed_scale from turn magnitude
│       ├─ front slowdown if front_dist < 1.5m
│       └─ publish Twist(linear.x, angular.z)
│
└─ Store prev_time, prev_error (PD state)
```

### Wall-Following Logic (PD Controller)

The robot tries to keep `desired_distance` (default 1.0m via launch, 1.5m declared) from the left wall:

- **error > 0** → too far → turn left (positive `angular.z` in ROS)
- **error < 0** → too close → turn right (negative `angular.z`)
- **Derivative term** dampens oscillation using rate of error change

This is purely reactive: each computation uses only the current scan.

### Circle Detection (Current — Broken)

The `_valid_reading_ratio()` method counts what fraction of the 720 lidar rays return valid (non-inf, non-nan, within range) readings. Inside the circle, walls surround the robot on most sides → ratio rises above 60%.

**Current behavior:** When ratio ≥ 0.6 for 15 consecutive frames (`stop_confirm_count`), robot stops.

**Problems:**
1. Uses temporal memory (counter) — **violates assignment**
2. Stops instead of transitioning to centering mode
3. No center-seeking logic implemented

---

## 7. Current "60% Lidar" Behavior and Required Evolution

### What Currently Happens (lines 96–110)

```python
ratio = self._valid_reading_ratio(msg)
if ratio >= self.inside_ratio:          # inside_ratio = 0.6
    self.stop_confirm_count += 1        # ⚠️ MEMORY VIOLATION
    if self.stop_confirm_count >= self.stop_confirm_needed:  # 15 frames
        self.is_stopped = True          # Terminal — never recovers
        self.cmd_pub.publish(Twist())
        return
else:
    self.stop_confirm_count = 0         # ⚠️ MEMORY VIOLATION
```

### What Should Happen

1. **State transition (not termination):** When ≥60% of rays are valid *in the current scan*, switch from `WALL_FOLLOW` mode to `CENTERING` mode
2. **Center detection:** In `CENTERING` mode, check if all non-inf rays read approximately the same distance → robot is near center
3. **Motion toward center:** Move toward the direction with the greatest free distance
4. **Stop condition:** When non-inf readings are "equal enough" (within tolerance), stop
5. **Final heading:** Face outward (toward the gap — the direction with largest/inf readings)

### How to Detect "Near Center" — Purely Reactive

When the robot is at the center of a circle, all non-inf lidar rays hit the wall at approximately the same distance (the circle's inner radius). The metric:

```python
valid_ranges = [r for r in msg.ranges if not (math.isinf(r) or math.isnan(r)
                 or r < msg.range_min or r > msg.range_max)]
if len(valid_ranges) < 3:
    return  # not enough data

mean_dist = sum(valid_ranges) / len(valid_ranges)
max_dev = max(abs(r - mean_dist) for r in valid_ranges)
# If max_dev < tolerance (e.g. 0.15m), we are at center
```

This is **purely instantaneous** — no memory needed.

### How to Choose Direction — Reactive

```python
# Find the angular sector with maximum free distance
# Move toward it (set angular.z to steer toward that sector)
max_range = -1
max_angle = 0
for i, r in enumerate(msg.ranges):
    if not math.isinf(r) and r > max_range:
        max_range = r
        max_angle = msg.angle_min + i * msg.angle_increment
# Convert max_angle to steering command
```

Since the lidar is rotated 180°, the "forward" direction for the robot is at angle ±π in the scan. The steering command should turn the robot so that its forward axis points toward `max_angle`.

### Tolerance Strategy to Avoid Oscillation

- Use `center_tolerance = 0.15` (meters) — the maximum deviation of any non-inf reading from the mean
- Use `centering_speed = 0.08` (m/s) — slower than wall-following to prevent overshoot
- Scale speed proportionally to `max_dev`: `speed = centering_speed * min(max_dev / center_tolerance, 1.0)`
- This creates natural deceleration as the robot approaches center

### Assignment-Compliant Pseudocode

```python
def scan_callback(self, msg):
    ratio = self._valid_reading_ratio(msg)
    valid_ranges = self._get_valid_ranges(msg)

    if ratio >= 0.6 and len(valid_ranges) >= 3:
        # ── CENTERING MODE (reactive: decided each scan) ─────
        mean_dist = mean(valid_ranges)
        max_dev = max(|r - mean_dist| for r in valid_ranges)

        if max_dev < CENTER_TOLERANCE:
            # AT CENTER → stop, face outward
            # "outward" = toward the gap = direction of max range or inf
            cmd = self._face_outward(msg)
            # Once facing outward: publish zero twist
        else:
            # NOT AT CENTER → move toward largest distance
            cmd = self._move_toward_max_distance(msg)
    else:
        # ── WALL-FOLLOW MODE (existing logic) ────────────────
        cmd = self._wall_follow(msg)

    self.cmd_pub.publish(cmd)
```

> [!IMPORTANT]
> The mode decision (`ratio >= 0.6`) is re-evaluated **every scan**. There is no counter, no flag that persists. If the robot somehow exits the circle, ratio drops below 0.6 and it reverts to wall-following automatically. This is fully reactive.

### On "Reactive State" vs "Memory"

A mode variable like `self.mode = 'CENTERING'` **is acceptable** if the **transition condition** is re-evaluated from scratch each callback. The variable just prevents re-computing the mode decision, but the same conclusion would be reached from sensors alone. The key rule: **if you disconnect power and restart with the same sensor snapshot, the robot must make the same decision.**

However, the **safest** approach is to not even store a mode variable — just use `if ratio >= 0.6:` as a branch every time.

---

## 8. File-by-File Change Guide

### `wall_follower/wall_follower/wall_follower_node.py` — THE CORE FILE

This 200-line file contains **all** behavior. Every tuning change happens here or in the launch file.

#### Change 1: Remove `stop_confirm_count` (compliance fix)

**Lines to edit:** 31, 42, 47–51, 96–110

**What to do:** Remove `stop_confirmation_count` parameter, `stop_confirm_count` state variable, and the counter logic. Replace with instantaneous ratio check.

**Risk:** Without the confirmation counter, a single noisy scan could trigger centering mode prematurely. Mitigation: use a slightly higher threshold (e.g., 0.65) or require a minimum number of valid rays.

#### Change 2: Replace stop logic with centering logic

**Lines to edit:** 96–110 (the `if ratio >= self.inside_ratio:` block)

**What to do:** Instead of incrementing a counter and stopping, implement center-seeking:
- Compute mean and max deviation of valid ranges
- If deviation < tolerance → stop (or face outward first)
- Else → steer toward direction of maximum range, drive slowly

**Effect:** Robot enters circle, navigates to center, stops there.

#### Change 3: Add center-seeking parameters

**Lines to add after line 30:**
```python
self.declare_parameter('center_tolerance', 0.15)
self.declare_parameter('centering_speed', 0.08)
```

#### Change 4: Add helper methods

Add `_move_toward_max_distance(msg)` and `_face_outward(msg)` methods.

**`_move_toward_max_distance`:** Find the lidar ray index with the largest valid distance. Compute the angular offset from robot forward (±π due to lidar rotation). Set `angular.z` to steer toward that direction, `linear.x` to `centering_speed` scaled by deviation.

**`_face_outward`:** Find the direction of the gap (maximum range / inf readings). Rotate in place until robot forward aligns with that direction. Then stop.

#### Change 5: Speed/velocity tuning

| Parameter | Line | Current | Suggested | Effect of increase |
|-----------|------|---------|-----------|-------------------|
| `desired_distance` | 21/13(launch) | 1.5/1.0 | 1.0 | Follows farther from wall |
| `forward_speed` | 22/14(launch) | 0.15 | 0.12–0.18 | Faster traversal, less stable |
| `kp` | 23/15(launch) | 1.2/1.5 | 1.0–2.0 | More aggressive correction |
| `kd` | 24/16(launch) | 0.4/0.5 | 0.3–0.8 | More damping, slower response |
| `front_obstacle_dist` | 25/17(launch) | 0.8/0.5 | 0.5–1.0 | Earlier obstacle avoidance |
| `front_slowdown_dist` | 26 | 1.5 | 1.0–2.0 | Earlier deceleration |
| `inside_ratio_threshold` | 30 | 0.6 | 0.55–0.70 | Earlier/later circle detection |

### `wall_follower/launch/wall_follower.launch.py`

**Purpose:** Sets parameter defaults that override the `declare_parameter` defaults.

> [!WARNING]
> The launch file sets `desired_distance=1.0`, `kp=1.5`, `kd=0.5`, `front_obstacle_dist=0.5` — these **differ** from the declared defaults of 1.5, 1.2, 0.4, 0.8. The launch file values win at runtime. Always check both files.

**What you'd edit here:** Any parameter default. Also add new parameters like `center_tolerance` and `centering_speed`.

### `andino_gz/andino_gz/launch/assignment1.launch.py`

**Purpose:** Random spawn positioning.

**Key constants (lines 12–31):**
- `MIN_WALL_DIST = 0.5` — minimum clearance from walls
- `MAX_WALL_DIST = 2.0` — must be within this distance of some wall
- `CIRCLE_CENTER = (3.0, 2.0)`, `CIRCLE_R_INNER = 2.0`, `CIRCLE_R_OUTER = 3.0`

**When to edit:** If spawn range needs adjustment, or to spawn at a fixed position for debugging:
```python
# Replace random_spawn() call on line 99 with:
x, y, yaw = 0.0, 3.0, 0.0  # Fixed pose for debugging
```

### `andino_gz/andino_gz/worlds/assignment1.sdf`

**Purpose:** Defines the physical world — walls and circle.

**When to edit:** To change wall positions, circle radius, gap angle, or add obstacles. The circle is built from 18 box segments at 15° intervals covering 270° (gap from −135° to −45°, i.e., roughly south-facing).

### `andino_gz/andino_gz/urdf/andino_gz.urdf.xacro`

**Purpose:** Robot-specific Gazebo plugins and sensor overlays.

**Key editable parameters:**
- **Lidar samples** (line 55): `720` — affects scan resolution
- **Lidar range** (lines 62–63): `0.20` min, `12.0` max
- **Lidar noise** (lines 68–69): σ = `0.01` m
- **Lidar rate** (line 74): `10.0` Hz
- **Wheel separation** (line 12): `0.137` m
- **Wheel radius** (line 13): `0.033` m

### `andino_gz/andino_gz/config/bridge_config.yaml`

**Purpose:** Maps Gazebo topics to ROS topics.

**When to edit:** If you rename topics (e.g., using a namespace) or need to bridge additional sensors. The critical entries are `scan` (line 30–34) and `cmd_vel` (line 40–44).

---

## 9. Behavior Tuning Playbook

| Symptom | Likely Cause | What to Change | File |
|---------|-------------|----------------|------|
| Robot hugs wall too aggressively | `desired_distance` too small | Increase `desired_distance` to 1.2–1.5 | `wall_follower.launch.py` line 13 |
| Robot oscillates along wall | `kd` too low or `kp` too high | Increase `kd` to 0.6–0.8, decrease `kp` to 1.0 | `wall_follower.launch.py` lines 15–16 |
| Robot oscillates near circle center | `center_tolerance` too tight or `centering_speed` too high | Increase tolerance to 0.20–0.25, decrease speed to 0.05 | `wall_follower_node.py` (new params) |
| Robot stops too early (outside circle) | `inside_ratio_threshold` too low | Increase from 0.6 to 0.65–0.70 | `wall_follower_node.py` line 30 |
| Robot never detects the circle | `inside_ratio_threshold` too high or lidar max_range too short | Lower threshold to 0.55; check `andino_gz.urdf.xacro` max range | `wall_follower_node.py` line 30 |
| Robot turns the wrong way | Lidar 180° rotation not accounted for | Verify angle sectors in `scan_callback()` match the comments on lines 113–117 | `wall_follower_node.py` lines 119–126 |
| Robot spins in place | Front obstacle threshold too aggressive, or wall too close on both sides | Increase `front_obstacle_dist` slightly; check left sector angles | `wall_follower_node.py` line 25/`launch.py` line 17 |
| Lidar inf handling is broken | `range_min`/`range_max` filtering mismatch | Check `_get_min_range_sector()` and `_valid_reading_ratio()` filtering logic matches URDF values (0.20–12.0) | `wall_follower_node.py` lines 62–85 |
| Gazebo and ROS topics disagree | Bridge config placeholder not replaced | Verify `<entity>` and `<world>` are replaced correctly in `gz_ros_bridge.launch.py` | `bridge_config.yaml` + `gz_ros_bridge.launch.py` |
| Robot drives into the wall | `front_obstacle_dist` too small | Increase to 0.8; also check `front_slowdown_dist` (line 26) | `wall_follower_node.py` / `launch.py` |
| Robot can't enter circle gap | Following the wrong side, or gap approach angle is bad | The circle gap is at the bottom (~south). Left-wall following should naturally enter from the left side of the gap. Check that `wall_dist` weighting factor (line 129) is correct | `wall_follower_node.py` line 129 |

---

## 10. Parameter Reference

| Parameter | File(s) | Declared Default | Launch Default | Meaning | ↑ Effect | ↓ Effect | Notes |
|-----------|---------|-----------------|----------------|---------|----------|----------|-------|
| `desired_distance` | node L21, launch L13 | 1.5 m | 1.0 m | Target wall distance | Follows farther from wall | Hugs wall tighter | Launch value overrides |
| `forward_speed` | node L22, launch L14 | 0.15 m/s | 0.15 m/s | Base linear velocity | Faster but less stable | Safer but slower | Scaled down when turning |
| `kp` | node L23, launch L15 | 1.2 | 1.5 | Proportional gain | More aggressive steering | Sluggish response | Too high → oscillation |
| `kd` | node L24, launch L16 | 0.4 | 0.5 | Derivative gain | More damping | Less damping, overshoots | Too high → jitter from noise |
| `front_obstacle_dist` | node L25, launch L17 | 0.8 m | 0.5 m | Emergency turn threshold | Turns earlier | Turns later, risk collision | **Launch sets 0.5, risky** |
| `front_slowdown_dist` | node L26 | 1.5 m | *(not in launch)* | Progressive deceleration start | Slows earlier | Slows later | Hardcoded in node only |
| `max_wall_search_dist` | node L27 | 3.0 m | *(not in launch)* | Wall detection range limit | Searches farther | Gives up sooner | If too small, enters "searching" mode near walls |
| `inside_ratio_threshold` | node L30 | 0.6 | *(not in launch)* | Circle detection ratio | Needs more walls visible | Triggers earlier | **Hardcoded in node only** |
| `stop_confirmation_count` | node L31 | 15 | *(not in launch)* | Frames before stop ⚠️ | Needs more confirmation | Faster stop | **MEMORY — must remove** |
| `scan_topic` | node L28, launch L18 | `scan` | `scan` | Lidar input topic | — | — | Must match bridge config |
| `cmd_vel_topic` | node L29, launch L19 | `cmd_vel` | `cmd_vel` | Motor command topic | — | — | Must match bridge config |

**Not yet parameterized (hardcoded):**
- Angular clamp `[-1.0, 1.0]` (line 162)
- Search turn rate `0.3` rad/s (line 143)
- Front obstacle turn rate `-1.0` rad/s (line 135)
- Speed scale factor `0.5` (line 165)
- Minimum speed scale `0.3` (line 165)
- Left-forward weighting `1.1` (line 129)
- Turn anticipation angular threshold `-0.3` (line 171)

---

## 11. Data Flow and Topic Reference

### Subscriptions and Publications

| Node | Subscribes | Publishes |
|------|-----------|-----------|
| `wall_follower` | `/scan` (`LaserScan`) | `/cmd_vel` (`Twist`) |
| `ros_gz_bridge` (parameter_bridge) | Gz topics | `/scan`, `/odom`, `/tf`, `/clock`, `/joint_states`, `/image_raw`, `/camera_info` |
| `robot_state_publisher` | `/joint_states` | `/tf`, `/tf_static`, `/robot_description` |

### Expected Rates

| Topic | Expected Rate | How to check |
|-------|--------------|-------------|
| `/scan` | 10 Hz | `ros2 topic hz /scan` |
| `/cmd_vel` | ~10 Hz (triggered by scan) | `ros2 topic hz /cmd_vel` |
| `/odom` | 20 Hz | `ros2 topic hz /odom` |
| `/clock` | ~1000 Hz | `ros2 topic hz /clock` |

### Debugging Commands

```bash
# See what the robot is doing right now
ros2 topic echo /cmd_vel

# See raw lidar data (720 floats)
ros2 topic echo /scan --field ranges --once

# Check lidar message structure
ros2 topic echo /scan --once | head -10

# Monitor node parameters at runtime
ros2 param list /wall_follower
ros2 param get /wall_follower desired_distance

# Dynamic parameter tuning (no restart needed)
ros2 param set /wall_follower kp 2.0
ros2 param set /wall_follower forward_speed 0.1

# View TF tree
ros2 run tf2_tools view_frames

# Record a bag for replay/analysis
ros2 bag record /scan /cmd_vel /odom -o wall_test
```

> [!NOTE]
> Dynamic parameter changes via `ros2 param set` will **not** take effect with the current code because parameters are read only in `__init__`. To enable runtime tuning, add a timer callback that periodically re-reads parameters, or use a parameter callback.

---

## 12. Testing and Validation Procedure

### Reproducible Checklist

- [ ] **Build clean:** `colcon build` succeeds without warnings
- [ ] **Launch Gazebo:** Robot appears in the world
- [ ] **Verify `/scan`:** `ros2 topic hz /scan` reports ~10 Hz
- [ ] **Launch wall_follower:** `ros2 topic hz /cmd_vel` reports ~10 Hz
- [ ] **Wall-follow test:** Robot finds left wall and follows it smoothly
- [ ] **Corner test:** Robot navigates L-wall corner without collision
- [ ] **Circle entry test:** Robot enters the circle through the gap
- [ ] **Circle detection test:** Lidar ratio crosses 60% inside circle
- [ ] **Center-seeking test:** Robot navigates toward center (after implementing)
- [ ] **Center detection test:** Non-inf readings converge to ≈equal values
- [ ] **Final heading test:** Robot faces outward (toward gap) when stopped
- [ ] **Random spawn test:** Repeat 5+ times with different random spawns

### Testing Random Initial Poses

```bash
# Run 5 trials by re-launching each time:
for i in {1..5}; do
    echo "=== Trial $i ==="
    # Kill previous Gazebo, re-launch, observe behavior
    ros2 launch andino_gz assignment1.launch.py &
    sleep 10
    ros2 launch wall_follower wall_follower.launch.py &
    sleep 120  # observe for 2 minutes
    killall -SIGINT ros2
    sleep 5
done
```

Or for deterministic debugging, hardcode the spawn in `assignment1.launch.py` line 99.

### Verifying No-Memory Compliance

Ask these questions of every state variable:
1. **Can I delete this variable and still make the same decision from the current scan?**
   - `prev_error` / `prev_time`: No (needed for PD derivative), acceptable as controller state
   - `stop_confirm_count`: Yes → **violation**
   - `is_stopped`: Yes (could recompute from ratio) → safe to remove if ratio is checked each scan

2. **If I restart the node mid-run, does it make the same decision?**
   - With current code: No (loses counter state) → proves memory exists
   - After fix: Yes (decisions based only on current `/scan`)

### Validation Metrics

| Metric | How to measure | Target |
|--------|---------------|--------|
| Wall-follow stability | Observe cmd_vel angular.z variance | Low variance = smooth following |
| Circle entry success | Visual observation | 100% (should always enter if wall-following left) |
| Centering accuracy | Echo lidar ranges at stop, compute std dev | std dev < 0.15 m |
| Mission time | Stopwatch from launch to stop | < 3 min typical |
| Collision count | Visual observation | 0 |
| Stuck events | Robot velocity = 0 for > 5s during wall-follow | 0 |

---

## 13. Known Weaknesses and Failure Modes

| Weakness | Description | Severity | Mitigation |
|----------|-------------|----------|------------|
| **Memory violation** | `stop_confirm_count` is temporal memory | Critical | Remove; use instantaneous threshold |
| **No centering logic** | Robot stops instead of seeking center | Critical | Implement as described in §7 |
| **No outward heading** | Robot stops in arbitrary orientation | Medium | Add gap-detection and rotation |
| **Noisy lidar triggers** | Single noisy scan could cause wrong mode | Medium | Use slightly higher threshold (0.65) and/or require minimum valid ray count |
| **Partial circle visibility** | Near the gap, ratio may drop below 60% | Medium | Robot may "exit" centering mode briefly; since it's reactive, it recovers |
| **Corner getting stuck** | If robot approaches L-corner head-on with wall on both sides | Low | Front obstacle detection handles this; may need tuning |
| **Overfitting to one world** | Constants tuned for `assignment1.sdf` geometry | Medium | Parameterize and test with different circle radii |
| **PD not re-read at runtime** | Parameter changes via `ros2 param set` don't take effect | Low | Add parameter callback or timer re-read |
| **`front_obstacle_dist` mismatch** | Node declares 0.8, launch sets 0.5 | Low | Consolidate defaults; 0.5 may be too close |
| **Ambiguous geometry** | Non-circular enclosures could trigger 60% threshold | Low | In this world, only the circle triggers it; for generalization, use circularity metric |

---

## 14. Recommended Next Refactors (Without Breaking Assignment Rules)

1. **Remove `stop_confirm_count` and `is_stopped`** — Replace with purely instantaneous `ratio >= threshold` check each scan. This is the #1 priority.

2. **Extract mode logic into methods** — `_wall_follow(msg)`, `_center_seek(msg)`, `_face_outward(msg)` for clarity.

3. **Parameterize all hardcoded constants** — The magic numbers on lines 135, 143, 162, 165, 171 should become ROS parameters so they can be tuned without code changes.

4. **Add a parameter callback** — So `ros2 param set` works at runtime without restarting the node.

5. **Add structured logging** — Log the current mode, ratio, mean distance, max deviation to help debug centering.

6. **Create a YAML config file** — Move all parameters out of the launch file into a `config/wall_follower_params.yaml` loaded by the launch file.

7. **Add launch argument for fixed spawn** — Allow `ros2 launch andino_gz assignment1.launch.py fixed_pose:=true` for deterministic debugging.

8. **Consolidate parameter defaults** — Remove the mismatch between `declare_parameter` defaults and launch file defaults by using only one source of truth.

---

## 15. Minimal Implementation Blueprint for the Next Step

### Goal: Replace stop-on-60% with center-seeking behavior

### Files to Edit

1. **`wall_follower/wall_follower/wall_follower_node.py`** — all changes here

### Step-by-Step

**Step 1: Remove forbidden memory (5 min)**
- Delete `stop_confirmation_count` parameter declaration (line 31)
- Delete `self.stop_confirm_count` (line 51)
- Delete `self.is_stopped` (line 50)
- Delete the early return on `is_stopped` (lines 88–90)
- Delete the counter logic block (lines 96–110)

**Step 2: Add new parameters (2 min)**
```python
self.declare_parameter('center_tolerance', 0.15)    # meters
self.declare_parameter('centering_speed', 0.08)      # m/s
self.center_tolerance = self.get_parameter('center_tolerance').value
self.centering_speed = self.get_parameter('centering_speed').value
```

**Step 3: Add helper — get valid ranges with angles (5 min)**
```python
def _get_valid_ranges_with_angles(self, msg):
    result = []
    for i, r in enumerate(msg.ranges):
        if not (math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max):
            angle = msg.angle_min + i * msg.angle_increment
            result.append((r, angle))
    return result
```

**Step 4: Add center-seeking logic in `scan_callback` (15 min)**
Replace lines 96–110 with:
```python
ratio = self._valid_reading_ratio(msg)
if ratio >= self.inside_ratio:
    # INSIDE CIRCLE — seek center
    valid = self._get_valid_ranges_with_angles(msg)
    if len(valid) >= 3:
        ranges_only = [r for r, a in valid]
        mean_dist = sum(ranges_only) / len(ranges_only)
        max_dev = max(abs(r - mean_dist) for r in ranges_only)

        if max_dev < self.center_tolerance:
            # AT CENTER — face outward and stop
            # Find gap direction: max range or inf direction
            max_r, max_a = max(valid, key=lambda x: x[0])
            # Compute angular error to face that direction
            # Robot forward = ±π in lidar frame (180° rotation)
            # Angle to gap relative to robot forward:
            heading_err = max_a - math.pi if max_a > 0 else max_a + math.pi
            if abs(heading_err) < 0.1:  # ~6° tolerance
                cmd = Twist()  # stopped, facing outward
            else:
                cmd = Twist()
                cmd.angular.z = 0.3 if heading_err > 0 else -0.3
        else:
            # NOT AT CENTER — move toward max distance
            max_r, max_a = max(valid, key=lambda x: x[0])
            heading_err = max_a - math.pi if max_a > 0 else max_a + math.pi
            speed_factor = min(max_dev / self.center_tolerance, 1.0)
            cmd = Twist()
            cmd.linear.x = self.centering_speed * speed_factor
            cmd.angular.z = max(-0.5, min(0.5, heading_err))
        self.cmd_pub.publish(cmd)
        return
# If ratio < threshold, fall through to wall-following below
```

**Step 5: Add parameters to launch file (2 min)**
In `wall_follower.launch.py`, add to the parameters dict:
```python
'center_tolerance': 0.15,
'centering_speed': 0.08,
'inside_ratio_threshold': 0.6,
```

**Step 6: Rebuild and test (5 min)**
```bash
cd ~/tri_ws && colcon build --packages-select wall_follower && source install/setup.bash
```

### Reactive State Machine Diagram

```
                    ┌──────────────────────┐
                    │    Every /scan msg    │
                    └──────────┬───────────┘
                               │
                    ┌──────────▼───────────┐
                    │ ratio = valid_ratio() │
                    └──────────┬───────────┘
                               │
                  ┌────────────▼────────────┐
                  │   ratio >= 0.6 ?        │
                  └──┬───────────────────┬──┘
                 YES │                   │ NO
        ┌────────────▼──────────┐  ┌─────▼──────────────┐
        │ CENTERING MODE        │  │ WALL-FOLLOW MODE   │
        │ (compute mean, dev)   │  │ (existing PD logic) │
        └────────┬──────────────┘  └────────────────────┘
                 │
        ┌────────▼──────────────┐
        │ max_dev < tolerance ? │
        └──┬─────────────┬─────┘
       YES │             │ NO
   ┌───────▼──────┐  ┌───▼──────────────┐
   │ FACE OUTWARD │  │ DRIVE TOWARD     │
   │ (rotate to   │  │ MAX DISTANCE     │
   │  gap, stop)  │  │ (slow approach)  │
   └──────────────┘  └──────────────────┘
```

> [!IMPORTANT]
> Every decision in this diagram is made from the **current scan only**. No state persists between callbacks except PD internals (`prev_error`, `prev_time`). The mode is recomputed each time.

---

## 16. Quickstart for a New Teammate

### Clone to Running (5 minutes)

```bash
# 1. Clone
cd ~/tri_ws/src
git clone <repo-url> TRI

# 2. Install dependencies
sudo apt install ros-jazzy-ros-gz ros-jazzy-nav2-common ros-jazzy-nav2-bringup

# 3. Build
cd ~/tri_ws
colcon build
source install/setup.bash

# 4. Launch Gazebo (Terminal 1)
ros2 launch andino_gz assignment1.launch.py

# 5. Launch wall follower (Terminal 2)
source ~/tri_ws/install/setup.bash
ros2 launch wall_follower wall_follower.launch.py
```

### Changing Behavior Safely

1. **Edit only** `wall_follower/wall_follower/wall_follower_node.py`
2. **Rebuild only** the wall_follower package: `colcon build --packages-select wall_follower`
3. **Re-source:** `source install/setup.bash`
4. **Restart only Terminal 2** — Gazebo (Terminal 1) can keep running
5. **Test with fixed spawn** first: edit `assignment1.launch.py` line 99 to set a known pose

### Debugging Quickly

```bash
# What is the robot seeing?
ros2 topic echo /scan --field ranges --once | tr ',' '\n' | head -20

# What commands is it sending?
ros2 topic echo /cmd_vel

# What is the node logging?
# (Already visible in Terminal 2 output — look for FOLLOWING/SEARCHING/etc.)

# Is the node alive?
ros2 node info /wall_follower
```

---

## 17. Appendix

### Glossary

| Term | Meaning in this project |
|------|------------------------|
| Andino | Small differential-drive robot platform by Ekumen |
| Reactive | Decisions based only on current sensor snapshot, no history |
| PD controller | Proportional-Derivative controller (no integral term) |
| Inside ratio | Fraction of 720 lidar rays that return a valid (non-inf) reading |
| Valid range | A lidar reading that is not `inf`, not `nan`, and within `[range_min, range_max]` |
| Circle gap | The 90° opening at the bottom of the circle (from −135° to −45°) |
| Center tolerance | Maximum acceptable deviation among non-inf readings to declare "at center" |
| Lidar rotation | The RPLidar is mounted rotated 180° — scan angle 0 = robot rear, ±π = robot front |
| Wall distance | Minimum reading in the left sector [−120°, −60°] combined with left-forward sector |

### Useful Command Snippets

```bash
# Rebuild single package fast
colcon build --packages-select wall_follower && source install/setup.bash

# Kill Gazebo cleanly
pkill -f gz-sim

# Fixed-pose launch for debugging (edit assignment1.launch.py line 99)
# x, y, yaw = -3.0, 3.0, 0.0

# Record and replay
ros2 bag record /scan /cmd_vel -o test_run
ros2 bag play test_run

# Plot lidar in RViz
# Already configured in andino_gz.rviz — just open RViz from Terminal 1

# Check current parameter values
ros2 param dump /wall_follower
```

### Lidar Angle Mapping Quick Reference

Since the lidar is rotated 180°:

| Scan Angle | Robot Direction | Array Index (approx) |
|------------|----------------|---------------------|
| 0 rad | **Back** | 360 |
| +π/2 rad | **Right** | 540 |
| −π/2 rad | **Left** | 180 |
| ±π rad | **Front** | 0 or 720 |

Formula: `index = (angle - angle_min) / angle_increment = (angle + π) / (2π/720)`

### Assumptions and Inferred Details

> [!WARNING]
> The following items are **inferred** from code inspection, not explicitly documented:

- **Inference:** The circle inner radius (2.0 m) is the expected distance all non-inf rays should read when the robot is at center. Actual measurements will differ due to the segmented box approximation (18 segments, not a smooth circle).
- **Inference:** The gap direction corresponds roughly to the south side of the circle (angles −135° to −45° in world frame). The robot's lidar frame is different, so gap detection must use the direction of maximum range, not a fixed angle.
- **Inference:** `front_slowdown_dist` (1.5 m) not being set in the launch file means it uses the node default. This is inconsistent with other parameters where the launch file overrides the node default.
- **Assumption:** The assignment permits PD controller state (`prev_error`, `prev_time`) as they are standard control-loop internals, not world-model memory. Verify with the instructor.
- **Unverified:** Whether `ros2 param set` dynamically updates behavior — the current code reads parameters only in `__init__`, so runtime changes have no effect without code modification.
