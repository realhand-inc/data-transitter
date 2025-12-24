# XR Teleoperation Dataflow Pipeline

This document describes the complete dataflow pipeline for hand tracking teleoperation, including all coordinate transformations and the role of meshcat.

---

## High-Level Overview

```
┌─────────────┐
│  XR Device  │ (Quest, Vision Pro, etc.)
│  Hardware   │
└──────┬──────┘
       │ Raw pose data [x,y,z,qx,qy,qz,qw]
       ▼
┌─────────────────────┐
│  XRoboToolkit SDK   │
│  (xrt library)      │
└──────┬──────────────┘
       │ Pose arrays
       ▼
┌─────────────────────┐
│    XrClient         │
│ get_pose_by_name()  │
└──────┬──────────────┘
       │ Hand wrist pose, Headset pose
       ▼
┌──────────────────────────────┐
│  RemappedMujocoTeleopController │
│  _process_xr_pose()             │
│  ┌──────────────────────────┐  │
│  │ 1. Headset Parent Transform │ ◄── meshcat.transformations
│  │ 2. Pose Offset              │
│  │ 3. R_headset_world          │ ◄── meshcat.transformations
│  │ 4. Delta Calculation        │
│  └──────────────────────────┘  │
└──────┬───────────────────────────┘
       │ Delta pose (position, rotation)
       ▼
┌─────────────────────┐
│  apply_delta_pose() │ ◄── meshcat.transformations
│  (geometry.py)      │
└──────┬──────────────┘
       │ Target pose (world frame)
       ▼
┌─────────────────────┐
│  Placo IK Solver    │
│  (inverse kinematics)│
└──────┬──────────────┘
       │ Joint angles [q1, q2, ..., qn]
       ▼
┌─────────────────────┐
│  MuJoCo Simulation  │
│  or Robot Hardware  │
└─────────────────────┘
```

---

## Detailed Dataflow with Meshcat Integration

### Stage 1: XR Device → XrClient

**Input:** Raw XR device data
**Output:** Pose arrays `[x, y, z, qx, qy, qz, qw]`
**Meshcat:** Not used

```python
# In xr_client.py
def get_pose_by_name(self, name: str) -> np.ndarray | None:
    if name == "right_hand_wrist":
        hand_state = self.get_hand_tracking_state("right")
        return hand_state[1]  # Wrist joint (row 1 of 27x7 array)
    elif name == "headset":
        return xrt.get_headset_pose()
```

**Data format:**
```
[x, y, z, qx, qy, qz, qw]
 ↑       ↑
 position quaternion (x,y,z,w order from SDK)
```

---

### Stage 2: Headset Parent Transform Construction

**Location:** `RemappedMujocoTeleopController._process_xr_pose()` (lines 37-61)
**Meshcat:** `tf.quaternion_matrix()` used to create transform matrices
**Purpose:** Create a parent frame that follows headset position but maintains initial yaw orientation

```python
# Step 2a: Extract initial headset yaw (ONCE at startup)
headset_quat = np.array([
    headset_pose[6],  # w
    headset_pose[3],  # x
    headset_pose[4],  # y
    headset_pose[5],  # z
])
self.initial_headset_yaw = _extract_yaw_from_quat(headset_quat)

# Step 2b: Create parent transform matrix (EVERY frame)
T_parent = np.eye(4)
T_parent[:3, :3] = _create_yaw_rotation_matrix(self.initial_headset_yaw)  # Fixed rotation
T_parent[:3, 3] = current_headset_pos  # Current position

# Step 2c: Convert hand to relative transform
T_hand_world = tf.quaternion_matrix(hand_quat)  # ◄── MESHCAT USED HERE
T_hand_world[:3, 3] = hand_pos

T_hand_relative = np.linalg.inv(T_parent) @ T_hand_world

# Step 2d: Extract relative pose
relative_quat = tf.quaternion_from_matrix(T_hand_relative)  # ◄── MESHCAT USED HERE
```

**Transform hierarchy:**
```
World Frame
    │
    └── T_parent (headset position + initial yaw)
            │
            └── T_hand_relative (hand pose relative to parent)
```

**Meshcat functions used:**
- `tf.quaternion_matrix(quat)` - Convert quaternion [w,x,y,z] to 4x4 matrix
- `tf.quaternion_from_matrix(matrix)` - Extract quaternion from 4x4 matrix

---

### Stage 3: R_headset_world Transformation

**Location:** `BaseTeleopController._process_xr_pose()` (lines 82-113)
**Meshcat:** `tf.quaternion_from_matrix()`, `tf.quaternion_multiply()`
**Purpose:** Transform from headset coordinate frame to world coordinate frame

```python
# Step 3a: Apply R_headset_world to position
controller_xyz = self.R_headset_world @ controller_xyz

# Step 3b: Apply R_headset_world to rotation using quaternion math
R_transform = np.eye(4)
R_transform[:3, :3] = self.R_headset_world
R_quat = tf.quaternion_from_matrix(R_transform)  # ◄── MESHCAT USED HERE

controller_quat = tf.quaternion_multiply(  # ◄── MESHCAT USED HERE
    tf.quaternion_multiply(R_quat, controller_quat),
    tf.quaternion_conjugate(R_quat),
)
```

**R_headset_world matrix** (from geometry.py):
```python
R_HEADSET_TO_WORLD = np.array([
    [0, 0, -1],
    [-1, 0, 0],
    [0, 1, 0],
])
```

**Meshcat functions used:**
- `tf.quaternion_from_matrix()` - Extract rotation quaternion
- `tf.quaternion_multiply()` - Compose quaternion rotations
- `tf.quaternion_conjugate()` - Get inverse rotation

---

### Stage 4: Delta Calculation

**Location:** `BaseTeleopController._process_xr_pose()` (lines 103-113)
**Meshcat:** `quat_diff_as_angle_axis()` which uses meshcat internally
**Purpose:** Calculate movement delta from reference pose

```python
# First call: store reference
if self.ref_controller_xyz[src_name] is None:
    self.ref_controller_xyz[src_name] = controller_xyz
    self.ref_controller_quat[src_name] = controller_quat
    delta_xyz = np.zeros(3)
    delta_rot = np.array([0.0, 0.0, 0.0])
else:
    # Calculate deltas
    delta_xyz = (controller_xyz - self.ref_controller_xyz[src_name]) * self.scale_factor
    delta_rot = quat_diff_as_angle_axis(  # ◄── Uses meshcat internally
        self.ref_controller_quat[src_name],
        controller_quat
    )

return delta_xyz, delta_rot
```

**Output format:**
```
delta_xyz: [dx, dy, dz] in meters (scaled)
delta_rot: [ax*angle, ay*angle, az*angle] in radians (angle-axis)
```

---

### Stage 5: Apply Delta to Target Pose

**Location:** `BaseTeleopController._update_ik()` (lines 221-235)
**Meshcat:** `tf.quaternion_matrix()` via `apply_delta_pose()`
**Purpose:** Apply movement delta to end-effector reference pose

```python
# For full pose control
target_xyz, target_quat = apply_delta_pose(
    self.ref_ee_xyz[src_name],
    self.ref_ee_quat[src_name],
    delta_xyz,
    delta_rot,
)

# Convert to 4x4 matrix for Placo
target_pose = tf.quaternion_matrix(target_quat)  # ◄── MESHCAT USED HERE
target_pose[:3, 3] = target_xyz
```

**Inside apply_delta_pose()** (geometry.py):
```python
def apply_delta_pose(ref_xyz, ref_quat, delta_xyz, delta_rot):
    # Position: simple addition
    target_xyz = ref_xyz + delta_xyz

    # Rotation: convert angle-axis to quaternion and compose
    delta_angle = np.linalg.norm(delta_rot)
    if delta_angle > 1e-6:
        delta_axis = delta_rot / delta_angle
        delta_quat = tf.quaternion_about_axis(delta_angle, delta_axis)  # ◄── MESHCAT
        target_quat = tf.quaternion_multiply(delta_quat, ref_quat)  # ◄── MESHCAT
    else:
        target_quat = ref_quat

    return target_xyz, target_quat
```

**Meshcat functions used:**
- `tf.quaternion_matrix()` - Convert quaternion to 4x4 pose matrix
- `tf.quaternion_about_axis()` - Create quaternion from axis-angle
- `tf.quaternion_multiply()` - Compose rotations

---

### Stage 6: Placo IK Solver

**Location:** `BaseTeleopController._update_ik()` (line 246)
**Meshcat:** Not directly used (Placo has its own internal math)
**Input:** Target pose (4x4 matrix)
**Output:** Joint angles

```python
# Set target for each end-effector
self.effector_task[src_name].T_world_frame = target_pose

# Solve inverse kinematics
self.solver.solve(True)

# Result stored in: self.placo_robot.state.q
```

---

### Stage 7: Send Commands to Robot

**Location:** `MujocoTeleopController._send_command()` (lines 93-116)
**Meshcat:** `tf.quaternion_from_matrix()` for visualization
**Purpose:** Convert joint angles to robot commands

```python
# Get desired joint positions from Placo
qpos_desired = calc_mujoco_qpos_from_placo_q(
    self.mj_model,
    self.placo_robot,
    self.placo_robot.state.q,
    floating_base=self.floating_base,
)

# Set MuJoCo control
self.mj_data.ctrl = calc_mujoco_ctrl_from_qpos(self.mj_model, qpos_desired)

# Update visualization targets
for name, task in self.effector_task.items():
    T_world_target = task.T_world_frame
    self.mj_data.mocap_pos[mocap_idx] = T_world_target[:3, 3]
    self.mj_data.mocap_quat[mocap_idx] = tf.quaternion_from_matrix(T_world_target)  # ◄── MESHCAT
```

---

## Complete Meshcat Usage Map

| Stage | Function | Meshcat Function | Purpose |
|-------|----------|------------------|---------|
| **2. Headset Parent** | `_process_xr_pose()` | `tf.quaternion_matrix()` | Convert quaternion to 4x4 matrix |
| **2. Headset Parent** | `_process_xr_pose()` | `tf.quaternion_from_matrix()` | Extract quaternion from matrix |
| **3. Coordinate Transform** | `_process_xr_pose()` | `tf.quaternion_from_matrix()` | Extract rotation from R_headset_world |
| **3. Coordinate Transform** | `_process_xr_pose()` | `tf.quaternion_multiply()` | Apply rotation transformation |
| **3. Coordinate Transform** | `_process_xr_pose()` | `tf.quaternion_conjugate()` | Get inverse rotation |
| **4. Delta Calculation** | `quat_diff_as_angle_axis()` | `tf.quaternion_inverse()` | Calculate rotation difference |
| **5. Apply Delta** | `apply_delta_pose()` | `tf.quaternion_about_axis()` | Convert angle-axis to quaternion |
| **5. Apply Delta** | `apply_delta_pose()` | `tf.quaternion_multiply()` | Compose rotations |
| **5. Apply Delta** | `_update_ik()` | `tf.quaternion_matrix()` | Convert to 4x4 for Placo |
| **7. Visualization** | `_update_mocap_target()` | `tf.quaternion_from_matrix()` | Extract quat for MuJoCo mocap |

---

## Coordinate Frames Diagram

```
┌──────────────────────────────────────────────────────────────┐
│                        World Frame                           │
│                            (0,0,0)                           │
└────────────────────┬─────────────────────────────────────────┘
                     │
        ┌────────────┴────────────┐
        │                         │
   ┌────▼─────┐            ┌──────▼──────┐
   │ T_parent │            │ Robot Base  │
   │ (headset)│            │   Frame     │
   └────┬─────┘            └─────────────┘
        │
        │ T_parent = [current_headset_pos, initial_headset_yaw]
        │            ↑                    ↑
        │            Updates              Fixed at startup
        │
   ┌────▼──────────┐
   │ T_hand_relative│ ← Hand pose relative to parent
   └───────────────┘
        │
        │ Transformed by R_headset_world
        │
   ┌────▼──────────┐
   │ Transformed    │
   │ Hand Pose      │
   └───────────────┘
        │
        │ Calculate delta from reference
        │
   ┌────▼──────────┐
   │ Delta Pose    │ → Applied to robot end-effector target
   └───────────────┘
```

---

## Data Type Transformations

```
XR SDK Format:        [x, y, z, qx, qy, qz, qw]
                             ↓
Reorder to meshcat:   [x, y, z, qw, qx, qy, qz]
                             ↓ tf.quaternion_matrix()
4x4 Matrix:           [[R11, R12, R13, x  ],
                       [R21, R22, R23, y  ],
                       [R31, R32, R33, z  ],
                       [0,   0,   0,   1  ]]
                             ↓ Matrix math
Transformed Matrix:   [[R'11, R'12, R'13, x' ],
                       [R'21, R'22, R'23, y' ],
                       [R'31, R'32, R'33, z' ],
                       [0,    0,    0,    1  ]]
                             ↓ tf.quaternion_from_matrix()
Meshcat Format:       [qw, qx, qy, qz]
                             ↓
Reorder to XR:        [qx, qy, qz, qw]
```

---

## Key Quaternion Convention Differences

| Library | Quaternion Order | Notes |
|---------|-----------------|-------|
| **XRoboToolkit SDK** | `[qx, qy, qz, qw]` | Scalar last (common in robotics) |
| **meshcat.transformations** | `[qw, qx, qy, qz]` | Scalar first (Hamilton convention) |
| **MuJoCo** | `[qw, qx, qy, qz]` | Scalar first (matches meshcat) |

**⚠️ Important:** Always reorder when converting between XR SDK and meshcat!

```python
# XR SDK → Meshcat
xr_quat = [qx, qy, qz, qw]
meshcat_quat = [xr_quat[3], xr_quat[0], xr_quat[1], xr_quat[2]]  # [qw, qx, qy, qz]

# Meshcat → XR SDK
meshcat_quat = [qw, qx, qy, qz]
xr_quat = [meshcat_quat[1], meshcat_quat[2], meshcat_quat[3], meshcat_quat[0]]  # [qx, qy, qz, qw]
```

---

## Summary

**Meshcat's Role:**
- Provides robust quaternion and 4x4 matrix operations
- Handles coordinate frame transformations
- Ensures mathematical correctness in rotation composition
- Used throughout the pipeline for all rotation math

**Data Flow:**
1. XR Device → Raw pose
2. XrClient → Standardized pose array
3. **Headset Parent Transform** (meshcat) → Relative pose
4. **R_headset_world Transform** (meshcat) → World-aligned pose
5. **Delta Calculation** (meshcat) → Movement delta
6. **Apply Delta** (meshcat) → Target pose
7. Placo IK → Joint angles
8. Robot → Execute motion
