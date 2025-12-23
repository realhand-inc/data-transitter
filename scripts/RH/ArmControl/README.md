# UR5e Dual Arm Hand Tracking Control

Real-time hand tracking control for dual UR5e robot arms with MuJoCo visualization, speed limiting, and comprehensive data logging.

---

## Overview

This script provides:
- ✅ **Hand tracking control** - Control robot arms with VR hand movements
- ✅ **MuJoCo 3D visualization** - Real-time physics simulation
- ✅ **Velocity limiting** - Safe speed limits for robot protection
- ✅ **Real-time logging** - Monitor hand tracking, IK solver, and control data
- ✅ **GUI control panel** - Monitor and control robot status
- ✅ **Hardware or visualization only** - Works with or without physical robot

---

## Installation

### 1. Create Virtual Environment (Conda - Recommended)

```bash
# From project root
cd /home/richard/data-transitter

# Create conda environment
./setup_conda.sh --conda xr-robotics

# Activate environment
conda activate xr-robotics

# Install dependencies
./setup_conda.sh --install
```

### 2. Alternative: System Installation

```bash
# If not using conda
./setup.sh
```

### 3. Verify Installation

```bash
python -c "import mujoco, placo, rtde_control; print('✅ All dependencies installed')"
```

---

## Quick Start

### Run with Visualization Only (No Robot)

```bash
python hand_tracking_mujoco_viz.py
```

- Don't click "Connect to Robot"
- Click "Start Hand Tracking"
- Move hands in VR headset
- Watch MuJoCo 3D visualization

### Run with Physical Robot

```bash
python hand_tracking_mujoco_viz.py --hardware_ip=192.168.2.2
```

- Click "Connect to Robot"
- Click "Start Hand Tracking"
- Robot follows hand movements

---

## Command Line Options

```bash
python hand_tracking_mujoco_viz.py [OPTIONS]
```

| Option | Default | Description |
|--------|---------|-------------|
| `--hardware_ip` | `192.168.2.2` | IP address of left UR5e robot |
| `--max_joint_velocity` | `1.0` | Max joint velocity (rad/s) |
| `--scale_factor` | `1.5` | Hand movement scaling factor |
| `--visualize_placo` | `True` | Enable Placo web visualizer |

### Examples

**Conservative speed (safer):**
```bash
python hand_tracking_mujoco_viz.py --max_joint_velocity=0.5
```

**Faster movement:**
```bash
python hand_tracking_mujoco_viz.py --max_joint_velocity=1.5
```

**Different robot IP:**
```bash
python hand_tracking_mujoco_viz.py --hardware_ip=192.168.2.3
```

---

## Dataflow Pipeline

### Complete Linear Pipeline

```
┌─────────────────────────────────────────────────────────────┐
│ 1. VR Headset (Hand Tracking)                               │
│    - Captures left/right hand wrist poses                   │
│    - 7-DOF: position (xyz) + orientation (quaternion)       │
└────────────────────────────┬────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│ 2. XrClient (VR Interface)                                  │
│    - get_pose_by_name("left_hand_wrist")                    │
│    - get_pose_by_name("right_hand_wrist")                   │
└────────────────────────────┬────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│ 3. Pose Processing                                          │
│    - Transform to world coordinates                         │
│    - Calculate delta from reference pose                    │
│    - Scale by scale_factor                                  │
└────────────────────────────┬────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│ 4. Placo IK Solver                                          │
│    - Update frame tasks with target poses                   │
│    - solver.solve() → placo_robot.state.q                   │
│    - Output: Full robot state (all joint angles)            │
└────────────────────────────┬────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│ 5. Extract q_target                                         │
│    - Extract 6 arm joints from full robot state             │
│    - q_target = placo_robot.state.q[arm_joint_indices]      │
└────────────────────────────┬────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────┐
│ 6. Python Control Interface (GUI) ⚡ DECISION POINT         │
│                                                             │
│    Mode Selection:                                          │
│    • Hand Tracking → q_command = q_target                   │
│    • Manual        → q_command = q_manual (from sliders)    │
│    • Stopped       → q_command = q_actual (hold position)   │
│                                                             │
│    Velocity Limiting:                                       │
│    • Calculate velocity: (q_command - q_prev) / dt          │
│    • If velocity > limit: scale down proportionally         │
└────────────────────────────┬────────────────────────────────┘
                             │
                             │ q_command (final output)
                             │
             ┌───────────────┴───────────────┐
             │                               │
             ▼                               ▼
┌─────────────────────┐         ┌─────────────────────┐
│  Hardware Robot     │         │  MuJoCo Simulation  │
│  (UR5e via RTDE)    │         │  (3D Visualization) │
│                     │         │                     │
│  • rtde_c.servoJ()  │         │  • Update qpos      │
│  • 8ms cycle (125Hz)│         │  • mj_step()        │
│  • Servo control    │         │  • viewer.sync()    │
└──────────┬──────────┘         └─────────────────────┘
           │
           │ q_actual (feedback)
           │
           ▼
┌─────────────────────────────────────────────────────────────┐
│ 7. GUI Display & Logging                                    │
│    - Joint angle table (Target, Actual, Error)              │
│    - Data logging panel (Hand tracking, IK, Control)        │
│    - Real-time updates (60 Hz)                              │
└─────────────────────────────────────────────────────────────┘
```

### Key Data Variables

| Step | Variable | Type | Description |
|------|----------|------|-------------|
| 1-2 | `xr_pose` | [7x1] | Hand pose from VR headset |
| 3 | `delta_xyz, delta_rot` | [3x1], [3x1] | Movement deltas |
| 4 | `placo_robot.state.q` | [Nx1] | Full robot state from IK |
| 5 | `q_target` | [6x1] | Target joint angles for one arm |
| 6 | `q_command` | [6x1] | Final command (velocity limited) |
| 7 | `q_actual` | [6x1] | Actual joint angles from robot |

---

## GUI Interface

### Window Layout

```
┌─────────────────────────────────────────────────────────┐
│ Linear Pipeline Control - 192.168.2.2            [×]    │
├─────────────────────────────────────────────────────────┤
│  [Connect to Robot]                                     │
│  [Start Hand Tracking]  [Manual Control]                │
│  [EMERGENCY STOP]                                       │
│  Status: Hand Tracking ACTIVE                           │
├─────────────────────────────────────────────────────────┤
│  ┌─ Joint Angles (degrees) ───────────────────┐         │
│  │ Joint │ Target  │ Actual │ Error           │         │
│  │ ─────┼─────────┼────────┼────────          │         │
│  │  J0  │ 70.73°  │ 70.70° │ +0.03°          │         │
│  │  J1  │-32.54°  │-32.55° │ +0.01°          │         │
│  │  ... │   ...   │  ...   │  ...            │         │
│  └─────────────────────────────────────────────┘         │
├─────────────────────────────────────────────────────────┤
│  ┌─ Data Logging ──────────────────────────────┐        │
│  │ [HAND TRACKING INPUT]                       │        │
│  │   LEFT HAND:                                │        │
│  │     Position: [0.12, 0.56, -0.34]           │        │
│  │                                             │        │
│  │ [IK SOLVER OUTPUT]                          │        │
│  │   Target Joints: [70.73, -32.54, ...]      │        │
│  │   Max Velocity: 0.12 rad/s (limit: 1.0)    │        │
│  │                                             │        │
│  │ [CONTROL OUTPUT]                            │        │
│  │   Command Joints: [70.73, -32.54, ...]     │        │
│  │   Actual Joints:  [70.70, -32.55, ...]     │        │
│  │   Error: [+0.03, +0.01, ...]               │        │
│  └─────────────────────────────────────────────┘        │
└─────────────────────────────────────────────────────────┘
```

### Status Indicators

| Status | Color | Meaning |
|--------|-------|---------|
| Disconnected | Red | Not connected to robot |
| Connected (Stopped) | Orange | Connected but not controlling |
| Hand Tracking ACTIVE | Green | Hand tracking with robot control |
| Hand Tracking (Viz Only) | Blue | Hand tracking without robot |
| Manual Control ACTIVE | Purple | Manual slider control |
| STOPPED | Red | Emergency stop |

### Target Column Indicators

| Display | Meaning |
|---------|---------|
| `70.23°` | Normal IK target |
| `70.23° ⚡` | Velocity limited (orange) |
| `---` | No data |

---

## Control Modes

### 1. Hand Tracking Mode 🤚

**Activate:** Click "Start Hand Tracking"

**Behavior:**
- Robot continuously follows hand movements
- IK solver runs in real-time
- Velocity limiting applied
- Both hands control independently

**Without Robot:**
- MuJoCo visualization works
- GUI shows target angles
- Status: "Hand Tracking (Viz Only)"

### 2. Manual Mode 🎛️

**Activate:** Click "Manual Control" (requires robot connection)

**Behavior:**
- 6 sliders appear (one per joint)
- Direct joint angle control
- Velocity limiting applied
- Real-time position updates

### 3. Stopped Mode 🛑

**Activate:** Click "EMERGENCY STOP"

**Behavior:**
- Robot holds current position
- All movement stops
- Can resume hand tracking or manual mode

---

## Velocity Limiting

### How It Works

```python
# Calculate velocity for each joint
velocity = (q_desired - q_previous) / dt

# Find maximum velocity
max_vel = max(abs(velocity))

# If exceeds limit, scale proportionally
if max_vel > max_joint_velocity:
    scale = max_joint_velocity / max_vel
    q_command = q_previous + velocity * scale * dt
```

### Safety Values

| Speed | rad/s | deg/s | Use Case |
|-------|-------|-------|----------|
| Conservative | 0.3-0.5 | 17-29 | Testing, learning |
| **Normal** | **0.8-1.2** | **46-69** | **Default** |
| Aggressive | 1.5-2.0 | 86-115 | Experienced users |
| UR5e Max | ~3.14 | ~180 | Hardware limit |

### When Limiting Activates

- Target column shows `⚡` symbol in orange
- Console prints: `[VELOCITY LIMIT] Max velocity exceeded...`
- Robot moves in same direction, but slower

---

## Data Logging

### Console Output (Terminal)

**Every 1 second:**
```
[DEBUG] q_target extracted: [ 70.73 -32.54  56.58 -70.73  37.50 -18.39] deg
```

**When velocity limited:**
```
[VELOCITY LIMIT] Max velocity exceeded in hand_tracking mode
```

### GUI Log Panel

Updated in real-time (60 Hz):

**Hand Tracking Input:**
- Raw hand poses from VR
- Position and orientation deltas
- Separate for left and right hands

**IK Solver Output:**
- Target joint angles (rad and deg)
- Joint velocities
- Velocity limiting status

**Control Output:**
- Command joints (what's being sent)
- Actual joints (hardware feedback)
- Error per joint
- RMS error

---

## Troubleshooting

### Issue: GUI Target Column Shows "---"

**Symptoms:**
- MuJoCo 3D visualization works
- [HAND TRACKING INPUT] updates in logs
- [CONTROL OUTPUT] doesn't update

**Fix:**
Check console for:
```
ERROR: No left arm joints found! Hardware control will not work.
Warning: arm_joint_indices not set up. Target extraction disabled.
```

**Solution:** Already fixed in this version! The script now uses correct Placo API (`get_joint_offset()` instead of `get_joint_q_offset()`).

---

### Issue: Velocity Always Limited

**Symptoms:**
- Every target shows `⚡` symbol
- Constant `[VELOCITY LIMIT]` messages

**Solutions:**
```bash
# Increase limit
python hand_tracking_mujoco_viz.py --max_joint_velocity=1.5

# Or decrease sensitivity
python hand_tracking_mujoco_viz.py --scale_factor=1.0
```

---

### Issue: Robot Doesn't Connect

**Check:**
1. Robot powered on?
2. Network cable connected?
3. Correct IP address?
4. Firewall blocking port 30001/30002?

**Test connection:**
```bash
ping 192.168.2.2
```

---

### Issue: Hand Tracking Not Detected

**Check:**
1. VR headset connected?
2. XRoboToolkit SDK running?
3. Hand tracking enabled in VR settings?
4. Hands visible to cameras?

**Debug:**
- Check if [HAND TRACKING INPUT] updates in log panel
- If empty, VR data not flowing

---

## Hardware Setup

### Network Configuration

**Left UR5e:**
- IP: 192.168.2.2
- Subnet: 255.255.255.0
- Gateway: 192.168.2.1

**Right UR5e:**
- IP: 192.168.2.3
- Subnet: 255.255.255.0
- Gateway: 192.168.2.1

**Control PC:**
- IP: 192.168.2.100 (recommended)
- Same subnet as robots

### Robot Configuration

1. **Enable External Control:**
   - Robot → Settings → System → Remote Control
   - Check "Enable"

2. **Safety Configuration:**
   - Set appropriate speed/force limits
   - Configure safety zones
   - Test emergency stop

3. **Home Position:**
   ```python
   # Defined in main() function
   # Adjust as needed for your setup
   ```

---

## Performance

| Metric | Target | Typical |
|--------|--------|---------|
| Control Loop | 60 Hz | 58-62 Hz |
| Hardware Servo | 125 Hz | 125 Hz |
| GUI Update | 60 Hz | 55-60 Hz |
| IK Solve Time | <16 ms | 5-10 ms |
| Tracking Error | <2° | 0.5-1.5° |

---

## File Structure

```
scripts/RH/ArmControl/
├── hand_tracking_mujoco_viz.py    # Main script
└── README.md                       # This file
```

---

## Dependencies

### Core
- Python 3.10+
- NumPy
- Tkinter (GUI)

### Robotics
- MuJoCo (physics simulation)
- Placo 0.9.4 (inverse kinematics)
- ur_rtde (UR robot interface)

### XR
- XRoboToolkit SDK (VR interface)

### Optional
- placo_utils (visualization)
- meshcat (web visualization)

---

## Related Scripts

In `scripts/RH/`:
- `hand_tracking_ur5e_gui.py` - Single arm hand tracking
- `ur_joint_control_gui.py` - Manual joint control only
- `hand_tracking_teleop/` - Alternative implementations

---

## Credits

- **Framework:** XRoboToolkit SDK for VR interface
- **IK Solver:** Placo library
- **Simulation:** MuJoCo physics engine
- **Robot Interface:** Universal Robots RTDE

---

## License

See project root LICENSE file.

---

## Support

For issues:
1. Check troubleshooting section above
2. Verify installation with test commands
3. Check console output for errors
4. Review dataflow pipeline to identify where data stops flowing

**Console debug output is your friend!** 🐛
