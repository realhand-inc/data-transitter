# Linear Pipeline Refactoring Summary

## Overview
Refactored `hand_tracking_mujoco_viz.py` to implement a **linear dataflow pipeline** with a centralized Python control interface.

---

## Key Changes

### 1. **Removed Multi-threaded Architecture** ❌
**Before:**
- Separate background thread (`control_loop`) for hardware control
- Main thread for simulation and GUI
- Complex synchronization between threads

**After:** ✅
- Single linear pipeline in main `run()` loop
- All processing happens sequentially
- Clear, predictable execution flow

---

### 2. **Unified Data Flow** 🔄

```
VR Headset
    ↓
XrClient (hand tracking)
    ↓
IK Solver → q_target
    ↓
┌─────────────────────────────────┐
│  PYTHON CONTROL INTERFACE (GUI) │  ← Decision Point
│  • Shows: target, actual, error │
│  • Modes: hand_tracking/manual  │
│  • Selects: q_command           │
└─────────────────────────────────┘
    ↓
    ├─→ Hardware Robot (RTDE)
    └─→ MuJoCo Visualization
```

---

### 3. **State Variables Restructure**

**Old Variables:**
```python
self.hardware_enabled = False
self.manual_control_enabled = False
self.latest_q = None
self.manual_q = None
self.actual_q = None
```

**New Variables:**
```python
self.control_mode = "stopped"  # "stopped" | "hand_tracking" | "manual"
self.q_target = None    # From IK solver
self.q_manual = None    # From GUI sliders
self.q_actual = None    # From hardware feedback
self.q_command = None   # Final output (selected based on mode)
```

---

### 4. **Enhanced GUI Display**

**New Features:**
- Side-by-side comparison table
- Real-time error calculation
- Color-coded warnings (red if error > 5°)

```
┌──────────────────────────────────────────────────┐
│ Joint │   Target    │   Actual    │    Error     │
├───────┼─────────────┼─────────────┼──────────────┤
│  J0   │   45.23°    │   45.10°    │   +0.13°     │
│  J1   │  -30.50°    │  -30.48°    │   -0.02°     │
│  J2   │   90.00°    │   95.20°    │   -5.20° ⚠️  │
│  ...  │    ...      │    ...      │     ...      │
└──────────────────────────────────────────────────┘
```

---

## Linear Pipeline Steps (in `run()` method)

```python
while True:
    # STEP 1: Read actual joint angles from hardware
    q_actual = rtde_r.getActualQ()

    # STEP 2: Update robot state in simulation
    _update_robot_state()

    # STEP 3: Process hand tracking and run IK solver
    _update_ik()  # XR → IK → placo_robot.state.q

    # STEP 4: Extract q_target from IK solution
    _extract_q_target_from_ik()  # placo_robot → q_target

    # STEP 5: SELECT q_command based on control mode ⚡
    if control_mode == "hand_tracking":
        q_command = q_target
    elif control_mode == "manual":
        q_command = q_manual  # from sliders
    else:  # "stopped"
        q_command = q_actual  # hold position

    # STEP 6: Send q_command to BOTH outputs
    _send_command_to_hardware()  # → UR5e robot
    _send_command_to_mujoco()    # → Visualization

    # STEP 7: Update GUI display
    _update_gui_display()  # Show target, actual, error

    # STEP 8: Step simulation and render
    mj_step() + viewer.sync()
```

---

## New Methods

### `_extract_q_target_from_ik()`
Extracts the 6-DOF arm joint angles from the IK solver's full robot state.

### `_select_command_based_on_mode()`
**The decision point** - selects which data source to use based on `control_mode`.

### `_send_command_to_hardware()`
Sends `q_command` to physical robot via RTDE `servoJ()`.

### `_send_command_to_mujoco()`
Updates MuJoCo simulation with `q_command` for visualization.

### `_update_gui_display()`
Updates the joint angle comparison table with real-time data.

---

## Benefits of Linear Architecture

✅ **Clarity**: Single execution path, easy to understand and debug
✅ **Consistency**: Both outputs (hardware + viz) receive identical commands
✅ **Safety**: Single decision point for command selection
✅ **Simplicity**: No thread synchronization issues
✅ **Transparency**: GUI shows exactly what's being sent and received

---

## Control Modes

### 1. **Stopped** 🛑
- `q_command = q_actual` (hold current position)
- Robot maintains current pose
- Safe default state

### 2. **Hand Tracking** 🤚
- `q_command = q_target` (from IK solver)
- VR hand tracking controls robot
- Continuous updates from XR device

### 3. **Manual** 🎛️
- `q_command = q_manual` (from GUI sliders)
- Direct joint angle control
- Each joint independently adjustable via slider

---

## Testing Checklist

- [x] Code compiles without syntax errors
- [ ] GUI displays correctly
- [ ] Hand tracking mode works
- [ ] Manual control mode works
- [ ] Emergency stop functions properly
- [ ] Error calculations are accurate
- [ ] Both hardware and MuJoCo receive commands
- [ ] Mode switching works seamlessly

---

## File Modified
- `scripts/simulation/hand_tracking_mujoco_viz.py` (lines 20-377)

---

## Next Steps
1. Test with VR headset connected
2. Test with UR5e hardware connected
3. Verify smooth mode transitions
4. Add data logging if needed
5. Consider adding command rate limiting for safety
