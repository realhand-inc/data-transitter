# GUI Target Angle Display Fix

## Issues Fixed

### Issue 1: Target angles not shown when robot is disconnected
**Problem**: GUI "Target" column was blank when hand tracking without robot connection.

**Root Cause**: The GUI was displaying `q_command` instead of `q_target`. When robot wasn't connected, `q_command` wasn't being updated properly.

**Solution**: Changed GUI to display `q_target` (the IK solver output) in the "Target" column, which is available regardless of robot connection status.

---

### Issue 2: Target angles not updating after clicking "Start Hand Tracking"
**Problem**: GUI target values remained static even though MuJoCo 3D visualization was working.

**Root Cause**: Same as Issue 1 - GUI was showing `q_command` which wasn't being updated in all scenarios.

**Solution**: GUI now shows `q_target` directly from the IK solver output.

---

## Changes Made

### 1. Updated `_update_gui_display()` Method

**Before:**
```python
# Display target angle
if self.q_command is not None:
    target_deg = math.degrees(self.q_command[i])
    self.joint_display_labels[i]['target'].config(text=f"{target_deg:>7.2f}°")
```

**After:**
```python
# Display target angle from IK (always show if available, even without robot)
if self.q_target is not None and i < len(self.q_target):
    target_deg = math.degrees(self.q_target[i])
    # Add indicator if velocity limited
    if self.log_data['ik_result']['velocity_limited']:
        self.joint_display_labels[i]['target'].config(
            text=f"{target_deg:>7.2f}° ⚡", fg="orange"
        )
    else:
        self.joint_display_labels[i]['target'].config(
            text=f"{target_deg:>7.2f}°", fg="black"
        )
elif self.q_command is not None and i < len(self.q_command):
    # Fallback to command if target not available (manual mode)
    target_deg = math.degrees(self.q_command[i])
    self.joint_display_labels[i]['target'].config(text=f"{target_deg:>7.2f}°", fg="black")
```

**Benefits:**
- ✅ Shows IK target even without robot connection
- ✅ Indicates when velocity limiting is active (⚡ symbol)
- ✅ Falls back to q_command in manual mode
- ✅ Color-coded: black (normal), orange (velocity limited)

---

### 2. Updated `start_control()` Method

**Before:**
```python
def start_control(self):
    if self.is_connected:
        self.control_mode = "hand_tracking"
        # ...
```

**After:**
```python
def start_control(self):
    # Can start hand tracking even without robot connection (for visualization only)
    self.control_mode = "hand_tracking"
    self.sliders_frame.pack_forget()
    if self.is_connected:
        self.status_label.config(text="Status: Hand Tracking ACTIVE", fg="green")
    else:
        self.status_label.config(text="Status: Hand Tracking (Viz Only)", fg="blue")
    print(f"Hand tracking control STARTED. Connected: {self.is_connected}")
```

**Benefits:**
- ✅ Can start hand tracking without robot connection
- ✅ Clear status message: "Hand Tracking (Viz Only)" when not connected
- ✅ Color-coded status: green (connected), blue (viz only)

---

### 3. Enhanced `_extract_q_target_from_ik()` Method

**Added:**
- Debug output every 1 second showing extracted target angles
- Better error handling and warnings
- Ensures extraction happens regardless of connection status

```python
# Debug: Print q_target occasionally (every 60 frames = 1 second)
if self._extract_counter % 60 == 0:
    print(f"[DEBUG] q_target extracted: {np.degrees(self.q_target).round(2)} deg")
```

---

### 4. Initialize `q_manual` for Disconnected Operation

**Added:**
```python
# Initialize q_manual to zeros if not connected (for manual mode without hardware)
if self.q_manual is None:
    self.q_manual = np.zeros(6)
```

**Benefits:**
- ✅ Manual mode works even without connecting to robot
- ✅ Prevents None errors in manual mode

---

## How It Works Now

### Data Flow for Target Display

```
┌──────────────────────────────────────────────────────────────┐
│ VR Headset → Hand Tracking Data                              │
└──────────────────────────────────────────────────────────────┘
                       ↓
┌──────────────────────────────────────────────────────────────┐
│ IK Solver → Calculates q_target                              │
│ (Always runs when hand tracking mode is active)              │
└──────────────────────────────────────────────────────────────┘
                       ↓
┌──────────────────────────────────────────────────────────────┐
│ _extract_q_target_from_ik() → self.q_target                  │
│ (Extracts regardless of robot connection)                    │
└──────────────────────────────────────────────────────────────┘
                       ↓
┌──────────────────────────────────────────────────────────────┐
│ GUI "Target" Column → Shows q_target                         │
│ (Updates in real-time, even without robot)                   │
└──────────────────────────────────────────────────────────────┘
                       ↓
┌──────────────────────────────────────────────────────────────┐
│ Velocity Limiting → q_command (if robot connected)           │
│ (Only affects actual robot commands, not display)            │
└──────────────────────────────────────────────────────────────┘
```

---

## Testing Scenarios

### Scenario 1: Hand Tracking Without Robot Connection ✅

**Steps:**
1. Launch script: `python scripts/simulation/hand_tracking_mujoco_viz.py`
2. Do NOT click "Connect to Robot"
3. Click "Start Hand Tracking"
4. Move hands in VR headset

**Expected Result:**
- Status: "Hand Tracking (Viz Only)" (blue)
- Target column: Updates in real-time with IK output
- Actual column: Shows "---" (no robot)
- Error column: Shows "---" (no robot)
- MuJoCo 3D viewer: Robot moves
- Console: Debug output every 1 second showing target angles

---

### Scenario 2: Hand Tracking With Robot Connection ✅

**Steps:**
1. Launch script
2. Click "Connect to Robot" (robot connected)
3. Click "Start Hand Tracking"
4. Move hands

**Expected Result:**
- Status: "Hand Tracking ACTIVE" (green)
- Target column: Shows IK output (may have ⚡ if velocity limited)
- Actual column: Shows robot feedback
- Error column: Shows difference
- Both MuJoCo and physical robot move
- Console: Debug output showing target angles

---

### Scenario 3: Velocity Limiting Indicator ✅

**Steps:**
1. In hand tracking mode (with or without robot)
2. Move hands very quickly

**Expected Result:**
- Target values show ⚡ symbol in orange
- Indicates velocity limiting is active
- Console shows: "[VELOCITY LIMIT] Max velocity exceeded..."

---

### Scenario 4: Manual Mode Without Robot ✅

**Steps:**
1. Launch script
2. Do NOT connect to robot
3. Click "Manual Control"
4. Move sliders

**Expected Result:**
- Sliders appear and work
- Target column: Shows slider values
- Actual column: "---"
- MuJoCo visualization updates

---

## GUI Display Legend

### Target Column
| Display | Meaning |
|---------|---------|
| `70.23°` | Normal IK target (black) |
| `70.23° ⚡` | IK target with velocity limiting active (orange) |
| `---` | No target data available |

### Status Bar
| Status | Color | Meaning |
|--------|-------|---------|
| Disconnected | Red | Not connected to robot |
| Connected (Stopped) | Orange | Connected but not controlling |
| Hand Tracking ACTIVE | Green | Hand tracking with robot control |
| Hand Tracking (Viz Only) | Blue | Hand tracking without robot (visualization only) |
| Manual Control ACTIVE | Purple | Manual slider control active |
| STOPPED | Red | Emergency stop activated |

---

## Debug Output

When hand tracking is active, you'll see console output every 1 second:

```
[DEBUG] q_target extracted: [ 70.73 -32.54  56.58 -70.73  37.50 -18.39] deg
```

This confirms:
- ✅ IK solver is running
- ✅ Target extraction is working
- ✅ Values are being updated

---

## Troubleshooting

### Target column still shows "---"
**Possible causes:**
1. VR headset not connected
2. Hand tracking quality too low
3. Hands not visible to cameras
4. IK solver failing

**Check:**
- Console for "[DEBUG] q_target extracted" messages
- Console for "Warning: arm_joint_indices not set up"
- MuJoCo viewer - is robot moving?

---

### Target updates but shows same values
**Possible causes:**
1. Hands not moving
2. IK solver at local minimum

**Check:**
- Move hands more dramatically
- Check MuJoCo viewer for movement
- Check log panel for hand tracking delta values

---

### ⚡ symbol always showing
**Meaning**: Your hand movements are consistently exceeding velocity limit

**Solutions:**
- Increase velocity limit: `--max_joint_velocity=1.5`
- Move hands more slowly
- Increase scale factor: `--scale_factor=2.0` (less sensitive)

---

## Files Modified

- `scripts/simulation/hand_tracking_mujoco_viz.py`
  - Lines 148-151: Initialize q_manual
  - Lines 182-190: Updated start_control()
  - Lines 340-365: Enhanced _extract_q_target_from_ik()
  - Lines 378-424: Updated _update_gui_display()

---

## Summary

✅ **Target angles now display correctly with or without robot connection**
✅ **GUI updates in real-time in hand tracking mode**
✅ **Velocity limiting is visually indicated**
✅ **Hand tracking works standalone for visualization**
✅ **Debug output helps troubleshoot issues**

The key insight: The "Target" column should show what the IK solver *wants* (q_target), not what's being *commanded* (q_command). This makes the GUI useful for debugging and monitoring even without hardware connected.
