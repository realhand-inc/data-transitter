# Testing Guide for Linear Pipeline Implementation

## Quick Start

### 1. Launch the Script

```bash
conda activate xr-robotics
python scripts/simulation/hand_tracking_mujoco_viz.py --hardware_ip=192.168.2.2
```

**Arguments:**
- `--hardware_ip`: IP address of your UR5e robot (default: 192.168.2.2)
- `--xml_path`: Path to MuJoCo scene XML (optional)
- `--robot_urdf_path`: Path to robot URDF (optional)
- `--scale_factor`: Scaling factor for hand tracking (default: 1.5)

---

## Expected Behavior

### On Launch
You should see:
1. **MuJoCo 3D Viewer** window (robot visualization)
2. **GUI Control Panel** window (700x800 pixels)
   - Title: "Linear Pipeline Control - 192.168.2.2"
   - Blue "Connect to Robot" button
   - Mode selection buttons (disabled until connected)
   - Joint angle display table (empty until connected)

### GUI Layout
```
┌─────────────────────────────────────────────┐
│  Linear Pipeline Control - 192.168.2.2      │
├─────────────────────────────────────────────┤
│  [Connect to Robot]                         │
│                                             │
│  [Start Hand Tracking]  [Manual Control]    │
│                                             │
│  [EMERGENCY STOP]                           │
│                                             │
│  Status: Disconnected                       │
│                                             │
│  ┌── Joint Angles (degrees) ──────────┐    │
│  │ Joint │  Target  │ Actual │ Error  │    │
│  ├───────┼──────────┼────────┼────────┤    │
│  │  J0   │   ---    │  ---   │  ---   │    │
│  │  J1   │   ---    │  ---   │  ---   │    │
│  │  J2   │   ---    │  ---   │  ---   │    │
│  │  J3   │   ---    │  ---   │  ---   │    │
│  │  J4   │   ---    │  ---   │  ---   │    │
│  │  J5   │   ---    │  ---   │  ---   │    │
│  └──────────────────────────────────────┘   │
└─────────────────────────────────────────────┘
```

---

## Test Scenarios

### Test 1: Connection ✅

**Steps:**
1. Ensure UR5e robot is powered on and connected to network
2. Click "Connect to Robot" button

**Expected Result:**
- Status changes to "Status: Connected (Stopped)" (orange)
- Connect button becomes disabled and shows "Connected"
- Mode buttons become enabled
- Joint angle table populates with current robot positions
- All "Actual" columns show current joint angles
- "Target" and "Error" columns show "---"

**Console Output:**
```
Connecting to UR hardware at 192.168.2.2...
Successfully connected to UR hardware.
```

---

### Test 2: Hand Tracking Mode 🤚

**Prerequisites:** Robot connected, VR headset on

**Steps:**
1. Click "Start Hand Tracking" button
2. Move your hands in front of VR headset
3. Observe robot movement and GUI

**Expected Result:**
- Status changes to "Status: Hand Tracking ACTIVE" (green)
- Target column updates with IK solver output
- Actual column updates with robot feedback
- Error column shows difference (should be small, < 5°)
- MuJoCo viewer shows robot following your hands
- Physical robot moves to match hand poses

**What to Check:**
- [ ] Robot moves smoothly
- [ ] Target and Actual values update in real-time
- [ ] Error values remain small (< 5° typically)
- [ ] Both MuJoCo and hardware move identically
- [ ] No lag or jerky movements

**Console Output:**
```
Hand tracking control STARTED.
```

---

### Test 3: Manual Control Mode 🎛️

**Prerequisites:** Robot connected

**Steps:**
1. Click "Manual Control" button
2. Observe slider panel appears
3. Move slider for J0 (first joint)
4. Observe robot movement

**Expected Result:**
- Status changes to "Status: Manual Control ACTIVE" (purple)
- Slider panel appears below joint angle table
- Sliders initialize to current robot positions
- Moving a slider updates:
  - The slider label (e.g., "J0: 45.3°")
  - The Target column in the table
  - The robot (both MuJoCo and hardware)
- Error column shows difference between commanded and actual

**What to Check:**
- [ ] All 6 sliders appear
- [ ] Sliders range from -180° to +180°
- [ ] Moving slider updates robot immediately
- [ ] Target column matches slider value
- [ ] Robot moves smoothly to commanded position

**Console Output:**
```
Manual control STARTED.
```

---

### Test 4: Mode Switching 🔄

**Steps:**
1. Start in Hand Tracking mode (move hands around)
2. Click "Manual Control" button
3. Move a slider
4. Click "Start Hand Tracking" button again
5. Move hands

**Expected Result:**
- Smooth transitions between modes
- No sudden robot movements
- Sliders hide when switching to hand tracking
- Sliders re-appear when switching to manual
- Robot continuously controlled (no gaps)

**What to Check:**
- [ ] No jerky transitions
- [ ] Robot doesn't "jump" when switching modes
- [ ] GUI updates correctly for each mode
- [ ] Both outputs (hardware + viz) stay synchronized

---

### Test 5: Emergency Stop 🛑

**Steps:**
1. Start Hand Tracking or Manual mode
2. Get robot moving
3. Click "EMERGENCY STOP" button

**Expected Result:**
- Status changes to "Status: STOPPED" (red)
- Robot stops immediately
- Sliders disappear (if visible)
- Target column shows same values as Actual (holding position)
- Error values go to near-zero

**Console Output:**
```
Hardware control STOPPED (Emergency).
```

**What to Check:**
- [ ] Robot stops within one control cycle
- [ ] No servo errors
- [ ] Can re-start hand tracking or manual control after stop

---

### Test 6: Error Display and Color Coding ⚠️

**Steps:**
1. In Manual mode, quickly move a slider by large amount (e.g., 90°)
2. Observe the Error column

**Expected Result:**
- Error values start large (red color if > 5°)
- Error values decrease as robot catches up
- Error turns black when < 5°
- Final error settles to near-zero

**Example:**
```
Joint │  Target  │ Actual │  Error
──────┼──────────┼────────┼─────────
 J0   │  90.00°  │ 45.00° │ +45.00° (RED)   ← Initially large
 J0   │  90.00°  │ 70.00° │ +20.00° (RED)   ← Catching up
 J0   │  90.00°  │ 87.00° │  +3.00° (BLACK) ← Getting close
 J0   │  90.00°  │ 89.98° │  +0.02° (BLACK) ← Settled
```

---

## Data Flow Verification

### Verify Linear Pipeline

In Hand Tracking mode, trace the data flow:

1. **VR Headset** → Move your right hand
2. **GUI Display** → Watch "Target" column for right arm joints (J0-J5) update
3. **MuJoCo Viewer** → See virtual robot right arm move
4. **Physical Robot** → See actual robot right arm move
5. **GUI Display** → Watch "Actual" column update with feedback

**All should update simultaneously (within one control cycle ~60Hz)**

---

## Troubleshooting

### GUI doesn't appear
- Check that X11 forwarding is enabled if using SSH
- Try: `export DISPLAY=:0`

### Robot doesn't move in Hand Tracking mode
- Check VR headset is connected: `XrClient` should detect hand tracking
- Verify hand tracking quality is good (well-lit room)
- Check console for IK solver errors

### Large errors persist
- Robot may be at joint limit
- Check manipulability warnings in console
- Try different hand positions

### Hardware and MuJoCo don't match
- This indicates a bug in the linear pipeline
- Both should receive identical `q_command`
- Check console for errors in `_send_command_to_hardware()` or `_send_command_to_mujoco()`

### Sliders don't appear in Manual mode
- Check window size (may need to resize)
- Verify `control_mode == "manual"` in code

---

## Performance Metrics

**Expected Performance:**
- Control loop frequency: ~60 Hz
- Hardware servo rate: 125 Hz (8ms)
- GUI update rate: ~60 Hz
- Typical tracking error: < 2° per joint
- Mode switch latency: < 1 control cycle (~16ms)

---

## Success Criteria

✅ **All modes work correctly**
✅ **Smooth transitions between modes**
✅ **GUI displays accurate real-time data**
✅ **Hardware and visualization stay synchronized**
✅ **Emergency stop functions immediately**
✅ **Error calculations are accurate**
✅ **No crashes or exceptions during operation**

---

## Reporting Issues

If you encounter problems, provide:
1. Console output (full error messages)
2. Which test scenario failed
3. Control mode when issue occurred
4. Screenshot of GUI state
5. Joint angle values when it happened

---

## Next Steps After Testing

Once basic functionality is verified:
1. Test with dual arm configuration (both left and right)
2. Add data logging for analysis
3. Implement velocity/acceleration limits for safety
4. Add joint limit visualization in GUI
5. Consider adding trajectory recording/playback
