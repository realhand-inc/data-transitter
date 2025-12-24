# Meshcat Transform Visualization Guide

## Overview

The script now supports real-time visualization of headset and hand transforms in the meshcat interface, allowing you to see exactly how the parent transform works.

---

## How to Enable

### Method 1: Command Line (Default - Already Enabled)

```bash
python scripts/simulation/teleop_dual_ur5e_mujoco_copy.py
```

Both `visualize_placo` and `visualize_transforms` are `True` by default.

### Method 2: Disable Transform Visualization

```bash
python scripts/simulation/teleop_dual_ur5e_mujoco_copy.py --no-visualize-transforms
```

### Method 3: Disable All Visualization

```bash
python scripts/simulation/teleop_dual_ur5e_mujoco_copy.py --no-visualize-placo --no-visualize-transforms
```

---

## What You'll See in Meshcat

When you run the script with visualization enabled, a browser window will open showing the meshcat viewer with the following coordinate frames:

### 1. **Headset Pose** (`headset_pose`)
- **Color:** RGB axes (Red=X, Green=Y, Blue=Z)
- **Size:** Small (0.15m axes, thin 0.005m radius)
- **What it shows:** Current headset position and orientation in world space
- **Updates:** Every frame as you move your head

**Behavior:**
- Position: Changes as you walk around
- Rotation: Changes as you rotate your head (pitch, yaw, roll)

---

### 2. **Headset Parent Transform** (`headset_parent`)
- **Color:** RGB axes
- **Size:** Medium (0.20m axes, thick 0.008m radius)
- **What it shows:** The "ghost frame" that follows your position but maintains initial yaw
- **Updates:** Every frame

**Behavior:**
- Position: Follows headset position exactly (x, y, z)
- Rotation: **FIXED at initial yaw** - does NOT rotate with your head!

**This is the key visualization:**
- When you rotate your head, you'll see `headset_pose` rotate but `headset_parent` stays facing the same direction
- When you walk, both frames move together in position

---

### 3. **Hand World Pose** (`right_hand_world`, `left_hand_world`)
- **Color:** RGB axes
- **Size:** Small (0.10m axes, 0.004m radius)
- **What it shows:** Raw hand position and orientation in world space
- **Updates:** Every frame as you move your hands

**Behavior:**
- Shows the actual hand wrist pose from the XR tracking system
- This is the "before" transform - what the hand looks like in world coordinates

---

### 4. **Hand Relative Pose** (`right_hand_relative`, `left_hand_relative`)
- **Color:** RGB axes
- **Size:** Small (0.10m axes, 0.004m radius)
- **What it shows:** Hand pose relative to the parent transform
- **Updates:** Every frame

**Behavior:**
- This is the "after" transform - what the robot actually sees
- Position and rotation are relative to `headset_parent`
- This is what gets sent to the inverse kinematics solver

---

## Understanding the Visualization

### Scenario 1: Startup
```
At startup (facing North):
  headset_pose:      position=(0,0,0), yaw=0°
  headset_parent:    position=(0,0,0), yaw=0° (LOCKED)

  Both frames are aligned!
```

### Scenario 2: Walk Forward 2m
```
After walking forward:
  headset_pose:      position=(0,0,2), yaw=0°
  headset_parent:    position=(0,0,2), yaw=0° (still locked)

  Both frames move together in position
  right_hand_relative stays the same if you didn't move your hand
```

### Scenario 3: Rotate Head 90° Left
```
After rotating (while at same position):
  headset_pose:      position=(0,0,2), yaw=90° ← rotated
  headset_parent:    position=(0,0,2), yaw=0°  ← STILL facing North!

  You'll see headset_pose pointing West
  But headset_parent still pointing North
```

### Scenario 4: Move Hand While Rotated
```
With head rotated 90° left, move hand "forward" from your perspective:
  right_hand_world:     moves in -X direction (your forward is world's left)
  right_hand_relative:  shows hand moving in parent's +Z direction

  The robot uses right_hand_relative, so it moves in the original forward direction!
```

---

## Visual Legend in Meshcat

| Frame Name | Axes Length | Axes Thickness | What It Represents |
|------------|-------------|----------------|-------------------|
| `headset_pose` | 0.15m | Thin (0.005m) | Current headset pose (updates with all motion) |
| `headset_parent` | 0.20m | **Thick (0.008m)** | Parent frame (follows position, fixed yaw) |
| `{hand}_world` | 0.10m | Very thin (0.004m) | Hand in world coordinates |
| `{hand}_relative` | 0.10m | Very thin (0.004m) | Hand relative to parent |

**Tip:** The **thick axes** indicate the parent transform - easy to spot!

---

## Color Coding (Standard RGB)

All frames use standard RGB color coding:
- **Red axis:** X-axis (left/right)
- **Green axis:** Y-axis (up/down)
- **Blue axis:** Z-axis (forward/backward)

---

## Using the Visualization for Debugging

### Check 1: Parent Frame Is Fixed
1. Start the script and note the direction of `headset_parent` (thick blue axis)
2. Rotate your head left and right
3. ✅ **Expected:** `headset_pose` rotates, but `headset_parent` blue axis stays pointing same direction
4. ❌ **Problem:** If `headset_parent` rotates with your head, check the yaw extraction code

### Check 2: Parent Frame Follows Position
1. Walk around your VR space
2. ✅ **Expected:** Both `headset_pose` and `headset_parent` move together
3. ❌ **Problem:** If `headset_parent` doesn't move, the position update is broken

### Check 3: Hand Relative Transform
1. Hold your hand in front of you
2. Rotate your body 180°
3. Keep your hand in the same position relative to your body
4. ✅ **Expected:** `{hand}_relative` should stay roughly the same
5. ✅ **Expected:** `{hand}_world` should rotate 180°

### Check 4: Robot Control Frame
1. Move your hand "forward" from your perspective
2. Rotate 90° and move hand "forward" again
3. ✅ **Expected:** Robot arm should move in the same world direction both times
4. This confirms the parent transform is working correctly

---

## Meshcat Controls

In the meshcat browser window:

- **Left click + drag:** Rotate view
- **Right click + drag:** Pan view
- **Scroll wheel:** Zoom in/out
- **Double-click on object:** Focus on it

**Useful view angles:**
- Top-down view: See yaw rotations clearly
- Side view: See position changes
- Free camera: See all transforms in 3D space

---

## Performance Notes

- Visualization updates every frame (~100 Hz)
- Minimal performance impact (just drawing coordinate frames)
- Can be disabled with `--no-visualize-transforms` if needed
- Meshcat runs in a separate browser, doesn't affect robot control

---

## Troubleshooting

### Problem: Browser doesn't open
**Solution:** Meshcat URL is printed to console, copy and open manually:
```
Placo visualization URL: http://127.0.0.1:7000/static/
```

### Problem: Frames not visible
**Solutions:**
1. Check that `visualize_placo=True` (meshcat server must be running)
2. Zoom out - frames might be outside current view
3. Check console for errors

### Problem: Transforms look incorrect
**Solutions:**
1. Check that hand tracking is active (Quest hand tracking enabled)
2. Verify initial headset yaw was recorded (check console output)
3. Try restarting - sometimes initial pose can be off

---

## Example Console Output

```bash
$ python scripts/simulation/teleop_dual_ur5e_mujoco_copy.py

Joint names in the Placo model:
  ...

Initial headset yaw recorded: 45.23°  ← Confirms parent transform initialized

Placo visualization URL: http://127.0.0.1:7000/static/  ← Open this in browser

Joint names in the Mujoco model:
  ...

right_hand is activated.  ← Hand tracking started
left_hand is activated.
```

Now you can see in the meshcat window:
- `headset_pose` and `headset_parent` at origin
- `right_hand_world` and `right_hand_relative` showing your hand
- All updating in real-time!

---

## Summary

The meshcat visualization shows you exactly how the parent transform works:
1. **headset_pose** - where your head actually is
2. **headset_parent** - the "ghost frame" (position follows, rotation fixed)
3. **hand_world** - where your hand is in the world
4. **hand_relative** - where your hand is relative to the ghost frame ← **This controls the robot**

This makes debugging and understanding the coordinate transformations much easier!
