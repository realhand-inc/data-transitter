# UR5e Dual Arm Teleoperation Dataflow

This document outlines the dataflow pipeline and data structures for the `teleop_dual_ur5e_hardware.py` script, covering the flow from XR input to robot hardware control.

## System Overview

The system controls two Universal Robots UR5e arms and Robotiq grippers using inputs from an XR device (VR controllers). It utilizes a multi-threaded architecture to separate high-frequency hardware control from IK (Inverse Kinematics) calculations.

### Core Components

1.  **XrClient (`XrClient`)**: Interface to the XR SDK, providing controller poses and input states.
2.  **DualArmURController (`DualArmURController`)**: Main teleoperation class managing IK, state mapping, and visualization.
3.  **URController (`URController`)**: Low-level hardware interface for each arm using `rtde_control` and `rtde_receive`.
4.  **Placo (`placo`)**: Kinematics library used for IK solving.

---

## Dataflow Pipeline

### 1. Input Stage (XR Device)
**Frequency**: Polled in the IK thread.
*   **Source**: `XrClient` interacts with `xrobotoolkit_sdk`.
*   **Data**:
    *   **Pose**: 7D vector `[x, y, z, qx, qy, qz, qw]` representing controller position and orientation.
    *   **Inputs**: Float values `[0.0, 1.0]` for `left_grip`/`right_grip` (activation) and `left_trigger`/`right_trigger` (gripper).

### 2. Logic & Processing Stage (`DualArmURController`)
**Thread**: `run_ik_thread` (calculates targets)
1.  **State Sync**:
    *   Reads actual joint positions from the physical robots.
    *   Updates the `placo` internal robot model state `placo_robot.state.q`.
2.  **Input Mapping**:
    *   **Activation**: Checks if grip value > `CONTROLLER_DEADZONE`.
    *   **Pose Delta**: Calculates the change in pose relative to the controller pose at the moment of activation (`_process_xr_pose`).
    *   **Transform**: Applies `R_headset_world` rotation and a `scale_factor` to the position delta.
3.  **Inverse Kinematics (IK)**:
    *   Sets the target frame for the end-effector task in `placo`.
    *   Solves for target joint angles: `target_left_q`, `target_right_q`.
4.  **Visualization**:
    *   Updates Meshcat viewer with the solved state if enabled.

### 3. Output Stage (Hardware Control)
**Threads**: `run_left_controller_thread`, `run_right_controller_thread`
**Frequency**: ~500Hz (driven by `servo_time` wait period).
*   **Robot Control**:
    *   Reads `target_left_q` / `target_right_q`.
    *   Sends `servoJ` command via RTDE to the UR controller.
*   **Gripper Control**:
    *   Maps trigger value to gripper position (0-255).
    *   Sends move command to Robotiq gripper.

---

## Data Structures

### Robot Configuration (`manipulator_config`)
A dictionary defining how XR inputs map to robot links.
```python
{
    "left_arm": {
        "link_name": "left_tool0",       # URDF link to control
        "pose_source": "left_controller", # XR pose source name
        "control_trigger": "left_grip",   # Button to activate control
        "gripper_trigger": "left_trigger" # Button to control gripper
    },
    # ... right_arm config
}
```

### Pose Data
*   **Format**: `numpy.ndarray` (shape: 7)
*   **Order**: `[tx, ty, tz, qx, qy, qz, qw]`
*   **Usage**: Used for raw XR poses and internal frame tasks.

### Joint Data
*   **Format**: `numpy.ndarray` (shape: 6)
*   **Unit**: Radians
*   **Order**: Base, Shoulder, Elbow, Wrist 1, Wrist 2, Wrist 3 (Standard UR ordering).

### Threading Model
*   **Main Thread**: Initial setup and cleanup.
*   **Head Thread**: Controls the Dynamixel head (if equipped).
*   **Left/Right Arm Threads**: Dedicated loops for sending servo commands to ensure smooth motion.
*   **IK Thread**: Continuous loop for reading inputs and solving IK.

---

## File Structure Reference

*   **Entry Point**: `scripts/hardware/teleop_dual_ur5e_hardware.py`
*   **Controller Logic**: `xrobotoolkit_teleop/hardware/dual_arm_ur_controller.py`
*   **Hardware Interface**: `xrobotoolkit_teleop/hardware/interface/universal_robots.py`
*   **XR Client**: `xrobotoolkit_teleop/common/xr_client.py`
