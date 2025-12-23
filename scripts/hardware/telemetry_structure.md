# Telemetry Data Structure (JSON over ZMQ)

This document defines the JSON payload structure broadcast by the `teleop_dual_ur5e_hardware.py` script.

**Protocol:** ZeroMQ (PUB-SUB)
**Format:** JSON String
**Default Port:** 5555

## Root Object

| Field | Type | Description |
| :--- | :--- | :--- |
| `timestamp` | `float` | Unix timestamp (seconds) of the packet creation. |
| `left` | `object` | State data for the Left Arm. |
| `right` | `object` | State data for the Right Arm. |

---

## Arm Object (`left` / `right`)

| Field | Type | Description |
| :--- | :--- | :--- |
| `tcp_pose` | `array[7]` | The Tool Center Point Pose in World Frame. |
| `joints` | `array[6]` | The 6 joint angles in radians. |
| `gripper` | `float` | Gripper state (0.0 = Open, 1.0 = Closed). |

### Field Details

#### 1. `tcp_pose` (7-Float Array)
The position and orientation of the end-effector (TCP).
*   **Indices 0-2 (Position):** `[x, y, z]` in meters.
*   **Indices 3-6 (Orientation):** `[qx, qy, qz, qw]` (Quaternion).

#### 2. `joints` (6-Float Array)
The angular position of each motor in **Radians**.
Order: `[Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]`

#### 3. `gripper` (Float)
A normalized value representing the gripper status.
*   `0.0`: Fully Open
*   `1.0`: Fully Closed
*   *(Note: Intermediate values like 0.5 are possible if the gripper supports analog feedback)*

---

## Example Payload

```json
{
  "timestamp": 1734912345.123456,
  "left": {
    "tcp_pose": [
      0.45, 0.12, 0.33,      // Position (x, y, z)
      0.0, 0.707, 0.0, 0.707 // Quaternion (qx, qy, qz, qw)
    ],
    "joints": [
      0.0, -1.57, 1.57, -1.57, -1.57, 0.0
    ],
    "gripper": 0.0
  },
  "right": {
    "tcp_pose": [
      0.45, -0.12, 0.33,
      0.0, -0.707, 0.0, 0.707
    ],
    "joints": [
      0.0, -1.57, 1.57, -1.57, -1.57, 0.0
    ],
    "gripper": 1.0
  }
}
```
