# XRoboToolkit Teleop Setup and Data Reference

## Table of Contents
- [Installation on New Devices](#installation-on-new-devices)
- [Running the Project](#running-the-project)
- [XR Device Data Structures](#xr-device-data-structures)
- [Troubleshooting](#troubleshooting)

---

## Installation on New Devices

### Prerequisites
- **Operating System**: Ubuntu 22.04 or Ubuntu 24.04 (tested)
- **Python**: Python 3.10 or higher
- **XRoboToolkit PC Service**: Must be downloaded and installed separately

### Step 1: Install System Dependencies

```bash
# Update package lists
sudo apt update

# Install required system packages
sudo apt install -y cmake python3-pip python-is-python3 python3.12-venv

# Optional: Install git if not already installed
sudo apt install -y git
```

### Step 2: Clone the Repository

```bash
git clone https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python.git
cd XRoboToolkit-Teleop-Sample-Python
```

### Step 3: Choose Installation Method

#### Option A: Conda Environment (Recommended)

```bash
# Install Miniconda first if not installed
# Download from: https://docs.conda.io/en/latest/miniconda.html

# Create and activate conda environment
bash setup_conda.sh --conda xr-robotics
conda activate xr-robotics

# Install dependencies
bash setup_conda.sh --install
```

#### Option B: Python Virtual Environment

```bash
# Create virtual environment
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate

# Install pybind11 with CMake support
pip install "pybind11[global]"

# Install XRoboToolkit SDK
cd dependencies/XRoboToolkit-PC-Service-Pybind
pip install .
cd ../..

# Install ARX R5 SDK
cd dependencies/R5/py/ARX_R5_python
pip install .
cd ../../../..

# Install main project
pip install -e .
```

### Step 4: Download XRoboToolkit PC Service

1. Download XRoboToolkit PC Service from the official website
2. Install and run the service before running any demos
3. Ensure your XR device is connected

---

## Running the Project

### Activating the Environment

**For Conda:**
```bash
conda activate xr-robotics
```

**For Virtual Environment:**
```bash
source venv/bin/activate
```

### Available Demos

#### 1. XR Data Display (Debug Tool)
Display real-time XR device data for debugging and understanding coordinate systems:

```bash
python scripts/misc/xr_data_display.py
```

Press `Ctrl+C` to exit.

#### 2. MuJoCo Simulation Demos

**Dual UR5e Arms:**
```bash
python scripts/simulation/teleop_dual_ur5e_mujoco.py
```

**ARX X7S:**
```bash
python scripts/simulation/teleop_x7s_placo.py
```

**Shadow Hand:**
```bash
python scripts/simulation/teleop_shadow_hand_mujoco.py
```

**Flexiv Rizon4s:**
```bash
python scripts/simulation/teleop_flexiv_rizon4s_mujoco.py
```

#### 3. Placo Visualization Demos

**ARX X7S:**
```bash
python scripts/simulation/teleop_x7s_placo.py
```

**Inspire Hand:**
```bash
python scripts/simulation/teleop_inspire_hand_placo.py
```

#### 4. Hardware Demos

**Dual UR5e with Robotiq Grippers:**
```bash
# Normal operation
python scripts/hardware/teleop_dual_ur5e_hardware.py

# Reset arms to home position
python scripts/hardware/teleop_dual_ur5e_hardware.py --reset

# Visualize IK with Placo
python scripts/hardware/teleop_dual_ur5e_hardware.py --visualize_placo
```

**ARX R5 Dual Arm:**
```bash
python scripts/hardware/teleop_arx_hardware.py
```

**Galaxea R1 Lite Humanoid:**
```bash
python scripts/hardware/teleop_r1lite_hardware.py
```

---

## XR Device Data Structures

### Overview
The XRoboToolkit SDK provides access to XR device data through the `XrClient` class. All data is accessed via simple getter methods.

### Available Data Streams

#### 1. Device Poses
Returns 7-element numpy array: `[x, y, z, qx, qy, qz, qw]`

**Position:**
- `x, y, z`: 3D position in meters (float)

**Orientation (Quaternion):**
- `qx, qy, qz, qw`: Quaternion components (float)
- Format: [x, y, z, w] representation

**Methods:**
```python
from xrobotoolkit_teleop.common.xr_client import XrClient

xr_client = XrClient()

# Get headset pose
headset_pose = xr_client.get_pose_by_name("headset")
# Returns: np.ndarray([x, y, z, qx, qy, qz, qw])

# Get controller poses
left_controller = xr_client.get_pose_by_name("left_controller")
right_controller = xr_client.get_pose_by_name("right_controller")
```

**Valid pose names:**
- `"headset"`
- `"left_controller"`
- `"right_controller"`

#### 2. Trigger and Grip Values (Analog Inputs)
Returns float value: `0.0` (released) to `1.0` (fully pressed)

**Methods:**
```python
# Get trigger values
left_trigger = xr_client.get_key_value_by_name("left_trigger")   # float: 0.0-1.0
right_trigger = xr_client.get_key_value_by_name("right_trigger") # float: 0.0-1.0

# Get grip values
left_grip = xr_client.get_key_value_by_name("left_grip")         # float: 0.0-1.0
right_grip = xr_client.get_key_value_by_name("right_grip")       # float: 0.0-1.0
```

**Valid analog input names:**
- `"left_trigger"`
- `"right_trigger"`
- `"left_grip"`
- `"right_grip"`

#### 3. Button States (Digital Inputs)
Returns boolean: `True` (pressed) or `False` (released)

**Methods:**
```python
# Face buttons
a_button = xr_client.get_button_state_by_name("A")  # bool
b_button = xr_client.get_button_state_by_name("B")  # bool
x_button = xr_client.get_button_state_by_name("X")  # bool
y_button = xr_client.get_button_state_by_name("Y")  # bool

# Menu buttons
left_menu = xr_client.get_button_state_by_name("left_menu_button")   # bool
right_menu = xr_client.get_button_state_by_name("right_menu_button") # bool

# Joystick/axis clicks
left_click = xr_client.get_button_state_by_name("left_axis_click")   # bool
right_click = xr_client.get_button_state_by_name("right_axis_click") # bool
```

**Valid button names:**
- `"A"`, `"B"`, `"X"`, `"Y"` - Face buttons
- `"left_menu_button"`, `"right_menu_button"` - Menu buttons
- `"left_axis_click"`, `"right_axis_click"` - Joystick/thumbstick clicks

#### 4. Joystick/Thumbstick Positions
Returns list: `[x, y]` with float values typically in range `-1.0` to `+1.0`

**Methods:**
```python
# Get joystick positions
left_joystick = xr_client.get_joystick_state("left")   # list: [x, y]
right_joystick = xr_client.get_joystick_state("right") # list: [x, y]

# Example values:
# [0.0, 0.0]    - Centered
# [1.0, 0.0]    - Right
# [-1.0, 0.0]   - Left
# [0.0, 1.0]    - Up
# [0.0, -1.0]   - Down
```

**Valid joystick names:**
- `"left"`
- `"right"`

#### 5. Timestamp
Returns integer: Timestamp in nanoseconds

**Method:**
```python
timestamp = xr_client.get_timestamp_ns()  # int (nanoseconds)
```

#### 6. Hand Tracking (Optional)
Returns 27×7 numpy array or `None` if hand tracking is inactive

**Structure:**
- 27 hand joints, each with pose: `[x, y, z, qx, qy, qz, qw]`
- Returns `None` if hand tracking quality is low or unavailable

**Methods:**
```python
# Get hand tracking data
left_hand = xr_client.get_hand_tracking_state("left")   # np.ndarray (27, 7) or None
right_hand = xr_client.get_hand_tracking_state("right") # np.ndarray (27, 7) or None

if left_hand is not None:
    # Hand tracking is active
    for joint_idx in range(27):
        joint_pose = left_hand[joint_idx]  # [x, y, z, qx, qy, qz, qw]
```

#### 7. Motion Trackers (Optional)
Returns dictionary with tracker serial numbers as keys

**Structure:**
```python
tracker_data = xr_client.get_motion_tracker_data()
# Returns: dict or {} if no trackers

# Example structure:
# {
#     "PC2310BLH9020740B": {
#         "pose": np.ndarray([x, y, z, qx, qy, qz, qw]),          # 7 floats
#         "velocity": np.ndarray([vx, vy, vz, wx, wy, wz]),       # 6 floats
#         "acceleration": np.ndarray([ax, ay, az, wax, way, waz]) # 6 floats
#     },
#     ...
# }
```

**Methods:**
```python
tracker_data = xr_client.get_motion_tracker_data()

for serial, data in tracker_data.items():
    pose = data["pose"]              # [x, y, z, qx, qy, qz, qw]
    velocity = data["velocity"]      # [vx, vy, vz, wx, wy, wz]
    acceleration = data["acceleration"]  # [ax, ay, az, wax, way, waz]
```

#### 8. Body Tracking (Optional)
Returns dictionary with full body tracking data or `None` if unavailable

**Structure:**
```python
body_data = xr_client.get_body_tracking_data()
# Returns: dict or None

# Structure (if available):
# {
#     "poses": np.ndarray(24, 7),         # 24 joints × [x, y, z, qx, qy, qz, qw]
#     "velocities": np.ndarray(24, 6),    # 24 joints × [vx, vy, vz, wx, wy, wz]
#     "accelerations": np.ndarray(24, 6)  # 24 joints × [ax, ay, az, wax, way, waz]
# }
```

### Data Structure Summary Table

| Data Type | Method | Return Type | Format/Range |
|-----------|--------|-------------|--------------|
| **Headset Pose** | `get_pose_by_name("headset")` | `np.ndarray(7,)` | `[x, y, z, qx, qy, qz, qw]` |
| **Controller Poses** | `get_pose_by_name("left/right_controller")` | `np.ndarray(7,)` | `[x, y, z, qx, qy, qz, qw]` |
| **Triggers** | `get_key_value_by_name("left/right_trigger")` | `float` | `0.0 - 1.0` |
| **Grips** | `get_key_value_by_name("left/right_grip")` | `float` | `0.0 - 1.0` |
| **Buttons** | `get_button_state_by_name("A/B/X/Y/...")` | `bool` | `True/False` |
| **Joysticks** | `get_joystick_state("left/right")` | `list[float]` | `[x, y]` (±1.0) |
| **Timestamp** | `get_timestamp_ns()` | `int` | Nanoseconds |
| **Hand Tracking** | `get_hand_tracking_state("left/right")` | `np.ndarray(27,7)` or `None` | 27 joints × pose |
| **Motion Trackers** | `get_motion_tracker_data()` | `dict` | Serial → pose/vel/accel |
| **Body Tracking** | `get_body_tracking_data()` | `dict` or `None` | 24 joints × pose/vel/accel |

### Coordinate System

**XR Device Coordinate Frame:**
- Origin: Determined by XR system calibration
- The headset frame transformation to world frame is defined in `xrobotoolkit_teleop/utils/geometry.py`:

```python
R_HEADSET_TO_WORLD = np.array([
    [0, 0, -1],
    [-1, 0, 0],
    [0, 1, 0],
])
```

**Quaternion Convention:**
- XR poses use: `[qx, qy, qz, qw]` format
- For conversion to rotation matrix, use `meshcat.transformations` library
- Convert to `[qw, qx, qy, qz]` for meshcat compatibility

### Example: Reading All XR Data

```python
from xrobotoolkit_teleop.common.xr_client import XrClient
import numpy as np

# Initialize
xr_client = XrClient()

# Read all data
def read_all_xr_data():
    data = {
        # Poses
        'headset': xr_client.get_pose_by_name("headset"),
        'left_controller': xr_client.get_pose_by_name("left_controller"),
        'right_controller': xr_client.get_pose_by_name("right_controller"),

        # Analog inputs
        'left_trigger': xr_client.get_key_value_by_name("left_trigger"),
        'right_trigger': xr_client.get_key_value_by_name("right_trigger"),
        'left_grip': xr_client.get_key_value_by_name("left_grip"),
        'right_grip': xr_client.get_key_value_by_name("right_grip"),

        # Joysticks
        'left_joystick': xr_client.get_joystick_state("left"),
        'right_joystick': xr_client.get_joystick_state("right"),

        # Buttons
        'buttons': {
            'A': xr_client.get_button_state_by_name("A"),
            'B': xr_client.get_button_state_by_name("B"),
            'X': xr_client.get_button_state_by_name("X"),
            'Y': xr_client.get_button_state_by_name("Y"),
            'left_menu': xr_client.get_button_state_by_name("left_menu_button"),
            'right_menu': xr_client.get_button_state_by_name("right_menu_button"),
            'left_click': xr_client.get_button_state_by_name("left_axis_click"),
            'right_click': xr_client.get_button_state_by_name("right_axis_click"),
        },

        # Timestamp
        'timestamp': xr_client.get_timestamp_ns(),

        # Optional: Hand tracking
        'left_hand': xr_client.get_hand_tracking_state("left"),
        'right_hand': xr_client.get_hand_tracking_state("right"),

        # Optional: Motion trackers
        'motion_trackers': xr_client.get_motion_tracker_data(),

        # Optional: Body tracking
        'body_tracking': xr_client.get_body_tracking_data(),
    }

    return data

# Main loop
try:
    while True:
        data = read_all_xr_data()

        # Process data
        print(f"Headset position: {data['headset'][:3]}")
        print(f"Left trigger: {data['left_trigger']:.2f}")
        print(f"A button: {'Pressed' if data['buttons']['A'] else 'Released'}")

except KeyboardInterrupt:
    xr_client.close()
    print("Closed.")
```

---

## Troubleshooting

### Common Issues

#### 1. "Command 'cmake' not found"
```bash
sudo apt install cmake
```

#### 2. "No module named 'pip'"
```bash
sudo apt install python3-pip
```

#### 3. "python: command not found"
```bash
sudo apt install python-is-python3
```

#### 4. "externally-managed-environment" error
This happens on Ubuntu 24.04. Use a virtual environment:
```bash
python3 -m venv venv
source venv/bin/activate
# Then install packages
```

#### 5. "XRoboToolkit SDK not initialized"
Make sure XRoboToolkit PC Service is running before starting any script.

#### 6. Virtual environment activation fails
```bash
# Install venv package
sudo apt install python3.12-venv

# Then recreate venv
python3 -m venv venv
source venv/bin/activate
```

#### 7. Build errors with pybind11
```bash
# Install pybind11 with global CMake support
pip install "pybind11[global]"
```

### Performance Tips

- **Target Frame Rate**: Most demos target 30-60 Hz
- **Data Logging**: Press `B` button on controller to toggle logging (in hardware demos)
- **Emergency Stop**: Press right joystick click to discard current log
- **Visualization**: Use `--visualize_placo` flag with hardware demos to see IK solution in browser

### Getting Help

- **GitHub Issues**: https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python/issues
- **Documentation**: See README.md and teleop_details.md in the repository
- **XR Data Display**: Use `scripts/misc/xr_data_display.py` to debug XR device connections and data

---

## Quick Reference Commands

```bash
# Activate environment
source venv/bin/activate          # or: conda activate xr-robotics

# Run XR data display
python scripts/misc/xr_data_display.py

# Run MuJoCo demo
python scripts/simulation/teleop_dual_ur5e_mujoco.py

# Run hardware demo
python scripts/hardware/teleop_dual_ur5e_hardware.py

# Format code
black .

# Deactivate environment
deactivate                        # or: conda deactivate
```
