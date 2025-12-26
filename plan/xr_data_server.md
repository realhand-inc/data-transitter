# XR Data Server Plan

This document outlines the plan for setting up a Flask-based REST API to expose real-time XR (Extended Reality) device data to other machines on your local network.

## 1. Goal

To provide a `/data` endpoint that, when accessed, returns a JSON object containing the latest headset, left-hand (controller), right-hand (controller), and trigger data. This allows external applications (e.g., visualization tools on other PCs) to consume XR data without direct integration with the `XrClient` SDK.

Additionally, more granular endpoints for position and rotation will be provided for convenience.

## 2. Installation and Setup

### 2.1 Install Flask

The server relies on the Flask web framework. If you don't have it installed, you can do so using `pip`.

```bash
# Ensure your Python environment (conda or venv) is activated first.
# For example, if using conda:
# conda activate xr-robotics

pip install Flask
```

### 2.2 Server Script (`scripts/misc/xr_data_server.py`)

The following Python script sets up the Flask application and the API endpoints.

```python
import sys
import os
import time
from typing import Dict, Any
import numpy as np
from flask import Flask, jsonify

# Ensure xrobotoolkit_teleop is in the Python path
# This assumes the script is run from the project root or from scripts/misc
script_dir = os.path.dirname(__file__)
project_root = os.path.abspath(os.path.join(script_dir, os.pardir, os.pardir))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from xrobotoolkit_teleop.common.xr_client import XrClient

app = Flask(__name__)
xr_client: XrClient = None

def quaternion_to_euler(q: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion [qx, qy, qz, qw] to Euler angles [roll, pitch, yaw] in radians.
    """
    x, y, z, w = q
    # roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)

    # pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch_y = np.arcsin(t2)

    # yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)

    return np.array([roll_x, pitch_y, yaw_z])

def get_xr_data() -> Dict[str, Any]:
    """
    Fetches all relevant XR data from the XrClient and formats it for the API.
    """
    data = {}

    # Timestamp
    data['timestamp'] = xr_client.get_timestamp_ns()

    # Headset Pose
    head_pose = xr_client.get_pose_by_name("headset")
    if head_pose is not None:
        data['head'] = {
            "pos": head_pose[:3].tolist(),
            "rot": head_pose[3:].tolist(), # qx,qy,qz,qw
            "euler": quaternion_to_euler(head_pose[3:]).tolist()
        }
    else:
        data['head'] = None

    # Left Controller/Hand Data
    left_pose = xr_client.get_pose_by_name("left_controller")
    left_trigger = xr_client.get_key_value_by_name("left_trigger")
    left_grip = xr_client.get_key_value_by_name("left_grip")
    left_joystick = xr_client.get_joystick_state("left")

    # Buttons - A, B, X, Y, menu, click
    buttons = {
        'A': xr_client.get_button_state_by_name("A"),
        'B': xr_client.get_button_state_by_name("B"),
        'X': xr_client.get_button_state_by_name("X"),
        'Y': xr_client.get_button_state_by_name("Y"),
        'left_menu': xr_client.get_button_state_by_name("left_menu_button"),
        'right_menu': xr_client.get_button_state_by_name("right_menu_button"),
        'left_click': xr_client.get_button_state_by_name("left_axis_click"),
        'right_click': xr_client.get_button_state_by_name("right_axis_click"),
    }

    # Populate left hand data
    data['left'] = {
        "pos": left_pose[:3].tolist() if left_pose is not None else None,
        "rot": left_pose[3:].tolist() if left_pose is not None else None,
        "euler": quaternion_to_euler(left_pose[3:]).tolist() if left_pose is not None else None,
        "trigger": left_trigger,
        "grip": left_grip,
        "joystick": left_joystick,
        "buttons": {
            'X': buttons['X'],
            'Y': buttons['Y'],
            'menu': buttons['left_menu'],
            'click': buttons['left_click'],
        }
    }

    # Right Controller/Hand Data
    right_pose = xr_client.get_pose_by_name("right_controller")
    right_trigger = xr_client.get_key_value_by_name("right_trigger")
    right_grip = xr_client.get_key_value_by_name("right_grip")
    right_joystick = xr_client.get_joystick_state("right")

    # Populate right hand data
    data['right'] = {
        "pos": right_pose[:3].tolist() if right_pose is not None else None,
        "rot": right_pose[3:].tolist() if right_pose is not None else None,
        "euler": quaternion_to_euler(right_pose[3:]).tolist() if right_pose is not None else None,
        "trigger": right_trigger,
        "grip": right_grip,
        "joystick": right_joystick,
        "buttons": {
            'A': buttons['A'],
            'B': buttons['B'],
            'menu': buttons['right_menu'],
            'click': buttons['right_click'],
        }
    }

    return data

@app.route('/data', methods=['GET'])
def get_all_xr_data():
    """
    API endpoint to get all current XR data.
    """
    try:
        xr_data = get_xr_data()
        return jsonify(xr_data)
    except Exception as e:
        app.logger.error(f"Error fetching XR data: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/head', methods=['GET'])
def get_head_pose_euler():
    pose = xr_client.get_pose_by_name("headset")
    if pose is not None:
        return jsonify({
            "pos": pose[:3].tolist(),
            "euler": quaternion_to_euler(pose[3:]).tolist()
        })
    return jsonify(None), 404

@app.route('/headPosition', methods=['GET'])
def get_head_position():
    pose = xr_client.get_pose_by_name("headset")
    if pose is not None:
        return jsonify(pose[:3].tolist())
    return jsonify(None), 404

@app.route('/headRotation', methods=['GET'])
def get_head_rotation():
    pose = xr_client.get_pose_by_name("headset")
    if pose is not None:
        return jsonify(quaternion_to_euler(pose[3:]).tolist())
    return jsonify(None), 404

@app.route('/left', methods=['GET'])
def get_left_pose_euler():
    pose = xr_client.get_pose_by_name("left_controller")
    if pose is not None:
        return jsonify({
            "pos": pose[:3].tolist(),
            "euler": quaternion_to_euler(pose[3:]).tolist()
        })
    return jsonify(None), 404

@app.route('/leftPosition', methods=['GET'])
def get_left_position():
    pose = xr_client.get_pose_by_name("left_controller")
    if pose is not None:
        return jsonify(pose[:3].tolist())
    return jsonify(None), 404

@app.route('/leftRotation', methods=['GET'])
def get_left_rotation():
    pose = xr_client.get_pose_by_name("left_controller")
    if pose is not None:
        return jsonify(quaternion_to_euler(pose[3:]).tolist())
    return jsonify(None), 404

@app.route('/right', methods=['GET'])
def get_right_pose_euler():
    pose = xr_client.get_pose_by_name("right_controller")
    if pose is not None:
        return jsonify({
            "pos": pose[:3].tolist(),
            "euler": quaternion_to_euler(pose[3:]).tolist()
        })
    return jsonify(None), 404

@app.route('/rightPosition', methods=['GET'])
def get_right_position():
    pose = xr_client.get_pose_by_name("right_controller")
    if pose is not None:
        return jsonify(pose[:3].tolist())
    return jsonify(None), 404

@app.route('/rightRotation', methods=['GET'])
def get_right_rotation():
    pose = xr_client.get_pose_by_name("right_controller")
    if pose is not None:
        return jsonify(quaternion_to_euler(pose[3:]).tolist())
    return jsonify(None), 404

@app.before_request
def initialize_xr_client():
    """Initialize XrClient before the first request if not already initialized."""
    global xr_client
    if xr_client is None:
        try:
            print("Initializing XrClient...")
            xr_client = XrClient()
            print("XrClient initialized.")
        except Exception as e:
            app.logger.error(f"Failed to initialize XrClient: {e}")

@app.teardown_appcontext
def close_xr_client(exception=None):
    """Close XrClient when the app context tears down."""
    global xr_client
    if xr_client:
        try:
            print("Closing XrClient...")
            xr_client.close()
            print("XrClient closed.")
        except Exception as e:
            app.logger.error(f"Error closing XrClient: {e}")
        xr_client = None

if __name__ == '__main__':
    # Initialise XrClient here for when running directly
    try:
        print("Starting XR Data Server...")
        xr_client = XrClient()
        print("XrClient initialized for main process.")
    except Exception as e:
        print(f"ERROR: Failed to initialize XrClient before starting app: {e}")
        print("Please ensure XRoboToolkit PC Service is running and XR device is connected.")
        sys.exit(1)

    # Use a try-finally block to ensure xr_client is closed on exit
    try:
        # Host on all available network interfaces
        app.run(host='0.0.0.0', port=5000, debug=False)
    finally:
        if xr_client:
            print("Closing XrClient on server shutdown...")
            xr_client.close()
            print("XrClient closed.")
```

## 3. Running the Server

To start the server, navigate to your project's root directory in the terminal (after activating your Python environment if applicable) and run:

```bash
python scripts/misc/xr_data_server.py
```

The server will start on `http://0.0.0.0:5000`. Make sure the `XRoboToolkit PC Service` is running and your XR device is connected before starting this script.

## 4. Accessing the Data

Once the server is running, you can access the XR data from any machine on the same network by making an HTTP GET request to `http://<YOUR_PC_IP_ADDRESS>:5000/data`.

Replace `<YOUR_PC_IP_ADDRESS>` with the actual IP address of the machine running the server.

### Example using `curl`:

```bash
curl http://<YOUR_PC_IP_ADDRESS>:5000/data
```

### Expected JSON Output Structure (`/data`):

The endpoint will return a JSON object similar to this:

```json
{
  "timestamp": 1701513600000000000,
  "head": {
    "pos": [0.1, 1.5, 0.2],
    "rot": [0.001, 0.002, 0.003, 0.999],
    "euler": [0.002, 0.004, 0.006]
  },
  "left": {
    "pos": [-0.5, 1.2, -0.3],
    "rot": [0.1, 0.2, 0.3, 0.9],
    "euler": [0.2, 0.4, 0.6],
    "trigger": 0.75,
    "grip": 0.1,
    "joystick": [-0.5, 0.8],
    "buttons": {
      "X": true,
      "Y": false,
      "menu": false,
      "click": true
    }
  },
  "right": {
    "pos": [0.5, 1.2, -0.3],
    "rot": [-0.1, -0.2, -0.3, 0.9],
    "euler": [-0.2, -0.4, -0.6],
    "trigger": 0.2,
    "grip": 0.0,
    "joystick": [0.1, -0.2],
    "buttons": {
      "A": false,
      "B": true,
      "menu": false,
      "click": false
    }
  }
}
```

-   **`timestamp`**: Current time in nanoseconds.
-   **`head`**, **`left`**, **`right`**:
    -   `pos`: Position `[x, y, z]` in meters.
    -   `rot`: Orientation `[qx, qy, qz, qw]` as a quaternion.
    -   `euler`: Orientation `[roll, pitch, yaw]` in radians.
    -   `trigger`, `grip`, `joystick`, `buttons`: Controller-specific input states.

### Additional Endpoints

In addition to the main `/data` endpoint, the server provides more granular access to pose data. These endpoints return data in a simplified format and use Euler angles (roll, pitch, yaw in radians) for rotation. If a device is not tracking, these endpoints will return a `404 Not Found` error.

-   **/head**: Returns the position (`pos`) and Euler rotation (`euler`) of the headset.
    -   `curl http://<IP>:5000/head`
-   **/headPosition**: Returns just the `[x, y, z]` position of the headset.
    -   `curl http://<IP>:5000/headPosition`
-   **/headRotation**: Returns just the `[roll, pitch, yaw]` Euler rotation of the headset.
    -   `curl http://<IP>:5000/headRotation`

-   **/left**: Returns the position (`pos`) and Euler rotation (`euler`) of the left controller.
    -   `curl http://<IP>:5000/left`
-   **/leftPosition**: Returns just the position of the left controller.
    -   `curl http://<IP>:5000/leftPosition`
-   **/leftRotation**: Returns just the Euler rotation of the left controller.
    -   `curl http://<IP>:5000/leftRotation`

-   **/right**: Returns the position (`pos`) and Euler rotation (`euler`) of the right controller.
    -   `curl http://<IP>:5000/right`
-   **/rightPosition**: Returns just the position of the right controller.
    -   `curl http://<IP>:5000/rightPosition`
-   **/rightRotation**: Returns just the Euler rotation of the right controller.
    -   `curl http://<IP>:5000/rightRotation`


## 5. Local Robot Control Application Considerations

For your robot control application running on the *same PC*, it is highly recommended to directly import the `XrClient` and its functions. This avoids any network overhead introduced by the Flask server and ensures the lowest possible latency for critical control loops.

```python
# Example for your robot control script on the same PC
from xrobotoolkit_teleop.common.xr_client import XrClient

# Initialize XrClient directly
local_xr_client = XrClient()

while True:
    # Get headset pose for robot base control
    head_pose = local_xr_client.get_pose_by_name("headset")
    
    # Get controller pose for end-effector control
    right_controller_pose = local_xr_client.get_pose_by_name("right_controller")
    
    # Process data and send commands to your robot
    # ...
```
