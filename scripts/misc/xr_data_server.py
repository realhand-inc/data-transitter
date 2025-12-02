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