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
            "rot": head_pose[3:].tolist() # qx,qy,qz,qw
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
    
    # Optional: Add hand tracking if available and requested
    # For now, we'll stick to controller data as "hands" for simplicity,
    # as per the JSON structure preview focusing on controller-like data.
    # If explicit hand tracking is needed, this section can be expanded.

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
            # Depending on desired behavior, could exit or return an error response
            # For now, let's allow the app to start but subsequent requests will fail
            # if xr_client remains None or throws an error during data fetching.

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
