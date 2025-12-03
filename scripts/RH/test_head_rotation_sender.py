import zmq
import time
import math
import sys
import os
import numpy as np
from pynput import keyboard

# Ensure xrobotoolkit_teleop is in the Python path
script_dir = os.path.dirname(__file__)
project_root = os.path.abspath(os.path.join(script_dir, os.pardir, os.pardir)) # Go up two levels from scripts/RH/
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from xrobotoolkit_teleop.common.xr_client import XrClient

# Server address
LINUX_IP = "192.168.1.56"
PORT = 5555


# Create ZMQ context and socket
context = zmq.Context()
socket = context.socket(zmq.PUSH)
socket.connect(f"tcp://{LINUX_IP}:{PORT}")

xr_client: XrClient = None
last_valid_angles = np.array([0.0, 0.0, 0.0])  # Store last valid rotation data
x2_offset = 0.0  # Offset for x2 axis reset
current_x2_raw = 0.0  # Track current raw x2 value before offset
last_trigger_state = False  # Track trigger state for edge detection


def reset_x2():
    """Reset the x2 axis by storing current raw value as offset and unfreezing output."""
    global x2_offset, current_x2_raw, last_valid_angles
    x2_offset = current_x2_raw
    # Accept current position even if out of range (unfreeze)
    last_valid_angles = np.array([0.0, last_valid_angles[1], last_valid_angles[2]])
    print(f"\n[RESET] X2 offset set to {x2_offset:.2f}° (X2 output now: 0.00°)")


def on_press(key):
    """Keyboard callback function for handling reset button."""
    try:
        if hasattr(key, 'char') and key.char == 'r':
            reset_x2()
    except AttributeError:
        pass


def quaternion_to_euler(q: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion [qx, qy, qz, qw] to Euler angles [roll, pitch, yaw] in radians.
    Uses atan2 for continuous -π to π (-180° to 180°) range on all axes.
    """
    x, y, z, w = q

    # Roll (x-axis rotation): -π to π
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)

    # Pitch (y-axis rotation): -π to π (using atan2 instead of asin)
    t2 = 2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)  # Safety clamp
    # Use atan2 for full range
    pitch_y = np.arctan2(t2, np.sqrt(1.0 - t2 * t2))

    # Yaw (z-axis rotation): -π to π
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)

    return np.array([roll_x, pitch_y, yaw_z])


def map_and_clamp_euler(euler_rad: np.ndarray) -> np.ndarray:
    """
    Map headset Euler angles to output coordinate system with range protection.
    If any angle exceeds limits, returns the last valid rotation data instead.

    Args:
        euler_rad: numpy array [x1, y1, z1] in radians (roll, pitch, yaw)

    Returns:
        numpy array [x2, y2, z2] in degrees with axis remapping and clamping
    """
    global last_valid_angles, x2_offset, current_x2_raw

    x1, y1, z1 = euler_rad

    # Convert radians to degrees
    rad_to_deg = 180.0 / np.pi

    # Axis remapping: x2=y1 (pitch), y2=z1 (yaw), z2=x1 (roll)
    x2_deg = -y1 * rad_to_deg
    y2_deg = x1 * rad_to_deg
    z2_deg = -z1 * rad_to_deg

    # Store raw x2 value and apply offset
    current_x2_raw = x2_deg
    x2_deg = x2_deg - x2_offset



    # x2_deg = 0
    # y2_deg = 0
    # z2_deg = 0

    # Define angle limits
    x2_min, x2_max = -90, 90
    y2_min, y2_max = -100, 100
    z2_min, z2_max = -35, 35

    # Check if any angle exceeds limits
    if (x2_deg < x2_min or x2_deg > x2_max or
        y2_deg < y2_min or y2_deg > y2_max or
        z2_deg < z2_min or z2_deg > z2_max):
        # Return last valid angles if any limit is exceeded
        return last_valid_angles.copy()
    else:
        # All angles are within limits, update and return
        last_valid_angles = np.array([x2_deg, y2_deg, z2_deg])
        return last_valid_angles.copy()


print(f"[PC] Connected to Linux {LINUX_IP}:{PORT}, sending head rotation data...")

try:
    print("Initializing XrClient...")
    xr_client = XrClient()
    print("XrClient initialized.")
    print("\n" + "="*80)

    # Start keyboard listener for reset functionality
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    print("Keyboard listener started. Press 'r' to reset X2 axis.")
    print("="*80 + "\n")

    frame_count = 0
    start_time = time.time()

    while True:
        # Get timestamp before processing
        timestamp = time.time()

        head_pose = xr_client.get_pose_by_name("headset")
        if head_pose is not None:
            euler_angles = quaternion_to_euler(head_pose[3:])
            # Apply axis remapping and range clamping
            mapped_angles = map_and_clamp_euler(euler_angles)
            data_to_send = f"{mapped_angles[0]:.2f}, {mapped_angles[1]:.2f}, {mapped_angles[2]:.2f}, {timestamp:.6f}"
            status = "OK"
        else:
            # If headset data is not available, send zeros for the three mapped angles.
            data_to_send = f"0.0, 0.0, 0.0, {timestamp:.6f}"
            mapped_angles = np.array([0.0, 0.0, 0.0])
            status = "NO HEADSET"

        # Send the data
        socket.send_string(data_to_send)

        # Check A button for reset
        button_pressed = xr_client.get_button_state_by_name("A")

        # Detect rising edge (button just pressed)
        if button_pressed and not last_trigger_state:
            reset_x2()

        last_trigger_state = button_pressed

        # Calculate frequency
        frame_count += 1
        elapsed_time = time.time() - start_time
        frequency = frame_count / elapsed_time if elapsed_time > 0 else 0

        # Clear line and print updated status (use \r to return to start of line)
        print(f"\r[Frame {frame_count:05d}] [{status:^12}] X2: {mapped_angles[0]:>7.2f}° | "
              f"Y2: {mapped_angles[1]:>7.2f}° | Z2: {mapped_angles[2]:>7.2f}° | "
              f"Freq: {frequency:>6.2f} Hz", end='', flush=True)

        # Wait for a short period
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n[PC] Stopping sender.")
except Exception as e:
    print(f"ERROR: An error occurred: {e}")
finally:
    # Stop keyboard listener
    if 'listener' in locals():
        listener.stop()
        print("Keyboard listener stopped.")
    if xr_client:
        xr_client.close()
        print("XrClient closed.")
    socket.close()
    context.term()
    print("[PC] Socket closed.")