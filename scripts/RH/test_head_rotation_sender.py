import zmq
import time
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

# List of target endpoints (IP, PORT)
TARGET_ENDPOINTS = [
    ("192.168.1.56", 5555),
    ("127.0.0.1", 8080),
]


# Create ZMQ context and sockets for each endpoint
context = zmq.Context()
sockets = []
for ip, port in TARGET_ENDPOINTS:
    sock = context.socket(zmq.PUSH)
    sock.connect(f"tcp://{ip}:{port}")
    sockets.append(sock)
    print(f"[PC] Connected to {ip}:{port}")

xr_client: XrClient = None
keyboard_reset_triggered = False  # Flag for keyboard reset


def on_press(key):
    """Keyboard listener callback for key press events."""
    global keyboard_reset_triggered
    try:
        if hasattr(key, 'char') and key.char == 'r':
            keyboard_reset_triggered = True
    except AttributeError:
        pass


def quaternion_to_euler(q: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion [qx, qy, qz, qw] to XYZ Euler angles [yaw, pitch, roll] in radians.
    Rotation order: Roll (X) -> Pitch (Y) -> Yaw (Z)
    Returns: [yaw, pitch, roll]
    """
    x, y, z, w = q

    # Pitch: Compute from look-at vector's vertical component (independent of yaw/roll)
    # Get look-at (forward) vector by rotating default forward [0, 0, -1]
    look_x = 2.0 * (x*z + w*y)
    look_y = 2.0 * (y*z - w*x)
    look_z = 1.0 - 2.0 * (x*x + y*y)

    # Pitch angle from vertical component (Y-axis is up, negated for correct direction)
    pitch = -np.arcsin(np.clip(look_y, -1.0, 1.0))

    # Yaw (Z-axis rotation): -π to π (negated for correct output)
    sin_yaw = 2.0 * (w * y - z * x)
    cos_yaw = 1.0 - 2.0 * (y * y + x * x)
    yaw = -np.arctan2(sin_yaw, cos_yaw)

    # Roll (X-axis rotation): -π/2 to π/2 (negated for correct output)
    sin_roll = 2.0 * (w * z + x * y)
    roll = -np.arcsin(np.clip(sin_roll, -1.0, 1.0))

    # Return [yaw, pitch, roll] with negations already applied in calculations
    return np.array([yaw, pitch, roll])


def send_euler_data(euler_rad: np.ndarray, timestamp: float):
    """
    Send Euler angles in degrees and timestamp to all configured endpoints.

    Args:
        euler_rad: numpy array [yaw, pitch, roll] in radians
        timestamp: timestamp in seconds
    """
    yaw, pitch, roll = euler_rad

    # Convert to degrees
    rad_to_deg = 180.0 / np.pi
    yaw_deg = yaw * rad_to_deg
    pitch_deg = pitch * rad_to_deg
    roll_deg = roll * rad_to_deg

    # Send simple CSV format: yaw, pitch, roll, timestamp
    data_to_send = f"{yaw_deg:.2f}, {pitch_deg:.2f}, {roll_deg:.2f}, {timestamp:.6f}"

    for sock in sockets:
        sock.send_string(data_to_send)


try:
    print("Initializing XrClient...")
    xr_client = XrClient()
    print("XrClient initialized.\n")

    # Start keyboard listener in a separate thread
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    print("Keyboard listener started.\n")

    frame_count = 0
    start_time = time.time()
    yaw_offset = 0.0  # Yaw offset for reset functionality
    reset_button_pressed = False  # Track button state to detect single press

    while True:
        head_pose = xr_client.get_pose_by_name("headset")
        if head_pose is not None:
            # Extract quaternion data
            quaternion = head_pose[3:]  # [qx, qy, qz, qw]

            # Convert to Euler angles [yaw, pitch, roll]
            euler_angles = quaternion_to_euler(quaternion)

            # Check for reset triggers (A button or R key)
            a_button_state = xr_client.get_button_state_by_name("A")

            if (a_button_state and not reset_button_pressed) or keyboard_reset_triggered:
                # Reset triggered - capture current yaw as offset
                yaw_offset = euler_angles[0]
                reset_button_pressed = True
                keyboard_reset_triggered = False  # Reset keyboard flag
                source = "A Button" if a_button_state else "R Key"
                print(f"\n[RESET via {source}] Yaw offset captured: {yaw_offset * 180.0 / np.pi:.2f}°\n")
            elif not a_button_state:
                # Button released - ready for next press
                reset_button_pressed = False

            # Apply yaw offset to reset yaw to zero
            euler_angles[0] -= yaw_offset

            status = "OK"
        else:
            # If headset data is not available, send zeros
            euler_angles = np.array([0.0, 0.0, 0.0])
            status = "NO HEADSET"

        # Send Euler angles to all endpoints
        current_timestamp = time.time()
        send_euler_data(euler_angles, current_timestamp)

        # Calculate frequency
        frame_count += 1
        elapsed_time = time.time() - start_time
        frequency = frame_count / elapsed_time if elapsed_time > 0 else 0

        # Convert to degrees for display
        rad_to_deg = 180.0 / np.pi
        yaw_deg = euler_angles[0] * rad_to_deg
        pitch_deg = euler_angles[1] * rad_to_deg
        roll_deg = euler_angles[2] * rad_to_deg

        # Clear terminal and print static interface
        os.system('clear' if os.name != 'nt' else 'cls')
        print("=" * 80)
        print("HEAD ROTATION DATA SENDER (XYZ Euler)")
        print("=" * 80)
        print(f"\nEndpoints: {len(TARGET_ENDPOINTS)} active")
        for idx, (ip, port) in enumerate(TARGET_ENDPOINTS, 1):
            print(f"  [{idx}] {ip}:{port}")
        print("\nControls:")
        print("  - Press A button (VR) or R key (keyboard) to reset yaw to zero")
        print("  - Ctrl+C to exit")
        print("=" * 80)
        print(f"\nStatus: [{status:^12}]  Frame: {frame_count:05d}  Frequency: {frequency:>5.1f} Hz")
        print(f"Yaw Offset: {yaw_offset * 180.0 / np.pi:>8.2f}° {'[ACTIVE]' if abs(yaw_offset) > 0.01 else ''}")
        print("=" * 80)
        print("\nEuler Angles (XYZ rotation order):")
        print(f"  Yaw:   {yaw_deg:>8.2f}°")
        print(f"  Pitch: {pitch_deg:>8.2f}°")
        print(f"  Roll:  {roll_deg:>8.2f}°")
        print("=" * 80)

        # Wait for a short period
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n[PC] Stopping sender.")
except Exception as e:
    print(f"ERROR: An error occurred: {e}")
finally:
    if xr_client:
        xr_client.close()
        print("XrClient closed.")
    # Stop keyboard listener
    if 'listener' in locals():
        listener.stop()
        print("Keyboard listener stopped.")
    # Close all sockets
    for sock in sockets:
        sock.close()
    context.term()
    print("All sockets closed.")