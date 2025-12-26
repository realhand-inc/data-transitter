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


def quaternion_to_vectors(q: np.ndarray):
    """
    Convert quaternion [qx, qy, qz, qw] to Up and LookAt (Forward) vectors.
    Assuming standard frame:
    - Up is local +Y
    - LookAt is local -Z (typical camera convention)
    
    Returns:
        up_vector (np.array): [x, y, z]
        look_at_vector (np.array): [x, y, z]
    """
    x, y, z, w = q

    # Rotation Matrix R
    # Col 0 (Right): [1-2yy-2zz, 2xy+2zw, 2xz-2yw]
    # Col 1 (Up):    [2xy-2zw, 1-2xx-2zz, 2yz+2xw]
    # Col 2 (Back):  [2xz+2yw, 2yz-2xw, 1-2xx-2yy] (If Z is Back)

    # Up Vector (Col 1)
    up_x = 2.0 * (x * y - z * w)
    up_y = 1.0 - 2.0 * (x * x + z * z)
    up_z = 2.0 * (y * z + x * w)
    up = np.array([up_x, up_y, up_z])

    # Forward/LookAt Vector (Negative of Col 2, assuming -Z is forward)
    # Col 2
    z_x = 2.0 * (x * z + y * w)
    z_y = 2.0 * (y * z - x * w)
    z_z = 1.0 - 2.0 * (x * x + y * y)
    
    # LookAt = -Z axis
    look_at = -np.array([z_x, z_y, z_z])

    return up, look_at


def send_vector_data(up: np.ndarray, look_at: np.ndarray, timestamp: float):
    """
    Send Up and LookAt vectors and timestamp to all configured endpoints.
    Format: up_x, up_y, up_z, look_at_x, look_at_y, look_at_z, timestamp
    """
    # Send CSV format
    data_to_send = f"{up[0]:.4f}, {up[1]:.4f}, {up[2]:.4f}, {look_at[0]:.4f}, {look_at[1]:.4f}, {look_at[2]:.4f}, {timestamp:.6f}"

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
    
    # Yaw offset logic might not apply directly to vectors in the same way, 
    # but we'll keep the variable to avoid breaking structure, though we won't use it for vector transformation here 
    # unless we want to rotate the vectors. For now, we display raw headset vectors.
    yaw_offset = 0.0 
    reset_button_pressed = False

    while True:
        head_pose = xr_client.get_pose_by_name("headset")
        if head_pose is not None:
            # Extract quaternion data
            quaternion = head_pose[3:]  # [qx, qy, qz, qw]

            # Convert to Vectors
            up_vec, look_at_vec = quaternion_to_vectors(quaternion)

            # Reset logic (kept for completeness, though effect on vectors is not implemented)
            a_button_state = xr_client.get_button_state_by_name("A")
            if (a_button_state and not reset_button_pressed) or keyboard_reset_triggered:
                reset_button_pressed = True
                keyboard_reset_triggered = False
                # Reset functionality typically resets yaw. 
                # Implementing yaw reset on vectors would require rotating them around global Y.
                # For this viewer, we'll just acknowledge the press.
            elif not a_button_state:
                reset_button_pressed = False

            status = "OK"
        else:
            # Default vectors if no headset
            up_vec = np.array([0.0, 1.0, 0.0])
            look_at_vec = np.array([0.0, 0.0, -1.0])
            status = "NO HEADSET"

        # Send data
        current_timestamp = time.time()
        send_vector_data(up_vec, look_at_vec, current_timestamp)

        # Calculate frequency
        frame_count += 1
        elapsed_time = time.time() - start_time
        frequency = frame_count / elapsed_time if elapsed_time > 0 else 0

        # Clear terminal and print static interface
        os.system('clear' if os.name != 'nt' else 'cls')
        print("=" * 80)
        print("HEAD ROTATION DATA SENDER (Vectors)")
        print("=" * 80)
        print(f"\nEndpoints: {len(TARGET_ENDPOINTS)} active")
        for idx, (ip, port) in enumerate(TARGET_ENDPOINTS, 1):
            print(f"  [{idx}] {ip}:{port}")
        print("\nControls:")
        print("  - Press A button (VR) or R key (keyboard) to trigger reset (visual only)")
        print("  - Ctrl+C to exit")
        print("=" * 80)
        print(f"\nStatus: [{status:^12}]  Frame: {frame_count:05d}  Frequency: {frequency:>5.1f} Hz")
        print("=" * 80)
        print("\nHead Vectors:")
        print(f"  LookAt (Forward): [{look_at_vec[0]:>6.3f}, {look_at_vec[1]:>6.3f}, {look_at_vec[2]:>6.3f}]")
        print(f"  Up:               [{up_vec[0]:>6.3f}, {up_vec[1]:>6.3f}, {up_vec[2]:>6.3f}]")
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
