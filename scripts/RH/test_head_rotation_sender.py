import zmq
import time
import math
import sys
import os
import numpy as np

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


def map_and_clamp_euler(euler_rad: np.ndarray) -> np.ndarray:
    """
    Map headset Euler angles to output coordinate system with range protection.

    Args:
        euler_rad: numpy array [x1, y1, z1] in radians (roll, pitch, yaw)

    Returns:
        numpy array [x2, y2, z2] in degrees with axis remapping and clamping
    """
    x1, y1, z1 = euler_rad

    # Convert radians to degrees
    rad_to_deg = 180.0 / np.pi

    # Axis remapping: x2=y1 (pitch), y2=z1 (yaw), z2=x1 (roll)
    x2_deg = -y1 * rad_to_deg
    y2_deg = x1 * rad_to_deg
    z2_deg = z1 * rad_to_deg

    # x2_deg = 0
    # y2_deg = 0
    # z2_deg = 0

    # Range clamping for safety
    x2 = np.clip(x2_deg, -180, 180)
    y2 = np.clip(y2_deg, -100, 100)
    z2 = np.clip(z2_deg, -35, 35)

    return np.array([x2, y2, z2])


print(f"[Mac] Connected to Linux {LINUX_IP}:{PORT}, sending head rotation data...")

try:
    print("Initializing XrClient...")
    xr_client = XrClient()
    print("XrClient initialized.")
    print("\n" + "="*80)

    frame_count = 0
    start_time = time.time()

    while True:
        head_pose = xr_client.get_pose_by_name("headset")
        if head_pose is not None:
            euler_angles = quaternion_to_euler(head_pose[3:])
            # Apply axis remapping and range clamping
            mapped_angles = map_and_clamp_euler(euler_angles)
            data_to_send = f"{mapped_angles[0]:.2f}, {mapped_angles[1]:.2f}, {mapped_angles[2]:.2f}"
            status = "OK"
        else:
            # If headset data is not available, send zeros for the three mapped angles.
            data_to_send = "0.0, 0.0, 0.0"
            mapped_angles = np.array([0.0, 0.0, 0.0])
            status = "NO HEADSET"

        # Send the data
        socket.send_string(data_to_send)

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
    print("\n[Mac] Stopping sender.")
except Exception as e:
    print(f"ERROR: An error occurred: {e}")
finally:
    if xr_client:
        xr_client.close()
        print("XrClient closed.")
    socket.close()
    context.term()
    print("[Mac] Socket closed.")