import zmq
import time
import sys
import os
import numpy as np

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


def quaternion_to_euler(q: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion [qx, qy, qz, qw] to XYZ Euler angles [yaw, pitch, roll] in radians.
    Rotation order: Roll (X) -> Pitch (Y) -> Yaw (Z)
    Returns: [yaw, pitch, roll]
    """
    x, y, z, w = q

    # Pitch (Y-axis rotation, first): -π/2 to π/2
    sin_pitch = 2.0 * (w * x + y * z)
    sin_pitch = np.clip(sin_pitch, -1.0, 1.0)
    pitch = np.arcsin(sin_pitch)

    # Yaw (Z-axis rotation, second): -π to π
    sin_yaw = 2.0 * (w * y - z * x)
    cos_yaw = 1.0 - 2.0 * (y * y + x * x)
    yaw = np.arctan2(sin_yaw, cos_yaw)

    # Roll (X-axis rotation, third): -π/2 to π/2
    sin_roll = 2.0 * (w * z + x * y)
    sin_roll = np.clip(sin_roll, -1.0, 1.0)
    roll = np.arcsin(sin_roll)


    # DON'T CHANGE ORDER: return [yaw, pitch, roll]
    return np.array([yaw, pitch, roll])


def send_euler_data(euler_rad: np.ndarray):
    """
    Send Euler angles in degrees to all configured endpoints.

    Args:
        euler_rad: numpy array [yaw, pitch, roll] in radians
    """
    yaw, pitch, roll = euler_rad

    # Convert to degrees
    rad_to_deg = 180.0 / np.pi
    yaw_deg = yaw * rad_to_deg
    pitch_deg = pitch * rad_to_deg
    roll_deg = roll * rad_to_deg

    # Send simple CSV format: yaw, pitch, roll
    data_to_send = f"{yaw_deg:.2f}, {pitch_deg:.2f}, {roll_deg:.2f}"

    for sock in sockets:
        sock.send_string(data_to_send)


try:
    print("Initializing XrClient...")
    xr_client = XrClient()
    print("XrClient initialized.\n")

    frame_count = 0
    start_time = time.time()

    while True:
        head_pose = xr_client.get_pose_by_name("headset")
        if head_pose is not None:
            # Extract quaternion data
            quaternion = head_pose[3:]  # [qx, qy, qz, qw]

            # Convert to Euler angles [yaw, pitch, roll]
            euler_angles = quaternion_to_euler(quaternion)
            status = "OK"
        else:
            # If headset data is not available, send zeros
            euler_angles = np.array([0.0, 0.0, 0.0])
            status = "NO HEADSET"

        # Send Euler angles to all endpoints
        send_euler_data(euler_angles)

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
        print("  - Ctrl+C to exit")
        print("=" * 80)
        print(f"\nStatus: [{status:^12}]  Frame: {frame_count:05d}  Frequency: {frequency:>5.1f} Hz")
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
    # Close all sockets
    for sock in sockets:
        sock.close()
    context.term()
    print("All sockets closed.")