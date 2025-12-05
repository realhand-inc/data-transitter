import zmq
import time
import sys
import os
import numpy as np
from pynput import keyboard
import cv2
import threading
import subprocess
from collections import deque

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

# Joystick rotation configuration
JOYSTICK_ROTATION_SPEED = 90.0  # Degrees per second at full joystick deflection
JOYSTICK_DEADZONE = 0.1         # Ignore joystick values below this threshold


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


def apply_deadzone(value: float, threshold: float = JOYSTICK_DEADZONE) -> float:
    """Apply deadzone to joystick input to prevent drift."""
    if abs(value) < threshold:
        return 0.0
    return value


# ADB Commands
ADB_OPEN_APP = "adb shell monkey -p com.xrobotoolkit.client -c android.intent.category.LAUNCHER 1"
ADB_STOP_APP = "adb shell am force-stop com.xrobotoolkit.client"


def execute_adb_command(command):
    """Execute ADB command and return result"""
    try:
        result = subprocess.run(
            command,
            shell=True,
            capture_output=True,
            text=True,
            timeout=5
        )
        return result.returncode == 0, result.stdout
    except Exception as e:
        return False, str(e)


def get_adb_devices():
    """Get list of connected ADB devices"""
    try:
        result = subprocess.run(
            ["adb", "devices"],
            capture_output=True,
            text=True,
            timeout=3
        )

        print(f"[DEBUG] ADB return code: {result.returncode}")
        print(f"[DEBUG] ADB raw output:\n{repr(result.stdout)}\n")

        if result.returncode == 0:
            devices = []
            lines = result.stdout.strip().split('\n')

            # Process each line
            for i, line in enumerate(lines):
                original_line = line
                line = line.strip()

                print(f"[DEBUG] Line {i}: {repr(original_line)} -> stripped: {repr(line)}")

                # Skip header and empty lines
                if not line or 'List of devices attached' in line or '* daemon' in line:
                    print(f"[DEBUG]   -> Skipped (header/empty)")
                    continue

                # Split by any whitespace
                parts = line.split()
                print(f"[DEBUG]   -> Parts: {parts}, Length: {len(parts)}")

                # Accept lines with at least device ID and status
                if len(parts) >= 2:
                    device_id = parts[0]
                    status = parts[1]
                    devices.append({'id': device_id, 'status': status})
                    print(f"[DEBUG]   -> ADDED: {device_id} ({status})")
                else:
                    print(f"[DEBUG]   -> REJECTED: Not enough parts")

            print(f"\n[DEBUG] === ADB SCAN COMPLETE ===")
            print(f"[DEBUG] Total devices found: {len(devices)}")
            for dev in devices:
                print(f"[DEBUG]   - {dev['id']} ({dev['status']})")
            print(f"[DEBUG] ========================\n")

            return devices

        print(f"[DEBUG] ADB command failed: return code {result.returncode}")
        return []

    except FileNotFoundError:
        print("[ERROR] ADB command not found")
        return []
    except Exception as e:
        print(f"[ERROR] ADB scan exception: {e}")
        import traceback
        traceback.print_exc()
        return []


def scan_local_network():
    """Scan local network for active devices"""
    devices = []

    try:
        # Get local IP using simpler method
        result = subprocess.run(
            ["hostname", "-I"],
            capture_output=True,
            text=True,
            timeout=2
        )

        print(f"[DEBUG] Network scan - hostname -I: {repr(result.stdout)}")

        local_ips = result.stdout.strip().split()
        if not local_ips:
            # Fallback: try ip route
            result = subprocess.run(
                "ip route get 1.1.1.1 | grep -oE 'src [0-9.]+' | awk '{print $2}'",
                shell=True,
                capture_output=True,
                text=True,
                timeout=2
            )
            print(f"[DEBUG] Network scan - ip route fallback: {repr(result.stdout)}")
            if result.stdout.strip():
                local_ips = [result.stdout.strip()]

        if local_ips:
            # Use first non-localhost IP
            local_ip = None
            for ip in local_ips:
                if not ip.startswith('127.'):
                    local_ip = ip
                    break

            print(f"[DEBUG] Using local IP: {local_ip}")

            if local_ip:
                ip_parts = local_ip.split('.')
                network_prefix = '.'.join(ip_parts[:3])

                # Quick scan of common device IPs (faster than full subnet scan)
                test_ips = set()

                # Add target endpoints first
                for ip, port in TARGET_ENDPOINTS:
                    test_ips.add(ip)

                # Add some common ranges
                for i in [1, 50, 51, 52, 53, 54, 55, 56, 100, 101, 254]:
                    test_ip = f"{network_prefix}.{i}"
                    test_ips.add(test_ip)

                print(f"[DEBUG] Testing {len(test_ips)} IPs: {sorted(test_ips)}")

                # Quick ping test with timeout
                for ip in test_ips:
                    try:
                        result = subprocess.run(
                            ["ping", "-c", "1", "-W", "1", ip],
                            capture_output=True,
                            timeout=1.5
                        )
                        if result.returncode == 0:
                            devices.append(ip)
                            print(f"[DEBUG] Found device: {ip}")
                    except subprocess.TimeoutExpired:
                        pass
                    except Exception as e:
                        print(f"[DEBUG] Ping error for {ip}: {e}")

        print(f"[DEBUG] Network scan found {len(devices)} device(s): {devices}")

    except Exception as e:
        print(f"[DEBUG] Network scan error: {e}")
        import traceback
        traceback.print_exc()

    return devices


class Button:
    def __init__(self, x, y, width, height, text, callback):
        self.rect = (x, y, width, height)
        self.text = text
        self.callback = callback
        self.hover = False

    def draw(self, frame):
        x, y, w, h = self.rect
        color = (180, 180, 250) if self.hover else (200, 200, 200)
        cv2.rectangle(frame, (x, y), (x+w, y+h), color, -1)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (100, 100, 100), 2)

        # Draw text centered
        text_size = cv2.getTextSize(self.text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
        text_x = x + (w - text_size[0]) // 2
        text_y = y + (h + text_size[1]) // 2
        cv2.putText(frame, self.text, (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

    def is_clicked(self, mx, my):
        x, y, w, h = self.rect
        return x <= mx <= x+w and y <= my <= y+h


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


class HeadRotationGUI:
    """GUI window for displaying head rotation data with visualizations"""

    def __init__(self, window_name="Head Rotation Data Sender"):
        self.window_name = window_name
        self.width = 1280
        self.height = 720
        self.running = True

        # Data storage
        self.current_data = {
            'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
            'status': 'INIT', 'frequency': 0.0, 'frame_count': 0,
            'yaw_offset': 0.0, 'joystick_x': 0.0
        }

        # History for graphs (circular buffer)
        self.history_length = 100  # 10 seconds at 10Hz
        self.angle_history = {
            'time': deque(maxlen=self.history_length),
            'yaw': deque(maxlen=self.history_length),
            'pitch': deque(maxlen=self.history_length),
            'roll': deque(maxlen=self.history_length)
        }

        # Device lists (thread-safe)
        self.device_lock = threading.Lock()
        self.adb_devices = []
        self.network_devices = []
        self.last_device_scan = 0
        self.device_scanner_running = True

        # Buttons
        self.buttons = []
        self.init_buttons()

        # Create window
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

    def init_buttons(self):
        """Initialize ADB control buttons"""
        def open_app_callback():
            success, output = execute_adb_command(ADB_OPEN_APP)
            print(f"[ADB] Open app: {'Success' if success else 'Failed'}")

        def stop_app_callback():
            success, output = execute_adb_command(ADB_STOP_APP)
            print(f"[ADB] Stop app: {'Success' if success else 'Failed'}")

        self.buttons.append(Button(50, 670, 150, 40, "Open App", open_app_callback))
        self.buttons.append(Button(220, 670, 150, 40, "Stop App", stop_app_callback))

    def update_data(self, **kwargs):
        """Thread-safe update of current data"""
        self.current_data.update(kwargs)

        # Add to history
        if len(self.angle_history['time']) == 0 or time.time() - self.angle_history['time'][-1] > 0.05:
            current_time = time.time()
            self.angle_history['time'].append(current_time)
            self.angle_history['yaw'].append(self.current_data['yaw'])
            self.angle_history['pitch'].append(self.current_data['pitch'])
            self.angle_history['roll'].append(self.current_data['roll'])

    def refresh_devices(self):
        """Refresh device lists (thread-safe, called from background thread)"""
        adb_devs = get_adb_devices()
        net_devs = scan_local_network()

        with self.device_lock:
            self.adb_devices = adb_devs
            self.network_devices = net_devs
            self.last_device_scan = time.time()

    def get_device_info(self):
        """Get device info in thread-safe manner"""
        with self.device_lock:
            return {
                'adb_devices': self.adb_devices.copy(),
                'network_devices': self.network_devices.copy(),
                'last_scan': self.last_device_scan
            }

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks on buttons"""
        if event == cv2.EVENT_LBUTTONDOWN:
            for button in self.buttons:
                if button.is_clicked(x, y):
                    button.callback()

    def draw_frame(self):
        """Draw complete GUI frame"""
        # Create blank canvas (light gray background)
        frame = np.ones((self.height, self.width, 3), dtype=np.uint8) * 240

        # Draw all components
        self.draw_header(frame)
        self.draw_3d_orientation(frame, x=160, y=180, size=200)
        self.draw_angle_gauges(frame, x=550, y=150)
        self.draw_device_list(frame, x=1000, y=60)
        self.draw_angle_history(frame, x=50, y=400)
        self.draw_status_info(frame)
        self.draw_buttons(frame)

        return frame

    def draw_header(self, frame):
        """Draw header with title and status"""
        cv2.rectangle(frame, (0, 0), (self.width, 50), (60, 60, 60), -1)
        cv2.putText(frame, "HEAD ROTATION DATA SENDER", (20, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        status = self.current_data['status']
        freq = self.current_data['frequency']
        status_text = f"[{status}] {freq:.1f} Hz"
        cv2.putText(frame, status_text, (900, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 255, 100), 2)

    def draw_status_info(self, frame):
        """Draw status information"""
        y_pos = 610

        # Endpoints info
        endpoints_text = f"Endpoints: {', '.join([f'{ip}:{port}' for ip, port in TARGET_ENDPOINTS])}"
        cv2.putText(frame, endpoints_text, (50, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # Joystick and offset info
        y_pos += 25
        joystick = self.current_data['joystick_x']
        yaw_offset = self.current_data['yaw_offset']
        frame_count = self.current_data['frame_count']

        info_text = f"Joystick: {joystick:>5.2f}  |  Yaw Offset: {yaw_offset:>7.2f}°  |  Frame: {frame_count:05d}"
        cv2.putText(frame, info_text, (50, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    def draw_buttons(self, frame):
        """Draw ADB control buttons and controls info"""
        for button in self.buttons:
            button.draw(frame)

        # Draw controls info
        cv2.putText(frame, "Controls: R=Reset | ESC/Q=Quit", (400, 690),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    def draw_device_list(self, frame, x, y):
        """Draw list of connected devices"""
        box_width = 260
        box_height = 320

        # Get device info in thread-safe manner
        device_info = self.get_device_info()
        adb_devices = device_info['adb_devices']
        network_devices = device_info['network_devices']
        last_scan = device_info['last_scan']

        # Draw background box
        cv2.rectangle(frame, (x, y), (x + box_width, y + box_height), (255, 255, 255), -1)
        cv2.rectangle(frame, (x, y), (x + box_width, y + box_height), (100, 100, 100), 2)

        # Title
        cv2.putText(frame, "Connected Devices", (x + 10, y + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        y_offset = y + 50

        # ADB Devices Section
        cv2.putText(frame, "ADB Devices:", (x + 10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 100), 1)
        y_offset += 20

        if adb_devices:
            for device in adb_devices[:10]:  # Show max 10 devices
                device_id = device['id']
                if len(device_id) > 20:
                    device_id = device_id[:17] + "..."
                status = device['status']

                # Status indicator
                color = (0, 200, 0) if status == 'device' else (150, 150, 0)
                cv2.circle(frame, (x + 15, y_offset - 3), 4, color, -1)

                # Device info
                cv2.putText(frame, device_id, (x + 25, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
                y_offset += 18

            # Show truncation indicator if more devices
            if len(adb_devices) > 10:
                cv2.putText(frame, f"  ... and {len(adb_devices) - 10} more", (x + 15, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)
                y_offset += 16
        else:
            cv2.putText(frame, "  No devices", (x + 15, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
            y_offset += 18

        y_offset += 15

        # Network Devices Section
        cv2.putText(frame, "Network Devices:", (x + 10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 0), 1)
        y_offset += 20

        if network_devices:
            # Show target endpoints first
            target_ips = [ip for ip, port in TARGET_ENDPOINTS]

            for ip in target_ips:
                is_online = ip in network_devices
                color = (0, 200, 0) if is_online else (200, 0, 0)
                status_text = "ONLINE" if is_online else "OFFLINE"

                # Status indicator
                cv2.circle(frame, (x + 15, y_offset - 3), 4, color, -1)

                # IP and status
                cv2.putText(frame, f"{ip}", (x + 25, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
                cv2.putText(frame, status_text, (x + 180, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
                y_offset += 18

            # Show other discovered devices
            other_devices = [ip for ip in network_devices if ip not in target_ips]
            if other_devices and y_offset < y + box_height - 30:
                y_offset += 10
                cv2.putText(frame, "Other Devices:", (x + 10, y_offset),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)
                y_offset += 15

                for ip in other_devices[:3]:  # Show max 3 other devices
                    cv2.circle(frame, (x + 15, y_offset - 3), 4, (100, 100, 255), -1)
                    cv2.putText(frame, ip, (x + 25, y_offset),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1)
                    y_offset += 16
        else:
            cv2.putText(frame, "  Scanning...", (x + 15, y_offset),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

        # Last scan time
        if last_scan > 0:
            time_since_scan = int(time.time() - last_scan)
            scan_text = f"Updated {time_since_scan}s ago"
            cv2.putText(frame, scan_text, (x + 10, y + box_height - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 100, 100), 1)

    def draw_3d_orientation(self, frame, x, y, size):
        """Draw 3D cube representing headset orientation"""
        # Define cube vertices in object space
        vertices = np.array([
            [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],  # Back face
            [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]        # Front face
        ], dtype=np.float32) * (size / 4)

        # Get Euler angles in radians
        yaw_rad = self.current_data['yaw'] * np.pi / 180.0
        pitch_rad = self.current_data['pitch'] * np.pi / 180.0
        roll_rad = self.current_data['roll'] * np.pi / 180.0

        # Create rotation matrices
        # Yaw (Z-axis)
        Rz = np.array([
            [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
            [np.sin(yaw_rad), np.cos(yaw_rad), 0],
            [0, 0, 1]
        ])

        # Pitch (Y-axis)
        Ry = np.array([
            [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
            [0, 1, 0],
            [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
        ])

        # Roll (X-axis)
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll_rad), -np.sin(roll_rad)],
            [0, np.sin(roll_rad), np.cos(roll_rad)]
        ])

        # Combined rotation: Roll -> Pitch -> Yaw
        R = Rz @ Ry @ Rx

        # Apply rotation
        rotated = vertices @ R.T

        # Project to 2D (simple orthographic projection)
        projected = rotated[:, :2] + np.array([x, y])
        projected = projected.astype(np.int32)

        # Draw cube edges
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),  # Back face
            (4, 5), (5, 6), (6, 7), (7, 4),  # Front face
            (0, 4), (1, 5), (2, 6), (3, 7)   # Connecting edges
        ]

        for edge in edges:
            pt1 = tuple(projected[edge[0]])
            pt2 = tuple(projected[edge[1]])
            cv2.line(frame, pt1, pt2, (50, 50, 50), 2)

        # Draw axis arrows
        axis_length = size / 2
        axes = np.array([[axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]], dtype=np.float32)
        rotated_axes = axes @ R.T
        axis_2d = rotated_axes[:, :2] + np.array([x, y])
        axis_2d = axis_2d.astype(np.int32)

        # X-axis (Red)
        cv2.arrowedLine(frame, (x, y), tuple(axis_2d[0]), (0, 0, 255), 3, tipLength=0.3)
        # Y-axis (Green)
        cv2.arrowedLine(frame, (x, y), tuple(axis_2d[1]), (0, 255, 0), 3, tipLength=0.3)
        # Z-axis (Blue)
        cv2.arrowedLine(frame, (x, y), tuple(axis_2d[2]), (255, 0, 0), 3, tipLength=0.3)

        # Draw label
        cv2.putText(frame, "3D Orientation", (x - 80, y - 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

    def draw_angle_gauges(self, frame, x, y):
        """Draw circular gauges for yaw, pitch, roll"""
        radius = 70
        spacing = 200

        yaw = self.current_data['yaw']
        pitch = self.current_data['pitch']
        roll = self.current_data['roll']

        # Draw three gauges
        self.draw_single_gauge(frame, x, y, radius, yaw, -180, 180, "Yaw", (0, 0, 200))
        self.draw_single_gauge(frame, x + spacing, y, radius, pitch, -90, 90, "Pitch", (0, 150, 0))
        self.draw_single_gauge(frame, x + 2*spacing, y, radius, roll, -90, 90, "Roll", (200, 0, 0))

    def draw_single_gauge(self, frame, x, y, radius, angle, min_angle, max_angle, label, color):
        """Draw a single circular gauge"""
        # Draw outer circle
        cv2.circle(frame, (x, y), radius, (100, 100, 100), 3)

        # Draw angle markings (every 30 degrees)
        for mark_angle in range(int(min_angle), int(max_angle) + 1, 30):
            # Convert angle to gauge position (bottom = min, top = max)
            gauge_angle = -180 + (mark_angle - min_angle) / (max_angle - min_angle) * 180
            rad = gauge_angle * np.pi / 180.0

            outer_x = int(x + (radius - 5) * np.cos(rad))
            outer_y = int(y + (radius - 5) * np.sin(rad))
            inner_x = int(x + (radius - 15) * np.cos(rad))
            inner_y = int(y + (radius - 15) * np.sin(rad))

            cv2.line(frame, (outer_x, outer_y), (inner_x, inner_y), (100, 100, 100), 2)

        # Draw needle
        # Map angle to gauge position
        gauge_angle = -180 + (angle - min_angle) / (max_angle - min_angle) * 180
        rad = gauge_angle * np.pi / 180.0

        needle_x = int(x + (radius - 20) * np.cos(rad))
        needle_y = int(y + (radius - 20) * np.sin(rad))

        cv2.line(frame, (x, y), (needle_x, needle_y), color, 4)
        cv2.circle(frame, (x, y), 8, color, -1)

        # Draw label
        cv2.putText(frame, label, (x - 25, y + radius + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        # Draw value
        cv2.putText(frame, f"{angle:.1f}°", (x - 30, y + radius + 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    def draw_angle_history(self, frame, x, y):
        """Draw line graphs of angle history"""
        graph_width = 1180
        graph_height = 180

        # Draw background
        cv2.rectangle(frame, (x, y), (x + graph_width, y + graph_height), (255, 255, 255), -1)
        cv2.rectangle(frame, (x, y), (x + graph_width, y + graph_height), (100, 100, 100), 2)

        # Draw title
        cv2.putText(frame, "Angle History (Last 10 seconds)", (x + 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        # Draw legend
        cv2.line(frame, (x + graph_width - 250, y + 15), (x + graph_width - 220, y + 15), (0, 0, 255), 2)
        cv2.putText(frame, "Yaw", (x + graph_width - 210, y + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

        cv2.line(frame, (x + graph_width - 160, y + 15), (x + graph_width - 130, y + 15), (0, 255, 0), 2)
        cv2.putText(frame, "Pitch", (x + graph_width - 120, y + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

        cv2.line(frame, (x + graph_width - 70, y + 15), (x + graph_width - 40, y + 15), (255, 0, 0), 2)
        cv2.putText(frame, "Roll", (x + graph_width - 30, y + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

        # Draw grid lines
        for i in range(5):
            grid_y = y + int((i + 1) * graph_height / 5)
            cv2.line(frame, (x, grid_y), (x + graph_width, grid_y), (200, 200, 200), 1)

        # Plot data if available
        if len(self.angle_history['time']) > 1:
            times = list(self.angle_history['time'])
            yaws = list(self.angle_history['yaw'])
            pitches = list(self.angle_history['pitch'])
            rolls = list(self.angle_history['roll'])

            # Normalize time to graph width
            time_range = max(times) - min(times)
            if time_range > 0:
                # Plot each angle
                self.plot_line(frame, x, y, graph_width, graph_height, times, yaws, -180, 180, (0, 0, 255))
                self.plot_line(frame, x, y, graph_width, graph_height, times, pitches, -90, 90, (0, 255, 0))
                self.plot_line(frame, x, y, graph_width, graph_height, times, rolls, -90, 90, (255, 0, 0))

    def plot_line(self, frame, x_base, y_base, width, height, times, values, min_val, max_val, color):
        """Plot a single line on the graph"""
        if len(times) < 2:
            return

        time_min = min(times)
        time_max = max(times)
        time_range = time_max - time_min

        if time_range == 0:
            return

        points = []
        for t, v in zip(times, values):
            # Normalize time to width
            px = x_base + int((t - time_min) / time_range * width)
            # Normalize value to height (inverted Y-axis)
            py = y_base + height - int((v - min_val) / (max_val - min_val) * height)
            py = max(y_base, min(y_base + height, py))  # Clamp
            points.append((px, py))

        # Draw lines between points
        for i in range(len(points) - 1):
            cv2.line(frame, points[i], points[i+1], color, 2)

    def run(self):
        """Main GUI loop"""
        while self.running:
            frame = self.draw_frame()
            cv2.imshow(self.window_name, frame)

            key = cv2.waitKey(30)  # 30ms = ~33 FPS
            if key == 27 or key == ord('q'):  # ESC or Q
                self.running = False
                break

        # Stop device scanner
        self.device_scanner_running = False
        cv2.destroyAllWindows()


# Global GUI instance
gui = None


def gui_worker():
    """Worker function for GUI thread"""
    global gui
    gui = HeadRotationGUI()
    gui.run()


def device_scanner_worker():
    """Background worker for scanning devices"""
    global gui
    # Wait for GUI to initialize
    while gui is None:
        time.sleep(0.1)

    # Initial scan
    gui.refresh_devices()

    # Scan every 5 seconds
    while gui.device_scanner_running:
        time.sleep(5)
        if gui.device_scanner_running:
            gui.refresh_devices()


try:
    print("Initializing XrClient...")
    xr_client = XrClient()
    print("XrClient initialized.\n")

    # Start GUI thread
    gui_thread = threading.Thread(target=gui_worker, daemon=True)
    gui_thread.start()
    print("GUI thread started.\n")

    # Start device scanner thread
    device_scanner_thread = threading.Thread(target=device_scanner_worker, daemon=True)
    device_scanner_thread.start()
    print("Device scanner thread started.\n")

    # Wait a moment for GUI to initialize
    time.sleep(0.5)

    # Start keyboard listener in a separate thread
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    print("Keyboard listener started.\n")

    frame_count = 0
    start_time = time.time()
    last_time = time.time()  # Track delta time for smooth rotation
    yaw_offset = 0.0  # Yaw offset for reset functionality
    joystick_offset = 0.0  # Accumulated joystick rotation offset
    joystick_x = 0.0  # Current joystick horizontal input (for display)
    reset_button_pressed = False  # Track button state to detect single press

    while True:
        current_time = time.time()
        delta_time = current_time - last_time
        last_time = current_time
        head_pose = xr_client.get_pose_by_name("headset")
        if head_pose is not None:
            # Extract quaternion data
            quaternion = head_pose[3:]  # [qx, qy, qz, qw]

            # Convert to Euler angles [yaw, pitch, roll]
            euler_angles = quaternion_to_euler(quaternion)

            # Read joystick input from both controllers
            left_joystick = xr_client.get_joystick_state("left")
            right_joystick = xr_client.get_joystick_state("right")
            left_x = apply_deadzone(left_joystick[0])
            right_x = apply_deadzone(right_joystick[0])

            # Combine both joystick inputs
            joystick_x = left_x + right_x

            # Accumulate joystick rotation into offset
            joystick_rotation_delta = joystick_x * (JOYSTICK_ROTATION_SPEED * np.pi / 180.0) * delta_time
            joystick_offset += joystick_rotation_delta

            # Check for reset triggers (A button or R key)
            a_button_state = xr_client.get_button_state_by_name("A")

            if (a_button_state and not reset_button_pressed) or keyboard_reset_triggered:
                # Reset triggered - capture current yaw as offset and reset joystick offset
                yaw_offset = euler_angles[0]
                joystick_offset = 0.0  # Also reset joystick offset
                reset_button_pressed = True
                keyboard_reset_triggered = False  # Reset keyboard flag
                source = "A Button" if a_button_state else "R Key"
                print(f"\n[RESET via {source}] All offsets reset\n")
            elif not a_button_state:
                # Button released - ready for next press
                reset_button_pressed = False

            # Apply both offsets to yaw
            euler_angles[0] -= yaw_offset  # Subtract reset offset
            euler_angles[0] += joystick_offset  # Add joystick rotation offset

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

        # Update GUI instead of printing to terminal
        if gui is not None:
            gui.update_data(
                yaw=yaw_deg,
                pitch=pitch_deg,
                roll=roll_deg,
                status=status,
                frequency=frequency,
                frame_count=frame_count,
                yaw_offset=yaw_offset * 180.0 / np.pi,
                joystick_x=joystick_x
            )

        # Check if GUI window was closed
        if gui is not None and not gui.running:
            print("\n[PC] GUI closed, stopping sender.")
            break

        # Wait for a short period
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n[PC] Stopping sender.")
except Exception as e:
    print(f"ERROR: An error occurred: {e}")
    import traceback
    traceback.print_exc()
finally:
    # Stop GUI
    if gui is not None:
        gui.running = False
        print("GUI closed.")

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