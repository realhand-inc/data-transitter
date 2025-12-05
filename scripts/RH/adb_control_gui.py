import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter import scrolledtext
import numpy as np
from collections import deque
import sys
import os
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math
import zmq

# Ensure xrobotoolkit_teleop is in the Python path
script_dir = os.path.dirname(__file__)
project_root = os.path.abspath(os.path.join(script_dir, os.pardir, os.pardir))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from xrobotoolkit_teleop.common.xr_client import XrClient
from test_adb_simple import ADB_PACKAGE_NAME, execute_adb_command, get_connected_adb_devices


DEVICE_REFRESH_MS = 5000


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


class AdbControlApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("RealHand TeleOp")
        self.devices = []
        self.status_var = tk.StringVar(value="Idle")
        self.ip_var = tk.StringVar()

        # Head rotation data storage
        self.rotation_data = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}
        self.data_status = 'INIT'
        self.data_frequency = 0.0
        self.frame_count = 0
        self.data_lock = threading.Lock()

        # History buffer for angle data (last 100 samples, ~10 seconds at 10Hz)
        self.history_length = 100
        self.angle_history = {
            'time': deque(maxlen=self.history_length),
            'yaw': deque(maxlen=self.history_length),
            'pitch': deque(maxlen=self.history_length),
            'roll': deque(maxlen=self.history_length)
        }

        # XrClient (initialized later)
        self.xr_client = None
        self.data_thread = None
        self.running = True

        # ZMQ for rotation data sending
        self.zmq_context = zmq.Context()
        self.rotation_sockets = []
        self.rotation_endpoints = []
        self.rotation_ip_var = tk.StringVar(value="192.168.1.56:5555")

        main = ttk.Frame(root, padding=12)
        main.grid(column=0, row=0, sticky="nsew")
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        title = ttk.Label(main, text="RealHand TeleOp", font=("TkDefaultFont", 16, "bold"))
        title.grid(column=0, row=0, padx=8, pady=(0, 8), sticky="w")

        self.build_devices_frame(main)
        self.build_data_frame(main)
        self.build_actions_frame(main)
        self.build_log_frame(main)

        self.refresh_devices()
        self.schedule_refresh()

        # Initialize XrClient and start data collection
        self.init_xr_client()

        # Start UI update loop
        self.update_data_display()

    def build_devices_frame(self, parent: ttk.Frame):
        frame = ttk.LabelFrame(parent, text="Devices")
        frame.grid(column=0, row=1, padx=8, pady=4, sticky="nsew")
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(1, weight=1)

        self.devices_list = tk.Listbox(frame, height=8, selectmode=tk.MULTIPLE, exportselection=False)
        self.devices_list.grid(column=0, row=0, rowspan=4, padx=(8, 4), pady=8, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(3, weight=1)

        refresh_btn = ttk.Button(frame, text="Refresh", command=self.refresh_devices)
        refresh_btn.grid(column=1, row=0, padx=4, pady=(8, 2), sticky="ew")

        ttk.Label(frame, text="IP:Port").grid(column=1, row=1, padx=4, pady=2, sticky="w")
        ip_entry = ttk.Entry(frame, textvariable=self.ip_var, width=18)
        ip_entry.grid(column=1, row=2, padx=4, pady=2, sticky="ew")
        connect_btn = ttk.Button(frame, text="Connect over IP", command=self.connect_ip)
        connect_btn.grid(column=1, row=3, padx=4, pady=(2, 8), sticky="ew")

        self.status_label = ttk.Label(frame, textvariable=self.status_var, foreground="green")
        self.status_label.grid(column=0, row=4, columnspan=2, padx=8, pady=(0, 8), sticky="w")

        # Separator
        ttk.Separator(frame, orient='horizontal').grid(column=0, row=5, columnspan=2, padx=8, pady=8, sticky="ew")

        # Rotation data endpoint section
        ttk.Label(frame, text="Rotation Data Endpoint", font=("TkDefaultFont", 9, "bold")).grid(column=0, row=6, columnspan=2, padx=8, pady=(0, 4), sticky="w")

        ttk.Label(frame, text="IP:Port").grid(column=1, row=7, padx=4, pady=2, sticky="w")
        rotation_ip_entry = ttk.Entry(frame, textvariable=self.rotation_ip_var, width=18)
        rotation_ip_entry.grid(column=1, row=8, padx=4, pady=2, sticky="ew")

        rotation_connect_btn = ttk.Button(frame, text="Connect", command=self.connect_rotation_endpoint)
        rotation_connect_btn.grid(column=1, row=9, padx=4, pady=(2, 4), sticky="ew")

        # List of connected rotation endpoints
        ttk.Label(frame, text="Connected Endpoints:", font=("TkDefaultFont", 8)).grid(column=0, row=7, padx=8, pady=(0, 2), sticky="w")
        self.rotation_endpoints_listbox = tk.Listbox(frame, height=3, selectmode=tk.SINGLE)
        self.rotation_endpoints_listbox.grid(column=0, row=8, rowspan=2, padx=(8, 4), pady=2, sticky="nsew")

        disconnect_rotation_btn = ttk.Button(frame, text="Disconnect", command=self.disconnect_rotation_endpoint)
        disconnect_rotation_btn.grid(column=0, row=10, padx=8, pady=(2, 8), sticky="ew")

    def build_data_frame(self, parent: ttk.Frame):
        """Build the head rotation data display frame with gauges and history graph"""
        frame = ttk.LabelFrame(parent, text="Head Rotation Data")
        frame.grid(column=0, row=2, padx=8, pady=4, sticky="nsew")
        parent.rowconfigure(2, weight=1)

        # Left side: Gauges section
        gauges_frame = ttk.Frame(frame)
        gauges_frame.grid(column=0, row=0, padx=8, pady=8, sticky="nsew")

        # Status label
        self.data_status_label = ttk.Label(gauges_frame, text="Status: INIT | 0.0 Hz", font=("TkDefaultFont", 10))
        self.data_status_label.grid(column=0, row=0, columnspan=3, padx=4, pady=(0, 8))

        # Create three canvas widgets for gauges
        gauge_size = 120
        self.yaw_canvas = tk.Canvas(gauges_frame, width=gauge_size, height=gauge_size + 40, bg='white', highlightthickness=1)
        self.yaw_canvas.grid(column=0, row=1, padx=8, pady=4)

        self.pitch_canvas = tk.Canvas(gauges_frame, width=gauge_size, height=gauge_size + 40, bg='white', highlightthickness=1)
        self.pitch_canvas.grid(column=1, row=1, padx=8, pady=4)

        self.roll_canvas = tk.Canvas(gauges_frame, width=gauge_size, height=gauge_size + 40, bg='white', highlightthickness=1)
        self.roll_canvas.grid(column=2, row=1, padx=8, pady=4)

        # Draw initial gauges
        self.draw_gauge(self.yaw_canvas, 0, -180, 180, "Yaw", "#0000FF")
        self.draw_gauge(self.pitch_canvas, 0, -90, 90, "Pitch", "#009600")
        self.draw_gauge(self.roll_canvas, 0, -90, 90, "Roll", "#C80000")

        # Right side: History graph
        history_frame = ttk.Frame(frame)
        history_frame.grid(column=1, row=0, padx=8, pady=8, sticky="nsew")
        frame.columnconfigure(1, weight=1)
        frame.rowconfigure(0, weight=1)

        # Create matplotlib figure
        self.fig = Figure(figsize=(8, 3), dpi=80)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Angle History (Last 10 seconds)")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Angle (degrees)")
        self.ax.grid(True, alpha=0.3)
        self.ax.set_ylim(-180, 180)

        # Embed matplotlib figure in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=history_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def draw_gauge(self, canvas: tk.Canvas, angle: float, min_angle: float, max_angle: float, label: str, color: str):
        """Draw a circular gauge on the canvas"""
        canvas.delete("all")

        width = int(canvas.cget("width"))
        height = int(canvas.cget("height"))
        center_x = width // 2
        center_y = (height - 40) // 2
        radius = min(center_x, center_y) - 10

        # Draw outer circle
        canvas.create_oval(center_x - radius, center_y - radius,
                          center_x + radius, center_y + radius,
                          outline='gray', width=2)

        # Draw tick marks every 30 degrees
        for mark_angle in range(int(min_angle), int(max_angle) + 1, 30):
            # Convert angle to gauge position (bottom = min, top = max)
            gauge_angle = -180 + (mark_angle - min_angle) / (max_angle - min_angle) * 180
            rad = math.radians(gauge_angle)

            outer_x = center_x + int((radius - 5) * math.cos(rad))
            outer_y = center_y + int((radius - 5) * math.sin(rad))
            inner_x = center_x + int((radius - 15) * math.cos(rad))
            inner_y = center_y + int((radius - 15) * math.sin(rad))

            canvas.create_line(outer_x, outer_y, inner_x, inner_y, fill='gray', width=2)

        # Draw needle
        gauge_angle = -180 + (angle - min_angle) / (max_angle - min_angle) * 180
        rad = math.radians(gauge_angle)

        needle_x = center_x + int((radius - 20) * math.cos(rad))
        needle_y = center_y + int((radius - 20) * math.sin(rad))

        canvas.create_line(center_x, center_y, needle_x, needle_y, fill=color, width=3)
        canvas.create_oval(center_x - 6, center_y - 6, center_x + 6, center_y + 6, fill=color, outline=color)

        # Draw label
        canvas.create_text(center_x, height - 30, text=label, font=("TkDefaultFont", 10, "bold"))

        # Draw value
        canvas.create_text(center_x, height - 10, text=f"{angle:.1f}°", font=("TkDefaultFont", 9))

    def build_actions_frame(self, parent: ttk.Frame):
        frame = ttk.LabelFrame(parent, text="Actions")
        frame.grid(column=0, row=3, padx=8, pady=4, sticky="ew")

        start_btn = ttk.Button(frame, text="Start", command=self.restart_app)
        start_btn.grid(column=0, row=0, padx=6, pady=6, sticky="ew")

        stop_btn = ttk.Button(frame, text="Stop", command=self.stop_app)
        stop_btn.grid(column=1, row=0, padx=6, pady=6, sticky="ew")

        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)

    def build_log_frame(self, parent: ttk.Frame):
        frame = ttk.LabelFrame(parent, text="Log")
        frame.grid(column=0, row=4, padx=8, pady=4, sticky="nsew")
        parent.rowconfigure(4, weight=1)

        self.log_widget = scrolledtext.ScrolledText(frame, wrap=tk.WORD, height=12, state="disabled")
        self.log_widget.grid(column=0, row=0, padx=8, pady=8, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)

    def schedule_refresh(self):
        self.root.after(DEVICE_REFRESH_MS, self.periodic_refresh)

    def periodic_refresh(self):
        self.refresh_devices()
        self.schedule_refresh()

    def set_status(self, text: str):
        self.status_var.set(text)

    def log(self, message: str):
        timestamp = time.strftime("%H:%M:%S")
        def _append():
            self.log_widget.configure(state="normal")
            self.log_widget.insert(tk.END, f"[{timestamp}] {message}\n")
            self.log_widget.see(tk.END)
            self.log_widget.configure(state="disabled")
        self.root.after(0, _append)

    def refresh_devices(self):
        self.set_status("Refreshing devices...")
        devices = get_connected_adb_devices()
        self.devices = devices
        self.devices_list.delete(0, tk.END)
        for dev in devices:
            self.devices_list.insert(tk.END, dev)
        if devices:
            self.set_status(f"Found {len(devices)} device(s)")
        else:
            self.set_status("No devices found")

    def selected_devices(self):
        indices = self.devices_list.curselection()
        if indices:
            return [self.devices[i] for i in indices]
        return self.devices[:1]  # Default to first device if none selected

    def connect_ip(self):
        ip = self.ip_var.get().strip()
        if not ip:
            messagebox.showinfo("ADB Control", "Enter an IP (with optional port) before connecting.")
            return

        def _connect():
            self.set_status(f"Connecting to {ip}...")
            success, output = execute_adb_command(f"adb connect {ip}")
            self.log(f"connect {ip}: {'ok' if success else 'failed'} | {output.strip()}")
            self.refresh_devices()
            self.set_status("Idle")

        threading.Thread(target=_connect, daemon=True).start()

    def disconnect_all(self):
        def _disconnect():
            self.set_status("Disconnecting all devices...")
            success, output = execute_adb_command("adb disconnect")
            self.log(f"disconnect all: {'ok' if success else 'failed'} | {output.strip()}")
            self.refresh_devices()
            self.set_status("Idle")

        threading.Thread(target=_disconnect, daemon=True).start()

    def connect_rotation_endpoint(self):
        """Connect to a rotation data endpoint (IP:Port)"""
        endpoint = self.rotation_ip_var.get().strip()
        if not endpoint:
            messagebox.showinfo("Rotation Data", "Enter an IP:Port before connecting.")
            return

        try:
            # Parse IP and port
            if ':' not in endpoint:
                messagebox.showerror("Rotation Data", "Format must be IP:Port (e.g., 192.168.1.56:5555)")
                return

            ip, port = endpoint.split(':')
            port = int(port)

            # Check if already connected
            if endpoint in self.rotation_endpoints:
                messagebox.showinfo("Rotation Data", f"Already connected to {endpoint}")
                return

            # Create and connect ZMQ socket
            sock = self.zmq_context.socket(zmq.PUSH)
            sock.connect(f"tcp://{ip}:{port}")

            # Add to lists
            self.rotation_sockets.append(sock)
            self.rotation_endpoints.append(endpoint)
            self.rotation_endpoints_listbox.insert(tk.END, endpoint)

            self.log(f"Connected rotation data endpoint: {endpoint}")
            self.set_status(f"Connected to {endpoint}")

        except Exception as e:
            messagebox.showerror("Rotation Data", f"Failed to connect: {e}")
            self.log(f"Failed to connect to {endpoint}: {e}")

    def disconnect_rotation_endpoint(self):
        """Disconnect selected rotation data endpoint"""
        selection = self.rotation_endpoints_listbox.curselection()
        if not selection:
            messagebox.showinfo("Rotation Data", "Select an endpoint to disconnect.")
            return

        index = selection[0]
        endpoint = self.rotation_endpoints[index]

        try:
            # Close socket
            sock = self.rotation_sockets[index]
            sock.close()

            # Remove from lists
            del self.rotation_sockets[index]
            del self.rotation_endpoints[index]
            self.rotation_endpoints_listbox.delete(index)

            self.log(f"Disconnected rotation data endpoint: {endpoint}")
            self.set_status("Idle")

        except Exception as e:
            messagebox.showerror("Rotation Data", f"Failed to disconnect: {e}")
            self.log(f"Failed to disconnect from {endpoint}: {e}")

    def stop_app(self):
        self.run_for_devices("stop app", self._stop_app_on_device)

    def restart_app(self):
        self.run_for_devices("restart app", self._restart_app_on_device)

    def run_for_devices(self, label: str, action):
        devices = self.selected_devices()
        if not devices:
            self.log(f"{label}: no connected devices")
            self.set_status("No devices")
            return

        for device in devices:
            threading.Thread(target=self._run_action, args=(device, label, action), daemon=True).start()

    def _run_action(self, device: str, label: str, action):
        self.set_status(f"{label} on {device}")
        try:
            action(device)
        finally:
            self.set_status("Idle")

    def _open_app_on_device(self, device: str):
        cmd = f"adb -s {device} shell monkey -p {ADB_PACKAGE_NAME} -c android.intent.category.LAUNCHER 1"
        success, output = execute_adb_command(cmd)
        self.log(f"[{device}] open app: {'ok' if success else 'failed'} | {output.strip()}")

    def _stop_app_on_device(self, device: str):
        cmd = f"adb -s {device} shell am force-stop {ADB_PACKAGE_NAME}"
        success, output = execute_adb_command(cmd)
        self.log(f"[{device}] stop app: {'ok' if success else 'failed'} | {output.strip()}")

    def _restart_app_on_device(self, device: str):
        self._stop_app_on_device(device)
        time.sleep(1)
        self._open_app_on_device(device)

    def init_xr_client(self):
        """Initialize XrClient and start data collection thread"""
        try:
            self.log("Initializing XrClient...")
            self.xr_client = XrClient()
            self.log("XrClient initialized successfully")

            # Start data collection thread
            self.data_thread = threading.Thread(target=self.data_collection_worker, daemon=True)
            self.data_thread.start()
            self.log("Data collection thread started")
        except Exception as e:
            self.log(f"Failed to initialize XrClient: {e}")
            self.data_status = 'ERROR'

    def send_rotation_data(self, yaw_deg: float, pitch_deg: float, roll_deg: float, timestamp: float):
        """Send rotation data to all connected ZMQ endpoints"""
        if not self.rotation_sockets:
            return

        try:
            # Format: CSV format matching test_head_rotation_sender.py
            data_to_send = f"{yaw_deg:.2f}, {pitch_deg:.2f}, {roll_deg:.2f}, {timestamp:.6f}"

            # Send to all connected endpoints
            for sock in self.rotation_sockets:
                try:
                    sock.send_string(data_to_send, zmq.NOBLOCK)
                except zmq.Again:
                    pass  # Socket buffer full, skip this send
                except Exception as e:
                    pass  # Silently ignore individual socket errors

        except Exception as e:
            pass  # Silently ignore sending errors to avoid flooding logs

    def data_collection_worker(self):
        """Background worker for collecting head rotation data from XrClient"""
        start_time = time.time()
        frame_count = 0

        while self.running and self.xr_client:
            try:
                # Get headset pose
                head_pose = self.xr_client.get_pose_by_name("headset")

                if head_pose is not None:
                    # Extract quaternion and convert to Euler angles
                    quaternion = head_pose[3:]  # [qx, qy, qz, qw]
                    euler_angles = quaternion_to_euler(quaternion)

                    # Convert to degrees
                    rad_to_deg = 180.0 / np.pi
                    yaw_deg = euler_angles[0] * rad_to_deg
                    pitch_deg = euler_angles[1] * rad_to_deg
                    roll_deg = euler_angles[2] * rad_to_deg

                    # Send rotation data to ZMQ endpoints
                    # Use original yaw in -180/180 range for data transmission
                    current_timestamp = time.time()
                    self.send_rotation_data(yaw_deg, pitch_deg, roll_deg, current_timestamp)

                    # Update data with thread lock (keep yaw in -180/180 range)
                    with self.data_lock:
                        self.rotation_data['yaw'] = yaw_deg
                        self.rotation_data['pitch'] = pitch_deg
                        self.rotation_data['roll'] = roll_deg
                        self.data_status = 'OK'

                        # Add to history
                        current_time = time.time()
                        if len(self.angle_history['time']) == 0 or current_time - self.angle_history['time'][-1] > 0.05:
                            self.angle_history['time'].append(current_time)
                            self.angle_history['yaw'].append(yaw_deg)
                            self.angle_history['pitch'].append(pitch_deg)
                            self.angle_history['roll'].append(roll_deg)

                        # Calculate frequency
                        frame_count += 1
                        elapsed = time.time() - start_time
                        if elapsed > 0:
                            self.data_frequency = frame_count / elapsed
                            self.frame_count = frame_count
                else:
                    with self.data_lock:
                        self.data_status = 'NO HEADSET'

                # Sleep to maintain ~10Hz update rate
                time.sleep(0.1)

            except Exception as e:
                with self.data_lock:
                    self.data_status = f'ERROR: {str(e)}'
                time.sleep(0.1)

    def update_data_display(self):
        """Update the data display (gauges and graph) periodically"""
        try:
            # Get current data with thread lock
            with self.data_lock:
                yaw = self.rotation_data['yaw']
                pitch = self.rotation_data['pitch']
                roll = self.rotation_data['roll']
                status = self.data_status
                freq = self.data_frequency

                # Copy history for plotting
                times = list(self.angle_history['time'])
                yaws = list(self.angle_history['yaw'])
                pitches = list(self.angle_history['pitch'])
                rolls = list(self.angle_history['roll'])

            # Update status label
            self.data_status_label.config(text=f"Status: {status} | {freq:.1f} Hz")

            # Update gauges
            self.draw_gauge(self.yaw_canvas, yaw, -180, 180, "Yaw", "#0000FF")
            self.draw_gauge(self.pitch_canvas, pitch, -90, 90, "Pitch", "#009600")
            self.draw_gauge(self.roll_canvas, roll, -90, 90, "Roll", "#C80000")

            # Update history graph
            if len(times) > 1:
                self.ax.clear()
                self.ax.set_title("Angle History (Last 10 seconds)")
                self.ax.set_xlabel("Time (s)")
                self.ax.set_ylabel("Angle (degrees)")
                self.ax.grid(True, alpha=0.3)

                # Normalize time to show last 10 seconds
                current_time = times[-1]
                relative_times = [t - current_time for t in times]

                # Plot angles
                self.ax.plot(relative_times, yaws, 'b-', label='Yaw', linewidth=2)
                self.ax.plot(relative_times, pitches, 'g-', label='Pitch', linewidth=2)
                self.ax.plot(relative_times, rolls, 'r-', label='Roll', linewidth=2)

                self.ax.legend(loc='upper right')
                self.ax.set_xlim(-10, 0)
                self.ax.set_ylim(-180, 180)

                self.canvas.draw()

        except Exception as e:
            pass  # Silently ignore display update errors

        # Schedule next update (100ms)
        if self.running:
            self.root.after(100, self.update_data_display)

    def cleanup(self):
        """Cleanup resources before closing"""
        self.running = False

        # Close XrClient
        if self.xr_client:
            try:
                self.xr_client.close()
                self.log("XrClient closed")
            except Exception as e:
                self.log(f"Error closing XrClient: {e}")

        # Close all ZMQ sockets
        for sock in self.rotation_sockets:
            try:
                sock.close()
            except Exception:
                pass

        # Terminate ZMQ context
        try:
            self.zmq_context.term()
            self.log("ZMQ context terminated")
        except Exception as e:
            self.log(f"Error terminating ZMQ context: {e}")


def main():
    root = tk.Tk()
    app = AdbControlApp(root)

    # Register cleanup handler for window close
    def on_closing():
        app.cleanup()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
