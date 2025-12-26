import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
from collections import deque
import sys
import os
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math
import zmq
import re
import base64
import subprocess
import json
# Ensure xrobotoolkit_teleop and local app are in the Python path
script_dir = os.path.dirname(__file__)
project_root = os.path.abspath(os.path.join(script_dir, os.pardir, os.pardir))
if project_root not in sys.path:
    sys.path.insert(0, project_root)
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from xrobotoolkit_teleop.utils.dex_hand_utils import pico_hand_state_to_mediapipe, pico_to_mediapipe
from app.tabs.dashboard_tab import DashboardTabMixin
from app.tabs.raw_data_tab import RawDataTabMixin
from app.tabs.mediapipe_tab import MediaPipeTabMixin

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


class AdbControlApp(DashboardTabMixin, RawDataTabMixin, MediaPipeTabMixin):
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("RealHand TeleOp")
        self.root.geometry("1200x1600")
        self.log_visible = False
        self.devices = []
        self.status_var = tk.StringVar(value="Idle")
        self.ip_var = tk.StringVar()
        self.device_vars = {}
        self.palette = {}

        # Head rotation data storage
        self.rotation_data = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}
        self.raw_data_snapshot = {}  # Store full raw data
        self.last_good_raw_snapshot = {}  # Persist last good raw data to freeze UI on loss
        self.mediapipe_points = {"left": None, "right": None}
        self.last_good_mediapipe_points = {"left": None, "right": None}
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
        self.last_data_ts = 0.0
        self.last_send_ts = 0.0

        # XrClient (initialized later)
        self.xr_client = None
        self.data_thread = None
        self.running = True

        # ZMQ for rotation data sending
        self.zmq_context = zmq.Context()
        self.rotation_sockets = []
        self.rotation_endpoints = []
        self.rotation_ip_var = tk.StringVar(value="192.168.1.56:5555")

        # ZMQ for MediaPipe hand data broadcasting
        self.hand_data_sockets = []
        self.hand_data_endpoints = []
        self.hand_data_ip_var = tk.StringVar(value="localhost:5557")

        # ZMQ for full XR data broadcasting (teleop bridge)
        self.xr_data_socket = None
        self.xr_data_port = 5558
        self.xr_data_send_count = 0
        self.xr_data_last_send_ts = 0.0
        self.xr_data_bind_error = None

        # Screenshot viewer state
        self.screenshot_window = None
        self.screenshot_label = None
        self.screenshot_image = None  # keep reference to avoid GC
        
        # Raw Data Sheet Cells Map
        self.sheet_cells = {}

        self.configure_styles()

        main = ttk.Frame(root, padding=12, style="App.TFrame")
        main.grid(column=0, row=0, sticky="nsew")
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        # Header
        header = tk.Frame(main, bg=self.palette["accent"])
        header.grid(column=0, row=0, sticky="ew", pady=(0, 10))
        header.columnconfigure(0, weight=1)

        title = ttk.Label(header, text="RealHand TeleOp", style="Header.TLabel")
        title.grid(column=0, row=0, padx=10, pady=8, sticky="w")
        self.log_toggle_btn = ttk.Button(header, text="Show Logs", style="Ghost.TButton", command=self.toggle_log_frame)
        self.log_toggle_btn.grid(column=1, row=0, padx=10, pady=8, sticky="e")

        # Main Notebook
        self.main_notebook = ttk.Notebook(main)
        self.main_notebook.grid(column=0, row=1, sticky="nsew")
        main.rowconfigure(1, weight=1)
        main.columnconfigure(0, weight=1)

        # Tab 1: Dashboard
        self.dashboard_tab = ttk.Frame(self.main_notebook, style="App.TFrame")
        self.main_notebook.add(self.dashboard_tab, text="Dashboard")
        self.dashboard_tab.columnconfigure(0, weight=1)
        
        # Tab 2: Raw Data Monitor
        self.raw_data_tab = ttk.Frame(self.main_notebook, style="App.TFrame")
        self.main_notebook.add(self.raw_data_tab, text="Raw Data Monitor")
        self.raw_data_tab.columnconfigure(0, weight=1)
        self.raw_data_tab.rowconfigure(0, weight=1)

        # Tab 3: MediaPipe
        self.mediapipe_tab = ttk.Frame(self.main_notebook, style="App.TFrame")
        self.main_notebook.add(self.mediapipe_tab, text="MediaPipe")
        self.mediapipe_tab.columnconfigure(0, weight=1)
        self.mediapipe_tab.columnconfigure(1, weight=1)
        self.mediapipe_tab.rowconfigure(0, weight=1)

        # Populate Dashboard
        self.build_devices_frame(self.dashboard_tab)
        self.build_visualizer_frame(self.dashboard_tab)
        self.build_actions_frame(self.dashboard_tab)
        self.build_log_frame(self.dashboard_tab)
        
        # Populate Raw Data Tab
        self.build_raw_data_sheet(self.raw_data_tab)
        # Populate MediaPipe Tab
        self.build_mediapipe_tab(self.mediapipe_tab)

        self.refresh_devices()
        self.schedule_refresh()

        # Initialize XrClient and start data collection
        self.init_xr_client()
        self._init_xr_data_pub()

        # Auto-connect to hand data endpoint on startup
        endpoint = self.hand_data_ip_var.get().strip()
        if endpoint and endpoint not in self.hand_data_endpoints:
            self.root.after(1000, self.connect_hand_data_endpoint)  # Delay 1s for UI to settle

        # Start UI update loop
        self.update_data_display()

    def configure_styles(self):
        """Apply a simple modern palette and ttk styles."""
        self.palette = {
            "bg": "#eef2f8",
            "panel": "#ffffff",
            "accent": "#2563eb",
            "muted": "#5f6b7a",
            "text": "#0f172a"
        }
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except Exception:
            pass

        self.root.configure(bg=self.palette["bg"])

        style.configure("App.TFrame", background=self.palette["bg"])
        style.configure("Surface.TFrame", background=self.palette["panel"])
        style.configure("Section.TLabelframe", background=self.palette["panel"], foreground=self.palette["text"], borderwidth=1, relief="solid")
        style.configure("Section.TLabelframe.Label", background=self.palette["panel"], foreground=self.palette["muted"], font=("Segoe UI", 10, "bold"))
        style.configure("Header.TLabel", background=self.palette["accent"], foreground="white", font=("Segoe UI", 14, "bold"))
        style.configure("TLabel", background=self.palette["panel"], foreground=self.palette["text"])
        style.configure("Device.TCheckbutton", background=self.palette["panel"], foreground=self.palette["text"])

        # Spreadsheet Styles
        style.configure("SheetHeader.TLabel", background="#e2e8f0", foreground=self.palette["text"], font=("Segoe UI", 9, "bold"), padding=4, borderwidth=1, relief="solid")
        style.configure("SheetCell.TLabel", background="white", foreground=self.palette["text"], font=("Consolas", 9), padding=4, borderwidth=1, relief="solid")
        style.configure("SheetLabel.TLabel", background="#f1f5f9", foreground=self.palette["text"], font=("Segoe UI", 9, "bold"), padding=4, borderwidth=1, relief="solid")

        # Treeview Styles (for MediaPipe tab tables)
        style.configure("Treeview", rowheight=25, font=("Consolas", 9))
        style.configure("Treeview.Heading", font=("Segoe UI", 9, "bold"), padding=4)


        style.configure("Accent.TButton", background=self.palette["accent"], foreground="white", padding=6, relief="flat")
        style.map("Accent.TButton", background=[("active", "#3b82f6")], foreground=[("disabled", "#a1acc0")])
        style.configure("Ghost.TButton", background=self.palette["panel"], foreground=self.palette["text"], padding=6, relief="flat", borderwidth=1)
        style.map("Ghost.TButton",
                  background=[("active", "#e5ebf5")],
                  relief=[("pressed", "flat")])
        style.configure("BadgeIdle.TLabel", background=self.palette["panel"], foreground=self.palette["muted"], padding=4, borderwidth=1, relief="solid")
        style.configure("BadgeGood.TLabel", background="#d9f5d3", foreground="#14532d", padding=4, borderwidth=1, relief="solid")
        style.configure("BadgeWarn.TLabel", background="#fff4d6", foreground="#92400e", padding=4, borderwidth=1, relief="solid")

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
            if not hasattr(self, "log_text"):
                return
            self.log_text.configure(state="normal")
            self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
            self.log_text.see(tk.END)
            self.log_text.configure(state="disabled")
        self.root.after(0, _append)

    def refresh_devices(self):
        self.set_status("Refreshing devices...")
        devices = get_connected_adb_devices()
        self.devices = devices
        if devices:
            self.set_status(f"Found {len(devices)} device(s)")
        else:
            self.set_status("No devices found")
        self.render_device_rows()

    def render_device_rows(self):
        """Rebuild device list with inline action buttons"""
        for child in self.devices_container.winfo_children():
            child.destroy()
        self.device_vars = {}

        if not self.devices:
            ttk.Label(self.devices_container, text="No devices connected").grid(column=0, row=0, padx=4, pady=2, sticky="w")
            return

        for idx, device in enumerate(self.devices):
            row = ttk.Frame(self.devices_container)
            row.grid(column=0, row=idx, sticky="ew", pady=2)
            self.devices_container.rowconfigure(idx, weight=0)
            self.devices_container.columnconfigure(0, weight=1)

            var = tk.IntVar(value=0)
            self.device_vars[device] = var
            check = ttk.Checkbutton(row, text=device, variable=var)
            check.grid(column=0, row=0, padx=(0, 6), sticky="w")

            restart_btn = ttk.Button(row, text="Restart", command=lambda d=device: self.reboot_device(d))
            restart_btn.grid(column=1, row=0, padx=4, sticky="ew")

            power_btn = ttk.Button(row, text="Power Off", command=lambda d=device: self.power_off_device(d))
            power_btn.grid(column=2, row=0, padx=4, sticky="ew")

            # Column 3: WiFi button if USB (no colon in name)
            if ":" not in device:
                wifi_btn = ttk.Button(row, text="Enable WiFi", command=lambda d=device: self.start_wifi_for_device(d))
                wifi_btn.grid(column=3, row=0, padx=4, sticky="ew")

            screenshot_btn = ttk.Button(row, text="Screenshot", command=lambda d=device: self.capture_screenshot_device(d))
            screenshot_btn.grid(column=4, row=0, padx=4, sticky="ew")

            row.columnconfigure(0, weight=1)
            row.columnconfigure(1, weight=0)
            row.columnconfigure(2, weight=0)
            row.columnconfigure(3, weight=0)
            row.columnconfigure(4, weight=0)

    def selected_devices(self):
        selected = [dev for dev, var in self.device_vars.items() if var.get() == 1]
        if selected:
            return selected
        return self.devices  # Return all found devices if none are selected

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

    def _get_device_ip(self, device: str) -> str | None:
        """Try to fetch the Wi-Fi IP of a device via adb shell."""
        cmds = [
            f"adb -s {device} shell ip -f inet addr show wlan0",
            f"adb -s {device} shell ip addr show wlan0",
            f"adb -s {device} shell ip route",
        ]

        for cmd in cmds:
            success, output = execute_adb_command(cmd)
            if not success or not output:
                continue
            match = re.search(r"inet (\d+\.\d+\.\d+\.\d+)", output)
            if match:
                return match.group(1)
        return None

    def start_wifi_for_device(self, device: str):
        """Switch specific USB device to TCPIP:5555 and connect over Wi-Fi."""
        port = "5555"

        def _run():
            self.set_status(f"Enabling WiFi for {device}...")
            
            ip_found = self._get_device_ip(device)
            if not ip_found:
                ip_fallback = self.ip_var.get().strip()
                if not ip_fallback:
                    self.log(f"Wi-Fi connect: failed to detect IP for {device}; enter IP manually.")
                    self.set_status("IP not detected")
                    return
                ip_found = ip_fallback.split(":")[0]

            target = f"{ip_found}:{port}"
            self.log(f"Wi-Fi connect: {device} -> {target} (port {port})")

            success_tcpip, output_tcpip = execute_adb_command(f"adb -s {device} tcpip {port}")
            self.log(f"tcpip {port} on {device}: {'ok' if success_tcpip else 'failed'} | {output_tcpip.strip()}")
            if not success_tcpip:
                self.set_status("TCPIP failed")
                self.refresh_devices()
                return

            success_conn, output_conn = execute_adb_command(f"adb connect {target}")
            self.log(f"connect {target}: {'ok' if success_conn else 'failed'} | {output_conn.strip()}")
            self.refresh_devices()
            self.set_status("Idle")

        threading.Thread(target=_run, daemon=True).start()

    def _ensure_screenshot_window(self):
        """Create the screenshot viewer window if needed."""
        if self.screenshot_window and self.screenshot_window.winfo_exists():
            return
        self.screenshot_window = tk.Toplevel(self.root)
        self.screenshot_window.title("Headset Screenshot")
        self.screenshot_label = ttk.Label(self.screenshot_window, text="Waiting for screenshot...")
        self.screenshot_label.pack(padx=8, pady=8)

    def capture_screenshot(self):
        """Capture a screenshot from the selected device and display it."""
        selected = self.selected_devices()
        if not selected:
            messagebox.showinfo("ADB Control", "Select a device before capturing a screenshot.")
            return

        device = selected[0]
        self.capture_screenshot_device(device)

    def capture_screenshot_device(self, device: str):
        """Capture screenshot for a specific device."""
        self.set_status(f"Capturing screenshot from {device}...")

        def _run():
            try:
                cmd = ["adb", "-s", device, "exec-out", "screencap", "-p"]
                result = subprocess.run(cmd, capture_output=True, timeout=10)
                if result.returncode != 0 or not result.stdout:
                    msg = result.stderr.decode("utf-8", errors="ignore") if result.stderr else "No data"
                    self.log(f"screenshot {device}: failed | {msg.strip()}")
                    self.set_status("Screenshot failed")
                    return

                png_bytes = result.stdout
                encoded = base64.b64encode(png_bytes).decode("ascii")

                def _update_ui():
                    self._ensure_screenshot_window()
                    try:
                        self.screenshot_image = tk.PhotoImage(data=encoded)
                        self.screenshot_label.configure(image=self.screenshot_image, text="")
                    except tk.TclError as e:
                        self.log(f"screenshot display error: {e}")
                        messagebox.showerror("Screenshot", f"Failed to display screenshot: {e}")
                        self.set_status("Screenshot display failed")
                        return
                    self.set_status("Idle")
                    self.log(f"screenshot {device}: ok")

                self.root.after(0, _update_ui)
            except Exception as e:
                self.log(f"screenshot error: {e}")
                self.set_status("Screenshot failed")

        threading.Thread(target=_run, daemon=True).start()

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
            self.render_rotation_endpoints()
            self.update_mediapipe_targets_only(self.rotation_endpoints)

            self.log(f"Connected rotation data endpoint: {endpoint}")
            self.set_status(f"Connected to {endpoint}")

        except Exception as e:
            messagebox.showerror("Rotation Data", f"Failed to connect: {e}")
            self.log(f"Failed to connect to {endpoint}: {e}")

    def disconnect_rotation_endpoint(self, endpoint: str):
        """Disconnect a specific rotation data endpoint"""
        if endpoint not in self.rotation_endpoints:
            return
        index = self.rotation_endpoints.index(endpoint)

        try:
            sock = self.rotation_sockets[index]
            sock.close()

            del self.rotation_sockets[index]
            del self.rotation_endpoints[index]
            self.render_rotation_endpoints()
            self.update_mediapipe_targets_only(self.rotation_endpoints)

            self.log(f"Disconnected rotation data endpoint: {endpoint}")
            self.set_status("Idle")

        except Exception as e:
            messagebox.showerror("Rotation Data", f"Failed to disconnect: {e}")
            self.log(f"Failed to disconnect from {endpoint}: {e}")

    def disconnect_all_rotation_endpoints(self):
        """Disconnect all rotation endpoints"""
        for endpoint in list(self.rotation_endpoints):
            self.disconnect_rotation_endpoint(endpoint)
        self.update_mediapipe_targets_only(self.rotation_endpoints)

    def render_rotation_endpoints(self):
        """Render connected rotation endpoints with per-row disconnect buttons"""
        for child in self.rotation_endpoints_frame.winfo_children():
            child.destroy()

        if not self.rotation_endpoints:
            ttk.Label(self.rotation_endpoints_frame, text="No endpoints connected", style="TLabel").grid(column=0, row=0, padx=6, pady=4, sticky="w")
            self.update_mediapipe_targets_only(self.rotation_endpoints)
            return

        for idx, endpoint in enumerate(self.rotation_endpoints):
            row = ttk.Frame(self.rotation_endpoints_frame, style="Surface.TFrame")
            row.grid(column=0, row=idx, sticky="ew", pady=2)
            self.rotation_endpoints_frame.rowconfigure(idx, weight=0)
            self.rotation_endpoints_frame.columnconfigure(0, weight=1)

            ttk.Label(row, text=endpoint, style="TLabel").grid(column=0, row=0, padx=(0, 6), sticky="w")
            ttk.Button(row, text="Connect", style="Ghost.TButton", command=lambda ep=endpoint: self._reconnect_rotation_endpoint(ep)).grid(column=1, row=0, padx=4, sticky="ew")
            ttk.Button(row, text="Disconnect", style="Ghost.TButton", command=lambda ep=endpoint: self.disconnect_rotation_endpoint(ep)).grid(column=2, row=0, padx=4, sticky="ew")

            row.columnconfigure(0, weight=1)
            row.columnconfigure(1, weight=0)
            row.columnconfigure(2, weight=0)
        self.update_mediapipe_targets_only(self.rotation_endpoints)

    def _reconnect_rotation_endpoint(self, endpoint: str):
        """Attempt to reconnect an existing endpoint"""
        # Disconnect first, then connect again
        self.disconnect_rotation_endpoint(endpoint)
        self.rotation_ip_var.set(endpoint)
        self.connect_rotation_endpoint()

    def connect_hand_data_endpoint(self):
        """Connect to a MediaPipe hand data endpoint (IP:Port)"""
        endpoint = self.hand_data_ip_var.get().strip()
        if not endpoint:
            messagebox.showinfo("Hand Data", "Enter an IP:Port before connecting.")
            return

        try:
            # Parse port from endpoint (format: "host:port")
            if ':' not in endpoint:
                messagebox.showerror("Hand Data", "Format must be Host:Port (e.g., localhost:5556)")
                return

            _, port_str = endpoint.split(':', 1)
            port = int(port_str)

            # Check if already connected
            if endpoint in self.hand_data_endpoints:
                messagebox.showinfo("Hand Data", f"Already connected to {endpoint}")
                return

            # Create and bind ZMQ socket (we are the server, bridge connects to us)
            sock = self.zmq_context.socket(zmq.PUB)
            sock.bind(f"tcp://*:{port}")

            self.log(f"Creating ZMQ PUB socket for hand data on tcp://*:{port}")

            # Add to lists
            self.hand_data_sockets.append(sock)
            self.hand_data_endpoints.append(endpoint)
            self.render_hand_data_endpoints()

            self.log(f"Bound hand data endpoint: {endpoint} (Total sockets: {len(self.hand_data_sockets)})")
            self.set_status(f"Connected to {endpoint}")

        except ValueError:
            messagebox.showerror("Hand Data", "Port must be a number")
        except Exception as e:
            messagebox.showerror("Hand Data", f"Failed to connect: {e}")
            self.log(f"Failed to connect to {endpoint}: {e}")

    def _init_xr_data_pub(self):
        """Bind a PUB socket for broadcasting full XR data snapshots."""
        try:
            self.xr_data_socket = self.zmq_context.socket(zmq.PUB)
            self.xr_data_socket.bind(f"tcp://*:{self.xr_data_port}")
            self.log(f"XR data PUB bound on tcp://*:{self.xr_data_port}")
            print(f"[ADB GUI] XR data PUB bound on tcp://*:{self.xr_data_port}")
            self.xr_data_bind_error = None
        except Exception as e:
            self.xr_data_socket = None
            self.xr_data_bind_error = str(e)
            self.log(f"Failed to bind XR data PUB socket on {self.xr_data_port}: {e}")
            print(f"[ADB GUI] Failed to bind XR data PUB socket on {self.xr_data_port}: {e}")

    def disconnect_hand_data_endpoint(self, endpoint):
        """Disconnect from a specific hand data endpoint"""
        try:
            idx = self.hand_data_endpoints.index(endpoint)

            # Close socket
            sock = self.hand_data_sockets[idx]
            sock.close()

            # Remove from lists
            del self.hand_data_sockets[idx]
            del self.hand_data_endpoints[idx]

            self.render_hand_data_endpoints()
            self.log(f"Disconnected hand data endpoint: {endpoint}")
            self.set_status(f"Disconnected from {endpoint}")

        except ValueError:
            pass  # Endpoint not in list
        except Exception as e:
            self.log(f"Error disconnecting {endpoint}: {e}")

    def render_hand_data_endpoints(self):
        """Render the list of connected hand data endpoints"""
        # Clear existing widgets
        for widget in self.hand_endpoints_frame.winfo_children():
            widget.destroy()

        if not self.hand_data_endpoints:
            ttk.Label(self.hand_endpoints_frame,
                      text="No endpoints connected",
                      foreground=self.palette["muted"]).grid(column=0, row=0, sticky="w")
            return

        ttk.Label(self.hand_endpoints_frame,
                  text="Connected Endpoints:",
                  font=("TkDefaultFont", 9, "bold")).grid(column=0, row=0, sticky="w", pady=(0, 4))

        for idx, endpoint in enumerate(self.hand_data_endpoints):
            row_frame = ttk.Frame(self.hand_endpoints_frame, style="Surface.TFrame")
            row_frame.grid(column=0, row=idx+1, sticky="ew", pady=2)
            row_frame.columnconfigure(0, weight=1)

            ttk.Label(row_frame, text=f"• {endpoint}").grid(column=0, row=0, sticky="w")

            disconnect_btn = ttk.Button(row_frame, text="Disconnect", style="Ghost.TButton",
                                        command=lambda ep=endpoint: self.disconnect_hand_data_endpoint(ep))
            disconnect_btn.grid(column=1, row=0, padx=(8, 0))

    def reboot_device(self, device: str):
        """Restart a single headset"""
        threading.Thread(target=self._run_device_command, args=(device, "reboot", f"adb -s {device} reboot"), daemon=True).start()

    def power_off_device(self, device: str):
        """Power off a single headset"""
        threading.Thread(target=self._run_device_command, args=(device, "power off", f"adb -s {device} shell reboot -p"), daemon=True).start()

    def _run_device_command(self, device: str, label: str, cmd: str):
        """Execute a simple ADB command per device"""
        self.set_status(f"{label} {device}")
        try:
            success, output = execute_adb_command(cmd)
            self.log(f"[{device}] {label}: {'ok' if success else 'failed'} | {output.strip()}")
        finally:
            self.set_status("Idle")

    def stop_app(self):
        self.run_for_devices("stop app", self._stop_app_on_device)

    def restart_app(self):
        self.run_for_devices("restart app", self._restart_app_on_device)

        # Auto-connect rotation endpoint if set and not connected
        endpoint = self.rotation_ip_var.get().strip()
        if endpoint and endpoint not in self.rotation_endpoints:
            self.connect_rotation_endpoint()

    def open_app(self):
        self.run_for_devices("open app", self._open_app_on_device)

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
            self.last_send_ts = time.time()

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

    def send_xr_data(self, full_data: dict):
        """Broadcast full XR snapshot data to teleop subscribers."""
        if not self.xr_data_socket:
            return
        try:
            headset = full_data.get("headset", {})
            left_ctrl = full_data.get("leftController", {})
            right_ctrl = full_data.get("rightController", {})
            left_hand = full_data.get("leftHand", {})
            right_hand = full_data.get("rightHand", {})
            raw_payload = {
                "headset_pose": headset.get("pose_raw"),
                "left_controller_pose": left_ctrl.get("pose_raw"),
                "right_controller_pose": right_ctrl.get("pose_raw"),
                "left_hand_joints": left_hand.get("joints_raw") if left_hand.get("isActive") else None,
                "right_hand_joints": right_hand.get("joints_raw") if right_hand.get("isActive") else None,
                "timestamp_ns": headset.get("timeStampNs"),
            }
            json_data = json.dumps(raw_payload)
            self.xr_data_socket.send_string(json_data, zmq.NOBLOCK)
            self.xr_data_send_count += 1
            self.xr_data_last_send_ts = time.time()
            if self.xr_data_send_count == 1 or self.xr_data_send_count % 50 == 0:
                self.log(
                    "XR ZMQ send #{count} headset_status={hs} left_active={la} right_active={ra}".format(
                        count=self.xr_data_send_count,
                        hs=headset.get("status"),
                        la=left_hand.get("isActive"),
                        ra=right_hand.get("isActive"),
                    )
                )
                print(json.dumps(raw_payload))
        except zmq.Again:
            pass
        except Exception:
            pass

    def _mediapipe_to_landmark_list(self, mediapipe_array):
        """Convert MediaPipe numpy array to list of {x, y, z} dicts

        Args:
            mediapipe_array: Numpy array of shape (21, 3) or None

        Returns:
            List of 21 dicts with x, y, z keys (floats), or empty list if None
        """
        if mediapipe_array is None:
            return []
        try:
            return [
                {"x": float(pos[0]), "y": float(pos[1]), "z": float(pos[2])}
                for pos in mediapipe_array
            ]
        except Exception:
            return []

    def send_hand_data(self):
        """Send MediaPipe hand data to all connected ZMQ endpoints

        Sends JSON format:
        {
          "left": [{"x": 0.1, "y": 0.2, "z": 0.3}, ...],  // 21 landmarks
          "right": [{"x": 0.1, "y": 0.2, "z": 0.3}, ...]  // 21 landmarks
        }
        """
        if not self.hand_data_sockets:
            return

        try:
            # Get MediaPipe data snapshot (thread-safe copy)
            with self.data_lock:
                left_mp = self.mediapipe_points.get("left")
                right_mp = self.mediapipe_points.get("right")
                # Make copies if not None
                left_copy = np.copy(left_mp) if left_mp is not None else None
                right_copy = np.copy(right_mp) if right_mp is not None else None

            # Convert to landmark list format
            hand_message = {
                "left": self._mediapipe_to_landmark_list(left_copy),
                "right": self._mediapipe_to_landmark_list(right_copy)
            }

            # Serialize to JSON
            json_data = json.dumps(hand_message)

            # Debug: Log data status
            if not hasattr(self, '_hand_data_send_count'):
                self._hand_data_send_count = 0

            self._hand_data_send_count += 1

            # Log every 100 sends or if first send
            if self._hand_data_send_count == 1 or self._hand_data_send_count % 100 == 0:
                left_count = len(hand_message['left'])
                right_count = len(hand_message['right'])
                self.log(f"Hand data send #{self._hand_data_send_count}: Left={left_count} landmarks, Right={right_count} landmarks")

                # Log raw data state
                if left_copy is None and right_copy is None:
                    self.log(f"  WARNING: Both hands are None in mediapipe_points")
                elif left_copy is None:
                    self.log(f"  Left hand is None, Right shape: {right_copy.shape if right_copy is not None else 'None'}")
                elif right_copy is None:
                    self.log(f"  Left shape: {left_copy.shape if left_copy is not None else 'None'}, Right hand is None")

            # Send to all connected endpoints
            for sock in self.hand_data_sockets:
                try:
                    sock.send_string(json_data, zmq.NOBLOCK)
                except zmq.Again:
                    pass  # Socket buffer full, skip this send
                except Exception as e:
                    self.log(f"Error sending hand data: {e}")

        except Exception as e:
            self.log(f"Error in send_hand_data: {e}")

    def collect_full_xr_data(self) -> dict:
        """Collect all available XR data into a dictionary structure."""
        data = {}
        
        # Helper to format pose array to string "x,y,z,qx,qy,qz,qw"
        def fmt_pose(p):
            if p is None: return "0.0,0.0,0.0,0.0,0.0,0.0,0.0"
            return ",".join([f"{x:.4f}" for x in p])
        
        def pose_to_list(p):
            if p is None:
                return None
            return np.asarray(p, dtype=float).tolist()

        ts = self.xr_client.get_timestamp_ns()
        
        # 1. Headset
        headset_pose = self.xr_client.get_pose_by_name("headset")
        data["headset"] = {
            "pose": fmt_pose(headset_pose),
            "pose_raw": pose_to_list(headset_pose),
            "status": 3 if headset_pose is not None else 0,
            "timeStampNs": ts
        }

        # 2. Controllers
        def get_controller_info(side):
            # side is "left" or "right"
            prefix = side.lower()
            try:
                pose = self.xr_client.get_pose_by_name(f"{prefix}_controller")
                joystick = self.xr_client.get_joystick_state(prefix)
                
                # Button mapping based on available methods
                if prefix == "left":
                    menu = self.xr_client.get_button_state_by_name("left_menu_button")
                    primary = self.xr_client.get_button_state_by_name("X")
                    secondary = self.xr_client.get_button_state_by_name("Y")
                    axis_click = self.xr_client.get_button_state_by_name("left_axis_click")
                else:
                    menu = self.xr_client.get_button_state_by_name("right_menu_button")
                    primary = self.xr_client.get_button_state_by_name("A")
                    secondary = self.xr_client.get_button_state_by_name("B")
                    axis_click = self.xr_client.get_button_state_by_name("right_axis_click")
                
                return {
                    "axisX": joystick[0],
                    "axisY": joystick[1],
                    "grip": self.xr_client.get_key_value_by_name(f"{prefix}_grip"),
                    "trigger": self.xr_client.get_key_value_by_name(f"{prefix}_trigger"),
                    "primaryButton": primary,
                    "secondaryButton": secondary,
                    "menuButton": menu,
                    "axisClick": axis_click,
                    "pose": fmt_pose(pose),
                    "pose_raw": pose_to_list(pose),
                    "timeStampNs": ts
                }
            except Exception:
                # Return empty/default structure on error
                return {
                    "axisX": 0.0, "axisY": 0.0, "grip": 0.0, "trigger": 0.0,
                    "primaryButton": False, "secondaryButton": False, "menuButton": False, "axisClick": False,
                    "pose": fmt_pose(None), "pose_raw": None, "timeStampNs": ts
                }

        data["leftController"] = get_controller_info("left")
        data["rightController"] = get_controller_info("right")

        # 3. Hands
        def get_hand_info(side):
            prefix = side.lower()
            try:
                hand_state = self.xr_client.get_hand_tracking_state(prefix)
                joints = []
                joints_raw = None
                is_active = 0
                
                if hand_state is not None:
                    hand_state_arr = np.array(hand_state)
                    is_active = 1
                    joints_raw = hand_state_arr.tolist()
                    for j in hand_state_arr:
                        # each j is [x,y,z,qx,qy,qz,qw]
                        joints.append(fmt_pose(j))
                else:
                    # Fill with 26 empty joints
                    joints = [fmt_pose(None)] * 26

                return {
                    "isActive": is_active,
                    "scale": 1.0,
                    "timeStampNs": ts,
                    "joints": joints,
                    "joints_raw": joints_raw
                }
            except Exception:
                return {
                    "isActive": 0, "scale": 1.0,
                    "timeStampNs": ts,
                    "joints": [fmt_pose(None)] * 26,
                    "joints_raw": None
                }

        data["leftHand"] = get_hand_info("left")
        data["rightHand"] = get_hand_info("right")
        
        return data

    def mediapipe_from_raw(self, joints_raw):
        """Convert raw PICO joints into MediaPipe-relative coordinates."""
        if joints_raw is None:
            return None
        try:
            arr = np.array(joints_raw, dtype=float)
            required_len = max(pico_to_mediapipe.keys()) + 1
            if arr.shape[0] < required_len:
                pad = np.zeros((required_len - arr.shape[0], arr.shape[1]))
                arr = np.concatenate([arr, pad], axis=0)
            return pico_hand_state_to_mediapipe(arr)
        except Exception:
            return None

    def data_collection_worker(self):
        """Background worker for collecting head rotation data from XrClient"""
        start_time = time.time()
        frame_count = 0

        while self.running and self.xr_client:
            try:
                # 1. Existing Rotation Logic
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
                    current_timestamp = time.time()
                    self.send_rotation_data(yaw_deg, pitch_deg, roll_deg, current_timestamp)

                    # Update data with thread lock
                    with self.data_lock:
                        self.rotation_data['yaw'] = yaw_deg
                        self.rotation_data['pitch'] = pitch_deg
                        self.rotation_data['roll'] = roll_deg
                        self.data_status = 'OK'
                        self.last_data_ts = time.time()

                        # Add to history
                        if len(self.angle_history['time']) == 0 or current_timestamp - self.angle_history['time'][-1] > 0.05:
                            self.angle_history['time'].append(current_timestamp)
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

                # 2. Collect Full Raw Data
                full_data = self.collect_full_xr_data()
                left_mp = self.mediapipe_from_raw(full_data.get("leftHand", {}).get("joints_raw"))
                right_mp = self.mediapipe_from_raw(full_data.get("rightHand", {}).get("joints_raw"))

                # Debug: Log hand data collection (first time only)
                if not hasattr(self, '_hand_collection_logged'):
                    self._hand_collection_logged = True
                    left_has_data = "leftHand" in full_data and full_data["leftHand"].get("joints_raw") is not None
                    right_has_data = "rightHand" in full_data and full_data["rightHand"].get("joints_raw") is not None
                    self.log(f"Hand data collection: Left={left_has_data}, Right={right_has_data}")
                    if left_mp is not None:
                        self.log(f"  Left MediaPipe shape: {left_mp.shape}")
                    if right_mp is not None:
                        self.log(f"  Right MediaPipe shape: {right_mp.shape}")

                with self.data_lock:
                    self.raw_data_snapshot = full_data
                    if self.data_status == 'OK' and full_data:
                        self.last_good_raw_snapshot = full_data
                    self.mediapipe_points["left"] = left_mp
                    self.mediapipe_points["right"] = right_mp
                    if left_mp is not None:
                        self.last_good_mediapipe_points["left"] = left_mp
                    if right_mp is not None:
                        self.last_good_mediapipe_points["right"] = right_mp

                # Broadcast MediaPipe hand data via ZMQ
                self.send_hand_data()
                # Broadcast full XR data for teleop bridge
                self.send_xr_data(full_data)

                # Sleep to maintain ~10Hz update rate
                time.sleep(0.1)

            except Exception as e:
                with self.data_lock:
                    self.data_status = f'ERROR: {str(e)}'
                time.sleep(0.1)

    def update_io_indicators(self, history_times):
        """Refresh send/receive badges without breaking the UI loop."""
        now = time.time()

        # Receive badge
        last_recv = self.last_data_ts or (history_times[-1] if history_times else 0)
        if last_recv and now - last_recv < 1.0:
            recv_style = "BadgeGood.TLabel"
            recv_text = f"Receive: {self.data_frequency:.1f} Hz"
        elif last_recv:
            age_ms = (now - last_recv) * 1000
            recv_style = "BadgeWarn.TLabel"
            recv_text = f"Receive: stale ({age_ms:.0f} ms)"
        else:
            recv_style = "BadgeIdle.TLabel"
            recv_text = "Receive: idle"
        self.recv_status_badge.configure(text=recv_text, style=recv_style)

        # Send badge
        last_send = self.last_send_ts
        if last_send and now - last_send < 1.0:
            send_style = "BadgeGood.TLabel"
            send_text = "Send: active"
        elif last_send:
            age_ms = (now - last_send) * 1000
            send_style = "BadgeWarn.TLabel"
            send_text = f"Send: stale ({age_ms:.0f} ms)"
        else:
            send_style = "BadgeIdle.TLabel"
            send_text = "Send: idle"
        self.send_status_badge.configure(text=send_text, style=send_style)

    def update_xr_data_badge(self):
        """Refresh XR data broadcast status badge."""
        if not hasattr(self, "xr_data_status_badge"):
            return
        if not self.xr_data_socket:
            error = self.xr_data_bind_error or "bind failed"
            self.xr_data_status_badge.configure(text=f"XR ZMQ: unavailable ({error})", style="BadgeWarn.TLabel")
            return
        now = time.time()
        last_send = self.xr_data_last_send_ts
        if last_send and now - last_send < 1.0:
            style = "BadgeGood.TLabel"
            text = f"XR ZMQ: active ({self.xr_data_send_count})"
        elif last_send:
            age_ms = (now - last_send) * 1000
            style = "BadgeWarn.TLabel"
            text = f"XR ZMQ: stale ({age_ms:.0f} ms)"
        else:
            style = "BadgeIdle.TLabel"
            text = "XR ZMQ: idle"
        self.xr_data_status_badge.configure(text=text, style=style)

    def update_data_display(self):
        """Update the data display (gauges, graph, or raw data) periodically"""
        try:
            # Check which tab is active
            current_tab = self.main_notebook.select()
            
            # Determine which view to update based on selected tab
            update_visualizer = (current_tab == str(self.dashboard_tab))
            update_raw_data = (current_tab == str(self.raw_data_tab))
            update_mediapipe = (current_tab == str(self.mediapipe_tab))

            # Get current data with thread lock
            with self.data_lock:
                yaw = self.rotation_data['yaw']
                pitch = self.rotation_data['pitch']
                roll = self.rotation_data['roll']
                status = self.data_status
                freq = self.data_frequency
                raw_snapshot = self.raw_data_snapshot.copy()
                last_good_snapshot = self.last_good_raw_snapshot.copy() if self.last_good_raw_snapshot else {}
                mp_left = np.array(self.mediapipe_points["left"]) if self.mediapipe_points.get("left") is not None else None
                mp_right = np.array(self.mediapipe_points["right"]) if self.mediapipe_points.get("right") is not None else None
                mp_left_last = np.array(self.last_good_mediapipe_points["left"]) if self.last_good_mediapipe_points.get("left") is not None else None
                mp_right_last = np.array(self.last_good_mediapipe_points["right"]) if self.last_good_mediapipe_points.get("right") is not None else None

                # Copy history for plotting (only if visualizer active)
                if update_visualizer:
                    times = list(self.angle_history['time'])
                    yaws = list(self.angle_history['yaw'])
                    pitches = list(self.angle_history['pitch'])
                    rolls = list(self.angle_history['roll'])
                else:
                    times = []

            # Update status label (common for both)
            self.data_status_label.config(text=f"Status: {status} | {freq:.1f} Hz")
            self.update_io_indicators(times if update_visualizer else []) # Pass empty if not visualizing history
            self.update_xr_data_badge()

            if update_visualizer:
                # Update history graph
                if len(times) > 1:
                    self.ax.clear()
                    self.ax.set_title("Angle History (Last 10 seconds)")
                    self.ax.set_xlabel("Time (s)")
                    self.ax.set_ylabel("Angle (degrees)")
                    self.ax.grid(True, alpha=0.3)

                    # Normalize time
                    current_time = times[-1]
                    relative_times = [t - current_time for t in times]

                    self.ax.plot(relative_times, yaws, 'b-', label='Yaw', linewidth=2)
                    self.ax.plot(relative_times, pitches, 'g-', label='Pitch', linewidth=2)
                    self.ax.plot(relative_times, rolls, 'r-', label='Roll', linewidth=2)

                    self.ax.legend(loc='upper right')
                    self.ax.set_xlim(-10, 0)
                    self.ax.set_ylim(-180, 180)

                    self.canvas.draw()
            
            elif update_mediapipe:
                now = time.time()
                stale = self.last_data_ts and (now - self.last_data_ts > 1.0)
                mp_left_disp = mp_left if (mp_left is not None and not stale and status == 'OK') else (mp_left_last if mp_left_last is not None else mp_left)
                mp_right_disp = mp_right if (mp_right is not None and not stale and status == 'OK') else (mp_right_last if mp_right_last is not None else mp_right)
                endpoints = list(self.rotation_endpoints)
                self.update_mediapipe_send_badge(active=not stale and status == 'OK', stale=stale, endpoints=endpoints)
                self.update_mediapipe_rotation_data(yaw, pitch, roll)
                self.update_mediapipe_table("left", mp_left_disp)
                self.update_mediapipe_table("right", mp_right_disp)

            elif update_raw_data:
                # Freeze display to last good snapshot if data is lost/stale
                display_snapshot = raw_snapshot
                now = time.time()
                stale = self.last_data_ts and (now - self.last_data_ts > 1.0)
                if (not display_snapshot) or status != 'OK' or stale:
                    if last_good_snapshot:
                        display_snapshot = last_good_snapshot

                if not display_snapshot:
                    return

                # Update Sheet Labels
                
                # 1. Headset
                hs = display_snapshot.get("headset", {})
                self.sheet_cells["headset_pose"].configure(text=hs.get("pose", "0"))
                self.sheet_cells["headset_status"].configure(text=str(hs.get("status", "0")))
                self.sheet_cells["headset_time"].configure(text=str(hs.get("timeStampNs", "0")))

                # 2. Controllers
                for side in ["Left", "Right"]:
                    key = side.lower()
                    c_key = f"{key}Controller"
                    c_data = display_snapshot.get(c_key, {})
                    
                    self.sheet_cells[f"{key}_c_pose"].configure(text=c_data.get("pose", "0"))
                    self.sheet_cells[f"{key}_c_axis"].configure(text=f"{c_data.get('axisX',0):.2f}, {c_data.get('axisY',0):.2f}")
                    self.sheet_cells[f"{key}_c_trigger"].configure(text=f"T:{c_data.get('trigger',0):.2f} G:{c_data.get('grip',0):.2f}")
                    
                    btns = []
                    if c_data.get("primaryButton"): btns.append("Prim")
                    if c_data.get("secondaryButton"): btns.append("Sec")
                    if c_data.get("menuButton"): btns.append("Menu")
                    if c_data.get("axisClick"): btns.append("Click")
                    self.sheet_cells[f"{key}_c_buttons"].configure(text=" | ".join(btns) if btns else "-")

                # 3. Hands Summary
                for side in ["Left", "Right"]:
                    key = side.lower()
                    h_key = f"{key}Hand"
                    h_data = display_snapshot.get(h_key, {})
                    
                    self.sheet_cells[f"{key}_h_active"].configure(text="Yes" if h_data.get("isActive") else "No")
                    self.sheet_cells[f"{key}_h_scale"].configure(text=f"{h_data.get('scale', 1.0):.2f}")
                    
                # 4. Hand Joints
                # We need to loop carefully. 26 rows.
                left_h = display_snapshot.get("leftHand", {}).get("joints", [])
                right_h = display_snapshot.get("rightHand", {}).get("joints", [])
                
                for i in range(26):
                    # Left
                    val_l = left_h[i] if i < len(left_h) else "0"
                    self.sheet_cells[f"joint_{i}_left"].configure(text=val_l)
                    
                    # Right
                    val_r = right_h[i] if i < len(right_h) else "0"
                    self.sheet_cells[f"joint_{i}_right"].configure(text=val_r)

        except Exception as e:
            # print(f"Display update error: {e}")
            pass

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

        # Close all hand data ZMQ sockets
        for sock in self.hand_data_sockets:
            try:
                sock.close()
            except Exception:
                pass

        if self.xr_data_socket:
            try:
                self.xr_data_socket.close()
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
