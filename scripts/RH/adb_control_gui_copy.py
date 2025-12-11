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
import re
import base64
import subprocess
import json
from xrobotoolkit_teleop.utils.dex_hand_utils import pico_hand_state_to_mediapipe, pico_to_mediapipe

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
        self.log_visible = False
        self.devices = []
        self.status_var = tk.StringVar(value="Idle")
        self.ip_var = tk.StringVar()
        self.device_vars = {}
        self.palette = {}

        # Head rotation data storage
        self.rotation_data = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}
        self.raw_data_snapshot = {}  # Store full raw data
        self.mediapipe_points = {"left": None, "right": None}
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


        style.configure("Accent.TButton", background=self.palette["accent"], foreground="white", padding=6, relief="flat")
        style.map("Accent.TButton", background=[("active", "#3b82f6")], foreground=[("disabled", "#a1acc0")])
        style.configure("Ghost.TButton", background=self.palette["panel"], foreground=self.palette["text"], padding=6, relief="flat", borderwidth=1)
        style.map("Ghost.TButton",
                  background=[("active", "#e5ebf5")],
                  relief=[("pressed", "flat")])
        style.configure("BadgeIdle.TLabel", background=self.palette["panel"], foreground=self.palette["muted"], padding=4, borderwidth=1, relief="solid")
        style.configure("BadgeGood.TLabel", background="#d9f5d3", foreground="#14532d", padding=4, borderwidth=1, relief="solid")
        style.configure("BadgeWarn.TLabel", background="#fff4d6", foreground="#92400e", padding=4, borderwidth=1, relief="solid")

    def build_devices_frame(self, parent: ttk.Frame):
        frame = ttk.LabelFrame(parent, text="Devices", style="Section.TLabelframe")
        frame.grid(column=0, row=0, padx=8, pady=4, sticky="nsew")
        parent.rowconfigure(0, weight=0)

        # Device rows with inline actions
        self.devices_container = ttk.Frame(frame, style="Surface.TFrame")
        self.devices_container.grid(column=0, row=0, rowspan=6, padx=(8, 4), pady=8, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(5, weight=1)

        refresh_btn = ttk.Button(frame, text="Refresh", style="Accent.TButton", command=self.refresh_devices)
        refresh_btn.grid(column=1, row=0, padx=4, pady=(8, 2), sticky="ew")

        ttk.Label(frame, text="IP:Port").grid(column=1, row=1, padx=4, pady=2, sticky="w")
        ip_entry = ttk.Entry(frame, textvariable=self.ip_var, width=18)
        ip_entry.grid(column=1, row=2, padx=4, pady=2, sticky="ew")
        connect_btn = ttk.Button(frame, text="Connect over IP", style="Ghost.TButton", command=self.connect_ip)
        connect_btn.grid(column=1, row=3, padx=4, pady=(2, 8), sticky="ew")

        wifi_btn = ttk.Button(frame, text="USB→WiFi (tcpip+connect)", style="Ghost.TButton", command=self.auto_connect_wifi)
        wifi_btn.grid(column=1, row=4, padx=4, pady=(0, 4), sticky="ew")

        screenshot_btn = ttk.Button(frame, text="Screenshot", style="Ghost.TButton", command=self.capture_screenshot)
        screenshot_btn.grid(column=1, row=5, padx=4, pady=(0, 8), sticky="ew")

        self.status_label = ttk.Label(frame, textvariable=self.status_var, foreground=self.palette["accent"])
        self.status_label.grid(column=0, row=6, columnspan=2, padx=8, pady=(0, 8), sticky="w")

        # Separator
        ttk.Separator(frame, orient='horizontal').grid(column=0, row=7, columnspan=2, padx=8, pady=8, sticky="ew")

        # Rotation data endpoint section
        ttk.Label(frame, text="Rotation Data Endpoint", font=("TkDefaultFont", 9, "bold")).grid(column=0, row=8, columnspan=2, padx=8, pady=(0, 4), sticky="w")

        rotation_entry_row = ttk.Frame(frame, style="Surface.TFrame")
        rotation_entry_row.grid(column=0, row=9, columnspan=2, padx=8, pady=(0, 4), sticky="ew")
        rotation_entry_row.columnconfigure(0, weight=1)
        rotation_entry_row.columnconfigure(1, weight=0)
        rotation_entry_row.columnconfigure(2, weight=0)

        rotation_ip_entry = ttk.Entry(rotation_entry_row, textvariable=self.rotation_ip_var, width=18)
        rotation_ip_entry.grid(column=0, row=0, padx=(0, 6), pady=2, sticky="ew")

        rotation_connect_btn = ttk.Button(rotation_entry_row, text="Connect", style="Ghost.TButton", command=self.connect_rotation_endpoint)
        rotation_connect_btn.grid(column=1, row=0, padx=4, pady=2, sticky="ew")

        rotation_disconnect_btn = ttk.Button(rotation_entry_row, text="Disconnect All", style="Ghost.TButton", command=self.disconnect_all_rotation_endpoints)
        rotation_disconnect_btn.grid(column=2, row=0, padx=4, pady=2, sticky="ew")

        # List of connected rotation endpoints
        ttk.Label(frame, text="Connected Endpoints:", font=("TkDefaultFont", 8)).grid(column=0, row=10, padx=8, pady=(0, 2), sticky="w")
        self.rotation_endpoints_frame = ttk.Frame(frame, style="Surface.TFrame")
        self.rotation_endpoints_frame.grid(column=0, row=11, columnspan=2, padx=8, pady=2, sticky="nsew")
        frame.rowconfigure(11, weight=1)
        self.render_rotation_endpoints()

    def build_visualizer_frame(self, parent: ttk.Frame):
        """Build the dashboard visualizer (Gauges + Graph)"""
        frame = ttk.LabelFrame(parent, text="Head Rotation Visualizer", style="Section.TLabelframe")
        frame.grid(column=0, row=1, padx=8, pady=4, sticky="nsew")
        parent.rowconfigure(1, weight=1)
        
        frame.columnconfigure(0, weight=0) # Gauges
        frame.columnconfigure(1, weight=1) # Graph
        frame.rowconfigure(0, weight=1)

        # Left side: Gauges section
        gauges_frame = ttk.Frame(frame)
        gauges_frame.grid(column=0, row=0, padx=8, pady=8, sticky="nsew")

        # Status label
        self.data_status_label = ttk.Label(gauges_frame, text="Status: INIT | 0.0 Hz", font=("TkDefaultFont", 10))
        self.data_status_label.grid(column=0, row=0, columnspan=3, padx=4, pady=(0, 8))
        status_row = ttk.Frame(gauges_frame)
        status_row.grid(column=0, row=1, columnspan=3, padx=4, pady=(0, 8), sticky="w")
        self.recv_status_badge = ttk.Label(status_row, text="Receive: idle", style="BadgeIdle.TLabel")
        self.recv_status_badge.grid(column=0, row=0, padx=(0, 6))
        self.send_status_badge = ttk.Label(status_row, text="Send: idle", style="BadgeIdle.TLabel")
        self.send_status_badge.grid(column=1, row=0, padx=(0, 6))

        # Create three canvas widgets for gauges
        gauge_size = 120
        self.yaw_canvas = tk.Canvas(gauges_frame, width=gauge_size, height=gauge_size + 40, bg=self.palette["panel"], highlightthickness=1, highlightbackground="#d7deea")
        self.yaw_canvas.grid(column=0, row=2, padx=8, pady=4)

        self.pitch_canvas = tk.Canvas(gauges_frame, width=gauge_size, height=gauge_size + 40, bg=self.palette["panel"], highlightthickness=1, highlightbackground="#d7deea")
        self.pitch_canvas.grid(column=1, row=2, padx=8, pady=4)

        self.roll_canvas = tk.Canvas(gauges_frame, width=gauge_size, height=gauge_size + 40, bg=self.palette["panel"], highlightthickness=1, highlightbackground="#d7deea")
        self.roll_canvas.grid(column=2, row=2, padx=8, pady=4)

        # Draw initial gauges
        self.draw_gauge(self.yaw_canvas, 0, -180, 180, "Yaw", "#0000FF")
        self.draw_gauge(self.pitch_canvas, 0, -90, 90, "Pitch", "#009600")
        self.draw_gauge(self.roll_canvas, 0, -90, 90, "Roll", "#C80000")

        # Right side: History graph
        history_frame = ttk.Frame(frame)
        history_frame.grid(column=1, row=0, padx=8, pady=8, sticky="nsew")

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

    def build_raw_data_sheet(self, parent: ttk.Frame):
        """Build a spreadsheet-like grid for raw data"""
        self.sheet_cells = {}
        
        # Create Scrollable Canvas
        canvas = tk.Canvas(parent, bg=self.palette["bg"], highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas, style="App.TFrame")

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Mousewheel scrolling
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        canvas.bind_all("<MouseWheel>", _on_mousewheel)
        # Linux Support
        canvas.bind_all("<Button-4>", lambda e: canvas.yview_scroll(-1, "units"))
        canvas.bind_all("<Button-5>", lambda e: canvas.yview_scroll(1, "units"))

        canvas.grid(column=0, row=0, sticky="nsew")
        scrollbar.grid(column=1, row=0, sticky="ns")
        parent.columnconfigure(0, weight=1)
        parent.rowconfigure(0, weight=1)

        # --- Spreadsheet Construction ---
        
        # Helper for creating cells
        def make_header(p, text, r, c, w=20):
            lbl = ttk.Label(p, text=text, style="SheetHeader.TLabel", width=w, anchor="center")
            lbl.grid(row=r, column=c, sticky="nsew", padx=1, pady=1)
            return lbl
            
        def make_label(p, text, r, c, w=20):
            lbl = ttk.Label(p, text=text, style="SheetLabel.TLabel", width=w, anchor="e")
            lbl.grid(row=r, column=c, sticky="nsew", padx=1, pady=1)
            return lbl

        def make_value(p, key, r, c, w=40):
            lbl = ttk.Label(p, text="0", style="SheetCell.TLabel", width=w, anchor="w")
            lbl.grid(row=r, column=c, sticky="nsew", padx=1, pady=1)
            self.sheet_cells[key] = lbl
            return lbl

        # SECTION 1: Headset & System (Row 0)
        sec1 = ttk.LabelFrame(scrollable_frame, text="Headset System", style="Section.TLabelframe")
        sec1.grid(row=0, column=0, columnspan=2, sticky="ew", padx=8, pady=8)
        
        make_header(sec1, "Property", 0, 0)
        make_header(sec1, "Value", 0, 1, w=60)
        
        make_label(sec1, "Pose", 1, 0)
        make_value(sec1, "headset_pose", 1, 1)
        
        make_label(sec1, "Status", 2, 0)
        make_value(sec1, "headset_status", 2, 1)
        
        make_label(sec1, "Timestamp", 3, 0)
        make_value(sec1, "headset_time", 3, 1)


        # SECTION 2: Controllers (Row 1)
        sec2 = ttk.LabelFrame(scrollable_frame, text="Controllers", style="Section.TLabelframe")
        sec2.grid(row=1, column=0, columnspan=2, sticky="ew", padx=8, pady=4)
        
        make_header(sec2, "Property", 0, 0)
        make_header(sec2, "Left Controller", 0, 1, w=45)
        make_header(sec2, "Right Controller", 0, 2, w=45)
        
        rows = ["Pose", "Axis (X,Y)", "Trigger / Grip", "Buttons"]
        keys = ["c_pose", "c_axis", "c_trigger", "c_buttons"]
        
        for i, (label, key_suffix) in enumerate(zip(rows, keys)):
            r = i + 1
            make_label(sec2, label, r, 0)
            make_value(sec2, f"left_{key_suffix}", r, 1, w=45)
            make_value(sec2, f"right_{key_suffix}", r, 2, w=45)


        # SECTION 3: Hands Summary (Row 2)
        sec3 = ttk.LabelFrame(scrollable_frame, text="Hands Summary", style="Section.TLabelframe")
        sec3.grid(row=2, column=0, columnspan=2, sticky="ew", padx=8, pady=4)
        
        make_header(sec3, "Property", 0, 0)
        make_header(sec3, "Left Hand", 0, 1, w=45)
        make_header(sec3, "Right Hand", 0, 2, w=45)
        
        make_label(sec3, "Active", 1, 0)
        make_value(sec3, "left_h_active", 1, 1, w=45)
        make_value(sec3, "right_h_active", 1, 2, w=45)
        
        make_label(sec3, "Scale", 2, 0)
        make_value(sec3, "left_h_scale", 2, 1, w=45)
        make_value(sec3, "right_h_scale", 2, 2, w=45)


        # SECTION 4: Hand Joints (Row 3) - The Big Table
        sec4 = ttk.LabelFrame(scrollable_frame, text="Hand Joints (26)", style="Section.TLabelframe")
        sec4.grid(row=3, column=0, columnspan=2, sticky="ew", padx=8, pady=4)
        
        make_header(sec4, "Joint ID", 0, 0, w=10)
        make_header(sec4, "Left Hand Pose (x,y,z,qx,qy,qz,qw)", 0, 1, w=60)
        make_header(sec4, "Right Hand Pose (x,y,z,qx,qy,qz,qw)", 0, 2, w=60)
        
        for i in range(26):
            r = i + 1
            make_label(sec4, f"Joint {i}", r, 0, w=10)
            make_value(sec4, f"joint_{i}_left", r, 1, w=60)
            make_value(sec4, f"joint_{i}_right", r, 2, w=60)


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
                          outline='#d0d7e4', width=2)

        # Draw tick marks every 30 degrees
        for mark_angle in range(int(min_angle), int(max_angle) + 1, 30):
            # Convert angle to gauge position (bottom = min, top = max)
            gauge_angle = -180 + (mark_angle - min_angle) / (max_angle - min_angle) * 180
            rad = math.radians(gauge_angle)

            outer_x = center_x + int((radius - 5) * math.cos(rad))
            outer_y = center_y + int((radius - 5) * math.sin(rad))
            inner_x = center_x + int((radius - 15) * math.cos(rad))
            inner_y = center_y + int((radius - 15) * math.sin(rad))

            canvas.create_line(outer_x, outer_y, inner_x, inner_y, fill='#d0d7e4', width=2)

        # Draw needle
        gauge_angle = -180 + (angle - min_angle) / (max_angle - min_angle) * 180
        rad = math.radians(gauge_angle)

        needle_x = center_x + int((radius - 20) * math.cos(rad))
        needle_y = center_y + int((radius - 20) * math.sin(rad))

        canvas.create_line(center_x, center_y, needle_x, needle_y, fill=color, width=3)
        canvas.create_oval(center_x - 6, center_y - 6, center_x + 6, center_y + 6, fill=color, outline=color)

        # Draw label
        canvas.create_text(center_x, height - 30, text=label, font=("TkDefaultFont", 10, "bold"), fill=self.palette["text"])

        # Draw value
        canvas.create_text(center_x, height - 10, text=f"{angle:.1f}°", font=("TkDefaultFont", 9), fill=self.palette["muted"])

    def build_actions_frame(self, parent: ttk.Frame):
        frame = ttk.LabelFrame(parent, text="Actions", style="Section.TLabelframe")
        frame.grid(column=0, row=2, padx=8, pady=4, sticky="ew")

        start_btn = ttk.Button(frame, text="Start", style="Ghost.TButton", command=self.restart_app)
        start_btn.grid(column=0, row=0, padx=6, pady=6, sticky="ew")

        stop_btn = ttk.Button(frame, text="Stop", style="Accent.TButton", command=self.stop_app)
        stop_btn.grid(column=1, row=0, padx=6, pady=6, sticky="ew")

        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)

    def build_log_frame(self, parent: ttk.Frame):
        frame = ttk.LabelFrame(parent, text="Log", style="Section.TLabelframe")
        frame.grid(column=0, row=3, padx=8, pady=4, sticky="nsew")
        parent.rowconfigure(3, weight=1)

        self.log_widget = scrolledtext.ScrolledText(
            frame,
            wrap=tk.WORD,
            height=10,
            state="disabled",
            background="#f5f7fb",
            foreground=self.palette["text"],
            insertbackground=self.palette["text"],
            borderwidth=0
        )
        self.log_widget.grid(column=0, row=0, padx=8, pady=8, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)

        self.log_frame = frame
        self.hide_log_frame()

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

    def toggle_log_frame(self):
        if self.log_visible:
            self.hide_log_frame()
        else:
            self.show_log_frame()

    def hide_log_frame(self):
        if hasattr(self, "log_frame"):
            self.log_frame.grid_remove()
            self.log_visible = False
            if hasattr(self, "log_toggle_btn"):
                self.log_toggle_btn.configure(text="Show Logs")

    def show_log_frame(self):
        if hasattr(self, "log_frame"):
            self.log_frame.grid()
            self.log_visible = True
            if hasattr(self, "log_toggle_btn"):
                self.log_toggle_btn.configure(text="Hide Logs")

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

            screenshot_btn = ttk.Button(row, text="Screenshot", command=lambda d=device: self.capture_screenshot_device(d))
            screenshot_btn.grid(column=3, row=0, padx=4, sticky="ew")

            row.columnconfigure(0, weight=1)
            row.columnconfigure(1, weight=0)
            row.columnconfigure(2, weight=0)
            row.columnconfigure(3, weight=0)

    def selected_devices(self):
        selected = [dev for dev, var in self.device_vars.items() if var.get() == 1]
        if selected:
            return selected
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

    def auto_connect_wifi(self):
        """Switch selected USB device to TCPIP:5555 and connect over Wi-Fi."""
        port = "5555"

        def _run():
            self.set_status("Preparing ADB over Wi-Fi...")
            # Prefer explicitly selected USB device; fall back to first detected USB device.
            usb_candidates = [d for d in self.selected_devices() if ":" not in d]
            if not usb_candidates:
                usb_candidates = [d for d in get_connected_adb_devices() if ":" not in d]

            if not usb_candidates:
                self.log("Wi-Fi connect: no USB-connected devices detected")
                self.set_status("No USB device found")
                return

            device = usb_candidates[0]
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

            self.log(f"Disconnected rotation data endpoint: {endpoint}")
            self.set_status("Idle")

        except Exception as e:
            messagebox.showerror("Rotation Data", f"Failed to disconnect: {e}")
            self.log(f"Failed to disconnect from {endpoint}: {e}")

    def disconnect_all_rotation_endpoints(self):
        """Disconnect all rotation endpoints"""
        for endpoint in list(self.rotation_endpoints):
            self.disconnect_rotation_endpoint(endpoint)

    def render_rotation_endpoints(self):
        """Render connected rotation endpoints with per-row disconnect buttons"""
        for child in self.rotation_endpoints_frame.winfo_children():
            child.destroy()

        if not self.rotation_endpoints:
            ttk.Label(self.rotation_endpoints_frame, text="No endpoints connected", style="TLabel").grid(column=0, row=0, padx=6, pady=4, sticky="w")
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

    def _reconnect_rotation_endpoint(self, endpoint: str):
        """Attempt to reconnect an existing endpoint"""
        # Disconnect first, then connect again
        self.disconnect_rotation_endpoint(endpoint)
        self.rotation_ip_var.set(endpoint)
        self.connect_rotation_endpoint()

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

    def collect_full_xr_data(self) -> dict:
        """Collect all available XR data into a dictionary structure."""
        data = {}
        
        # Helper to format pose array to string "x,y,z,qx,qy,qz,qw"
        def fmt_pose(p):
            if p is None: return "0.0,0.0,0.0,0.0,0.0,0.0,0.0"
            return ",".join([f"{x:.4f}" for x in p])

        ts = self.xr_client.get_timestamp_ns()
        
        # 1. Headset
        headset_pose = self.xr_client.get_pose_by_name("headset")
        data["headset"] = {
            "pose": fmt_pose(headset_pose),
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
                    "timeStampNs": ts
                }
            except Exception:
                # Return empty/default structure on error
                return {
                    "axisX": 0.0, "axisY": 0.0, "grip": 0.0, "trigger": 0.0,
                    "primaryButton": False, "secondaryButton": False, "menuButton": False, "axisClick": False,
                    "pose": fmt_pose(None), "timeStampNs": ts
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
                with self.data_lock:
                    self.raw_data_snapshot = full_data
                    self.mediapipe_points["left"] = left_mp
                    self.mediapipe_points["right"] = right_mp

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
                mp_left = np.array(self.mediapipe_points["left"]) if self.mediapipe_points.get("left") is not None else None
                mp_right = np.array(self.mediapipe_points["right"]) if self.mediapipe_points.get("right") is not None else None

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

            if update_visualizer:
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
                self.update_mediapipe_table("left", mp_left)
                self.update_mediapipe_table("right", mp_right)

            elif update_raw_data and raw_snapshot:
                # Update Sheet Labels
                
                # 1. Headset
                hs = raw_snapshot.get("headset", {})
                self.sheet_cells["headset_pose"].configure(text=hs.get("pose", "0"))
                self.sheet_cells["headset_status"].configure(text=str(hs.get("status", "0")))
                self.sheet_cells["headset_time"].configure(text=str(hs.get("timeStampNs", "0")))

                # 2. Controllers
                for side in ["Left", "Right"]:
                    key = side.lower()
                    c_key = f"{key}Controller"
                    c_data = raw_snapshot.get(c_key, {})
                    
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
                    h_data = raw_snapshot.get(h_key, {})
                    
                    self.sheet_cells[f"{key}_h_active"].configure(text="Yes" if h_data.get("isActive") else "No")
                    self.sheet_cells[f"{key}_h_scale"].configure(text=f"{h_data.get('scale', 1.0):.2f}")
                    
                # 4. Hand Joints
                # We need to loop carefully. 26 rows.
                left_h = raw_snapshot.get("leftHand", {}).get("joints", [])
                right_h = raw_snapshot.get("rightHand", {}).get("joints", [])
                
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

    def build_mediapipe_tab(self, parent: ttk.Frame):
        """Display MediaPipe-converted hand keypoints for both hands."""
        self.mediapipe_tables = {}

        def build_table(col: int, title: str, key: str):
            frame = ttk.LabelFrame(parent, text=title, style="Section.TLabelframe")
            frame.grid(row=0, column=col, padx=8, pady=8, sticky="nsew")
            frame.columnconfigure(0, weight=1)
            frame.rowconfigure(0, weight=1)

            columns = ("joint", "x", "y", "z")
            tree = ttk.Treeview(frame, columns=columns, show="headings", height=22)
            tree.heading("joint", text="Joint #")
            tree.heading("x", text="X (rel)")
            tree.heading("y", text="Y (rel)")
            tree.heading("z", text="Z (rel)")
            tree.column("joint", width=60, anchor="center")
            tree.column("x", width=80, anchor="e")
            tree.column("y", width=80, anchor="e")
            tree.column("z", width=80, anchor="e")

            vsb = ttk.Scrollbar(frame, orient="vertical", command=tree.yview)
            tree.configure(yscrollcommand=vsb.set)
            tree.grid(row=0, column=0, sticky="nsew")
            vsb.grid(row=0, column=1, sticky="ns")

            # Pre-populate rows
            for idx in range(21):
                tree.insert("", "end", iid=str(idx), values=(idx, "--", "--", "--"))

            self.mediapipe_tables[key] = tree

            note = ttk.Label(frame, text="Relative to wrist (MediaPipe coords)", style="TLabel")
            note.grid(row=1, column=0, columnspan=2, sticky="w", padx=4, pady=(4, 2))

        build_table(0, "Left Hand (MediaPipe)", "left")
        build_table(1, "Right Hand (MediaPipe)", "right")

    def update_mediapipe_table(self, side: str, mp_data):
        """Populate MediaPipe tables with converted hand data."""
        tree = self.mediapipe_tables.get(side)
        if not tree:
            return

        def set_row(idx, x_val, y_val, z_val):
            tree.set(str(idx), column="x", value=x_val)
            tree.set(str(idx), column="y", value=y_val)
            tree.set(str(idx), column="z", value=z_val)

        if mp_data is None or len(mp_data) == 0:
            for i in range(21):
                set_row(i, "--", "--", "--")
            return

        for i in range(21):
            if i < len(mp_data):
                x, y, z = mp_data[i]
                set_row(i, f"{x:.3f}", f"{y:.3f}", f"{z:.3f}")
            else:
                set_row(i, "--", "--", "--")

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
