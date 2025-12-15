"""Dashboard tab builders and helpers."""

import math
import tkinter as tk
from tkinter import ttk

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class DashboardTabMixin:
    def build_dashboard_frames(self, parent: ttk.Frame):
        """Populate the dashboard tab with its sub-sections."""
        self.build_devices_frame(parent)
        self.build_visualizer_frame(parent)
        self.build_actions_frame(parent)
        self.build_log_frame(parent)

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
        self.rotation_endpoints_frame = ttk.Frame(frame, style="Surface.TFrame")
        self.rotation_endpoints_frame.grid(column=0, row=11, columnspan=2, padx=8, pady=2, sticky="nsew")

        self.render_rotation_endpoints()

        # MediaPipe Hand Data Broadcasting section
        ttk.Label(frame, text="MediaPipe Hand Data Broadcasting", font=("TkDefaultFont", 9, "bold")).grid(column=0, row=12, columnspan=2, padx=8, pady=(12, 4), sticky="w")

        hand_entry_row = ttk.Frame(frame, style="Surface.TFrame")
        hand_entry_row.grid(column=0, row=13, columnspan=2, padx=8, pady=(0, 4), sticky="ew")
        hand_entry_row.columnconfigure(0, weight=1)
        hand_entry_row.columnconfigure(1, weight=0)

        hand_ip_entry = ttk.Entry(hand_entry_row, textvariable=self.hand_data_ip_var, width=18)
        hand_ip_entry.grid(column=0, row=0, padx=(0, 6), pady=2, sticky="ew")

        hand_connect_btn = ttk.Button(hand_entry_row, text="Connect", style="Ghost.TButton", command=self.connect_hand_data_endpoint)
        hand_connect_btn.grid(column=1, row=0, padx=4, pady=2, sticky="ew")

        # List of connected hand data endpoints
        self.hand_endpoints_frame = ttk.Frame(frame, style="Surface.TFrame")
        self.hand_endpoints_frame.grid(column=0, row=14, columnspan=2, padx=8, pady=2, sticky="nsew")

        self.render_hand_data_endpoints()

    def build_visualizer_frame(self, parent: ttk.Frame):
        """Build the dashboard visualizer (Gauges + Graph)."""
        frame = ttk.LabelFrame(parent, text="Head Rotation Visualizer", style="Section.TLabelframe")
        frame.grid(column=0, row=1, padx=8, pady=4, sticky="nsew")
        parent.rowconfigure(1, weight=1)

        frame.columnconfigure(0, weight=1)  # Graph fills width
        frame.rowconfigure(0, weight=0)
        frame.rowconfigure(1, weight=1)

        # Status + badges row (no gauges)
        header = ttk.Frame(frame)
        header.grid(column=0, row=0, padx=8, pady=(8, 0), sticky="ew")
        header.columnconfigure(0, weight=1)
        header.columnconfigure(1, weight=0)
        header.columnconfigure(2, weight=0)

        self.data_status_label = ttk.Label(header, text="Status: INIT | 0.0 Hz", font=("TkDefaultFont", 10))
        self.data_status_label.grid(column=0, row=0, padx=(0, 8), pady=(0, 4), sticky="w")

        self.recv_status_badge = ttk.Label(header, text="Receive: idle", style="BadgeIdle.TLabel")
        self.recv_status_badge.grid(column=1, row=0, padx=(0, 6), sticky="e")
        self.send_status_badge = ttk.Label(header, text="Send: idle", style="BadgeIdle.TLabel")
        self.send_status_badge.grid(column=2, row=0, sticky="e")

        # History graph (takes remaining space)
        history_frame = ttk.Frame(frame)
        history_frame.grid(column=0, row=1, padx=8, pady=8, sticky="nsew")

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

        # Gauges removed; keep attributes for guard checks
        self.yaw_canvas = None
        self.pitch_canvas = None
        self.roll_canvas = None

    def draw_gauge(self, canvas: tk.Canvas, angle_deg: float, min_angle: float, max_angle: float, label: str, color: str):
        """Draw a simple semicircular gauge."""
        canvas.delete("all")
        center_x = int(canvas['width']) // 2
        center_y = int(canvas['height']) - 20
        radius = min(center_x, center_y) - 10

        # Draw arc
        canvas.create_arc(center_x - radius, center_y - radius, center_x + radius, center_y + radius,
                          start=180, extent=180, style=tk.ARC, width=2)

        # Needle
        angle_rad = math.radians(180 * (angle_deg - min_angle) / (max_angle - min_angle))
        x = center_x + radius * math.cos(angle_rad)
        y = center_y + radius * math.sin(angle_rad)
        canvas.create_line(center_x, center_y, x, y, fill=color, width=3)

        # Label and value
        canvas.create_text(center_x, center_y - radius - 10, text=label, font=("TkDefaultFont", 10, "bold"))
        canvas.create_text(center_x, center_y + 10, text=f"{angle_deg:.1f}°", font=("TkDefaultFont", 10))

    def build_actions_frame(self, parent: ttk.Frame):
        frame = ttk.LabelFrame(parent, text="Actions", style="Section.TLabelframe")
        frame.grid(column=0, row=2, padx=8, pady=4, sticky="ew")

        btn_params = dict(style="Accent.TButton", width=16)
        ttk.Button(frame, text="Open App", command=self.open_app, **btn_params).grid(column=0, row=0, padx=4, pady=4)
        ttk.Button(frame, text="Stop App", command=self.stop_app, **btn_params).grid(column=1, row=0, padx=4, pady=4)
        ttk.Button(frame, text="Restart App", command=self.restart_app, **btn_params).grid(column=2, row=0, padx=4, pady=4)

        ttk.Button(frame, text="Reboot Device", command=lambda: self.run_for_devices("reboot", self.reboot_device), **btn_params).grid(column=0, row=1, padx=4, pady=4)
        ttk.Button(frame, text="Power Off", command=lambda: self.run_for_devices("power off", self.power_off_device), **btn_params).grid(column=1, row=1, padx=4, pady=4)
        ttk.Button(frame, text="Disconnect All", command=self.disconnect_all, **btn_params).grid(column=2, row=1, padx=4, pady=4)

    def build_log_frame(self, parent: ttk.Frame):
        frame = ttk.LabelFrame(parent, text="Log", style="Section.TLabelframe")
        frame.grid(column=0, row=3, padx=8, pady=4, sticky="nsew")
        parent.rowconfigure(3, weight=1)

        self.log_text = tk.Text(frame, height=8, width=80, bg="#0f172a", fg="#e2e8f0", insertbackground="white")
        self.log_text.grid(column=0, row=0, sticky="nsew")
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

        # Show/hide toggle button moved to header; visibility managed via toggle_log_frame.
        self.log_frame = frame
        self.hide_log_frame()

    def toggle_log_frame(self):
        self.log_visible = not self.log_visible
        if self.log_visible:
            self.log_frame.grid()
            self.log_toggle_btn.config(text="Hide Logs")
        else:
            self.log_frame.grid_remove()
            self.log_toggle_btn.config(text="Show Logs")

    def hide_log_frame(self):
        self.log_visible = False
        if hasattr(self, "log_frame"):
            self.log_frame.grid_remove()
        if hasattr(self, "log_toggle_btn"):
            self.log_toggle_btn.config(text="Show Logs")

    def show_log_frame(self):
        self.log_visible = True
        if hasattr(self, "log_frame"):
            self.log_frame.grid()
        if hasattr(self, "log_toggle_btn"):
            self.log_toggle_btn.config(text="Hide Logs")
