"""MediaPipe tab UI and update helpers."""

from tkinter import ttk


# MediaPipe hand landmark names (21 landmarks per hand)
MEDIAPIPE_JOINT_NAMES = [
    "WRIST",
    "THUMB_CMC",
    "THUMB_MCP",
    "THUMB_IP",
    "THUMB_TIP",
    "INDEX_MCP",
    "INDEX_PIP",
    "INDEX_DIP",
    "INDEX_TIP",
    "MIDDLE_MCP",
    "MIDDLE_PIP",
    "MIDDLE_DIP",
    "MIDDLE_TIP",
    "RING_MCP",
    "RING_PIP",
    "RING_DIP",
    "RING_TIP",
    "PINKY_MCP",
    "PINKY_PIP",
    "PINKY_DIP",
    "PINKY_TIP",
]


class MediaPipeTabMixin:
    def build_mediapipe_tab(self, parent: ttk.Frame):
        """Display MediaPipe-converted hand keypoints for both hands."""
        self.mediapipe_tables = {}

        # Status and rotation data section at the top
        top_section = ttk.Frame(parent, style="Surface.TFrame")
        top_section.grid(row=0, column=0, columnspan=2, padx=8, pady=(8, 4), sticky="ew")
        top_section.columnconfigure(0, weight=1)
        top_section.columnconfigure(1, weight=0)

        # Left column: Rotation data display
        rotation_frame = ttk.LabelFrame(top_section, text="Head Rotation Data", style="Section.TLabelframe")
        rotation_frame.grid(row=0, column=0, padx=(0, 4), pady=0, sticky="ew")
        rotation_frame.columnconfigure(0, weight=0)
        rotation_frame.columnconfigure(1, weight=1)

        # Create rotation data labels with proper spacing
        ttk.Label(rotation_frame, text="Yaw:", font=("TkDefaultFont", 9, "bold")).grid(row=0, column=0, padx=(8, 4), pady=4, sticky="w")
        self.mediapipe_yaw_label = ttk.Label(rotation_frame, text="0.0°", style="TLabel")
        self.mediapipe_yaw_label.grid(row=0, column=1, padx=(0, 8), pady=4, sticky="w")

        ttk.Label(rotation_frame, text="Pitch:", font=("TkDefaultFont", 9, "bold")).grid(row=1, column=0, padx=(8, 4), pady=4, sticky="w")
        self.mediapipe_pitch_label = ttk.Label(rotation_frame, text="0.0°", style="TLabel")
        self.mediapipe_pitch_label.grid(row=1, column=1, padx=(0, 8), pady=4, sticky="w")

        ttk.Label(rotation_frame, text="Roll:", font=("TkDefaultFont", 9, "bold")).grid(row=2, column=0, padx=(8, 4), pady=4, sticky="w")
        self.mediapipe_roll_label = ttk.Label(rotation_frame, text="0.0°", style="TLabel")
        self.mediapipe_roll_label.grid(row=2, column=1, padx=(0, 8), pady=4, sticky="w")

        # Right column: Targets and status
        status_frame = ttk.LabelFrame(top_section, text="Connection Status", style="Section.TLabelframe")
        status_frame.grid(row=0, column=1, padx=(4, 0), pady=0, sticky="nsew")
        status_frame.columnconfigure(0, weight=1)

        self.mediapipe_target_label = ttk.Label(status_frame, text="Targets: none", style="TLabel", wraplength=250)
        self.mediapipe_target_label.grid(column=0, row=0, padx=8, pady=(8, 4), sticky="w")

        self.mediapipe_send_badge = ttk.Label(status_frame, text="Send: idle", style="BadgeIdle.TLabel")
        self.mediapipe_send_badge.grid(column=0, row=1, padx=8, pady=(4, 8), sticky="w")

        def build_table(col: int, title: str, key: str):
            frame = ttk.LabelFrame(parent, text=title, style="Section.TLabelframe")
            frame.grid(row=1, column=col, padx=8, pady=8, sticky="nsew")
            frame.columnconfigure(0, weight=1)
            frame.rowconfigure(0, weight=1)

            columns = ("joint", "x", "y", "z")
            tree = ttk.Treeview(frame, columns=columns, show="headings", height=22)
            tree.heading("joint", text="Joint Name")
            tree.heading("x", text="X (rel)")
            tree.heading("y", text="Y (rel)")
            tree.heading("z", text="Z (rel)")
            tree.column("joint", width=120, anchor="w")
            tree.column("x", width=80, anchor="e")
            tree.column("y", width=80, anchor="e")
            tree.column("z", width=80, anchor="e")

            vsb = ttk.Scrollbar(frame, orient="vertical", command=tree.yview)
            tree.configure(yscrollcommand=vsb.set)
            tree.grid(row=0, column=0, sticky="nsew")
            vsb.grid(row=0, column=1, sticky="ns")

            # Pre-populate rows with joint names
            for idx in range(21):
                joint_name = MEDIAPIPE_JOINT_NAMES[idx] if idx < len(MEDIAPIPE_JOINT_NAMES) else f"Joint {idx}"
                tree.insert("", "end", iid=str(idx), values=(joint_name, "--", "--", "--"))

            self.mediapipe_tables[key] = tree

            note = ttk.Label(frame, text="Relative to wrist (MediaPipe coords)", style="TLabel")
            note.grid(row=1, column=0, columnspan=2, sticky="w", padx=4, pady=(4, 2))

        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)
        parent.rowconfigure(0, weight=0)
        parent.rowconfigure(1, weight=1)

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

    def update_mediapipe_send_badge(self, active: bool, stale: bool, endpoints=None):
        """Update send-status badge and target list for MediaPipe tab."""
        if not hasattr(self, "mediapipe_send_badge"):
            return

        targets = endpoints if endpoints is not None else []
        targets_text = ", ".join(targets) if targets else "none"
        if hasattr(self, "mediapipe_target_label"):
            self.mediapipe_target_label.configure(text=f"Targets: {targets_text}")

        if active:
            style = "BadgeGood.TLabel"
            text = "Send: active"
        elif stale:
            style = "BadgeWarn.TLabel"
            text = "Send: stale"
        else:
            style = "BadgeIdle.TLabel"
            text = "Send: idle"
        self.mediapipe_send_badge.configure(text=text, style=style)

    def update_mediapipe_targets_only(self, endpoints=None):
        """Refresh targets label without touching send badge."""
        targets = endpoints if endpoints is not None else []
        targets_text = ", ".join(targets) if targets else "none"
        if hasattr(self, "mediapipe_target_label"):
            self.mediapipe_target_label.configure(text=f"Targets: {targets_text}")

    def update_mediapipe_rotation_data(self, yaw: float, pitch: float, roll: float):
        """Update rotation data labels in MediaPipe tab."""
        if hasattr(self, "mediapipe_yaw_label"):
            self.mediapipe_yaw_label.configure(text=f"{yaw:.1f}°")
        if hasattr(self, "mediapipe_pitch_label"):
            self.mediapipe_pitch_label.configure(text=f"{pitch:.1f}°")
        if hasattr(self, "mediapipe_roll_label"):
            self.mediapipe_roll_label.configure(text=f"{roll:.1f}°")
