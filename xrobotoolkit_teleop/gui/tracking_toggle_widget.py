"""Simple toggle widget for enabling/disabling hand tracking control."""

from PyQt5.QtWidgets import (
    QWidget,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QSlider,
    QLabel,
    QGroupBox,
    QScrollArea,
    QSizePolicy,
    QFrame,
)
from PyQt5.QtCore import Qt, QTimer
import math
import time
import numpy as np

# Try to import UR interfaces for hardware connection
try:
    from xrobotoolkit_teleop.hardware.interface.universal_robots import URReader, URController
    RTDE_AVAILABLE = True
except ImportError:
    RTDE_AVAILABLE = False
    print("RTDE libraries not available. Hardware connection disabled.")


class TrackingToggleWidget(QWidget):
    """Widget with a toggle button and sliders to display simulation and hardware joint angles."""

    # Left arm joint names for UR5e (simulation)
    LEFT_ARM_JOINTS = [
        "left_shoulder_pan_joint",
        "left_shoulder_lift_joint",
        "left_elbow_joint",
        "left_wrist_1_joint",
        "left_wrist_2_joint",
        "left_wrist_3_joint",
    ]

    # Hardware joint names (without "left_" prefix)
    HARDWARE_JOINTS = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]

    # Slider scale factor (1 radian = 100 slider units for precision)
    SLIDER_SCALE = 100
    DEFAULT_SHOULDER_PAN_OFFSET_DEG = 180.0

    def __init__(self, parent=None):
        super().__init__(parent)
        self._tracking_enabled = True
        self.sliders = {}
        self.value_labels = {}
        self._slider_callback = None
        self.rotation_sliders = {}
        self.rotation_value_labels = {}
        self._rotation_callback = None
        self.shoulder_pan_offset_deg = self.DEFAULT_SHOULDER_PAN_OFFSET_DEG

        # Hardware connection variables
        self.hardware_sliders = {}
        self.hardware_value_labels = {}
        self.hardware_connected = False
        self.ur_reader = None

        # Hardware control variables
        self.control_arm_enabled = False
        self.ur_controller = None
        self.current_hw_positions = np.zeros(6)  # Current hardware joint positions
        self.target_hw_positions = np.zeros(6)   # Target positions from sliders
        self.smoothed_target_hw_positions = np.zeros(6)
        self.target_filter_alpha = 0.25  # Low-pass filter on target positions
        self.max_joint_speed_rad_s = 0.8  # Speed limit per joint (rad/s)
        self.movement_threshold = 0.0  # Send even small deltas for smooth motion

        # Timer for automatic hardware updates
        self.hardware_timer = QTimer()
        self.hardware_timer.timeout.connect(self._update_hardware_sliders)
        self.hardware_timer.setInterval(100)  # Update every 100ms (10 Hz)

        # Timer for sending control commands
        self.control_timer = QTimer()
        self.control_timer.timeout.connect(self._send_hardware_command)
        self.control_timer.setInterval(100)  # Send commands at 10 Hz

        self._init_ui()

    def _wrap_angle_rad(self, angle_rad: float) -> float:
        """Wrap angle to [-pi, pi] for slider display."""
        return (angle_rad + math.pi) % (2 * math.pi) - math.pi

    def _apply_shoulder_pan_offset(self, angle_rad: float, direction: int) -> float:
        """Apply shoulder pan offset (+1 for sim->hw, -1 for hw->sim)."""
        return angle_rad + direction * math.radians(self.shoulder_pan_offset_deg)

    def _init_ui(self):
        """Initialize the user interface."""
        outer_layout = QVBoxLayout()
        outer_layout.setContentsMargins(12, 12, 12, 12)
        outer_layout.setSpacing(12)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QFrame.NoFrame)
        outer_layout.addWidget(scroll_area)

        content = QWidget()
        scroll_area.setWidget(content)

        layout = QVBoxLayout(content)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(16)

        title = QLabel("Dual UR5e Teleoperation")
        title.setStyleSheet("font-size: 20pt; font-weight: 600;")
        layout.addWidget(title, alignment=Qt.AlignCenter)

        # Create toggle button
        self.toggle_button = QPushButton("Hand Tracking: ON")
        self.toggle_button.setCheckable(True)
        self.toggle_button.setChecked(True)  # Start in ON state
        self.toggle_button.setMinimumSize(260, 60)
        self.toggle_button.setStyleSheet(
            """
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-size: 16pt;
                border-radius: 10px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            """
        )

        # Connect signal
        self.toggle_button.toggled.connect(self._on_toggle)

        tracking_group = QGroupBox("Hand Tracking")
        tracking_layout = QVBoxLayout()
        tracking_layout.addWidget(self.toggle_button, alignment=Qt.AlignCenter)
        rot_grid = QGridLayout()
        rot_grid.setHorizontalSpacing(12)
        rot_grid.setVerticalSpacing(6)
        rot_grid.setColumnStretch(1, 1)
        tracking_layout.addLayout(rot_grid)
        self._add_rotation_slider(rot_grid, 0, "X", 45)
        self._add_rotation_slider(rot_grid, 1, "Y", 0)
        self._add_rotation_slider(rot_grid, 2, "Z", 0)
        self._add_shoulder_pan_offset_control(tracking_layout)
        self.xr_status_label = QLabel("XR ZMQ: idle")
        self.xr_status_label.setStyleSheet("color: #475569; font-size: 11pt;")
        tracking_layout.addWidget(self.xr_status_label, alignment=Qt.AlignLeft)
        tracking_group.setLayout(tracking_layout)
        layout.addWidget(tracking_group)

        sim_group = QGroupBox("Simulation Joints (Left Arm)")
        sim_layout = QGridLayout()
        sim_layout.setHorizontalSpacing(16)
        sim_layout.setVerticalSpacing(8)
        sim_layout.setColumnStretch(1, 1)
        sim_group.setLayout(sim_layout)
        layout.addWidget(sim_group)

        for row, joint_name in enumerate(self.LEFT_ARM_JOINTS):
            self._add_joint_row(
                sim_layout,
                row,
                joint_name,
                self.sliders,
                self.value_labels,
                handle_color="#4CAF50",
                enabled=False,
                display_prefix="left_",
            )

        hw_group = QGroupBox("Hardware (RTDE) Left Arm")
        hw_group_layout = QVBoxLayout()
        hw_group_layout.setSpacing(12)

        # Status and buttons row
        status_button_layout = QHBoxLayout()

        # Status indicator
        self.status_label = QLabel("Status:")
        self.status_label.setStyleSheet("font-size: 14pt; font-weight: 600;")
        status_button_layout.addWidget(self.status_label)

        self.status_indicator = QLabel("● Disconnected")
        self.status_indicator.setStyleSheet("color: #f44336; font-size: 14pt;")
        status_button_layout.addWidget(self.status_indicator)

        status_button_layout.addStretch(1)

        # Connect button
        self.connect_button = QPushButton("Connect")
        self.connect_button.setMinimumSize(160, 48)
        self.connect_button.setStyleSheet(
            """
            QPushButton {
                background-color: #2196F3;
                color: white;
                font-size: 12pt;
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #1976D2; }
            """
        )
        self.connect_button.clicked.connect(self._on_connect_clicked)
        status_button_layout.addWidget(self.connect_button)

        status_button_layout.addSpacing(20)

        # Read button
        self.read_button = QPushButton("Read Joint Data")
        self.read_button.setMinimumSize(200, 48)
        self.read_button.setEnabled(False)  # Disabled until connected
        self.read_button.setStyleSheet(
            """
            QPushButton {
                background-color: #FF9800;
                color: white;
                font-size: 12pt;
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #F57C00; }
            QPushButton:disabled {
                background-color: #CCCCCC;
                color: #666666;
            }
            """
        )
        self.read_button.clicked.connect(self._on_read_clicked)
        status_button_layout.addWidget(self.read_button)

        status_button_layout.addSpacing(20)

        # Control Arm toggle button
        self.control_arm_button = QPushButton("Control Arm: OFF")
        self.control_arm_button.setCheckable(True)
        self.control_arm_button.setChecked(False)
        self.control_arm_button.setMinimumSize(200, 48)
        self.control_arm_button.setEnabled(False)  # Disabled until connected
        self.control_arm_button.setStyleSheet(
            """
            QPushButton {
                background-color: #9E9E9E;
                color: white;
                font-size: 12pt;
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #757575; }
            QPushButton:disabled {
                background-color: #CCCCCC;
                color: #666666;
            }
            QPushButton:checked {
                background-color: #4CAF50;
            }
            QPushButton:checked:hover {
                background-color: #45a049;
            }
            """
        )
        self.control_arm_button.toggled.connect(self._on_control_arm_toggled)
        status_button_layout.addWidget(self.control_arm_button)

        hw_group_layout.addLayout(status_button_layout)

        hw_grid = QGridLayout()
        hw_grid.setHorizontalSpacing(16)
        hw_grid.setVerticalSpacing(8)
        hw_grid.setColumnStretch(1, 1)
        hw_group_layout.addLayout(hw_grid)

        # Create 6 hardware sliders
        for row, joint_name in enumerate(self.HARDWARE_JOINTS):
            self._add_joint_row(
                hw_grid,
                row,
                joint_name,
                self.hardware_sliders,
                self.hardware_value_labels,
                handle_color="#FF9800",
                enabled=False,
                display_prefix="",
            )

        hw_group.setLayout(hw_group_layout)
        layout.addWidget(hw_group)
        layout.addStretch(1)
        self.setLayout(outer_layout)

        # Set window properties
        self.setMinimumSize(1400, 900)

    def _add_joint_row(
        self,
        grid_layout,
        row,
        joint_name,
        sliders,
        value_labels,
        handle_color,
        enabled,
        display_prefix,
    ):
        """Add a labeled slider row to a grid layout."""
        display_name = joint_name.replace(display_prefix, "").replace("_", " ").title()

        name_label = QLabel(display_name)
        name_label.setMinimumWidth(200)
        name_label.setStyleSheet("font-size: 12pt;")
        grid_layout.addWidget(name_label, row, 0)

        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(int(-math.pi * self.SLIDER_SCALE))
        slider.setMaximum(int(math.pi * self.SLIDER_SCALE))
        slider.setValue(0)
        slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        slider.setStyleSheet(
            f"""
            QSlider::groove:horizontal {{
                height: 12px;
                background: #ddd;
                border-radius: 6px;
            }}
            QSlider::handle:horizontal {{
                background: {handle_color};
                width: 24px;
                margin: -6px 0;
                border-radius: 12px;
            }}
            """
        )
        slider.setEnabled(enabled)
        if sliders is self.sliders:
            slider.valueChanged.connect(self._on_slider_changed)
        else:
            slider.valueChanged.connect(self._on_hardware_slider_changed)
        sliders[joint_name] = slider
        grid_layout.addWidget(slider, row, 1)

        value_label = QLabel("0.00")
        value_label.setMinimumWidth(90)
        value_label.setStyleSheet("font-size: 12pt;")
        value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        value_labels[joint_name] = value_label
        grid_layout.addWidget(value_label, row, 2)

    def _on_toggle(self, checked):
        """Handle toggle button state changes."""
        self._tracking_enabled = checked

        # Enable/disable sliders based on tracking state
        for slider in self.sliders.values():
            slider.setEnabled(not checked)  # Enabled when tracking OFF
        for slider in self.hardware_sliders.values():
            slider.setEnabled(self.control_arm_enabled or not checked)

        if checked:
            self.toggle_button.setText("Hand Tracking: ON")
            self.toggle_button.setStyleSheet(
                """
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    font-size: 16pt;
                    border-radius: 10px;
                }
                QPushButton:hover {
                    background-color: #45a049;
                }
                """
            )
        else:
            self.toggle_button.setText("Hand Tracking: OFF")
            self.toggle_button.setStyleSheet(
                """
                QPushButton {
                    background-color: #f44336;
                    color: white;
                    font-size: 16pt;
                    border-radius: 10px;
                }
                QPushButton:hover {
                    background-color: #da190b;
                }
                """
            )

    def update_xr_status(self, xr_client):
        """Update XR ZMQ receive status if available."""
        if not hasattr(self, "xr_status_label"):
            return
        if not hasattr(xr_client, "last_receive_ts"):
            self.xr_status_label.setText("XR ZMQ: n/a")
            return
        now = time.time()
        last_ts = getattr(xr_client, "last_receive_ts", 0.0)
        count = getattr(xr_client, "recv_count", 0)
        if last_ts and (now - last_ts) < 1.0:
            self.xr_status_label.setText(f"XR ZMQ: active ({count})")
        elif last_ts:
            age_ms = (now - last_ts) * 1000
            self.xr_status_label.setText(f"XR ZMQ: stale ({age_ms:.0f} ms)")
        else:
            self.xr_status_label.setText("XR ZMQ: idle")

    def is_tracking_enabled(self):
        """Return True if hand tracking is enabled."""
        return self._tracking_enabled

    def set_slider_callback(self, callback):
        """Set callback function to be called when sliders change.

        Args:
            callback: Function that takes a dict of {joint_name: angle_rad}
        """
        self._slider_callback = callback

    def set_rotation_callback(self, callback):
        """Set callback function to be called when rotation sliders change."""
        self._rotation_callback = callback
        self._on_rotation_changed()

    def get_manual_joint_values(self):
        """Get current slider positions as joint values.

        Returns:
            Dictionary mapping joint names to angles in radians
        """
        joint_values = {}
        for joint_name in self.LEFT_ARM_JOINTS:
            slider = self.sliders[joint_name]
            angle_rad = slider.value() / self.SLIDER_SCALE
            joint_values[joint_name] = angle_rad
        return joint_values

    def _on_slider_changed(self):
        """Handle slider value changes."""
        # Only trigger callback when tracking is OFF and callback is set
        if not self._tracking_enabled and self._slider_callback is not None:
            joint_values = self.get_manual_joint_values()
            self._slider_callback(joint_values)

    def _add_rotation_slider(self, grid_layout, row, axis_label, default_value):
        """Add a rotation slider row (degrees) to a grid layout."""
        name_label = QLabel(f"Rot {axis_label} (deg)")
        name_label.setMinimumWidth(200)
        name_label.setStyleSheet("font-size: 12pt;")
        grid_layout.addWidget(name_label, row, 0)

        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(-180)
        slider.setMaximum(180)
        slider.setValue(default_value)
        slider.setSingleStep(45)
        slider.setPageStep(45)
        slider.setTickInterval(45)
        slider.setTickPosition(QSlider.TicksBelow)
        slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        slider.valueChanged.connect(self._on_rotation_changed)
        self.rotation_sliders[axis_label] = slider
        grid_layout.addWidget(slider, row, 1)

        value_label = QLabel(f"{default_value}°")
        value_label.setMinimumWidth(90)
        value_label.setStyleSheet("font-size: 12pt;")
        value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.rotation_value_labels[axis_label] = value_label
        grid_layout.addWidget(value_label, row, 2)

    def _on_rotation_changed(self):
        """Handle rotation slider value changes."""
        x_val = self.rotation_sliders["X"].value()
        y_val = self.rotation_sliders["Y"].value()
        z_val = self.rotation_sliders["Z"].value()
        self.rotation_value_labels["X"].setText(f"{x_val}°")
        self.rotation_value_labels["Y"].setText(f"{y_val}°")
        self.rotation_value_labels["Z"].setText(f"{z_val}°")
        if self._rotation_callback is not None:
            self._rotation_callback(np.array([x_val, y_val, z_val], dtype=float))

    def _add_shoulder_pan_offset_control(self, parent_layout):
        """Add UI control for shoulder pan offset (degrees)."""
        row = QHBoxLayout()
        label = QLabel("Shoulder Pan Offset (deg)")
        label.setStyleSheet("font-size: 12pt;")
        row.addWidget(label)

        self.shoulder_pan_offset_slider = QSlider(Qt.Horizontal)
        self.shoulder_pan_offset_slider.setMinimum(-180)
        self.shoulder_pan_offset_slider.setMaximum(180)
        self.shoulder_pan_offset_slider.setValue(int(self.shoulder_pan_offset_deg))
        self.shoulder_pan_offset_slider.setSingleStep(5)
        self.shoulder_pan_offset_slider.setPageStep(15)
        self.shoulder_pan_offset_slider.setTickInterval(30)
        self.shoulder_pan_offset_slider.setTickPosition(QSlider.TicksBelow)
        self.shoulder_pan_offset_slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.shoulder_pan_offset_slider.valueChanged.connect(self._on_shoulder_pan_offset_changed)
        row.addWidget(self.shoulder_pan_offset_slider, stretch=1)

        self.shoulder_pan_offset_value = QLabel(f"{int(self.shoulder_pan_offset_deg)}°")
        self.shoulder_pan_offset_value.setMinimumWidth(90)
        self.shoulder_pan_offset_value.setStyleSheet("font-size: 12pt;")
        self.shoulder_pan_offset_value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        row.addWidget(self.shoulder_pan_offset_value)

        parent_layout.addLayout(row)

    def _on_shoulder_pan_offset_changed(self, value):
        self.shoulder_pan_offset_deg = float(value)
        if hasattr(self, "shoulder_pan_offset_value"):
            self.shoulder_pan_offset_value.setText(f"{int(value)}°")

    def update_joint_positions(self, joint_values):
        """Update slider positions to match joint values from the scene.

        Args:
            joint_values: Dictionary mapping joint names to angle values in radians
        """
        for joint_name in self.LEFT_ARM_JOINTS:
            if joint_name in joint_values:
                angle_rad = joint_values[joint_name]
                angle_deg = math.degrees(angle_rad)

                # Update slider (block signals to avoid triggering events)
                slider = self.sliders[joint_name]
                slider.blockSignals(True)
                slider.setValue(int(self._wrap_angle_rad(angle_rad) * self.SLIDER_SCALE))
                slider.blockSignals(False)

                # Update value label in degrees
                self.value_labels[joint_name].setText(f"{angle_deg:.1f}°")
        if self._tracking_enabled:
            self._sync_hardware_from_sim_joint_values(joint_values)


    def _on_connect_clicked(self):
        """Handle Connect/Disconnect button click."""
        if not RTDE_AVAILABLE:
            error_msg = "RTDE libraries not available. Cannot connect to hardware."
            print(error_msg)
            self.status_indicator.setText("● RTDE Not Available")
            self.status_indicator.setStyleSheet("color: #FF0000; font-size: 20pt;")
            return

        if not self.hardware_connected:
            # Connect to robot
            try:
                print("Connecting to robot at 192.168.2.2...")
                self.status_indicator.setText("● Connecting...")
                self.status_indicator.setStyleSheet("color: #FF9800; font-size: 20pt;")

                # Create and connect URReader (read-only, no gripper)
                self.ur_reader = URReader(robot_ip="192.168.2.2")
                self.ur_reader.connect()

                self.hardware_connected = True
                self.connect_button.setText("Disconnect")
                self.read_button.setEnabled(True)
                self.control_arm_button.setEnabled(True)  # Enable Control Arm button
                self.status_indicator.setText("● Connected")
                self.status_indicator.setStyleSheet("color: #4CAF50; font-size: 20pt;")

                # Start automatic updates
                self.hardware_timer.start()
                print("Connected successfully! Auto-update started.")
            except Exception as e:
                error_str = str(e)
                print(f"\n=== Connection Error ===")
                print(f"Error: {error_str}")
                print(f"\nTroubleshooting:")
                print(f"1. Check robot IP is correct (currently: 192.168.2.2)")
                print(f"2. Ping the robot: ping 192.168.2.2")
                print(f"3. Ensure robot is powered on and network cable connected")
                print(f"4. Check if RTDE is enabled on robot (URCaps settings)")
                print(f"5. Verify firewall is not blocking connection")
                print(f"=======================\n")

                # Show helpful error message
                if "111" in error_str or "Connection refused" in error_str:
                    self.status_indicator.setText("● Connection Refused (Check IP/Network)")
                elif "113" in error_str or "No route to host" in error_str:
                    self.status_indicator.setText("● No Route to Host (Check Network)")
                elif "timeout" in error_str.lower():
                    self.status_indicator.setText("● Connection Timeout (Check Robot)")
                else:
                    self.status_indicator.setText(f"● Error: {error_str[:40]}")
                self.status_indicator.setStyleSheet("color: #FF0000; font-size: 18pt;")
        else:
            # Disconnect
            try:
                # Stop automatic updates
                self.hardware_timer.stop()

                # Turn off control mode if active
                if self.control_arm_enabled:
                    self.control_arm_button.setChecked(False)  # Triggers _on_control_arm_toggled

                if self.ur_reader:
                    self.ur_reader.close()
                    self.ur_reader = None
                self.hardware_connected = False
                self.connect_button.setText("Connect")
                self.read_button.setEnabled(False)
                self.control_arm_button.setEnabled(False)  # Disable Control Arm button
                self.status_indicator.setText("● Disconnected")
                self.status_indicator.setStyleSheet("color: #f44336; font-size: 20pt;")
                print("Disconnected successfully!")
            except Exception as e:
                print(f"Disconnect error: {e}")

    def _update_hardware_sliders(self, update_3d=False):
        """Update hardware sliders with current joint positions (called by timer).

        Args:
            update_3d: If True, also update the 3D simulation to match hardware pose
        """
        if self.hardware_connected and self.ur_reader:
            try:
                # Read joint positions from hardware
                joint_positions = self.ur_reader.get_current_joint_positions()

                # Update hardware sliders
                for i, joint_name in enumerate(self.HARDWARE_JOINTS):
                    angle_rad = joint_positions[i]
                    angle_deg = math.degrees(angle_rad)

                    slider = self.hardware_sliders[joint_name]
                    slider.blockSignals(True)
                    slider.setValue(int(self._wrap_angle_rad(angle_rad) * self.SLIDER_SCALE))
                    slider.blockSignals(False)

                    self.hardware_value_labels[joint_name].setText(f"{angle_deg:.1f}°")

                # Update 3D simulation if requested or tracking is OFF
                if update_3d or not self._tracking_enabled:
                    self._sync_sim_from_hardware_positions(joint_positions)

            except Exception as e:
                print(f"Hardware read failed: {e}")
                self.status_indicator.setText(f"● Read Error")
                self.status_indicator.setStyleSheet("color: #FF0000; font-size: 20pt;")
                # Stop timer on error
                self.hardware_timer.stop()

    def _on_read_clicked(self):
        """Handle Read Joint Data button click (manual update with 3D visualization)."""
        # Turn off hand tracking to allow manual control
        if self._tracking_enabled:
            self.toggle_button.setChecked(False)  # This triggers _on_toggle
            print("Hand tracking disabled - switching to hardware pose")

        self._update_hardware_sliders(update_3d=True)
        print(f"Manual read triggered - updating 3D visualization")

    def _on_hardware_slider_changed(self):
        """Handle hardware slider value changes - update target positions."""
        if self.control_arm_enabled:
            for i, joint_name in enumerate(self.HARDWARE_JOINTS):
                slider = self.hardware_sliders[joint_name]
                angle_rad = slider.value() / self.SLIDER_SCALE
                self.target_hw_positions[i] = angle_rad
        if not self._tracking_enabled:
            self._sync_sim_from_hardware_sliders()

    def _on_control_arm_toggled(self, checked):
        """Handle Control Arm button toggle."""
        if checked:
            # Turn ON control mode
            try:
                print("\n=== Enabling Control Arm ===")
                # Create URController (without gripper)
                self.ur_controller = URController(
                    robot_ip="192.168.2.2",
                    initial_joint_positions=np.zeros(6),
                    with_gripper=False
                )
                # Don't call reset() - stay at current position

                # Read current positions
                self.current_hw_positions = self.ur_controller.get_current_joint_positions()
                self.target_hw_positions = self.current_hw_positions.copy()
                self.smoothed_target_hw_positions = self.current_hw_positions.copy()

                # Update sliders to current positions
                for i, joint_name in enumerate(self.HARDWARE_JOINTS):
                    angle_rad = self.current_hw_positions[i]
                    angle_deg = math.degrees(angle_rad)
                    slider = self.hardware_sliders[joint_name]
                    slider.blockSignals(True)
                    slider.setValue(int(self._wrap_angle_rad(angle_rad) * self.SLIDER_SCALE))
                    slider.blockSignals(False)
                    self.hardware_value_labels[joint_name].setText(f"{angle_deg:.1f}°")

                # Enable hardware sliders for user control
                for slider in self.hardware_sliders.values():
                    slider.setEnabled(True)

                # Start control timer
                self.control_timer.start()

                self.control_arm_enabled = True
                self.control_arm_button.setText("Control Arm: ON")
                print("Control Arm enabled - you can now use sliders to control the robot")

            except Exception as e:
                print(f"Failed to enable Control Arm: {e}")
                self.control_arm_button.setChecked(False)
                return
        else:
            # Turn OFF control mode
            print("\n=== Disabling Control Arm ===")
            # Stop control timer
            self.control_timer.stop()

            # Disable hardware sliders
            for slider in self.hardware_sliders.values():
                slider.setEnabled(not self._tracking_enabled)

            # Close controller
            if self.ur_controller:
                self.ur_controller.close()
                self.ur_controller = None

            self.control_arm_enabled = False
            self.control_arm_button.setText("Control Arm: OFF")
            print("Control Arm disabled")

    def _send_hardware_command(self):
        """Send lerped command to hardware robot (called by control timer)."""
        if not self.control_arm_enabled or not self.ur_controller:
            return

        try:
            # Read current positions
            self.current_hw_positions = self.ur_controller.get_current_joint_positions()

            # Smooth target and apply speed limit per joint
            self.smoothed_target_hw_positions = (
                (1.0 - self.target_filter_alpha) * self.smoothed_target_hw_positions
                + self.target_filter_alpha * self.target_hw_positions
            )
            delta = self.smoothed_target_hw_positions - self.current_hw_positions
            # Wrap shoulder pan delta to avoid 180/-180 jumps
            delta[0] = self._wrap_angle_rad(delta[0])
            max_step = self.max_joint_speed_rad_s * (self.control_timer.interval() / 1000.0)
            step = np.clip(delta, -max_step, max_step)
            lerped_positions = self.current_hw_positions + step

            # Log command
            print(f"\n[HW CMD] Current: {np.degrees(self.current_hw_positions).round(1)}°")
            print(f"[HW CMD] Target:  {np.degrees(self.smoothed_target_hw_positions).round(1)}°")
            print(f"[HW CMD] Sending: {np.degrees(lerped_positions).round(1)}°")
            print(f"[HW CMD] Delta:   {np.degrees(lerped_positions - self.current_hw_positions).round(2)}°")

            # Send command to robot
            self.ur_controller.servo_joints(lerped_positions)

        except Exception as e:
            print(f"[HW CMD ERROR] Failed to send command: {e}")
            # Don't disable control mode on error, just skip this command

    def cleanup(self):
        """Cleanup hardware connection when closing."""
        # Stop timers
        self.hardware_timer.stop()
        self.control_timer.stop()

        # Close controller if active
        if self.ur_controller:
            try:
                self.ur_controller.close()
                print("Hardware controller closed.")
            except Exception as e:
                print(f"Controller cleanup error: {e}")

        if self.ur_reader:
            try:
                self.ur_reader.close()
                print("Hardware connection closed.")
            except Exception as e:
                print(f"Cleanup error: {e}")

    def _sync_sim_from_hardware_positions(self, joint_positions):
        """Update 3D scene and sim sliders from hardware joint positions."""
        sim_joint_values = {}
        for i, hw_joint_name in enumerate(self.HARDWARE_JOINTS):
            sim_joint_name = f"left_{hw_joint_name}"
            angle_rad = joint_positions[i]
            if hw_joint_name == "shoulder_pan_joint":
                angle_rad = self._apply_shoulder_pan_offset(angle_rad, direction=-1)
            sim_joint_values[sim_joint_name] = angle_rad

        if self._slider_callback is not None:
            self._slider_callback(sim_joint_values)

        for sim_joint_name, angle_rad in sim_joint_values.items():
            if sim_joint_name in self.sliders:
                angle_deg = math.degrees(angle_rad)
                slider = self.sliders[sim_joint_name]
                slider.blockSignals(True)
                slider.setValue(int(self._wrap_angle_rad(angle_rad) * self.SLIDER_SCALE))
                slider.blockSignals(False)
                self.value_labels[sim_joint_name].setText(f"{angle_deg:.1f}°")

    def _sync_sim_from_hardware_sliders(self):
        """Update 3D scene from current hardware slider values."""
        joint_positions = np.zeros(len(self.HARDWARE_JOINTS))
        for i, joint_name in enumerate(self.HARDWARE_JOINTS):
            joint_positions[i] = self.hardware_sliders[joint_name].value() / self.SLIDER_SCALE
        self._sync_sim_from_hardware_positions(joint_positions)

    def _sync_hardware_from_sim_joint_values(self, joint_values):
        """Update hardware sliders from sim joint values (tracking ON)."""
        for i, hw_joint_name in enumerate(self.HARDWARE_JOINTS):
            sim_joint_name = f"left_{hw_joint_name}"
            if sim_joint_name not in joint_values:
                continue
            angle_rad = joint_values[sim_joint_name]
            if hw_joint_name == "shoulder_pan_joint":
                angle_rad = self._apply_shoulder_pan_offset(angle_rad, direction=1)
            angle_deg = math.degrees(angle_rad)
            slider = self.hardware_sliders[hw_joint_name]
            slider.blockSignals(True)
            slider.setValue(int(self._wrap_angle_rad(angle_rad) * self.SLIDER_SCALE))
            slider.blockSignals(False)
            self.hardware_value_labels[hw_joint_name].setText(f"{angle_deg:.1f}°")
            if self.control_arm_enabled:
                self.target_hw_positions[i] = angle_rad
