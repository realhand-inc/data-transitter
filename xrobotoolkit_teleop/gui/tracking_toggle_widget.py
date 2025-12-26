"""Simple toggle widget for enabling/disabling hand tracking control."""

from PyQt5.QtWidgets import QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QSlider, QLabel
from PyQt5.QtCore import Qt
import math
import numpy as np

# Try to import URController for hardware connection
try:
    from xrobotoolkit_teleop.hardware.interface.universal_robots import URController
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

    def __init__(self, parent=None):
        super().__init__(parent)
        self._tracking_enabled = True
        self.sliders = {}
        self.value_labels = {}
        self._slider_callback = None

        # Hardware connection variables
        self.hardware_sliders = {}
        self.hardware_value_labels = {}
        self.hardware_connected = False
        self.ur_controller = None

        self._init_ui()

    def _init_ui(self):
        """Initialize the user interface."""
        layout = QVBoxLayout()

        # Create toggle button
        self.toggle_button = QPushButton("Hand Tracking: ON")
        self.toggle_button.setCheckable(True)
        self.toggle_button.setChecked(True)  # Start in ON state
        self.toggle_button.setMinimumSize(800, 200)
        self.toggle_button.setStyleSheet(
            """
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-size: 28pt;
                font-weight: bold;
                border-radius: 10px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            """
        )

        # Connect signal
        self.toggle_button.toggled.connect(self._on_toggle)

        layout.addWidget(self.toggle_button, alignment=Qt.AlignCenter)

        # Add spacing
        layout.addSpacing(40)

        # Create 6 sliders for left arm joints
        for joint_name in self.LEFT_ARM_JOINTS:
            joint_layout = self._create_joint_slider(joint_name)
            layout.addLayout(joint_layout)
            layout.addSpacing(20)

        # Add hardware section
        layout.addSpacing(60)

        # Hardware section title
        hw_title = QLabel("HARDWARE JOINT DATA (Left Arm - RTDE)")
        hw_title.setStyleSheet("font-size: 28pt; font-weight: bold; color: #2196F3;")
        layout.addWidget(hw_title, alignment=Qt.AlignCenter)
        layout.addSpacing(20)

        # Status and buttons row
        status_button_layout = QHBoxLayout()

        # Status indicator
        self.status_label = QLabel("Status:")
        self.status_label.setStyleSheet("font-size: 20pt; font-weight: bold;")
        status_button_layout.addWidget(self.status_label)

        self.status_indicator = QLabel("● Disconnected")
        self.status_indicator.setStyleSheet("color: #f44336; font-size: 20pt; font-weight: bold;")
        status_button_layout.addWidget(self.status_indicator)

        status_button_layout.addSpacing(40)

        # Connect button
        self.connect_button = QPushButton("Connect")
        self.connect_button.setMinimumSize(200, 80)
        self.connect_button.setStyleSheet(
            """
            QPushButton {
                background-color: #2196F3;
                color: white;
                font-size: 20pt;
                font-weight: bold;
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
        self.read_button.setMinimumSize(300, 80)
        self.read_button.setEnabled(False)  # Disabled until connected
        self.read_button.setStyleSheet(
            """
            QPushButton {
                background-color: #FF9800;
                color: white;
                font-size: 20pt;
                font-weight: bold;
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

        status_button_layout.addStretch()
        layout.addLayout(status_button_layout)
        layout.addSpacing(30)

        # Create 6 hardware sliders
        for joint_name in self.HARDWARE_JOINTS:
            hw_slider_layout = self._create_hardware_slider(joint_name)
            layout.addLayout(hw_slider_layout)
            layout.addSpacing(20)

        layout.addStretch()
        self.setLayout(layout)

        # Set window properties
        self.setWindowTitle("Hand Tracking Control")
        self.setFixedSize(1600, 1600)

    def _create_joint_slider(self, joint_name):
        """Create a slider row for a single joint."""
        row_layout = QHBoxLayout()

        # Joint name label
        name_label = QLabel(joint_name.replace("left_", "").replace("_", " ").title())
        name_label.setMinimumWidth(400)
        name_label.setStyleSheet("font-size: 24pt; font-weight: bold;")
        row_layout.addWidget(name_label)

        # Slider (range -π to π, scaled by 100)
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(int(-math.pi * self.SLIDER_SCALE))
        slider.setMaximum(int(math.pi * self.SLIDER_SCALE))
        slider.setValue(0)
        slider.setMinimumWidth(800)
        slider.setStyleSheet(
            """
            QSlider::groove:horizontal {
                height: 20px;
                background: #ddd;
                border-radius: 10px;
            }
            QSlider::handle:horizontal {
                background: #4CAF50;
                width: 40px;
                margin: -10px 0;
                border-radius: 20px;
            }
            """
        )
        slider.setEnabled(False)  # Start disabled (tracking ON by default)
        slider.valueChanged.connect(self._on_slider_changed)
        self.sliders[joint_name] = slider
        row_layout.addWidget(slider)

        # Value label
        value_label = QLabel("0.00")
        value_label.setMinimumWidth(200)
        value_label.setStyleSheet("font-size: 24pt;")
        value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.value_labels[joint_name] = value_label
        row_layout.addWidget(value_label)

        return row_layout

    def _on_toggle(self, checked):
        """Handle toggle button state changes."""
        self._tracking_enabled = checked

        # Enable/disable sliders based on tracking state
        for slider in self.sliders.values():
            slider.setEnabled(not checked)  # Enabled when tracking OFF

        if checked:
            self.toggle_button.setText("Hand Tracking: ON")
            self.toggle_button.setStyleSheet(
                """
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    font-size: 28pt;
                    font-weight: bold;
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
                    font-size: 28pt;
                    font-weight: bold;
                    border-radius: 10px;
                }
                QPushButton:hover {
                    background-color: #da190b;
                }
                """
            )

    def is_tracking_enabled(self):
        """Return True if hand tracking is enabled."""
        return self._tracking_enabled

    def set_slider_callback(self, callback):
        """Set callback function to be called when sliders change.

        Args:
            callback: Function that takes a dict of {joint_name: angle_rad}
        """
        self._slider_callback = callback

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

    def update_joint_positions(self, joint_values):
        """Update slider positions to match joint values from the scene.

        Args:
            joint_values: Dictionary mapping joint names to angle values in radians
        """
        for joint_name in self.LEFT_ARM_JOINTS:
            if joint_name in joint_values:
                angle_rad = joint_values[joint_name]

                # Update slider (block signals to avoid triggering events)
                slider = self.sliders[joint_name]
                slider.blockSignals(True)
                slider.setValue(int(angle_rad * self.SLIDER_SCALE))
                slider.blockSignals(False)

                # Update value label
                self.value_labels[joint_name].setText(f"{angle_rad:.2f}")

    def _create_hardware_slider(self, joint_name):
        """Create a slider row for a hardware joint."""
        row_layout = QHBoxLayout()

        # Joint name label
        display_name = joint_name.replace("_", " ").title()
        name_label = QLabel(display_name)
        name_label.setMinimumWidth(400)
        name_label.setStyleSheet("font-size: 24pt; font-weight: bold;")
        row_layout.addWidget(name_label)

        # Slider (range -π to π, scaled by 100)
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(int(-math.pi * self.SLIDER_SCALE))
        slider.setMaximum(int(math.pi * self.SLIDER_SCALE))
        slider.setValue(0)
        slider.setMinimumWidth(800)
        slider.setStyleSheet(
            """
            QSlider::groove:horizontal {
                height: 20px;
                background: #ddd;
                border-radius: 10px;
            }
            QSlider::handle:horizontal {
                background: #FF9800;
                width: 40px;
                margin: -10px 0;
                border-radius: 20px;
            }
            """
        )
        slider.setEnabled(False)  # Hardware sliders are always read-only
        self.hardware_sliders[joint_name] = slider
        row_layout.addWidget(slider)

        # Value label
        value_label = QLabel("0.00")
        value_label.setMinimumWidth(200)
        value_label.setStyleSheet("font-size: 24pt;")
        value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.hardware_value_labels[joint_name] = value_label
        row_layout.addWidget(value_label)

        return row_layout

    def _on_connect_clicked(self):
        """Handle Connect/Disconnect button click."""
        if not RTDE_AVAILABLE:
            error_msg = "RTDE libraries not available. Cannot connect to hardware."
            print(error_msg)
            self.status_indicator.setText("● RTDE Not Available")
            self.status_indicator.setStyleSheet("color: #FF0000; font-size: 20pt; font-weight: bold;")
            return

        if not self.hardware_connected:
            # Connect to robot
            try:
                print("Connecting to robot at 192.168.2.2...")
                self.status_indicator.setText("● Connecting...")
                self.status_indicator.setStyleSheet("color: #FF9800; font-size: 20pt; font-weight: bold;")

                self.ur_controller = URController(
                    robot_ip="192.168.2.2",
                    initial_joint_positions=np.zeros(6),  # Don't move on connect
                )
                # Don't call reset() - we don't want to move the robot
                self.hardware_connected = True
                self.connect_button.setText("Disconnect")
                self.read_button.setEnabled(True)
                self.status_indicator.setText("● Connected")
                self.status_indicator.setStyleSheet("color: #4CAF50; font-size: 20pt; font-weight: bold;")
                print("Connected successfully!")
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
                self.status_indicator.setStyleSheet("color: #FF0000; font-size: 18pt; font-weight: bold;")
        else:
            # Disconnect
            try:
                if self.ur_controller:
                    self.ur_controller.close()
                    self.ur_controller = None
                self.hardware_connected = False
                self.connect_button.setText("Connect")
                self.read_button.setEnabled(False)
                self.status_indicator.setText("● Disconnected")
                self.status_indicator.setStyleSheet("color: #f44336; font-size: 20pt; font-weight: bold;")
                print("Disconnected successfully!")
            except Exception as e:
                print(f"Disconnect error: {e}")

    def _on_read_clicked(self):
        """Handle Read Joint Data button click."""
        if self.hardware_connected and self.ur_controller:
            try:
                # Read joint positions from hardware
                joint_positions = self.ur_controller.get_current_joint_positions()
                print(f"Read joint positions: {joint_positions}")

                # Update hardware sliders
                for i, joint_name in enumerate(self.HARDWARE_JOINTS):
                    angle_rad = joint_positions[i]

                    slider = self.hardware_sliders[joint_name]
                    slider.blockSignals(True)
                    slider.setValue(int(angle_rad * self.SLIDER_SCALE))
                    slider.blockSignals(False)

                    self.hardware_value_labels[joint_name].setText(f"{angle_rad:.2f}")
            except Exception as e:
                print(f"Read failed: {e}")
                self.status_indicator.setText(f"● Read Error")
                self.status_indicator.setStyleSheet("color: #FF0000; font-size: 20pt; font-weight: bold;")

    def cleanup(self):
        """Cleanup hardware connection when closing."""
        if self.ur_controller:
            try:
                self.ur_controller.close()
                print("Hardware connection closed.")
            except Exception as e:
                print(f"Cleanup error: {e}")
