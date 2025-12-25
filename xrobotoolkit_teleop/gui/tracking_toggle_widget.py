"""Simple toggle widget for enabling/disabling hand tracking control."""

from PyQt5.QtWidgets import QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QSlider, QLabel
from PyQt5.QtCore import Qt
import math


class TrackingToggleWidget(QWidget):
    """Widget with a toggle button and 6 sliders to display left arm joint angles."""

    # Left arm joint names for UR5e
    LEFT_ARM_JOINTS = [
        "left_shoulder_pan_joint",
        "left_shoulder_lift_joint",
        "left_elbow_joint",
        "left_wrist_1_joint",
        "left_wrist_2_joint",
        "left_wrist_3_joint",
    ]

    # Slider scale factor (1 radian = 100 slider units for precision)
    SLIDER_SCALE = 100

    def __init__(self, parent=None):
        super().__init__(parent)
        self._tracking_enabled = True
        self.sliders = {}
        self.value_labels = {}
        self._slider_callback = None
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
