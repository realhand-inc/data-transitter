import os
import sys

import tyro
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer
from mujoco import viewer as mj_viewer

from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import MujocoTeleopController
from xrobotoolkit_teleop.gui import TrackingToggleWidget
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH


class DualUR5eToggleApp(QMainWindow):
    """PyQt5 application with toggle button for hand tracking control."""

    def __init__(self, controller, viewer):
        super().__init__()
        self.controller = controller
        self.viewer = viewer

        # Create and set the toggle widget as central widget
        self.widget = TrackingToggleWidget()
        self.setCentralWidget(self.widget)

        # Connect slider callback for manual control
        self.widget.set_slider_callback(self.on_manual_control)
        self.widget.set_rotation_callback(self.on_rotation_change)

        # Set up QTimer to drive simulation loop at 100 Hz (10ms interval)
        self.timer = QTimer()
        self.timer.timeout.connect(self.simulation_step)
        self.timer.start(10)  # 10ms = 100 Hz

        # Set window properties
        self.setWindowTitle("Dual UR5e Teleoperation")
        self.setMinimumSize(1500, 1800)

        # Setup viewer camera (do this once)
        self.viewer.cam.azimuth = 0
        self.viewer.cam.elevation = -50
        self.viewer.cam.distance = 2.0
        self.viewer.cam.lookat = [0.2, 0, 0]

    def on_manual_control(self, joint_values):
        """Called when sliders are moved in manual mode.

        Args:
            joint_values: Dictionary of {joint_name: angle_rad} from sliders
        """
        if not self.widget.is_tracking_enabled():
            self.controller.update3DJoint(joint_values)

    def simulation_step(self):
        """Execute one simulation step, called by QTimer."""
        try:
            # Check if hand tracking is enabled
            skip_ik = not self.widget.is_tracking_enabled()

            # Execute one simulation step
            self.controller.step_once(self.viewer, skip_ik=skip_ik)

            # Get current joint positions from the scene
            joint_positions = self.controller.get_joint_positions()

            # Update GUI sliders with joint data
            self.widget.update_joint_positions(joint_positions)
            # Update XR ZMQ status badge
            self.widget.update_xr_status(self.controller.xr_client)

        except KeyboardInterrupt:
            print("\nTeleoperation stopped.")
            self.timer.stop()
            self.close()

    def on_rotation_change(self, rot_offset_deg):
        """Update XR rotation offsets used for delta rotations."""
        self.controller.set_xr_rot_offset_deg(rot_offset_deg)

    def closeEvent(self, event):
        """Handle window close event."""
        self.timer.stop()
        self.controller._stop_event.set()
        self.widget.cleanup()  # Cleanup hardware connection
        event.accept()


def main(
    xml_path: str = os.path.join(ASSET_PATH, "universal_robots_ur5e/scene_dual_arm.xml"),
    robot_urdf_path: str = os.path.join(ASSET_PATH, "universal_robots_ur5e/dual_ur5e.urdf"),
    scale_factor: float = 1,
    visualize_placo: bool = False,
    xr_zmq_endpoint: str | None = "tcp://127.0.0.1:5558",
):
    """
    Main function to run the dual UR5e teleoperation with PyQt5 GUI toggle button.
    """
    if not xr_zmq_endpoint:
        xr_zmq_endpoint = "tcp://127.0.0.1:5558"
    config = {
        "right_hand": {
            "link_name": "right_tool0",  # control the IK target
            "pose_source": "right_hand_wrist",  # position information
            # "control_trigger": "right_grip",
            "vis_target": "right_target",
        },
        "left_hand": {
            "link_name": "left_tool0",
            "pose_source": "right_hand_wrist",
            # "control_trigger": "left_grip",
            "vis_target": "left_target",
        },
    }

    # Create QApplication first (required for PyQt5)
    app = QApplication(sys.argv)

    # Create and initialize the teleoperation controller
    controller = MujocoTeleopController(
        xml_path=xml_path,
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
        xr_zmq_endpoint=xr_zmq_endpoint,
    )

    # Additional constraints hardcoded here for now
    joints_task = controller.solver.add_joints_task()
    joints_task.set_joints({joint: 0.0 for joint in controller.placo_robot.joint_names()})
    joints_task.configure("joints_regularization", "soft", 1e-4)

    # Launch MuJoCo viewer (passive mode)
    viewer = mj_viewer.launch_passive(controller.mj_model, controller.mj_data).__enter__()

    # Create and show GUI application
    gui_app = DualUR5eToggleApp(controller, viewer)
    gui_app.show()

    # Start Qt event loop
    sys.exit(app.exec_())


if __name__ == "__main__":
    tyro.cli(main)
