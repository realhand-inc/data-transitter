import os
import threading
import tkinter as tk

import tyro
import mujoco
import numpy as np
from mujoco import viewer as mj_viewer
from scipy.spatial.transform import Rotation

from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import (
    MujocoTeleopController,
)
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH


class RotatedHandTeleopController(MujocoTeleopController):
    """Custom controller that applies 90-degree z-axis rotation to hand tracking data."""

    def __init__(self, *args, z_rotation_deg=90.0, **kwargs):
        """
        Initialize controller with hand tracking rotation.

        Args:
            z_rotation_deg: Rotation angle around z-axis in degrees (default: 90.0)
        """
        super().__init__(*args, **kwargs)
        # Create rotation quaternion for z-axis rotation
        self.z_rotation = Rotation.from_euler('z', z_rotation_deg, degrees=True)
        self._rotated_poses = {}
        print(f"Hand tracking rotation: {z_rotation_deg}° around z-axis")

        # Wrap the XR client's get_pose_by_name to return rotated poses
        self._original_get_pose = self.xr_client.get_pose_by_name
        self.xr_client.get_pose_by_name = self._get_rotated_pose

    def _get_rotated_pose(self, pose_name):
        """Wrapper for XR client's get_pose_by_name that applies rotation."""
        # Get original pose
        original_pose = self._original_get_pose(pose_name)

        if original_pose is None:
            return None

        # Check if this is a hand wrist pose (needs rotation)
        if "hand_wrist" not in pose_name:
            return original_pose

        # Apply z-axis rotation to the orientation
        # Pose format: [x, y, z, qx, qy, qz, qw]
        position = original_pose[:3]
        orientation_quat = original_pose[3:]  # [qx, qy, qz, qw]

        # Convert to scipy Rotation format and apply z-rotation
        original_rot = Rotation.from_quat(orientation_quat)
        rotated_rot = self.z_rotation * original_rot

        # Get rotated quaternion
        rotated_quat = rotated_rot.as_quat()

        # Create rotated pose
        rotated_pose = np.concatenate([position, rotated_quat])

        return rotated_pose


def run_with_gui(controller, gui):
    """
    Run the controller with GUI updates integrated into the main loop.

    Args:
        controller: MujocoTeleopController instance
        gui: TeleopMonitorGUI instance
    """
    with mj_viewer.launch_passive(controller.mj_model, controller.mj_data) as viewer:
        # Set up viewer camera
        viewer.cam.azimuth = 0
        viewer.cam.elevation = -50
        viewer.cam.distance = 2.0
        viewer.cam.lookat = [0.2, 0, 0]

        while not controller._stop_event.is_set():
            try:
                # Only update control if hand tracking is enabled
                if gui.hand_tracking_enabled:
                    # Update robot state and IK (from controller's run loop)
                    controller._update_robot_state()
                    controller._update_ik()
                    controller._update_gripper_target()
                    controller._update_mocap_target()
                    controller._send_command()

                    # Step simulation and update MuJoCo viewer
                    mujoco.mj_step(controller.mj_model, controller.mj_data)

                # Send commands to real robot if robot control is enabled
                if gui.robot_control_enabled:
                    gui._send_joints_to_robot()

                # Always sync viewer (so camera controls still work)
                viewer.sync()

                # Update GUI
                gui.update_once()

            except KeyboardInterrupt:
                print("\nTeleoperation stopped.")
                controller._stop_event.set()
            except tk.TclError:
                # GUI window was closed
                print("\nGUI closed. Stopping teleoperation.")
                controller._stop_event.set()


def main(
    xml_path: str = os.path.join(ASSET_PATH, "universal_robots_ur5e/scene_dual_arm.xml"),
    robot_urdf_path: str = os.path.join(ASSET_PATH, "universal_robots_ur5e/dual_ur5e.urdf"),
    scale_factor: float = 1.5,
    visualize_placo: bool = False,
    show_gui: bool = True,
    right_robot_ip: str = "192.168.2.2",
    enable_rtde: bool = True,
    hand_z_rotation: float = 90.0,
):
    """
    Main function to run the dual UR5e teleoperation in MuJoCo (Right hand only).

    Args:
        xml_path: Path to MuJoCo scene XML file
        robot_urdf_path: Path to robot URDF file
        scale_factor: Scaling factor for XR input
        visualize_placo: Whether to show Placo meshcat visualization
        show_gui: Whether to show monitoring GUI (default: True)
        right_robot_ip: IP address of right UR5e robot for RTDE (default: 192.168.2.2)
        enable_rtde: Enable RTDE connection to real robot (default: True)
        hand_z_rotation: Z-axis rotation for hand tracking in degrees (default: 90.0)
    """
    config = {
        "right_hand": {
            "link_name": "right_tool0",
            "pose_source": "right_hand_wrist",
            "vis_target": "right_target",
        },
    }

    # Create and initialize the teleoperation controller with hand rotation
    controller = RotatedHandTeleopController(
        xml_path=xml_path,
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
        z_rotation_deg=hand_z_rotation,
    )

    # additional constraints hardcoded here for now
    joints_task = controller.solver.add_joints_task()
    joints_task.set_joints({joint: 0.0 for joint in controller.placo_robot.joint_names()})
    joints_task.configure("joints_regularization", "soft", 1e-4)

    # Setup RTDE connection if enabled
    rtde_receiver = None
    rtde_controller = None
    if enable_rtde:
        try:
            print(f"Connecting to right robot via RTDE at {right_robot_ip}...")
            import rtde_receive
            import rtde_control

            rtde_receiver = rtde_receive.RTDEReceiveInterface(right_robot_ip)
            rtde_controller = rtde_control.RTDEControlInterface(right_robot_ip)
            print(f"RTDE connected (receive + control): {right_robot_ip}")
        except Exception as e:
            print(f"Failed to connect to robot via RTDE: {e}")
            print("Continuing without RTDE connection...")
            rtde_receiver = None
            rtde_controller = None

    # Run with or without GUI
    if show_gui:
        try:
            print("Creating monitoring GUI...")
            # Import GUI (only if needed)
            from monitoring_gui import TeleopMonitorGUI

            # Create GUI but don't run mainloop yet
            print("Initializing GUI window...")
            gui = TeleopMonitorGUI(
                controller=controller,
                start_monitoring=False,
                rtde_receiver=rtde_receiver,
                rtde_controller=rtde_controller
            )
            print("GUI created successfully!")

            # Run controller with GUI updates integrated in the loop
            print("Starting controller with GUI integration...")
            run_with_gui(controller, gui)
        except Exception as e:
            print(f"Error with GUI: {e}")
            print("Falling back to running without GUI...")
            import traceback
            traceback.print_exc()
            controller.run()
    else:
        # Run controller normally (blocking)
        controller.run()


if __name__ == "__main__":
    tyro.cli(main)
