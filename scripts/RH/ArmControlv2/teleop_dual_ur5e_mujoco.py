import os
import threading
import tkinter as tk

import tyro
import mujoco
from mujoco import viewer as mj_viewer

from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import (
    MujocoTeleopController,
)
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH


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
    """
    config = {
        "right_hand": {
            "link_name": "right_tool0",
            "pose_source": "right_hand_wrist",
            "vis_target": "right_target",
        },
    }

    # Create and initialize the teleoperation controller
    controller = MujocoTeleopController(
        xml_path=xml_path,
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
    )

    # additional constraints hardcoded here for now
    joints_task = controller.solver.add_joints_task()
    joints_task.set_joints({joint: 0.0 for joint in controller.placo_robot.joint_names()})
    joints_task.configure("joints_regularization", "soft", 1e-4)

    # Setup RTDE connection if enabled
    rtde_receiver = None
    if enable_rtde:
        try:
            print(f"Connecting to right robot via RTDE at {right_robot_ip}...")
            import rtde_receive

            rtde_receiver = rtde_receive.RTDEReceiveInterface(right_robot_ip)
            print(f"RTDE connected: {right_robot_ip}")
        except Exception as e:
            print(f"Failed to connect to robot via RTDE: {e}")
            print("Continuing without RTDE connection...")
            rtde_receiver = None

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
                rtde_receiver=rtde_receiver
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
