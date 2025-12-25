import os
import threading
import tkinter as tk

import tyro
import mujoco
from mujoco import viewer as mj_viewer
import numpy as np
import rtde_receive
import rtde_control

from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import (
    MujocoTeleopController,
)
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH


class HardwareController:
    """RTDE interface for reading and controlling UR5e"""

    def __init__(self, robot_ip: str, max_joint_speed: float = 0.1):
        self.robot_ip = robot_ip
        self.rtde_r = None  # Receive interface (read)
        self.rtde_c = None  # Control interface (write)
        self.connected = False
        self.control_enabled = False  # Safety flag

        # RTDE servoJ parameters (from universal_robots.py)
        self.servo_time = 0.017
        self.lookahead_time = 0.1
        self.servo_gain = 300.0
        self.max_velocity = 0.5
        self.max_acceleration = 1.0

        # Speed limiting (rad/s per joint)
        self.max_joint_speed = max_joint_speed  # Max change per servo cycle
        self.last_command = None  # Track last command for speed limiting

    def connect(self):
        """Establish RTDE connection (both receive and control)"""
        try:
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)
            self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
            self.connected = True
            print(f"[Hardware] Connected to UR5e at {self.robot_ip}")
            print(f"[Hardware] Control interface ready (control disabled by default)")
        except Exception as e:
            print(f"[Hardware] Connection failed: {e}")
            self.connected = False
            raise

    def get_actual_joint_positions(self) -> np.ndarray:
        """Read current joint angles from robot (in radians)"""
        if not self.connected:
            return np.zeros(6)
        try:
            return np.array(self.rtde_r.getActualQ())
        except Exception as e:
            print(f"[Hardware] Error reading joints: {e}")
            return np.zeros(6)

    def _apply_speed_limit(self, target_positions: np.ndarray) -> np.ndarray:
        """Apply speed limiting to joint commands"""
        if self.last_command is None:
            # First command - use current actual position as baseline
            self.last_command = self.get_actual_joint_positions()

        # Calculate maximum allowed change per servo cycle
        max_change = self.max_joint_speed * self.servo_time

        # Limit the change for each joint
        delta = target_positions - self.last_command
        delta = np.clip(delta, -max_change, max_change)
        limited_positions = self.last_command + delta

        # Update last command
        self.last_command = limited_positions.copy()

        return limited_positions

    def send_joint_command(self, joint_positions: np.ndarray):
        """Send servoJ command to robot (only if control enabled)"""
        if not self.connected or not self.control_enabled:
            return  # Silent return if control disabled

        try:
            # Apply speed limiting
            limited_positions = self._apply_speed_limit(joint_positions)

            t_start = self.rtde_c.initPeriod()
            self.rtde_c.servoJ(
                limited_positions.tolist(),
                self.max_velocity,
                self.max_acceleration,
                self.servo_time,
                self.lookahead_time,
                self.servo_gain,
            )
            self.rtde_c.waitPeriod(t_start)
        except Exception as e:
            print(f"[Hardware] Error sending command: {e}")
            self.control_enabled = False  # Disable on error

    def enable_control(self):
        """Enable hardware control (safety check)"""
        if self.connected:
            self.control_enabled = True
            print("[Hardware] ⚠️  CONTROL ENABLED - Commands will be sent to robot")

    def disable_control(self):
        """Disable hardware control"""
        self.control_enabled = False
        print("[Hardware] Control disabled - Read-only mode")

    def disconnect(self):
        """Close RTDE connection"""
        if self.rtde_c:
            self.rtde_c.servoStop()
            self.rtde_c.stopScript()
        if self.rtde_r:
            self.rtde_r.disconnect()
        self.connected = False
        self.control_enabled = False
        print("[Hardware] Disconnected from UR5e")


def run_with_gui(controller, gui, hardware_controller=None):
    """
    Run the controller with GUI updates integrated into the main loop.

    Args:
        controller: MujocoTeleopController instance
        gui: TeleopMonitorGUI instance
        hardware_controller: Optional HardwareController for bidirectional robot control
    """
    # Debug: Print Placo joint structure (first iteration only)
    debug_printed = False

    with mj_viewer.launch_passive(controller.mj_model, controller.mj_data) as viewer:
        # Set up viewer camera
        viewer.cam.azimuth = 0
        viewer.cam.elevation = -50
        viewer.cam.distance = 2.0
        viewer.cam.lookat = [0.2, 0, 0]

        while not controller._stop_event.is_set():
            try:
                # Debug print (once)
                if not debug_printed and hardware_controller:
                    print(f"\n[Debug] Placo robot state.q shape: {controller.placo_robot.state.q.shape}")
                    print(f"[Debug] MuJoCo qpos shape: {controller.mj_data.qpos.shape}")
                    debug_printed = True

                # STEP 1: Read actual position from hardware
                actual_q_right = None
                if hardware_controller and hardware_controller.connected:
                    actual_q_right = hardware_controller.get_actual_joint_positions()

                # STEP 2: Check if hand tracking is enabled
                hand_tracking_enabled = gui.get_hand_tracking_enabled()

                # STEP 3: Decide what to show in MuJoCo based on mode
                if hand_tracking_enabled:
                    # Hand tracking mode: Run IK, show result in MuJoCo (but don't send to hardware)
                    # Don't overwrite qpos with hardware - let IK drive visualization
                    pass
                else:
                    # Visualization mode: MuJoCo shows hardware state
                    if actual_q_right is not None:
                        controller.mj_data.qpos[6:12] = actual_q_right

                # STEP 4: Update robot state and run IK solver
                controller._update_robot_state()

                # Only run IK if hand tracking is enabled
                if hand_tracking_enabled:
                    controller._update_ik()
                    controller._update_gripper_target()
                    controller._update_mocap_target()

                # STEP 5: Get IK result (target joint positions for right arm)
                # Only if hand tracking is enabled
                if hand_tracking_enabled:
                    # Extract right arm target from MuJoCo qpos (after IK has updated it)
                    ik_q_right = controller.mj_data.qpos[6:12].copy()

                    # STEP 6: Send IK command to hardware (if control enabled)
                    if hardware_controller and hardware_controller.control_enabled:
                        hardware_controller.send_joint_command(ik_q_right)

                # STEP 6: Update MuJoCo visualization
                controller._send_command()
                mujoco.mj_step(controller.mj_model, controller.mj_data)
                viewer.sync()

                # STEP 7: Update GUI
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
    robot_ip: str = "192.168.2.2",  # UR5e robot IP address (empty = simulation only)
):
    """
    Main function to run the dual UR5e teleoperation in MuJoCo.

    Args:
        xml_path: Path to MuJoCo scene XML file
        robot_urdf_path: Path to robot URDF file
        scale_factor: Scaling factor for XR input
        visualize_placo: Whether to show Placo meshcat visualization
        show_gui: Whether to show monitoring GUI (default: True)
    """
    # Only control right arm (left arm disabled since not connected to hardware)
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

    # Create hardware controller if IP provided
    hardware_controller = None
    if robot_ip:
        try:
            print(f"\n[Hardware] Connecting to UR5e at {robot_ip}...")
            hardware_controller = HardwareController(robot_ip)
            hardware_controller.connect()
            print("[Hardware] Connection successful! Robot state will be synced to MuJoCo scene.\n")
        except Exception as e:
            print(f"[Hardware] Connection failed: {e}")
            print("[Hardware] Continuing in simulation-only mode...\n")
            hardware_controller = None

    # Run with or without GUI
    if show_gui:
        try:
            print("Creating monitoring GUI...")
            # Import GUI (only if needed)
            from monitoring_gui import TeleopMonitorGUI

            # Create GUI but don't run mainloop yet
            print("Initializing GUI window...")
            gui = TeleopMonitorGUI(controller=controller, start_monitoring=False)
            print("GUI created successfully!")

            # Pass hardware controller to GUI and update connection status
            if hardware_controller and hardware_controller.connected:
                gui.set_hardware_controller(hardware_controller)

            # Run controller with GUI updates integrated in the loop
            print("Starting controller with GUI integration...")
            try:
                run_with_gui(controller, gui, hardware_controller)
            finally:
                if hardware_controller:
                    hardware_controller.disconnect()
        except Exception as e:
            print(f"Error with GUI: {e}")
            print("Falling back to running without GUI...")
            import traceback
            traceback.print_exc()
            try:
                controller.run()
            finally:
                if hardware_controller:
                    hardware_controller.disconnect()
    else:
        # Run controller normally (blocking)
        try:
            controller.run()
        finally:
            if hardware_controller:
                hardware_controller.disconnect()


if __name__ == "__main__":
    tyro.cli(main)
