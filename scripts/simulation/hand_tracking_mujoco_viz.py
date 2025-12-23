import os
import tkinter as tk

import tyro
import numpy as np
import mujoco
from mujoco import viewer as mj_viewer

from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import (
    MujocoTeleopController,
)
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH
from xrobotoolkit_teleop.hardware.interface.universal_robots import URController


class HardwareMujocoTeleopController(MujocoTeleopController):
    def __init__(self, *args, hardware_ip="192.168.2.2", **kwargs):
        self.hardware_ip = hardware_ip
        self.hw_controller = None
        self.left_arm_joint_indices = None
        self.hardware_enabled = False

        # Setup GUI
        self.gui_root = tk.Tk()
        self.gui_root.title("Control Panel")
        self.gui_root.geometry("300x150")
        
        self.start_btn = tk.Button(self.gui_root, text="Start Control", command=self.start_control, bg="green", fg="white", font=("Arial", 12))
        self.start_btn.pack(pady=10, fill="x", padx=20)
        
        self.stop_btn = tk.Button(self.gui_root, text="EMERGENCY STOP", command=self.emergency_stop, bg="red", fg="white", font=("Arial", 12, "bold"))
        self.stop_btn.pack(pady=10, fill="x", padx=20)
        
        self.status_label = tk.Label(self.gui_root, text="Status: Hardware Disabled", fg="red")
        self.status_label.pack(pady=5)

        # Initialize parent
        super().__init__(*args, **kwargs)

    def start_control(self):
        self.hardware_enabled = True
        self.status_label.config(text="Status: Hardware ENABLED", fg="green")
        print("Hardware control STARTED.")

    def emergency_stop(self):
        self.hardware_enabled = False
        self.status_label.config(text="Status: Hardware STOPPED", fg="red")
        print("Hardware control STOPPED (Emergency).")
        if self.hw_controller:
            try:
                # Stop the robot immediately
                self.hw_controller.rtde_c.servoStop()
            except Exception as e:
                print(f"Error stopping hardware: {e}")

    def _placo_setup(self):
        super()._placo_setup()
        
        # Identify left arm joint indices in Placo
        left_joint_names = [
            "left_shoulder_pan_joint",
            "left_shoulder_lift_joint",
            "left_elbow_joint",
            "left_wrist_1_joint",
            "left_wrist_2_joint",
            "left_wrist_3_joint",
        ]
        
        self.left_arm_joint_indices = []
        for name in left_joint_names:
            try:
                # In Placo, for revolute joints, q_offset is the index in q
                idx = self.placo_robot.get_joint_q_offset(name)
                self.left_arm_joint_indices.append(idx)
            except Exception as e:
                print(f"Warning: Could not find joint {name} in Placo model: {e}")

        print(f"Connecting to UR hardware at {self.hardware_ip}...")
        try:
            # Initialize with current positions from IK
            q_init = np.array([self.placo_robot.state.q[i] for i in self.left_arm_joint_indices])
            self.hw_controller = URController(
                robot_ip=self.hardware_ip,
                initial_joint_positions=q_init
            )
            print("Successfully connected to UR hardware. Moving to initial position...")
            self.hw_controller.reset()
            print("Reached initial position.")
        except Exception as e:
            print(f"Failed to connect to UR hardware: {e}")
            self.hw_controller = None

    def _robot_setup(self):
        super()._robot_setup()

    def _send_command(self):
        # Update MuJoCo simulation
        super()._send_command()
        
        # Send to hardware if connected and enabled
        if self.hw_controller and self.left_arm_joint_indices and self.hardware_enabled:
            left_q = np.array([self.placo_robot.state.q[i] for i in self.left_arm_joint_indices])
            self.hw_controller.servo_joints(left_q)

    def run(self):
        # Override run to include gui update
        with mj_viewer.launch_passive(self.mj_model, self.mj_data) as viewer:
            # Set up viewer camera
            viewer.cam.azimuth = 0
            viewer.cam.elevation = -50
            viewer.cam.distance = 2.0
            viewer.cam.lookat = [0.2, 0, 0]

            while not self._stop_event.is_set():
                try:
                    # Update GUI
                    self.gui_root.update()
                    
                    self._update_robot_state()
                    self._update_ik()
                    self._update_gripper_target()
                    self._update_mocap_target()
                    self._send_command()

                    # Step simulation and update viewer
                    mujoco.mj_step(self.mj_model, self.mj_data)
                    viewer.sync()
                except KeyboardInterrupt:
                    print("\nTeleoperation stopped.")
                    self._stop_event.set()
                except tk.TclError:
                    # Window closed
                    print("\nGUI Window closed. Stopping.")
                    self._stop_event.set()

        # Cleanup
        if self.hw_controller:
            print("Closing UR hardware connection...")
            self.hw_controller.close()
        try:
            self.gui_root.destroy()
        except:
            pass


def main(
    xml_path: str = os.path.join(ASSET_PATH, "universal_robots_ur5e/scene_dual_arm.xml"),
    robot_urdf_path: str = os.path.join(ASSET_PATH, "universal_robots_ur5e/dual_ur5e.urdf"),
    scale_factor: float = 1.5,
    visualize_placo: bool = True,
    hardware_ip: str = "192.168.2.2",
):
    """
    Main function to run dual UR5e hand tracking visualization in MuJoCo with hardware control.

    Uses hand wrist tracking (no VR controllers needed).
    Continuous control - arms follow hands automatically.
    """
    config = {
        "right_hand": {
            "link_name": "right_tool0",
            "pose_source": "right_hand_wrist",  # Hand tracking instead of controller
            "vis_target": "right_target",
            # No control_trigger needed - continuous hand tracking control
        },
        "left_hand": {
            "link_name": "left_tool0",
            "pose_source": "left_hand_wrist",  # Hand tracking instead of controller
            "vis_target": "left_target",
            # No control_trigger needed - continuous hand tracking control
        },
    }

    # Create and initialize the teleoperation controller
    controller = HardwareMujocoTeleopController(
        xml_path=xml_path,
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
        hardware_ip=hardware_ip,
    )

    # additional constraints hardcoded here for now
    joints_task = controller.solver.add_joints_task()
    joints_task.set_joints({joint: 0.0 for joint in controller.placo_robot.joint_names()})
    joints_task.configure("joints_regularization", "soft", 1e-4)

    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
