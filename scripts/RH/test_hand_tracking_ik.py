import time
import os
import sys
import numpy as np
import json
import zmq
import meshcat.transformations as tf

# --- Setup Python Path for Imports ---
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, os.pardir, os.pardir))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from xrobotoolkit_teleop.common.xr_client import XrClient
from xrobotoolkit_teleop.utils.geometry import (
    R_HEADSET_TO_WORLD,
    apply_delta_pose,
    quat_diff_as_angle_axis,
)
import placo

# --- Constants & Configuration ---
# Path relative to project root
URDF_PATH = os.path.join(project_root, "assets/universal_robots_ur5e/dual_ur5e.urdf")
DEFAULT_SCALE_FACTOR = 1.2  # Slightly higher scale for hand tracking usually feels better

# Telemetry Configuration
TELEMETRY_IP = "127.0.0.1"
TELEMETRY_PORT = 5555

# Initial Joint Positions (Home) - Radians
LEFT_HOME_Q = np.deg2rad([165.26, -47.50, 118.93, -38.96, 87.51, 149.56])
RIGHT_HOME_Q = np.deg2rad([193.53, -164.17, -114.02, 58.01, 101.87, -138.40])

CONFIG = {
    "left": {
        "link": "left_tool0",
        "pose_source": "left_hand_wrist",
        "grip_source": "left_grip",
        "home_q": LEFT_HOME_Q,
        "offset_idx": slice(7, 13)  # Placo state indices for Left Arm
    },
    "right": {
        "link": "right_tool0",
        "pose_source": "right_hand_wrist",
        "grip_source": "right_grip",
        "home_q": RIGHT_HOME_Q,
        "offset_idx": slice(13, 19)  # Placo state indices for Right Arm
    }
}


class TeleopSimulator:
    def __init__(self):
        self.xr = XrClient()

        # Initialize ZMQ Publisher
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        # Bind to all interfaces to allow external access, or specific IP
        bind_addr = f"tcp://*:{TELEMETRY_PORT}"
        self.socket.bind(bind_addr)
        print(f"ZMQ Publisher bound to {bind_addr}")

        # Initialize Placo (Kinematics)
        print(f"Loading URDF: {URDF_PATH}")
        self.robot = placo.RobotWrapper(URDF_PATH)
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.dt = 0.01  # Set time step (100Hz) for regularization tasks
        self.solver.mask_fbase(True)  # Fix the base in space

        # Regularization (keeps robot close to comfortable pose)
        self.solver.add_kinetic_energy_regularization_task(1e-6)

        # Setup Initial State
        self.robot.state.q[CONFIG["left"]["offset_idx"]] = CONFIG["left"]["home_q"]
        self.robot.state.q[CONFIG["right"]["offset_idx"]] = CONFIG["right"]["home_q"]
        self.robot.update_kinematics()

        # State storage for the "Clutch" logic
        self.state = {
            "left": {"active": False, "init_ee": None, "init_ctrl_xyz": None, "init_ctrl_quat": None},
            "right": {"active": False, "init_ee": None, "init_ctrl_xyz": None, "init_ctrl_quat": None}
        }

        # Frame Tasks (The IK Targets)
        self.tasks = {}
        for side, cfg in CONFIG.items():
            # Create task at current position
            T_current = self.robot.get_T_world_frame(cfg["link"])
            task = self.solver.add_frame_task(cfg["link"], T_current)
            task.configure(f"{side}_eef", "soft", 1.0)
            self.tasks[side] = task

    def process_side(self, side):
        cfg = CONFIG[side]
        st = self.state[side]

        # 1. Get Inputs
        xr_pose = self.xr.get_pose_by_name(cfg["pose_source"])
        
        # If tracking lost, treat as inactive and clear references
        if xr_pose is None:
            st["active"] = False
            st["init_ee"] = None
            st["init_ctrl_xyz"] = None
            st["init_ctrl_quat"] = None
            return None, None

        # 2. Handle Continuous Tracking
        if not st["active"]:
            # FIRST VALID FRAME -> Store Initial References (Anchor)
            st["active"] = True
            
            # Store Robot's Current Physical EE Pose (Home)
            T_ee = self.robot.get_T_world_frame(cfg["link"])
            st["init_ee"] = (T_ee[:3, 3].copy(), tf.quaternion_from_matrix(T_ee))
            
            # Store Hand's Current Pose
            ctrl_xyz, ctrl_quat = self._transform_input(xr_pose)
            st["init_ctrl_xyz"] = ctrl_xyz
            st["init_ctrl_quat"] = ctrl_quat
            
        # 3. Calculate Target (Continuous)
        curr_xyz, curr_quat = self._transform_input(xr_pose)
        
        # Calculate Delta (How much hand moved since first frame)
        delta_xyz = (curr_xyz - st["init_ctrl_xyz"]) * DEFAULT_SCALE_FACTOR
        delta_rot = quat_diff_as_angle_axis(st["init_ctrl_quat"], curr_quat)
        
        # Apply Delta to Robot's Initial Home Pose
        init_robot_xyz, init_robot_quat = st["init_ee"]
        target_xyz, target_quat = apply_delta_pose(
            init_robot_xyz, init_robot_quat, delta_xyz, delta_rot
        )
        
        # Update IK Task
        T_target = tf.quaternion_matrix(target_quat)
        T_target[:3, 3] = target_xyz
        self.tasks[side].T_world_frame = T_target

        return xr_pose, self.robot.state.q[cfg["offset_idx"]]

    def _transform_input(self, xr_pose):
        """Standard coordinate transformation from HMD to World."""
        # Unpack [x, y, z, qx, qy, qz, qw]
        pos = np.array(xr_pose[:3])
        quat = np.array([xr_pose[6], xr_pose[3], xr_pose[4], xr_pose[5]])  # w, x, y, z for transform

        # Rotate position by Headset mounting angle
        pos_world = R_HEADSET_TO_WORLD @ pos

        # Rotate orientation
        R_transform = np.eye(4)
        R_transform[:3, :3] = R_HEADSET_TO_WORLD
        R_quat_mount = tf.quaternion_from_matrix(R_transform)

        # q_new = q_mount * q_input * q_mount_conj
        quat_world = tf.quaternion_multiply(
            tf.quaternion_multiply(R_quat_mount, quat),
            tf.quaternion_conjugate(R_quat_mount)
        )
        return pos_world, quat_world

    def step(self):
        # Update inputs
        l_in, l_out = self.process_side("left")
        r_in, r_out = self.process_side("right")

        # Solve IK
        self.solver.solve(True)

        # Broadcast Telemetry
        try:
            payload = {
                "timestamp": time.time(),
                "left": {
                    "joints": l_out.tolist() if l_out is not None else [],
                    "gripper": 0.0  # Placeholder
                },
                "right": {
                    "joints": r_out.tolist() if r_out is not None else [],
                    "gripper": 0.0  # Placeholder
                }
            }
            self.socket.send_json(payload)
        except Exception as e:
            pass  # Don't crash if network fails

        return (l_in, l_out), (r_in, r_out)

    def close(self):
        print("Closing ZMQ socket...")
        self.socket.close()
        self.context.term()


# --- UI Helpers ---
def clear_screen():
    print("\033[H\033[J", end="")


def format_vec(v, precision=2):
    if v is None:
        return "NO DATA"
    return f"[{', '.join([f'{x:.{precision}f}' for x in v])}]"


def main():
    sim = TeleopSimulator()

    try:
        while True:
            (l_in, l_q), (r_in, r_q) = sim.step()

            clear_screen()
            print("================================================================")
            print(f"   HAND TRACKING -> IK SIMULATOR (Streaming to Port {TELEMETRY_PORT})")
            print("================================================================")
            print("Instructions:")
            print(" 1. Put on headset. Enable Hand Tracking.")
            print(" 2. Pinch 'Grip' (Middle Finger + Thumb) to clutch/move arm.")
            print("================================================================\n")

            # --- LEFT ARM ---
            l_status = "ACTIVE (MOVING)" if sim.state["left"]["active"] else "IDLE (HOLDING)"
            print(f"LEFT ARM  | Status: {l_status}")
            print(f"  > Input (Wrist):  {format_vec(l_in[:3] if l_in is not None else None)}")
            # Convert radians to degrees for readability
            l_deg = np.rad2deg(l_q) if l_q is not None else None
            print(f"  > Output (Joints): {format_vec(l_deg, 1)}")
            print("-" * 64)

            # --- RIGHT ARM ---
            r_status = "ACTIVE (MOVING)" if sim.state["right"]["active"] else "IDLE (HOLDING)"
            print(f"RIGHT ARM | Status: {r_status}")
            print(f"  > Input (Wrist):  {format_vec(r_in[:3] if r_in is not None else None)}")
            r_deg = np.rad2deg(r_q) if r_q is not None else None
            print(f"  > Output (Joints): {format_vec(r_deg, 1)}")
            print("================================================================")

            time.sleep(0.05)  # 20Hz refresh

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        sim.close()


if __name__ == "__main__":
    main()
