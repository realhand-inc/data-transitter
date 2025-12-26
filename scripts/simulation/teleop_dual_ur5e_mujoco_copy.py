import os

import meshcat.transformations as tf
import numpy as np
import tyro
from placo_utils.visualization import frame_viz
from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import (
    MujocoTeleopController,
)
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH


class RemappedMujocoTeleopController(MujocoTeleopController):
    """MujocoTeleopController with headset-based parent transform for hand tracking."""

    def __init__(self, *args, visualize_transforms=False, **kwargs):
        super().__init__(*args, **kwargs)
        self.initial_headset_yaw = None  # Fixed yaw rotation recorded at startup
        self.visualize_transforms = visualize_transforms  # Enable transform visualization in meshcat

    def _extract_yaw_from_quat(self, quat):
        """Extract yaw angle from quaternion [w, x, y, z]."""
        qw, qx, qy, qz = quat[0], quat[1], quat[2], quat[3]
        yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        return yaw

    def _create_yaw_rotation_matrix(self, yaw):
        """Create 3x3 rotation matrix for yaw rotation around Y-axis."""
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        R_yaw = np.array([[cos_yaw, 0, sin_yaw], [0, 1, 0], [-sin_yaw, 0, cos_yaw]])
        return R_yaw

    def _process_xr_pose(self, xr_pose, src_name):
        """Process XR pose with headset parent transform."""
        config = self.manipulator_config[src_name]

        # Check if headset parent transform is enabled for this hand
        if config.get("use_headset_parent", False):
            # Initialize headset yaw on first call
            if self.initial_headset_yaw is None:
                headset_pose = self.xr_client.get_pose_by_name("headset")
                if headset_pose is not None:
                    headset_quat = np.array(
                        [
                            headset_pose[6],  # w
                            headset_pose[3],  # x
                            headset_pose[4],  # y
                            headset_pose[5],  # z
                        ]
                    )
                    self.initial_headset_yaw = self._extract_yaw_from_quat(headset_quat)
                    print(f"Initial headset yaw recorded: {np.degrees(self.initial_headset_yaw):.2f}°")

            # Get current headset position
            headset_pose = self.xr_client.get_pose_by_name("headset")
            if headset_pose is not None and self.initial_headset_yaw is not None:
                current_headset_pos = headset_pose[:3]
                current_headset_quat = np.array(
                    [
                        headset_pose[6],  # w
                        headset_pose[3],  # x
                        headset_pose[4],  # y
                        headset_pose[5],  # z
                    ]
                )

                # Create parent transform: current position + initial yaw rotation
                T_parent = np.eye(4)
                T_parent[:3, :3] = self._create_yaw_rotation_matrix(self.initial_headset_yaw)
                T_parent[:3, 3] = current_headset_pos

                # Create hand transform from world pose
                hand_pos = np.array(xr_pose[:3])
                hand_quat = np.array(
                    [
                        xr_pose[6],  # w
                        xr_pose[3],  # x
                        xr_pose[4],  # y
                        xr_pose[5],  # z
                    ]
                )

                T_hand_world = tf.quaternion_matrix(hand_quat)
                T_hand_world[:3, 3] = hand_pos

                # Transform hand to be child of parent: T_child = T_parent^-1 @ T_hand_world
                T_parent_inv = np.linalg.inv(T_parent)
                T_hand_relative = T_parent_inv @ T_hand_world

                # Visualize transforms in meshcat if enabled and placo viz is active
                if self.visualize_transforms and hasattr(self, "placo_vis"):
                    # Visualize headset current pose
                    T_headset = tf.quaternion_matrix(current_headset_quat)
                    T_headset[:3, 3] = current_headset_pos
                    frame_viz("headset_pose", T_headset, length=0.15, radius=0.005)

                    # Visualize parent transform (follows headset position, fixed yaw)
                    frame_viz("headset_parent", T_parent, length=0.2, radius=0.008)

                    # Visualize hand world pose
                    frame_viz(f"{src_name}_world", T_hand_world, length=0.1, radius=0.004)

                    # Visualize hand relative pose (child of parent)
                    frame_viz(f"{src_name}_relative", T_hand_relative, length=0.1, radius=0.004)

                # Extract relative position and quaternion
                xr_pose = xr_pose.copy()
                xr_pose[:3] = T_hand_relative[:3, 3]

                relative_quat = tf.quaternion_from_matrix(T_hand_relative)
                xr_pose[6] = relative_quat[0]  # w
                xr_pose[3] = relative_quat[1]  # x
                xr_pose[4] = relative_quat[2]  # y
                xr_pose[5] = relative_quat[3]  # z

        # Apply pose_offset if configured
        if "pose_offset" in config:
            offset = np.array(config["pose_offset"])
            xr_pose = xr_pose.copy()
            xr_pose[0:3] += offset

        # Call parent implementation
        return super()._process_xr_pose(xr_pose, src_name)


def main(
    xml_path: str = os.path.join(ASSET_PATH, "universal_robots_ur5e/scene_dual_arm.xml"),
    robot_urdf_path: str = os.path.join(ASSET_PATH, "universal_robots_ur5e/dual_ur5e.urdf"),
    scale_factor: float = 1.5,
    visualize_placo: bool = True,
    visualize_transforms: bool = True,
):
    """
    Main function to run the dual UR5e teleoperation in MuJoCo.

    Args:
        visualize_placo: Enable Placo robot visualization in meshcat
        visualize_transforms: Enable headset/hand transform visualization in meshcat
    """
    config = {
        "right_hand": {
            "link_name": "right_tool0",
            "pose_source": "right_hand_wrist",
            "use_headset_parent": True,
            "pose_offset": [0.0, 0.0, 0.0],
            "vis_target": "right_target",
        },
        "left_hand": {
            "link_name": "left_tool0",
            "pose_source": "left_hand_wrist",
            "use_headset_parent": True,
            "pose_offset": [0.0, 0.0, 0.0],
            "vis_target": "left_target",
        },
    }

    # Create and initialize the teleoperation controller
    controller = RemappedMujocoTeleopController(
        xml_path=xml_path,
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        scale_factor=scale_factor,
        visualize_placo=visualize_placo,
        visualize_transforms=visualize_transforms,
    )

    # additional constraints hardcoded here for now
    joints_task = controller.solver.add_joints_task()
    joints_task.set_joints({joint: 0.0 for joint in controller.placo_robot.joint_names()})
    joints_task.configure("joints_regularization", "soft", 1e-4)

    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
