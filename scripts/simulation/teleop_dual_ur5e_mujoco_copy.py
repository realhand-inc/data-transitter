import os

import numpy as np
import tyro
from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import (
    MujocoTeleopController,
)
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH


class RemappedMujocoTeleopController(MujocoTeleopController):
    """MujocoTeleopController with pose offset remapping for hand tracking."""

    def _process_xr_pose(self, xr_pose, src_name):
        """Process XR pose with optional position offset applied before scaling."""
        config = self.manipulator_config[src_name]
        if "pose_offset" in config:
            offset = np.array(config["pose_offset"])
            # Apply offset to position (first 3 elements of xr_pose)
            xr_pose = xr_pose.copy()
            xr_pose[0:3] += offset

        # Call parent implementation
        return super()._process_xr_pose(xr_pose, src_name)


def main(
    xml_path: str = os.path.join(ASSET_PATH, "universal_robots_ur5e/scene_dual_arm.xml"),
    robot_urdf_path: str = os.path.join(ASSET_PATH, "universal_robots_ur5e/dual_ur5e.urdf"),
    scale_factor: float = 1.5,
    visualize_placo: bool = True,
):
    """
    Main function to run the dual UR5e teleoperation in MuJoCo.
    """
    config = {
        "right_hand": {
            "link_name": "right_tool0",
            "pose_source": "right_hand_wrist",
            "pose_offset": [0.0, 0.0, 0.0],
            "vis_target": "right_target",
        },
        "left_hand": {
            "link_name": "left_tool0",
            "pose_source": "left_hand_wrist",
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
    )

    # additional constraints hardcoded here for now
    joints_task = controller.solver.add_joints_task()
    joints_task.set_joints({joint: 0.0 for joint in controller.placo_robot.joint_names()})
    joints_task.configure("joints_regularization", "soft", 1e-4)

    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
