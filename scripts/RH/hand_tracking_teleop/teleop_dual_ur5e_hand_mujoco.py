"""
Hand tracking-based teleoperation for dual UR5e arms in MuJoCo simulation.
Uses hand wrist pose (joint 1 from 27-joint array) for robot arm control.
"""
import os

import tyro
from xrobotoolkit_teleop.simulation.mujoco_teleop_controller import (
    MujocoTeleopController,
)
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH


def main(
    xml_path: str = os.path.join(ASSET_PATH, "universal_robots_ur5e/scene_dual_arm.xml"),
    robot_urdf_path: str = os.path.join(ASSET_PATH, "universal_robots_ur5e/dual_ur5e.urdf"),
    scale_factor: float = 1.5,
    visualize_placo: bool = True,
):
    """
    Main function to run hand tracking-based dual UR5e teleoperation in MuJoCo.

    Args:
        xml_path: Path to MuJoCo scene XML
        robot_urdf_path: Path to robot URDF
        scale_factor: Motion scaling factor for robot movements
        visualize_placo: Show Placo IK visualization in browser
    """
    # Hybrid configuration for testing:
    # - Left hand: Controller (original behavior with grip trigger)
    # - Right hand: Hand tracking (always active, wrist pose)
    config = {
        "right_hand": {
            "link_name": "right_tool0",
            "pose_source": "right_hand_wrist",  # Use hand wrist tracking
            # Note: No "control_trigger" - hand tracking is always active
            "vis_target": "right_target",
        },
        "left_hand": {
            "link_name": "left_tool0",
            "pose_source": "left_hand_wrist",   # Use controller (original)
            # "control_trigger": "left_grip",     # Grip button to activate/deactivate
            "vis_target": "left_target",
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

    # Additional constraints (same as controller-based version)
    joints_task = controller.solver.add_joints_task()
    joints_task.set_joints({joint: 0.0 for joint in controller.placo_robot.joint_names()})
    joints_task.configure("joints_regularization", "soft", 1e-4)

    print("=" * 60)
    print("Hybrid Teleoperation Active (Testing Mode)")
    print("=" * 60)
    print("LEFT ARM:")
    print("  - Using LEFT CONTROLLER (original behavior)")
    print("  - Press LEFT GRIP button to activate/deactivate")
    print("")
    print("RIGHT ARM:")
    print("  - Using RIGHT HAND WRIST tracking")
    print("  - Control is ALWAYS ACTIVE (no button needed)")
    print("  - Hand must be visible (green in headset)")
    print("  - If hand disappears, robot freezes at last pose")
    print("=" * 60)

    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
