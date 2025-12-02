#!/usr/bin/env python3
"""
XRoboToolkit Real-Time XR Data Display

This script displays live XR device data (headset, controllers, buttons, triggers, joysticks)
in a terminal interface for debugging and understanding coordinate systems/transformations.

Usage:
    python xr_data_display.py

    Press Ctrl+C to exit.

Requirements:
    - XRoboToolkit PC Service must be running
    - XR device must be connected
"""

import sys
import os
import time
from typing import Tuple
import numpy as np
import meshcat.transformations as tf

from xrobotoolkit_teleop.common.xr_client import XrClient


def quaternion_to_euler(quat: np.ndarray) -> Tuple[float, float, float]:
    """
    Convert quaternion to Euler angles (roll, pitch, yaw) in degrees.

    Args:
        quat: Quaternion in [x, y, z, w] format (from XR pose data)

    Returns:
        Tuple of (roll, pitch, yaw) in degrees
    """
    # Convert from [x,y,z,w] to [w,x,y,z] for meshcat
    quat_wxyz = np.array([quat[3], quat[0], quat[1], quat[2]])

    # Convert to rotation matrix
    rot_matrix = tf.quaternion_matrix(quat_wxyz)

    # Extract Euler angles (in radians)
    # Using ZYX convention (yaw, pitch, roll)
    sy = np.sqrt(rot_matrix[0, 0] ** 2 + rot_matrix[1, 0] ** 2)

    singular = sy < 1e-6

    if not singular:
        roll = np.arctan2(rot_matrix[2, 1], rot_matrix[2, 2])
        pitch = np.arctan2(-rot_matrix[2, 0], sy)
        yaw = np.arctan2(rot_matrix[1, 0], rot_matrix[0, 0])
    else:
        roll = np.arctan2(-rot_matrix[1, 2], rot_matrix[1, 1])
        pitch = np.arctan2(-rot_matrix[2, 0], sy)
        yaw = 0

    # Convert to degrees
    return (np.degrees(roll), np.degrees(pitch), np.degrees(yaw))


def create_progress_bar(value: float, width: int = 20) -> str:
    """
    Create a visual progress bar for analog values (0.0 to 1.0).

    Args:
        value: Float value between 0.0 and 1.0
        width: Width of the progress bar in characters

    Returns:
        Progress bar string like "████░░░░░░░░░░░░░░░░"
    """
    filled = int(value * width)
    empty = width - filled
    return "█" * filled + "░" * empty


def format_float_with_sign(value: float, decimals: int = 3) -> str:
    """
    Format float with explicit + or - sign.

    Args:
        value: Float value to format
        decimals: Number of decimal places

    Returns:
        Formatted string like "+1.234" or "-0.567"
    """
    return f"{value:+.{decimals}f}"


class XrDataFormatter:
    """Formats XR device data for terminal display."""

    def format_position(self, pose: np.ndarray) -> str:
        """
        Format position from pose array.

        Args:
            pose: 7-element array [x, y, z, qx, qy, qz, qw]

        Returns:
            Formatted position string
        """
        x, y, z = pose[0], pose[1], pose[2]
        return f"[{format_float_with_sign(x)}, {format_float_with_sign(y)}, {format_float_with_sign(z)}] m"

    def format_quaternion(self, pose: np.ndarray) -> str:
        """
        Format quaternion from pose array.

        Args:
            pose: 7-element array [x, y, z, qx, qy, qz, qw]

        Returns:
            Formatted quaternion string
        """
        qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]
        return f"[x:{format_float_with_sign(qx)}, y:{format_float_with_sign(qy)}, z:{format_float_with_sign(qz)}, w:{format_float_with_sign(qw)}]"

    def format_euler(self, pose: np.ndarray) -> str:
        """
        Format Euler angles from pose array.

        Args:
            pose: 7-element array [x, y, z, qx, qy, qz, qw]

        Returns:
            Formatted Euler angles string
        """
        quat = pose[3:7]  # Extract quaternion [qx, qy, qz, qw]
        roll, pitch, yaw = quaternion_to_euler(quat)
        return f"[Roll:{format_float_with_sign(roll, 1)}°, Pitch:{format_float_with_sign(pitch, 1)}°, Yaw:{format_float_with_sign(yaw, 1)}°]"

    def format_analog(self, value: float, width: int = 20) -> str:
        """
        Format analog input (trigger/grip) with progress bar.

        Args:
            value: Float value between 0.0 and 1.0
            width: Width of progress bar

        Returns:
            Formatted string with value and progress bar
        """
        bar = create_progress_bar(value, width)
        return f"{value:.2f}  |{bar}"

    def format_joystick(self, xy: list) -> str:
        """
        Format joystick 2D position.

        Args:
            xy: List with [x, y] values

        Returns:
            Formatted joystick string
        """
        x, y = xy[0], xy[1]
        return f"[x:{format_float_with_sign(x, 2)}, y:{format_float_with_sign(y, 2)}]"

    def format_button(self, state: bool, label: str) -> str:
        """
        Format button state with visual indicator.

        Args:
            state: Boolean button state
            label: Button label

        Returns:
            Formatted button string like "[A:ON★]" or "[B:OFF]"
        """
        state_str = "ON★" if state else "OFF"
        return f"[{label}:{state_str}]"


class TerminalManager:
    """Manages terminal display and timing."""

    def __init__(self):
        """Initialize terminal manager with timing variables."""
        self.last_time = time.perf_counter()
        self.delta_time = 0.0
        self.fps = 0.0

    def clear_screen(self):
        """Clear terminal screen (cross-platform)."""
        os.system('clear' if os.name == 'posix' else 'cls')

    def update_timing(self):
        """Update delta time and FPS calculations."""
        current_time = time.perf_counter()
        self.delta_time = current_time - self.last_time
        self.last_time = current_time

        if self.delta_time > 0:
            self.fps = 1.0 / self.delta_time
        else:
            self.fps = 0.0

    def get_delta_time(self) -> float:
        """Get delta time in seconds."""
        return self.delta_time

    def get_fps(self) -> float:
        """Get current FPS."""
        return self.fps


def display_header(frame_count: int, delta_time: float, fps: float):
    """
    Display header with frame info.

    Args:
        frame_count: Current frame number
        delta_time: Time since last frame (seconds)
        fps: Current frames per second
    """
    delta_ms = delta_time * 1000.0
    print(f"XR DATA DISPLAY - Frame {frame_count} | Delta: {delta_ms:.2f}ms | FPS: {fps:.1f}")
    print("═" * 63)
    print()


def display_pose_data(device_name: str, pose: np.ndarray, formatter: XrDataFormatter):
    """
    Display pose data for a device.

    Args:
        device_name: Name of device (HEADSET, LEFT CONTROLLER, RIGHT CONTROLLER)
        pose: 7-element pose array
        formatter: XrDataFormatter instance
    """
    print(f"{device_name}")
    print(f"  Position:    {formatter.format_position(pose)}")
    print(f"  Quaternion:  {formatter.format_quaternion(pose)}")
    print(f"  Euler (RPY): {formatter.format_euler(pose)}")
    print()


def display_analog_data(
    device_name: str,
    trigger: float,
    grip: float,
    joystick: list,
    formatter: XrDataFormatter
):
    """
    Display analog data (trigger, grip, joystick) for a controller.

    Args:
        device_name: Controller name
        trigger: Trigger value (0-1)
        grip: Grip value (0-1)
        joystick: Joystick [x, y]
        formatter: XrDataFormatter instance
    """
    print(f"  Trigger:     {formatter.format_analog(trigger)}")
    print(f"  Grip:        {formatter.format_analog(grip)}")
    print(f"  Joystick:    {formatter.format_joystick(joystick)}")


def display_button_data(buttons: dict, formatter: XrDataFormatter):
    """
    Display button states.

    Args:
        buttons: Dictionary of button states
        formatter: XrDataFormatter instance
    """
    print("BUTTONS")

    # Left hand buttons
    left_buttons = f"  Left:   {formatter.format_button(buttons['left_menu'], 'Menu')} "
    left_buttons += f"{formatter.format_button(buttons['left_click'], 'Click')} "
    left_buttons += f"{formatter.format_button(buttons['X'], 'X')} "
    left_buttons += f"{formatter.format_button(buttons['Y'], 'Y')}"
    print(left_buttons)

    # Right hand buttons
    right_buttons = f"  Right:  {formatter.format_button(buttons['right_menu'], 'Menu')} "
    right_buttons += f"{formatter.format_button(buttons['right_click'], 'Click')} "
    right_buttons += f"{formatter.format_button(buttons['A'], 'A')} "
    right_buttons += f"{formatter.format_button(buttons['B'], 'B')}"
    print(right_buttons)
    print()


def display_timestamp(timestamp: int):
    """
    Display timestamp.

    Args:
        timestamp: Timestamp in nanoseconds
    """
    print(f"TIMESTAMP: {timestamp} ns")
    print("═" * 63)


def main():
    """Main function to run XR data display."""
    xr_client = None

    try:
        # Initialize
        print("Initializing XRoboToolkit SDK...")
        xr_client = XrClient()
        formatter = XrDataFormatter()
        term_mgr = TerminalManager()
        frame_count = 0

        print("XR Data Display started. Press Ctrl+C to exit.")
        print("Make sure XRoboToolkit PC Service is running and XR device is connected.")
        time.sleep(2)

        # Main loop
        while True:
            # Update timing
            term_mgr.update_timing()
            delta_time = term_mgr.get_delta_time()
            fps = term_mgr.get_fps()

            # Clear screen
            term_mgr.clear_screen()

            # Read all XR data
            head_pose = xr_client.get_pose_by_name("headset")
            left_pose = xr_client.get_pose_by_name("left_controller")
            right_pose = xr_client.get_pose_by_name("right_controller")

            left_trigger = xr_client.get_key_value_by_name("left_trigger")
            right_trigger = xr_client.get_key_value_by_name("right_trigger")
            left_grip = xr_client.get_key_value_by_name("left_grip")
            right_grip = xr_client.get_key_value_by_name("right_grip")

            left_joystick = xr_client.get_joystick_state("left")
            right_joystick = xr_client.get_joystick_state("right")

            buttons = {
                'A': xr_client.get_button_state_by_name("A"),
                'B': xr_client.get_button_state_by_name("B"),
                'X': xr_client.get_button_state_by_name("X"),
                'Y': xr_client.get_button_state_by_name("Y"),
                'left_menu': xr_client.get_button_state_by_name("left_menu_button"),
                'right_menu': xr_client.get_button_state_by_name("right_menu_button"),
                'left_click': xr_client.get_button_state_by_name("left_axis_click"),
                'right_click': xr_client.get_button_state_by_name("right_axis_click"),
            }

            timestamp = xr_client.get_timestamp_ns()

            # Display header
            display_header(frame_count, delta_time, fps)

            # Display pose data
            display_pose_data("HEADSET", head_pose, formatter)
            display_pose_data("LEFT CONTROLLER", left_pose, formatter)

            # Display left controller analog data
            display_analog_data("LEFT", left_trigger, left_grip, left_joystick, formatter)
            print()

            display_pose_data("RIGHT CONTROLLER", right_pose, formatter)

            # Display right controller analog data
            display_analog_data("RIGHT", right_trigger, right_grip, right_joystick, formatter)
            print()

            # Display button data
            display_button_data(buttons, formatter)

            # Display timestamp
            display_timestamp(timestamp)

            frame_count += 1

            # Target 60 Hz (16.67ms per frame)
            target_frame_time = 1.0 / 60.0
            sleep_time = max(0, target_frame_time - delta_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\nShutting down gracefully...")
        if xr_client:
            xr_client.close()
        print("XR Data Display closed. Goodbye!")
        sys.exit(0)

    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()

        if xr_client:
            try:
                xr_client.close()
            except:
                pass

        sys.exit(1)


if __name__ == "__main__":
    main()
