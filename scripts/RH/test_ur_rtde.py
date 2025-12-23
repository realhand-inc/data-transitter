#!/usr/bin/env python3
"""
Test script for UR robot using ur_rtde library.
Reads joint positions and TCP pose in real-time.
"""
import rtde_receive
import time
import argparse
import sys


ROBOT_IP = "192.168.2.2"


def clear_screen():
    """Clear the terminal screen."""
    print("\033[2J\033[H", end="")


def display_robot_state(joint_positions, tcp_pose, robot_ip, update_count):
    """Display robot state in a formatted way."""
    print("="*70)
    print(f"UR ROBOT REAL-TIME MONITOR - {robot_ip}")
    print(f"Updates: {update_count} | Press Ctrl+C to exit")
    print("="*70)

    print("\nJOINT POSITIONS")
    print("-"*70)
    for i, pos in enumerate(joint_positions):
        print(f"  Joint {i}: {pos:8.4f} rad  ({pos * 57.2958:7.2f}°)")

    print("\nTCP POSE (Tool Center Point)")
    print("-"*70)
    print(f"  X:  {tcp_pose[0]:8.4f} m")
    print(f"  Y:  {tcp_pose[1]:8.4f} m")
    print(f"  Z:  {tcp_pose[2]:8.4f} m")
    print(f"  RX: {tcp_pose[3]:8.4f} rad  ({tcp_pose[3] * 57.2958:7.2f}°)")
    print(f"  RY: {tcp_pose[4]:8.4f} rad  ({tcp_pose[4] * 57.2958:7.2f}°)")
    print(f"  RZ: {tcp_pose[5]:8.4f} rad  ({tcp_pose[5] * 57.2958:7.2f}°)")
    print("="*70)


def test_connection(robot_ip):
    """Test if we can connect to the robot."""
    print(f"Testing connection to {robot_ip}...")
    try:
        rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
        print(f"✓ Successfully connected to {robot_ip}")
        rtde_r.disconnect()
        return True
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return False


def monitor_robot(robot_ip, realtime=False, rate=10):
    """Monitor robot state."""
    print(f"Connecting to robot at {robot_ip}...")

    try:
        rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
        print("Connected successfully!")

        if not realtime:
            time.sleep(0.5)

        update_count = 0

        if realtime:
            print(f"Starting real-time monitor at {rate} Hz...")
            time.sleep(1)

            try:
                while True:
                    # Read joint positions and TCP pose
                    joint_positions = rtde_r.getActualQ()
                    tcp_pose = rtde_r.getActualTCPPose()

                    update_count += 1
                    clear_screen()
                    display_robot_state(joint_positions, tcp_pose, robot_ip, update_count)

                    time.sleep(1.0 / rate)

            except KeyboardInterrupt:
                print("\n\nStopping real-time monitor...")
        else:
            # Single read
            joint_positions = rtde_r.getActualQ()
            tcp_pose = rtde_r.getActualTCPPose()

            update_count = 1
            display_robot_state(joint_positions, tcp_pose, robot_ip, update_count)

        rtde_r.disconnect()
        return True

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Test UR robot connection and read position data using ur_rtde"
    )
    parser.add_argument(
        "robot_ip",
        nargs='?',
        default=ROBOT_IP,
        help=f"IP address of the UR robot (default: {ROBOT_IP})"
    )
    parser.add_argument(
        "-r", "--realtime",
        action="store_true",
        help="Continuously read and display data in real-time"
    )
    parser.add_argument(
        "--rate",
        type=int,
        default=10,
        help="Update rate in Hz for real-time mode (default: 10)"
    )
    parser.add_argument(
        "--test-only",
        action="store_true",
        help="Only test connection without reading data"
    )

    args = parser.parse_args()

    print("="*70)
    print("UR Robot Connection Test (using ur_rtde)")
    print("="*70)

    # Test connection
    if not test_connection(args.robot_ip):
        sys.exit(1)

    # Monitor robot (unless --test-only is specified)
    if not args.test_only:
        if not monitor_robot(args.robot_ip, realtime=args.realtime, rate=args.rate):
            sys.exit(1)

    if not args.realtime:
        print("\nTest completed successfully!")


if __name__ == "__main__":
    main()
