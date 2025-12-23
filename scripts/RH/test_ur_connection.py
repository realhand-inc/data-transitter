#!/usr/bin/env python3
"""
Test script for UR robot connection and position reading.
Connects to the robot's real-time interface (port 30003) and reads joint/TCP positions.
"""
import socket
import struct
import sys
import argparse
import time
import os


def test_connection(robot_ip, port=30003):
    """Test if we can connect to the robot on the specified port."""
    print(f"Testing connection to {robot_ip}:{port}...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)  # 5 second timeout
        sock.connect((robot_ip, port))
        print(f"✓ Successfully connected to {robot_ip}:{port}")
        sock.close()
        return True
    except socket.timeout:
        print(f"✗ Connection timeout - robot may be offline or IP incorrect")
        return False
    except ConnectionRefusedError:
        print(f"✗ Connection refused - check if robot is running")
        return False
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return False


def clear_screen():
    """Clear the terminal screen."""
    # Use ANSI escape codes to clear screen and move cursor to top
    print("\033[2J\033[H", end="")


def read_robot_state_once(sock):
    """Read and parse one packet of robot state data."""
    # Read packet - size varies by UR model/firmware (typically 1060-1116 bytes)
    data = sock.recv(2048)  # Read more than needed to ensure we get full packet

    # Check if we have enough data for joint positions (need at least 252 + 48 = 300 bytes)
    if len(data) < 300:
        return None, None, f"Error: Packet too small ({len(data)} bytes), need at least 300 bytes"

    # Parse joint positions (6 doubles starting at byte 252)
    offset = 252
    joint_positions = []
    for i in range(6):
        pos = struct.unpack('!d', data[offset:offset+8])[0]  # Big-endian double
        joint_positions.append(pos)
        offset += 8

    # Check if we have enough data for TCP position (need at least 444 + 48 = 492 bytes)
    if len(data) < 492:
        return joint_positions, None, f"Warning: Packet size {len(data)} bytes - TCP data may be incomplete"

    # Parse TCP position (6 doubles starting at byte 444)
    offset = 444
    tcp_position = []
    for i in range(6):
        pos = struct.unpack('!d', data[offset:offset+8])[0]
        tcp_position.append(pos)
        offset += 8

    return joint_positions, tcp_position, None


def display_robot_state(joint_positions, tcp_position, robot_ip, port, update_count):
    """Display robot state in a formatted way."""
    print("="*70)
    print(f"UR ROBOT REAL-TIME MONITOR - {robot_ip}:{port}")
    print(f"Updates: {update_count} | Press Ctrl+C to exit")
    print("="*70)

    print("\nJOINT POSITIONS")
    print("-"*70)
    for i, pos in enumerate(joint_positions):
        print(f"  Joint {i}: {pos:8.4f} rad  ({pos * 57.2958:7.2f}°)")

    if tcp_position is not None:
        print("\nTCP POSITION (Tool Center Point)")
        print("-"*70)
        print(f"  X:  {tcp_position[0]:8.4f} m")
        print(f"  Y:  {tcp_position[1]:8.4f} m")
        print(f"  Z:  {tcp_position[2]:8.4f} m")
        print(f"  RX: {tcp_position[3]:8.4f} rad  ({tcp_position[3] * 57.2958:7.2f}°)")
        print(f"  RY: {tcp_position[4]:8.4f} rad  ({tcp_position[4] * 57.2958:7.2f}°)")
        print(f"  RZ: {tcp_position[5]:8.4f} rad  ({tcp_position[5] * 57.2958:7.2f}°)")
    else:
        print("\nTCP POSITION (Tool Center Point)")
        print("-"*70)
        print("  [Data not available]")

    print("="*70)


def read_robot_state(robot_ip, port=30003, realtime=False, rate=10):
    """Read and parse robot state data from the real-time interface."""
    print(f"Connecting to robot at {robot_ip}:{port}...")

    try:
        # Connect to the robot
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        sock.connect((robot_ip, port))
        print("Connected successfully!")

        if not realtime:
            time.sleep(0.5)

        update_count = 0

        if realtime:
            print(f"Starting real-time monitor at {rate} Hz...")
            time.sleep(1)

            try:
                while True:
                    joint_positions, tcp_position, error = read_robot_state_once(sock)

                    if error:
                        print(f"Error: {error}")
                        break

                    update_count += 1
                    clear_screen()
                    display_robot_state(joint_positions, tcp_position, robot_ip, port, update_count)

                    time.sleep(1.0 / rate)

            except KeyboardInterrupt:
                print("\n\nStopping real-time monitor...")
        else:
            # Single read
            joint_positions, tcp_position, error = read_robot_state_once(sock)

            if error:
                print(f"Error: {error}")
                sock.close()
                return False

            update_count = 1
            display_robot_state(joint_positions, tcp_position, robot_ip, port, update_count)

        sock.close()
        return True

    except socket.timeout:
        print("Error: Connection timeout")
        return False
    except Exception as e:
        print(f"Error reading robot state: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Test UR robot connection and read position data"
    )
    parser.add_argument(
        "robot_ip",
        nargs='?',
        default="192.168.2.2",
        help="IP address of the UR robot (default: 192.168.2.2)"
    )
    parser.add_argument(
        "-p", "--port",
        type=int,
        default=30003,
        help="Port number (default: 30003 for real-time interface)"
    )
    parser.add_argument(
        "--test-only",
        action="store_true",
        help="Only test connection without reading data"
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

    args = parser.parse_args()

    print("="*70)
    print("UR Robot Connection Test")
    print("="*70)

    # Test connection
    if not test_connection(args.robot_ip, args.port):
        sys.exit(1)

    # Read and display robot state (unless --test-only is specified)
    if not args.test_only:
        if not read_robot_state(args.robot_ip, args.port, realtime=args.realtime, rate=args.rate):
            sys.exit(1)

    if not args.realtime:
        print("\nTest completed successfully!")


if __name__ == "__main__":
    main()
