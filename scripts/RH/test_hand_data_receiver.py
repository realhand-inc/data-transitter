#!/usr/bin/env python3
"""
Test receiver for MediaPipe hand data from ADB Control GUI.
This script subscribes to the ZMQ publisher and prints received hand tracking data.

Usage:
    python test_hand_data_receiver.py [--port 5556] [--raw] [--verbose]
"""

import zmq
import json
import argparse
import time
from datetime import datetime


def main():
    parser = argparse.ArgumentParser(description='Test receiver for MediaPipe hand data')
    parser.add_argument('--host', type=str, default='localhost', help='ZMQ host (default: localhost)')
    parser.add_argument('--port', type=int, default=5556, help='ZMQ port (default: 5556)')
    parser.add_argument('--raw', action='store_true', help='Show raw JSON data')
    parser.add_argument('--verbose', action='store_true', help='Show all landmarks (not just first)')
    args = parser.parse_args()

    # Create ZMQ context and SUB socket
    context = zmq.Context()
    socket = context.socket(zmq.SUB)

    # Connect to the publisher
    endpoint = f"tcp://{args.host}:{args.port}"
    print(f"Connecting to {endpoint}...")
    socket.connect(endpoint)

    # Subscribe to all messages (empty string = subscribe to everything)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    print(f"✓ Connected and subscribed to {endpoint}")

    if args.raw:
        print("Mode: RAW JSON (showing complete JSON payloads)")
    elif args.verbose:
        print("Mode: VERBOSE (showing all 21 landmarks per hand)")
    else:
        print("Mode: SUMMARY (showing updates every 1 second)")

    print("Waiting for hand data... (Press Ctrl+C to stop)\n")

    message_count = 0
    start_time = time.time()
    last_print_time = start_time

    try:
        while True:
            # Receive message (blocking)
            message = socket.recv_string()
            message_count += 1
            current_time = time.time()

            # Parse JSON
            try:
                data = json.loads(message)

                # Validate structure
                if 'left' in data and 'right' in data:
                    left_landmarks = data['left']
                    right_landmarks = data['right']

                    # Show raw JSON if requested
                    if args.raw:
                        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        print(f"\n{'='*80}")
                        print(f"[{timestamp}] Message #{message_count}")
                        print(f"{'='*80}")
                        print(json.dumps(data, indent=2))
                        print()

                    # Show verbose landmark data
                    elif args.verbose:
                        timestamp = datetime.now().strftime("%H:%M:%S")
                        print(f"\n[{timestamp}] Message #{message_count}")
                        print(f"Left hand ({len(left_landmarks)} landmarks):")
                        for i, lm in enumerate(left_landmarks):
                            print(f"  [{i:2d}] x={lm['x']:7.4f}, y={lm['y']:7.4f}, z={lm['z']:7.4f}")
                        print(f"\nRight hand ({len(right_landmarks)} landmarks):")
                        for i, lm in enumerate(right_landmarks):
                            print(f"  [{i:2d}] x={lm['x']:7.4f}, y={lm['y']:7.4f}, z={lm['z']:7.4f}")
                        print()

                    # Default: Print update every second
                    elif current_time - last_print_time >= 1.0:
                        elapsed = current_time - start_time
                        rate = message_count / elapsed if elapsed > 0 else 0

                        timestamp = datetime.now().strftime("%H:%M:%S")
                        print(f"[{timestamp}] Messages: {message_count} | Rate: {rate:.1f} Hz")
                        print(f"  Left hand:  {len(left_landmarks)} landmarks")
                        print(f"  Right hand: {len(right_landmarks)} landmarks")

                        # Show sample data from wrist if available
                        if left_landmarks and len(left_landmarks) > 0:
                            first = left_landmarks[0]
                            if 'x' in first and 'y' in first and 'z' in first:
                                print(f"  Sample (left wrist): x={first['x']:.3f}, y={first['y']:.3f}, z={first['z']:.3f}")

                        if right_landmarks and len(right_landmarks) > 0:
                            first = right_landmarks[0]
                            if 'x' in first and 'y' in first and 'z' in first:
                                print(f"  Sample (right wrist): x={first['x']:.3f}, y={first['y']:.3f}, z={first['z']:.3f}")

                        print()
                        last_print_time = current_time

                else:
                    print(f"⚠ Unexpected data format: {data.keys()}")

            except json.JSONDecodeError as e:
                print(f"✗ JSON parse error: {e}")
                print(f"  Raw message: {message[:100]}...")
            except Exception as e:
                print(f"✗ Error processing message: {e}")

    except KeyboardInterrupt:
        print("\n\nStopping...")
        elapsed = time.time() - start_time
        rate = message_count / elapsed if elapsed > 0 else 0
        print(f"\nStatistics:")
        print(f"  Total messages: {message_count}")
        print(f"  Duration: {elapsed:.1f}s")
        print(f"  Average rate: {rate:.1f} Hz")

    finally:
        socket.close()
        context.term()
        print("✓ Disconnected")


if __name__ == "__main__":
    main()
