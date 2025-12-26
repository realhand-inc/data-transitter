#!/usr/bin/env python3
"""
Test various ADB commands for PICO 4u device
Focus on: boundary confirmation, wake/sleep, input events, power management
"""
import subprocess
import time
import sys

def execute_adb_command(command, device=None, timeout=5):
    """Execute an ADB command and return success status and output."""
    if device:
        if isinstance(command, str):
            if command.startswith("adb "):
                command = command.replace("adb ", f"adb -s {device} ", 1)
            else:
                command = f"adb -s {device} {command}"

    print(f"\n{'='*60}")
    print(f"Command: {command}")
    print(f"{'='*60}")

    try:
        result = subprocess.run(
            command,
            shell=True,
            capture_output=True,
            text=True,
            timeout=timeout
        )

        print(f"Return Code: {result.returncode}")
        if result.stdout:
            print("Output:")
            print(result.stdout)
        if result.stderr:
            print("Stderr:")
            print(result.stderr)

        return result.returncode == 0, result.stdout, result.stderr

    except subprocess.TimeoutExpired:
        print(f"TIMEOUT after {timeout}s")
        return False, "", "Timeout"
    except Exception as e:
        print(f"ERROR: {e}")
        return False, "", str(e)


def get_connected_devices():
    """Get list of connected ADB devices."""
    success, output, _ = execute_adb_command("adb devices")
    devices = []

    if success:
        for line in output.strip().split('\n')[1:]:
            if '\tdevice' in line:
                devices.append(line.split('\t')[0])

    return devices


def test_power_commands(device):
    """Test power-related commands."""
    print("\n" + "="*70)
    print("TESTING POWER COMMANDS")
    print("="*70)

    commands = [
        # Check power state
        ("Check power state", "shell dumpsys power | grep 'mWakefulness\\|Display Power'"),

        # Wake commands
        ("Wake device (input keyevent WAKEUP)", "shell input keyevent KEYCODE_WAKEUP"),
        ("Wake device (power button)", "shell input keyevent 26"),

        # Sleep/screen off commands
        ("Sleep device (input keyevent SLEEP)", "shell input keyevent KEYCODE_SLEEP"),
        ("Sleep device (power button)", "shell input keyevent 223"),

        # Check screen state
        ("Check screen state", "shell dumpsys input_method | grep mScreenOn"),
        ("Check display state", "shell dumpsys display | grep 'mScreenState\\|Display Power'"),
    ]

    for desc, cmd in commands:
        print(f"\n>>> {desc}")
        execute_adb_command(cmd, device)
        time.sleep(0.5)


def test_boundary_commands(device):
    """Test boundary-related commands for VR headsets."""
    print("\n" + "="*70)
    print("TESTING BOUNDARY/GUARDIAN COMMANDS")
    print("="*70)

    commands = [
        # Check boundary/guardian settings
        ("List all system settings", "shell settings list system"),
        ("List all global settings", "shell settings list global"),
        ("List all secure settings", "shell settings list secure"),

        # Common VR-specific settings keys to check
        ("Check VR settings", "shell dumpsys | grep -i 'vr\\|guardian\\|boundary\\|play.space'"),

        # Input events that might confirm boundary
        ("Send BUTTON_A (confirm)", "shell input keyevent KEYCODE_BUTTON_A"),
        ("Send ENTER (confirm)", "shell input keyevent KEYCODE_ENTER"),
        ("Send DPAD_CENTER (confirm)", "shell input keyevent KEYCODE_DPAD_CENTER"),

        # Check running activities
        ("Check current activity", "shell dumpsys activity activities | grep 'mResumedActivity'"),
    ]

    for desc, cmd in commands:
        print(f"\n>>> {desc}")
        execute_adb_command(cmd, device)
        time.sleep(0.5)


def test_input_commands(device):
    """Test various input commands."""
    print("\n" + "="*70)
    print("TESTING INPUT COMMANDS")
    print("="*70)

    commands = [
        # Basic inputs
        ("BACK button", "shell input keyevent KEYCODE_BACK"),
        ("HOME button", "shell input keyevent KEYCODE_HOME"),
        ("MENU button", "shell input keyevent KEYCODE_MENU"),

        # VR-specific buttons
        ("BUTTON_A", "shell input keyevent KEYCODE_BUTTON_A"),
        ("BUTTON_B", "shell input keyevent KEYCODE_BUTTON_B"),
        ("BUTTON_X", "shell input keyevent KEYCODE_BUTTON_X"),
        ("BUTTON_Y", "shell input keyevent KEYCODE_BUTTON_Y"),
        ("BUTTON_SELECT", "shell input keyevent KEYCODE_BUTTON_SELECT"),
        ("BUTTON_START", "shell input keyevent KEYCODE_BUTTON_START"),

        # Tap screen (might work for 2D UI overlays)
        ("Tap screen center", "shell input tap 500 500"),
    ]

    for desc, cmd in commands:
        print(f"\n>>> {desc}")
        print("(Not executing to avoid unwanted actions - uncomment to test)")
        # execute_adb_command(cmd, device)
        # time.sleep(1)


def test_system_info(device):
    """Get system information about the device."""
    print("\n" + "="*70)
    print("SYSTEM INFORMATION")
    print("="*70)

    commands = [
        ("Device model", "shell getprop ro.product.model"),
        ("Android version", "shell getprop ro.build.version.release"),
        ("SDK version", "shell getprop ro.build.version.sdk"),
        ("Device manufacturer", "shell getprop ro.product.manufacturer"),
        ("Device serial", "shell getprop ro.serialno"),

        # VR-specific info
        ("VR mode state", "shell dumpsys vrmanager"),

        # Running processes
        ("Running VR processes", "shell ps | grep -i 'vr\\|pico'"),
    ]

    for desc, cmd in commands:
        print(f"\n>>> {desc}")
        execute_adb_command(cmd, device)


def test_pico_specific(device):
    """Test PICO-specific commands."""
    print("\n" + "="*70)
    print("PICO-SPECIFIC COMMANDS")
    print("="*70)

    commands = [
        # PICO packages
        ("List PICO packages", "shell pm list packages | grep pico"),

        # PICO services
        ("Check PICO services", "shell dumpsys | grep -i pico | head -20"),

        # Boundary/Guardian specific
        ("Search for guardian", "shell dumpsys | grep -i guardian"),
        ("Search for boundary", "shell dumpsys | grep -i boundary"),
        ("Search for play space", "shell dumpsys | grep -i 'play.space\\|playspace'"),

        # Check activities
        ("List all activities", "shell pm list packages -3 | head -20"),
    ]

    for desc, cmd in commands:
        print(f"\n>>> {desc}")
        execute_adb_command(cmd, device)


def test_broadcast_commands(device):
    """Test broadcast intent commands that might trigger VR actions."""
    print("\n" + "="*70)
    print("TESTING BROADCAST/INTENT COMMANDS")
    print("="*70)

    commands = [
        # Common Android broadcasts
        ("Check current broadcasts", "shell dumpsys activity broadcasts | head -30"),

        # These are examples - may not work without knowing exact intent
        ("List broadcast receivers", "shell dumpsys activity broadcasts | grep -i 'vr\\|pico'"),
    ]

    for desc, cmd in commands:
        print(f"\n>>> {desc}")
        execute_adb_command(cmd, device)


def main():
    print("="*70)
    print("PICO 4U ADB COMMAND TESTER")
    print("="*70)

    # Get connected devices
    devices = get_connected_devices()

    if not devices:
        print("\nERROR: No devices connected!")
        print("Please connect your PICO 4u via ADB and try again.")
        sys.exit(1)

    print(f"\nConnected devices: {devices}")

    if len(devices) > 1:
        print("\nMultiple devices found. Using first device:", devices[0])

    device = devices[0]
    print(f"\nTesting on device: {device}")

    # Run tests
    print("\n\n")
    print("#" * 70)
    print("# Select test to run:")
    print("# 1. System Information")
    print("# 2. Power Commands (wake/sleep)")
    print("# 3. Boundary/Guardian Commands")
    print("# 4. Input Commands (button simulation)")
    print("# 5. PICO-specific Commands")
    print("# 6. Broadcast/Intent Commands")
    print("# 7. Run ALL tests")
    print("#" * 70)

    try:
        choice = input("\nEnter choice (1-7) or press Enter for ALL: ").strip()

        if choice == "1":
            test_system_info(device)
        elif choice == "2":
            test_power_commands(device)
        elif choice == "3":
            test_boundary_commands(device)
        elif choice == "4":
            test_input_commands(device)
        elif choice == "5":
            test_pico_specific(device)
        elif choice == "6":
            test_broadcast_commands(device)
        else:
            # Run all
            test_system_info(device)
            test_power_commands(device)
            test_boundary_commands(device)
            test_input_commands(device)
            test_pico_specific(device)
            test_broadcast_commands(device)

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")

    print("\n" + "="*70)
    print("TESTING COMPLETE")
    print("="*70)


if __name__ == "__main__":
    main()
