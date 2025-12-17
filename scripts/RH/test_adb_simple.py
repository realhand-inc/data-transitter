import subprocess
import time
import re
import platform
import os
import tempfile

# Define ADB commands for the application
ADB_PACKAGE_NAME = "com.xrobotoolkit.client"

# Constants for udev rules diagnostics
PICO_VENDOR_ID = "2d40"
UDEV_RULES_PATH = "/etc/udev/rules.d/51-android.rules"
UDEV_RULE_CONTENT = 'SUBSYSTEM=="usb", ATTR{idVendor}=="2d40", MODE="0666", GROUP="plugdev"'

def execute_adb_command(command_parts, timeout=5):
    """Helper to execute an ADB command and print its output."""
    # If command_parts is a string, split it for safer execution without shell=True
    if isinstance(command_parts, str):
        command_string = command_parts
        # Only use shell=True if the command contains shell-specific syntax
        use_shell = True # For now, keep it true for simplicity with adb shell commands
    else: # Assume it's a list
        command_string = " ".join(command_parts) # For printing
        use_shell = False # Prefer shell=False for security and clarity if possible

    print(f"\nExecuting command: '{command_string}'")
    try:
        if use_shell:
            result = subprocess.run(
                command_string,
                shell=True,
                capture_output=True,
                text=True,
                timeout=timeout
            )
        else: # For commands like ["adb", "-s", device_id, "devices"]
             result = subprocess.run(
                command_parts,
                capture_output=True,
                text=True,
                timeout=timeout
            )


        print(f"  Return Code: {result.returncode}")
        if result.stdout:
            print("  Stdout:")
            print(f"    {result.stdout}")
        if result.stderr:
            print("  Stderr:")
            print(f"    {result.stderr}")
        return result.returncode == 0, result.stdout
    except subprocess.TimeoutExpired:
        print(f"  Command timed out after {timeout} seconds.")
        return False, "Timeout"
    except Exception as e:
        print(f"  An exception occurred: {e}")
        return False, str(e)

def get_connected_adb_devices():
    """Gets a list of connected ADB device IDs."""
    success, output = execute_adb_command("adb devices")
    devices = []
    if success:
        lines = output.strip().split('\n')
        # Skip the header line "List of devices attached"
        if lines and "List of devices attached" in lines[0]:
            lines = lines[1:] # Skip the header
        
        for line in lines:
            line = line.strip()
            if not line: # Skip empty lines
                continue
            parts = line.split('\t') # Split by tab
            if len(parts) >= 2 and parts[1].strip() == "device":
                device_id = parts[0].strip()
                if device_id:
                    devices.append(device_id)
    return devices


# ===== Linux udev Rules Diagnostic Functions =====


def check_platform_is_linux():
    """Check if running on Linux."""
    return platform.system() == "Linux"


def get_usb_devices_by_vendor(vendor_id):
    """
    Get USB devices matching a vendor ID using lsusb.

    Args:
        vendor_id: Vendor ID in hex format (e.g., "2d40")

    Returns:
        List of dicts with keys: 'bus', 'device', 'vendor_id', 'product_id', 'description'
    """
    devices = []

    try:
        result = subprocess.run(
            ["lsusb"],
            capture_output=True,
            text=True,
            timeout=5
        )

        if result.returncode != 0:
            return devices

        # Parse lsusb output
        # Format: "Bus 001 Device 013: ID 2d40:00b5 Pico PICO 4 Ultra"
        lsusb_pattern = r"Bus (\d+) Device (\d+): ID ([0-9a-fA-F]{4}):([0-9a-fA-F]{4}) (.+)"

        for line in result.stdout.split('\n'):
            match = re.match(lsusb_pattern, line)
            if match:
                bus, device, vid, pid, description = match.groups()
                # Case-insensitive vendor ID comparison
                if vid.lower() == vendor_id.lower():
                    devices.append({
                        'bus': bus,
                        'device': device,
                        'vendor_id': vid,
                        'product_id': pid,
                        'description': description
                    })

    except FileNotFoundError:
        # lsusb not installed
        pass
    except Exception:
        # Other errors, return empty list
        pass

    return devices


def generate_udev_fix_commands():
    """
    Generate the exact shell commands needed to fix udev rules.

    Returns:
        List of command strings to execute sequentially
    """
    return [
        f"echo '{UDEV_RULE_CONTENT}' | sudo tee {UDEV_RULES_PATH}",
        "sudo udevadm control --reload-rules",
        "sudo udevadm trigger"
    ]


def diagnose_adb_udev_issue():
    """
    Diagnose potential udev rule issues for Pico devices.

    Returns:
        dict with keys:
            'platform': str ('linux' or 'other')
            'pico_usb_devices': list[dict] (from get_usb_devices_by_vendor)
            'adb_devices': list[str] (device IDs from get_connected_adb_devices)
            'issue_detected': bool (True if Pico in USB but not in ADB)
            'udev_rules_exist': bool (True if /etc/udev/rules.d/51-android.rules exists)
            'udev_rules_content': str (content of the file, or empty if not exists)
            'recommended_fix': str (instructions for the user)
    """
    result = {
        'platform': platform.system().lower(),
        'pico_usb_devices': [],
        'adb_devices': [],
        'issue_detected': False,
        'udev_rules_exist': False,
        'udev_rules_content': '',
        'recommended_fix': ''
    }

    # Check if running on Linux
    if not check_platform_is_linux():
        return result

    # Get Pico USB devices
    result['pico_usb_devices'] = get_usb_devices_by_vendor(PICO_VENDOR_ID)

    # Get ADB devices
    result['adb_devices'] = get_connected_adb_devices()

    # Check if udev rules file exists
    if os.path.exists(UDEV_RULES_PATH):
        result['udev_rules_exist'] = True
        try:
            with open(UDEV_RULES_PATH, 'r') as f:
                result['udev_rules_content'] = f.read()
        except Exception:
            pass

    # Determine if issue exists
    result['issue_detected'] = len(result['pico_usb_devices']) > 0 and len(result['adb_devices']) == 0

    # Generate recommended fix instructions
    if result['issue_detected']:
        fix_commands = generate_udev_fix_commands()
        result['recommended_fix'] = """Run these commands in terminal:

1. Create/update udev rule:
   {}

2. Reload udev rules:
   {}

3. Trigger udev:
   {}

4. Unplug and replug your device

5. Refresh device list in this app
""".format(fix_commands[0], fix_commands[1], fix_commands[2])

    return result


def execute_udev_fix_with_sudo():
    """
    Execute the udev fix commands using pkexec for GUI sudo prompt.

    Returns:
        (success: bool, message: str)
    """
    # Check if pkexec is available
    try:
        subprocess.run(["which", "pkexec"], capture_output=True, check=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False, "pkexec not available. Please apply the fix manually using the instructions above."

    # Create a temporary shell script with the fix commands
    try:
        with tempfile.NamedTemporaryFile(mode='w', suffix='.sh', delete=False) as f:
            script_path = f.name
            f.write("#!/bin/bash\n")
            f.write("set -e\n")  # Exit on error
            f.write(f"echo '{UDEV_RULE_CONTENT}' > {UDEV_RULES_PATH}\n")
            f.write("udevadm control --reload-rules\n")
            f.write("udevadm trigger\n")
            f.write("echo 'udev rules updated successfully'\n")

        # Make script executable
        os.chmod(script_path, 0o755)

        # Execute with pkexec
        result = subprocess.run(
            ["pkexec", "sh", script_path],
            capture_output=True,
            text=True,
            timeout=60
        )

        # Clean up
        try:
            os.unlink(script_path)
        except Exception:
            pass

        # Check result
        if result.returncode == 0:
            return True, "udev rules updated successfully!\n\nNext steps:\n1. Unplug and replug your device\n2. Run 'adb kill-server && adb start-server'\n3. Refresh device list"
        elif result.returncode in [126, 127]:
            return False, "Operation cancelled by user"
        else:
            error_msg = result.stderr if result.stderr else "Unknown error"
            return False, f"Failed to update udev rules: {error_msg}"

    except subprocess.TimeoutExpired:
        return False, "Operation timed out"
    except Exception as e:
        return False, f"Error: {str(e)}"


def test_adb():
    print("--- Testing ADB Command Execution ---")

    # 1. Get connected devices
    connected_devices = get_connected_adb_devices()
    print(f"Found ADB devices: {connected_devices}")

    if not connected_devices:
        print("No ADB devices found. Please ensure a device is connected and ADB is authorized.")
        return

    print(f"Found {len(connected_devices)} device(s).Executing commands on ALL connected devices...")

    for target_device in connected_devices:
        print(f"\n>>> Targeting device: {target_device} <<<")

        # Commands with target device
        ADB_STOP_APP = f"adb -s {target_device} shell am force-stop {ADB_PACKAGE_NAME}"
        ADB_OPEN_APP = f"adb -s {target_device} shell monkey -p {ADB_PACKAGE_NAME} -c android.intent.category.LAUNCHER 1"
            
        # 2. Test stopping the application
        print(f"[{target_device}] Attempting to force-stop the application...")
        execute_adb_command(ADB_STOP_APP)
        time.sleep(1) # Give it some time to stop

        # 3. Test running the application
        print(f"[{target_device}] Attempting to open the application...")
        execute_adb_command(ADB_OPEN_APP)
        time.sleep(2) # Give it some time to launch

    print("\n--- ADB Command Testing Complete ---")

if __name__ == "__main__":
    test_adb()
