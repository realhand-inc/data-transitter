import subprocess
import time
import re

# Define ADB commands for the application
ADB_PACKAGE_NAME = "com.xrobotoolkit.client"

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
