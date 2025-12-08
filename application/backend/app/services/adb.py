import os
import subprocess
import time
from typing import List, Tuple, Optional

ADB_PACKAGE_NAME = os.getenv("ADB_PACKAGE_NAME", "com.xrobotoolkit.client")
ADB_BIN = os.getenv("ADB_BIN", "adb")
DEFAULT_TIMEOUT = 5


def execute(cmd: str, timeout: int = DEFAULT_TIMEOUT) -> Tuple[bool, str]:
    """Run an adb command, returning (ok, stdout+stderr)."""
    try:
        result = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        output = (result.stdout or "") + (result.stderr or "")
        return result.returncode == 0, output
    except subprocess.TimeoutExpired:
        return False, "timeout"
    except Exception as exc:  # pragma: no cover
        return False, str(exc)


def get_connected_devices() -> List[str]:
    ok, output = execute(f"{ADB_BIN} devices")
    if not ok:
        return []
    devices: List[str] = []
    lines = output.strip().splitlines()
    if lines and "List of devices attached" in lines[0]:
        lines = lines[1:]
    for line in lines:
        line = line.strip()
        if not line:
            continue
        parts = line.split("\t")
        if len(parts) >= 2 and parts[1].strip() == "device":
            devices.append(parts[0].strip())
    return devices


def control_app(action: str, device: Optional[str] = None) -> Tuple[bool, str]:
    """
    Control app for a device. If device is None, apply to all connected.
    Returns (overall_ok, aggregated_output).
    """
    if device is None:
        devices = get_connected_devices()
        if not devices:
            return False, "no devices"
        outputs = []
        overall_ok = True
        for dev in devices:
            ok, out = control_app(action, device=dev)
            outputs.append(f"{dev}: {out.strip()}")
            overall_ok = overall_ok and ok
        return overall_ok, "\n".join(outputs)

    target = f"-s {device} "
    if action == "stop":
        cmd = f"{ADB_BIN} {target}shell am force-stop {ADB_PACKAGE_NAME}"
    elif action == "start":
        cmd = (
            f"{ADB_BIN} {target}shell monkey -p {ADB_PACKAGE_NAME} "
            "-c android.intent.category.LAUNCHER 1"
        )
    else:  # restart
        ok, out1 = control_app("stop", device=device)
        if not ok:
            return ok, out1
        time.sleep(0.5)  # small buffer to allow the stop to settle
        return control_app("start", device=device)
    return execute(cmd)
