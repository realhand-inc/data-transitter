import json
import time
from typing import Optional

import numpy as np
from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse

try:
    from xrobotoolkit_teleop.common.xr_client import XrClient
except Exception:  # pragma: no cover - allow import failures when not installed
    XrClient = None

router = APIRouter()


def quaternion_to_euler(q):
    x, y, z, w = q
    look_x = 2.0 * (x * z + w * y)
    look_y = 2.0 * (y * z - w * x)
    look_z = 1.0 - 2.0 * (x * x + y * y)
    pitch = -np.arcsin(np.clip(look_y, -1.0, 1.0))
    sin_yaw = 2.0 * (w * y - z * x)
    cos_yaw = 1.0 - 2.0 * (y * y + x * x)
    yaw = -np.arctan2(sin_yaw, cos_yaw)
    sin_roll = 2.0 * (w * z + x * y)
    roll = -np.arcsin(np.clip(sin_roll, -1.0, 1.0))
    return np.array([yaw, pitch, roll])


@router.get("/rotation-stream")
def rotation_stream(interval_ms: int = 100):
    if XrClient is None:
        raise HTTPException(status_code=500, detail="XrClient unavailable")

    client = XrClient()

    def gen():
        try:
            while True:
                pose = client.get_pose_by_name("headset")
                if pose is not None:
                    quat = pose[3:]
                    euler = quaternion_to_euler(quat) * 180.0 / np.pi
                    payload = {
                        "yaw": float(euler[0]),
                        "pitch": float(euler[1]),
                        "roll": float(euler[2]),
                        "timestamp": time.time(),
                    }
                    # Emit JSON for SSE consumers (frontend parses JSON.parse)
                    yield f"data: {json.dumps(payload)}\n\n"
                time.sleep(max(interval_ms, 20) / 1000.0)
        finally:
            client.close()

    return StreamingResponse(gen(), media_type="text/event-stream")


@router.get("/rotation-once")
def rotation_once():
    if XrClient is None:
        raise HTTPException(status_code=500, detail="XrClient unavailable")
    client = XrClient()
    pose = client.get_pose_by_name("headset")
    client.close()
    if pose is None:
        raise HTTPException(status_code=404, detail="no headset pose")
    quat = pose[3:]
    euler = quaternion_to_euler(quat) * 180.0 / np.pi
    return {
        "yaw": float(euler[0]),
        "pitch": float(euler[1]),
        "roll": float(euler[2]),
        "timestamp": time.time(),
    }
