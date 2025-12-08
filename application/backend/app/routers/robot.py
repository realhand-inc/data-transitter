import threading
import time
from typing import Optional, Set, Dict

import numpy as np
import zmq
from fastapi import APIRouter, HTTPException

try:
    from xrobotoolkit_teleop.common.xr_client import XrClient
except Exception:  # pragma: no cover
    XrClient = None

router = APIRouter()

ROTATION_ENDPOINTS: Set[str] = set()
_forward_thread: Optional[threading.Thread] = None
_forward_stop = threading.Event()
_socket_map: Dict[str, zmq.Socket] = {}
_lock = threading.Lock()


def quaternion_to_euler(q):
    x, y, z, w = q
    look_y = 2.0 * (y * z - w * x)
    pitch = -np.arcsin(np.clip(look_y, -1.0, 1.0))
    sin_yaw = 2.0 * (w * y - z * x)
    cos_yaw = 1.0 - 2.0 * (y * y + x * x)
    yaw = -np.arctan2(sin_yaw, cos_yaw)
    sin_roll = 2.0 * (w * z + x * y)
    roll = -np.arcsin(np.clip(sin_roll, -1.0, 1.0))
    return np.array([yaw, pitch, roll])


def _ensure_socket(endpoint: str, ctx: zmq.Context) -> zmq.Socket:
    if endpoint in _socket_map:
        return _socket_map[endpoint]
    sock = ctx.socket(zmq.PUSH)
    sock.connect(f"tcp://{endpoint}")
    _socket_map[endpoint] = sock
    return sock


def _close_removed_sockets(endpoints: Set[str]):
    for ep in list(_socket_map.keys()):
        if ep not in endpoints:
            try:
                _socket_map[ep].close()
            finally:
                _socket_map.pop(ep, None)


def _forward_loop(interval_ms: int):
    if XrClient is None:
        return
    client = XrClient()
    ctx = zmq.Context.instance()
    try:
        while not _forward_stop.is_set():
            pose = client.get_pose_by_name("headset")
            if pose is not None:
                quat = pose[3:]
                euler = quaternion_to_euler(quat) * 180.0 / np.pi
                payload = f"{euler[0]:.2f}, {euler[1]:.2f}, {euler[2]:.2f}, {time.time():.6f}"
                with _lock:
                    eps = set(ROTATION_ENDPOINTS)
                _close_removed_sockets(eps)
                for ep in eps:
                    try:
                        sock = _ensure_socket(ep, ctx)
                        sock.send_string(payload, flags=zmq.NOBLOCK)
                    except Exception:
                        pass
            time.sleep(max(interval_ms, 20) / 1000.0)
    finally:
        client.close()
        _close_removed_sockets(set())


@router.get("/endpoints")
def list_endpoints():
    with _lock:
        return sorted(ROTATION_ENDPOINTS)


@router.post("/endpoints")
def add_endpoint(endpoint: str):
    if ":" not in endpoint:
        raise HTTPException(status_code=400, detail="endpoint must be ip:port")
    with _lock:
        ROTATION_ENDPOINTS.add(endpoint)
    return list_endpoints()


@router.delete("/endpoints")
def remove_endpoint(endpoint: str):
    with _lock:
        ROTATION_ENDPOINTS.discard(endpoint)
    return list_endpoints()


@router.post("/start-forward")
def start_forward(interval_ms: int = 100):
    global _forward_thread
    if XrClient is None:
        raise HTTPException(status_code=500, detail="XrClient unavailable")
    with _lock:
        if not ROTATION_ENDPOINTS:
            raise HTTPException(status_code=400, detail="no endpoints configured")
    if _forward_thread and _forward_thread.is_alive():
        return {"status": "running"}
    _forward_stop.clear()
    _forward_thread = threading.Thread(target=_forward_loop, args=(interval_ms,), daemon=True)
    _forward_thread.start()
    return {"status": "started"}


@router.post("/stop-forward")
def stop_forward():
    global _forward_thread
    if not _forward_thread:
        return {"status": "stopped"}
    _forward_stop.set()
    _forward_thread.join(timeout=2)
    _forward_thread = None
    _close_removed_sockets(set())
    return {"status": "stopped"}


@router.post("/send-rotation")
def send_rotation(endpoint: str):
    if ":" not in endpoint:
        raise HTTPException(status_code=400, detail="endpoint must be ip:port")
    if XrClient is None:
        raise HTTPException(status_code=500, detail="XrClient unavailable")

    client = XrClient()
    pose = client.get_pose_by_name("headset")
    client.close()
    if pose is None:
        raise HTTPException(status_code=404, detail="no headset pose")

    quat = pose[3:]
    euler = quaternion_to_euler(quat) * 180.0 / np.pi
    payload = f"{euler[0]:.2f}, {euler[1]:.2f}, {euler[2]:.2f}, {time.time():.6f}"

    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.PUSH)
    try:
        sock.connect(f"tcp://{endpoint}")
        sock.send_string(payload)
    finally:
        sock.close()
    return {"status": "sent", "endpoint": endpoint, "payload": payload}
