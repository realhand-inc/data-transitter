from typing import List, Optional

from fastapi import APIRouter, HTTPException

from app.services import adb as adb_service

router = APIRouter()


@router.get("/devices")
def list_devices() -> List[str]:
    return adb_service.get_connected_devices()


@router.post("/connect")
def connect_device(ip: str):
    ok, output = adb_service.execute(f"adb connect {ip}")
    if not ok:
        raise HTTPException(status_code=500, detail=output.strip())
    return {"status": "connected", "output": output}


@router.post("/disconnect")
def disconnect(device: Optional[str] = None):
    target = f"-s {device} " if device else ""
    ok, output = adb_service.execute(f"adb {target}disconnect")
    if not ok:
        raise HTTPException(status_code=500, detail=output.strip())
    return {"status": "disconnected", "output": output}


@router.post("/app/{action}")
def control_app(action: str, device: Optional[str] = None):
    if action not in {"start", "stop", "restart"}:
        raise HTTPException(status_code=400, detail="action must be start|stop|restart")

    ok, output = adb_service.control_app(action, device=device)
    if not ok:
        raise HTTPException(status_code=500, detail=output.strip())
    return {"status": action, "output": output}
