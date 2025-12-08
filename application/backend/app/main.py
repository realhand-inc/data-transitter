from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from app.routers import adb, xr, robot

app = FastAPI(title="XRoboToolkit Teleop API", version="0.1.0")

# Allow frontend dev server
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(adb.router, prefix="/adb", tags=["adb"])
app.include_router(xr.router, prefix="/xr", tags=["xr"])
app.include_router(robot.router, prefix="/robot", tags=["robot"])

# Serve built frontend if present (application/frontend/dist)
project_root = Path(__file__).resolve().parents[2]
dist_dir = project_root / "frontend" / "dist"
if dist_dir.exists():
    app.mount("/", StaticFiles(directory=dist_dir, html=True), name="static")


@app.get("/health")
def health():
    return {"status": "ok"}
