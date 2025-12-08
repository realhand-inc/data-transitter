# RealHand Desktop App Plan

Goal: Develop a new, standalone desktop application that provides the functionality of the existing `scripts/RH/adb_control_gui.py` and `scripts/RH/test_adb_simple.py`. This new application will be installable and launchable with a double-click, eventually allowing for the deprecation and removal of the `scripts/RH` folder.

## Section 1: Features
- Launch XRoboToolkit PC application and keep connectivity healthy.
- Receive XR data and send data to the robot computer via configurable IP/port.
- Provide ADB controls for two headsets (connect, start app, restart app) via UI buttons.
- Automate ADB-over-WiFi: detect USB device, switch to `adb tcpip 5555`, prompt/auto-use last known IP, run `adb connect {ip}:5555`, and refresh device list (show both USB and WiFi entries).

## Section 2: Data Display
- Show headset angle data (yaw, pitch, roll) with live updates.
- Surface headset connection status and runtime data/throughput.

## Section 3: Shutdown
- One-click shutdown button that stops data streams, disconnects ADB targets, and exits cleanly.

## Section 4: Logging
- Default-hidden log panel for system/messages/ADB actions; expandable on demand.

## UX Style
- Modern, minimal B2B (clean layout, strong grouping, restrained color, no heavy visual flair).

## Technical Blueprint
- Stack: Python 3.10+, CustomTkinter (modern, rounded UI replacement for standard Tkinter), matplotlib for charts, pyzmq for transport, stdlib logging, and adb invoked via subprocess.
- Packaging: PyInstaller per OS (one-folder for easier adb/xrt dependencies, optional onefile), with entrypoint `realhand_app/main.py` that wires GUI + background services; create platform-specific installers (.exe/.msi, .deb/.AppImage).
- Layout: `application/src/realhand_app/` split into `ui/` (widgets/screens), `services/` (xr, adb, zmq, watchdog), `core/` (config, logging, utils), `api/` (local HTTP control).
- Config: YAML in `~/.realhand/config.yaml` for robot IP/ports, ADB package name, default headset targets, log level; fall back to sane defaults and allow in-app editing.
- Startup checks: verify XRoboToolkit PC service is running, adb is available, and required ports are open; surface status in the UI header.

## Endpoints & Protocols to Implement
- Outbound headset stream: ZeroMQ PUB to `tcp://{robot_ip}:{port}` (multi-endpoint fan-out); payload `yaw_deg,pitch_deg,roll_deg,timestamp_ns` CSV at 10–30 Hz; reconnect on drop with backoff.
- Local control API (FastAPI/uvicorn on `localhost:5080` so the UI and automation scripts share one backend):
  - `GET /health` → process + XR/ADB status
  - `GET /devices/adb` → list connected + wireless targets
  - `POST /adb/connect` → `{device_id?|ip}` to pair/connect
  - `POST /adb/app/start` and `POST /adb/app/restart` → target all/selected devices
  - `POST /streams/start` → `{endpoints:[{"ip":"x","port":5555}]}` to bind PUB sockets
  - `POST /streams/stop` → stop ZMQ sockets and XR polling
  - `POST /shutdown` → stop streams, disconnect adb targets, close app
- Logging: structured logs to `~/.realhand/logs/realhand.log` with rotation; optional `GET /logs/tail?lines=N` for debugging from automation.

## Entrypoint (current)
- Development launcher: `python application/run_realhand_app.py` (double-click friendly) which bootstraps `realhand_app.launcher` and runs `scripts/RH/adb_control_gui.py`.
- Module entry: from repo root `PYTHONPATH=application/src python -m realhand_app` does the same import and call into `adb_control_gui.main()`.
- Linux double-click: make `application/run_realhand_app.sh` executable (`chmod +x application/run_realhand_app.sh`) and double-click it in your file manager; it prefers `venv/bin/python` if present, otherwise falls back to `python3`. If the terminal closes immediately, ensure you are in a graphical session (DISPLAY set) or run from a terminal to see the printed error.
