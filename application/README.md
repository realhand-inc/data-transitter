# Application (FastAPI + React)

## Backend (FastAPI)
```
cd application/backend
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
uvicorn app.main:app --reload --port 8000
```
Env vars:
- `ADB_BIN` (default `adb`)
- `ADB_PACKAGE_NAME` (default `com.xrobotoolkit.client`)

## Frontend (React via Vite)
```
cd application/frontend
npm install
npm run dev     # dev server on :5173
npm run build   # outputs dist/ consumed by backend static mount
```
Optionally set `VITE_API_BASE` (default `http://localhost:8000`).

## Integrated serve
After `npm run build`, the backend will serve the static SPA from `application/frontend/dist` at `/` (API remains under `/adb/*` and `/xr/*`).

## Notes
- `/adb/*` endpoints wrap the legacy adb commands.
- `/xr/rotation-stream` emits JSON via SSE; `/xr/rotation-once` returns a single sample.
