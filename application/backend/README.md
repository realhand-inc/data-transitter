# FastAPI backend scaffold

## Setup

```bash
cd backend
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
uvicorn app.main:app --reload --port 8000
```

### Env
- `ADB_BIN` (default `adb`)
- `ADB_PACKAGE_NAME` (default `com.xrobotoolkit.client`)

## Endpoints
- `GET /health`
- `GET /adb/devices`
- `POST /adb/connect?ip=HOST:PORT`
- `POST /adb/disconnect` (optional `device` query)
- `POST /adb/app/{start|stop|restart}` (optional `device` query)
- `GET /xr/rotation-once`
- `GET /xr/rotation-stream` (SSE)
