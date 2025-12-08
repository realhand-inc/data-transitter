# React frontend scaffold

## Setup

```bash
cd frontend
npm install
npm run dev
```

Environment:
- `VITE_API_BASE` defaults to `http://localhost:8000`.

Current views:
- Device controls: refresh, connect over IP, disconnect all, start/stop/restart app.
- Rotation chart: subscribes to `GET /xr/rotation-stream` SSE.
- Simple log of actions.

Wire the backend FastAPI server from `../backend` before running the dev server.
