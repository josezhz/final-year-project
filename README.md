# Final Year Project

This repository contains the `MoCap` project, split into a Python backend and a React frontend.

## Structure

- `backend/`: motion-capture logic, camera calibration files, and the main tracking server in `index.py`
- `frontend/`: Vite + React client

## Backend

The backend in `backend/index.py` uses OpenCV, NumPy, `pseyepy`, and `websockets` to:

- load camera intrinsics and extrinsics from `backend/calibration/`
- detect LED points from connected cameras
- estimate 3D pose from multi-view triangulation
- serve pose updates over WebSocket on `ws://localhost:8765`

Install the required Python packages in your environment before running it.

## Frontend

The frontend is a Vite app in `frontend/`.

Typical commands:

```bash
cd frontend
npm install
npm run dev
```

## Notes

- `frontend/node_modules/` is intentionally ignored and should not be committed.
- Calibration assets under `backend/calibration/` are currently tracked in the repository.
