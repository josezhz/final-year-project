# Development of a Marker-based Infrared Motion Capture System for Real-time Drone Localization and Control

This repository contains the implementation for a final year project focused on building a marker-based infrared motion capture pipeline for real-time drone localization and control. The system combines a Python tracking backend, a React operator interface, and ESP32-based wireless communication/control modules to support pose estimation, control tuning, and onboard actuation.

## Project Overview

The project is built around a multi-camera infrared tracking workflow that detects active markers, reconstructs their 3D position, and uses that localization data to support closed-loop drone control. In its current repository form, the system includes:

- camera calibration utilities for intrinsic and extrinsic setup
- a backend tracking server for marker detection, triangulation, and pose estimation
- a frontend dashboard for serial setup, activation, and PID tuning
- an ESP32-S3 relay that bridges USB serial data to ESP-NOW
- an ESP32-S2 drone-side receiver that reads IMU data and runs a hover-oriented control loop

## Architecture

![System architecture diagram](architecture.png)

At a high level, the workflow is:

1. Multiple cameras observe infrared markers in the tracking space.
2. The backend loads calibration parameters, detects marker points, and estimates 3D pose from multi-view observations.
3. The frontend provides operator control for serial configuration, target values, and controller gains.
4. The backend sends compact control payloads to an ESP32-S3 over USB serial.
5. The ESP32-S3 forwards the payload over ESP-NOW to the ESP32-S2 on the drone.
6. The ESP32-S2 estimates attitude from the onboard MPU6050 and applies the control loop to the motors.

## Repository Structure

- `backend/`: tracking logic, calibration tools and assets, logs, and the main server in `backend/index.py`
- `frontend/`: Vite + React operator dashboard for activation, serial selection, and controller input
- `esp32-s3-sender/`: ESP32-S3 sketch for USB serial reception and ESP-NOW forwarding
- `esp32-s2-drone/`: ESP32-S2 sketch for IMU-based attitude estimation and motor control

## Backend

The backend in `backend/index.py` uses OpenCV, NumPy, `pseyepy`, `websockets`, and `pyserial` to:

- load camera intrinsics and extrinsics from `backend/calibration/`
- verify that the configured cameras are connected before transmission is allowed
- detect LED marker points from the configured camera feeds
- estimate 3D pose from multi-view triangulation
- accept activation, serial settings, target inputs, and PID values from the frontend over WebSocket
- stream `droneIndex + compact JSON + newline` frames to the ESP32-S3 only when camera and serial readiness checks pass

Calibration-related resources are stored under `backend/calibration/`, including scripts for image capture and intrinsic/extrinsic calibration.

## Frontend

The frontend is a Vite app in `frontend/` and connects to `ws://localhost:8765`.

Typical commands:

```bash
cd frontend
npm install
npm run dev
```

The dashboard is responsible for operator input, including:

- serial port selection
- baud rate
- stream activation and motor arming
- target `x`, `y`, `z`, and `yaw` hover coordinates
- outer-loop PID parameters for `x`, `y`, `z`, and `yaw`
- inner-loop PID parameters for `roll`, `pitch`, and `yaw rate`
- hover throttle and attitude limits

The repository also includes `backend/data_logs/imu_dashboard.html` for reviewing logged IMU data.

## Embedded Components

Flash `esp32-s3-sender/esp32-s3-sender.ino` to the ESP32-S3. It receives `droneIndex + JSON + newline` frames from the Python backend over USB serial, strips the leading index byte, and forwards the JSON payload to the matching ESP32-S2 peer over ESP-NOW.

Flash `esp32-s2-drone/esp32-s2-drone.ino` to the ESP32-S2. It receives ESP-NOW messages, estimates attitude from the onboard MPU6050, and runs a hover-oriented control loop for the brushed motor outputs.

Setup notes:

- Open the ESP32-S2 serial monitor once after flashing and note the printed station MAC address.
- Copy that MAC address into `DRONE_MAC_ADDRESSES` in `esp32-s3-sender/esp32-s3-sender.ino`.
- Index `0` in `DRONE_MAC_ADDRESSES` currently matches the backend's default `droneIndex` value.
- ESP-NOW payloads in this implementation are capped at the official 250-byte ESP-NOW limit.

## Running the System

1. Install the required Python packages for the backend environment.
2. Start the backend from the repository root with `python backend/index.py`.
3. Start the frontend from `frontend/` with `npm run dev`.
4. Connect the ESP32-S3 to the computer over USB.
5. Open the frontend, refresh serial ports, choose the ESP32-S3 COM port, enter controller values, and activate the stream.
6. The backend will only send payloads when the required cameras are connected and pose tracking is ready.

## Notes

- `frontend/node_modules/` is intentionally ignored and should not be committed.
- Calibration assets under `backend/calibration/` are currently tracked in the repository.
- This repository is structured as an implementation-focused project artifact for the final year project, so it contains both development code and supporting calibration/logging assets.
