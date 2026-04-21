# Development of a Marker-based Infrared Motion Capture System for Real-time Drone Localization and Control

This repository contains the implementation for a final year project focused on building a marker-based infrared motion capture pipeline for real-time drone localization and control.

This branch is the Betaflight flight-controller variant. Instead of running the full hover controller on an ESP-based drone board, the system now uses an ESP32 sender/receiver link to move mocap-derived control commands to a Betaflight-compatible flight controller.

## Project Overview

The project is built around a multi-camera infrared tracking workflow that detects active markers, reconstructs their 3D position, and uses that localization data to support closed-loop drone control. In this branch, the repository includes:

- camera calibration utilities for intrinsic and extrinsic setup
- a backend tracking server for marker detection, triangulation, pose estimation, logging, and serial transport
- a frontend dashboard for operator setup, activation, target selection, and PID tuning
- an `esp32-s3-sender` bridge that receives compact control frames over USB serial and forwards them over ESP-NOW
- an `esp32-s3-receiver` node that converts those mocap control payloads into CRSF RC frames for a Betaflight-compatible flight controller

## Control Flow

At a high level, the system works like this:

1. Multiple cameras observe infrared markers in the tracking space.
2. The backend loads calibration parameters, detects marker points, and estimates 3D pose from multi-view observations.
3. The frontend provides operator control for serial configuration, target values, stream state, and controller gains.
4. The backend sends compact serial frames to the USB-connected ESP32-S3 sender.
5. The sender forwards the command payload over ESP-NOW to the ESP32-S3 receiver on the vehicle.
6. The receiver runs the outer-loop mocap controller and emits CRSF-compatible channel data to the Betaflight flight controller over UART.

## Repository Structure

- `backend/`: tracking logic, calibration tools and assets, logs, and the main server in `backend/index.py`
- `frontend/`: Vite + React operator dashboard for setup, live monitoring, and tuning
- `esp32-s3-sender/`: ESP32-S3 sketch for USB serial reception and ESP-NOW forwarding
- `esp32-s3-receiver/`: ESP32-S3 sketch that receives mocap commands over ESP-NOW and outputs CRSF-style control frames to the flight controller

## Backend

The backend in `backend/index.py` uses OpenCV, NumPy, `pseyepy`, `websockets`, and `pyserial` to:

- load camera intrinsics and extrinsics from `backend/calibration/`
- verify that the configured cameras are connected before transmission is allowed
- detect LED marker points from the configured camera feeds
- estimate 3D pose from multi-view triangulation
- accept activation, serial settings, target inputs, limits, and PID values from the frontend over WebSocket
- maintain logging for motion, bridge, controller, and IMU-related telemetry
- stream compact serial frames to the ESP32-S3 sender only when camera and serial readiness checks pass

Calibration-related resources are stored under `backend/calibration/`, including scripts for image capture and intrinsic/extrinsic calibration.

## Frontend

The frontend is a Vite app in `frontend/` and connects to `ws://localhost:8765`.

Typical commands:

```bash
cd frontend
npm install
npm run dev
```

The dashboard is responsible for operator input and monitoring, including:

- serial port selection
- baud rate
- stream activation and motor arming
- target `x`, `y`, `z`, and `yaw` hover coordinates
- outer-loop PID parameters for `x`, `y`, `z`, and `yaw`
- inner-loop PID parameters for `roll`, `pitch`, and `yaw rate`
- hover throttle and attitude limits
- camera previews, pose plots, and readiness feedback
- logging and payload/debug visibility

The repository also includes `backend/data_logs/dashboard.html` for reviewing logged experiment data.

## ESP32 and Flight Controller Components

Flash `esp32-s3-sender/esp32-s3-sender.ino` to the USB-connected ESP32-S3 sender. It receives compact frames from the Python backend over USB serial and forwards the payload over ESP-NOW.

Flash `esp32-s3-receiver/esp32-s3-receiver.ino` to the vehicle-side ESP32-S3 receiver. It:

- receives the forwarded mocap control payload over ESP-NOW
- parses target, pose, limit, and PID data
- runs the outer-loop controller on the receiver side
- emits CRSF-style RC channel output to the flight controller over `Serial2`

Current receiver-side wiring/constants of note:

- `FC_RX_PIN = 7`
- `FC_TX_PIN = 6`
- `Serial2` is used for the flight-controller link

Setup notes:

- Open the `esp32-s3-receiver` serial monitor once after flashing and note the printed MAC address.
- Copy that MAC address into `DRONE_MAC_ADDRESS` in `esp32-s3-sender/esp32-s3-sender.ino`.
- Confirm the receiver-side CRSF scaling and angle/rate limits match the Betaflight configuration on the flight controller.
- ESP-NOW payloads in this implementation remain constrained by the standard ESP-NOW payload limit.

## Running the System

1. Install the required Python packages for the backend environment.
2. Start the backend from the repository root with `python backend/index.py`.
3. Start the frontend from `frontend/` with `npm run dev`.
4. Connect the ESP32-S3 sender to the computer over USB.
5. Power the ESP32-S3 receiver and wire its UART connection to the Betaflight flight controller.
6. Open the frontend, refresh serial ports, choose the sender COM port, enter controller values, and activate the stream.
7. The backend will only send payloads when the required cameras are connected and pose tracking is ready.

## Notes

- This README describes the Betaflight flight-controller branch, not the earlier ESP-drone control path.
- `frontend/node_modules/` is intentionally ignored and should not be committed.
- Calibration assets under `backend/calibration/` are currently tracked in the repository.
- This repository is structured as an implementation-focused project artifact, so it contains both development code and supporting calibration/logging assets.
