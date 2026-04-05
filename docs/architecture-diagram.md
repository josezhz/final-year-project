# Architecture Diagram

This diagram reflects the current implementation in the repository:

- the ground station runs the React frontend and Python backend on the host computer
- the current backend code is configured for **2 cameras**
- the ESP32-S3 acts as a USB-to-ESP-NOW relay, while the ESP32-S2 runs the onboard hover controller

```mermaid
flowchart TB
  classDef hardware fill:#FFF3D6,stroke:#C58A00,color:#2B1D00;
  classDef software fill:#DFF4FF,stroke:#0969DA,color:#0A3069;
  classDef embedded fill:#E8F5E9,stroke:#2DA44E,color:#0F5132;
  classDef artifact fill:#F6F8FA,stroke:#57606A,color:#24292F,stroke-dasharray: 5 3;

  subgraph GROUND["Ground Station"]
    direction TB
    HOST["Host PC / laptop"]
    CAMS["2 x PS Eye<br/>cameras"]
    FRONTEND["React frontend<br/>React + Vite<br/>@react-three/fiber + drei"]
    BACKEND["Python backend<br/>OpenCV + NumPy + pseyepy<br/>websockets + pyserial"]
    PIPELINE["Pose pipeline<br/>detect -> triangulate -> filter"]
    CAL["Calibration JSON<br/>intrinsics + extrinsics"]

    HOST -. hosts .-> FRONTEND
    HOST -. hosts .-> BACKEND
    CAMS -->|frames| PIPELINE
    CAL --> PIPELINE
    PIPELINE -->|pose + previews| BACKEND
    FRONTEND -->|WebSocket control| BACKEND
    BACKEND -->|WebSocket state| FRONTEND
  end

  subgraph RELAY["USB / Wireless Relay"]
    direction TB
    S3RELAY["ESP32-S3 relay<br/>board + firmware<br/>USB serial -> ESP-NOW"]
  end

  subgraph DRONE["Drone / Airframe"]
    direction TB
    FRAME["Drone frame<br/>3 LED markers"]
    S2CTRL["ESP32-S2 flight controller<br/>board + firmware<br/>parser + cascaded PID + motor mix"]
    IMU["MPU6050 IMU"]
    MOTORS["4 brushed motors"]

    IMU -->|I2C| S2CTRL
    S2CTRL -->|PWM| MOTORS
    MOTORS --> FRAME
  end

  BACKEND -->|USB serial| S3RELAY
  S3RELAY -->|ESP-NOW| S2CTRL
  FRAME -. observed LEDs .-> CAMS

  class HOST,CAMS,IMU,MOTORS,FRAME hardware;
  class FRONTEND,BACKEND,PIPELINE software;
  class S3RELAY,S2CTRL embedded;
  class CAL artifact;
```

## Component Notes

- `frontend/src/App.jsx`: browser-based operator console for stream control, arming, targets, PID tuning, telemetry, camera previews, and 3D scene view
- `backend/index.py`: WebSocket server, motion-capture pipeline, state broadcast, serial bridge, and compact control-payload builder
- `esp32-s3-sender/esp32-s3-sender.ino`: implementation behind the ESP32-S3 relay node that bridges USB serial to ESP-NOW
- `esp32-s2-drone/esp32-s2-drone.ino`: implementation behind the ESP32-S2 flight-controller node that parses commands, estimates attitude, and drives the motors

## Reuse

- GitHub will render the Mermaid block directly in this Markdown file
- the raw Mermaid source is available in `docs/architecture-diagram.mmd` if you want to export it as SVG or PNG for your final report
