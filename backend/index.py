import asyncio
import base64
import copy
import csv
import faulthandler
import itertools
import json
import math
import sys
import time
import traceback
from dataclasses import dataclass, field
from datetime import datetime
from http import HTTPStatus
from pathlib import Path

import cv2 as cv
import numpy as np
import websockets
from pseyepy import Camera
from websockets.datastructures import Headers
from websockets.http11 import Response

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


faulthandler.enable(all_threads=True)


BASE_DIR = Path(__file__).resolve().parent
INTRINSICS_PATH = BASE_DIR / "calibration" / "camera_intrinsics.json"
EXTRINSICS_PATH = BASE_DIR / "calibration" / "camera_extrinsics.json"
DATA_LOG_DIR = BASE_DIR / "data_logs"

HOST = "localhost"
PORT = 8765
EXPECTED_CAMERAS = 2
MIN_BRIGHTNESS = 50
MAX_LEDS = 3
MAX_FIT_ERROR = 0.05
TRACKING_HZ = 60
SERIAL_REFRESH_SECONDS = 1.0
SERIAL_RECONNECT_SECONDS = 1.0
SERIAL_SETTLE_SECONDS = 2.0
SERIAL_FRAME_TERMINATOR = b"\n"
SERIAL_MAX_CONSECUTIVE_FAILURES = 3
SERIAL_IMU_STALE_SECONDS = 0.5
CAMERA_RETRY_SECONDS = 5.0
DEFAULT_BAUD_RATE = 1000000
DEFAULT_DRONE_INDEX = 0
POSITION_JUMP_WEIGHT = 2.0
PREVIEW_HZ = 5
MAX_ESP_NOW_PAYLOAD_BYTES = 250
IMU_LEVEL_CALIBRATION_RETRY_SECONDS = 1.0
MOTION_STATE_PROCESS_NOISE = 1e-2
MOTION_STATE_MEASUREMENT_NOISE = 1.0
MOTION_STATE_ZERO_THRESHOLD = 0.01
MOTION_STATE_MAX_DT = 0.25
MOTION_STATE_MAX_ABS_VELOCITY = 2.5

CONTROL_PID_DEFAULTS = {
    "xyPos": {"kp": 0.0, "ki": 0.0, "kd": 0.0},
    "zPos": {"kp": 1.5, "ki": 0.0, "kd": 0.0},
    "yawPos": {"kp": 0.06, "ki": 0.0, "kd": 0.0},
    "xyVel": {"kp": 0.0, "ki": 0.0, "kd": 0.0},
    "zVel": {"kp": 0.18, "ki": 0.0, "kd": 0.02},
    "roll": {"kp": 0.008, "ki": 0.0, "kd": 0.0006},
    "pitch": {"kp": 0.008, "ki": 0.0, "kd": 0.0006},
    "yawRate": {"kp": 0.006, "ki": 0.0, "kd": 0.0},
}
# Mocap/body frame is +x front, +y left, +z up. A yaw target of 0 deg means
# the drone's nose should stay aligned with the world +x direction.
CONTROL_TARGET_DEFAULTS = {"x": 0.0, "y": 0.0, "z": 0.35, "yaw": 0.0}
CONTROL_LIMIT_DEFAULTS = {
    "hoverThrottle": 0.65,
    "minThrottle": 0.30,
    "maxThrottle": 0.95,
    "maxTiltDeg": 3.0,
    "maxYawRateDeg": 25.0,
}


# Body model uses the same mocap frame: +x front, +y left, +z up.
DRONE_LED_MODEL = np.array(
    [
        [0.050, 0.000, 0.025],   # Front
        [0.000, -0.050, 0.025],  # Back-Right
        [0.000, 0.050, 0.025],   # Back-Left
    ],
    dtype=np.float32,
)

DEFAULT_IMU_TELEMETRY = {
    "ready": False,
    "pitch": 0.0,
    "roll": 0.0,
    "pitch_rate": 0.0,
    "roll_rate": 0.0,
}

DEFAULT_TELEMETRY = {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
    "rotation": {"yaw": 0.0, "pitch": 0.0, "roll": 0.0},
    "imu": DEFAULT_IMU_TELEMETRY,
    "error": 0.0,
    "mapping_error_px": 0.0,
    "model_fit_error_m": 0.0,
    "scale_factor": 1.0,
    "solved_led_coordinates": [],
    "detected_leds_per_camera": [0, 0],
    "spatial_data_valid": False,
}
DEFAULT_VISION_METRICS = {
    "frames_processed": 0,
    "valid_pose_frames": 0,
    "all_markers_detected_frames": 0,
    "per_camera_full_detection_frames": [0 for _ in range(EXPECTED_CAMERAS)],
    "pose_success_rate": 0.0,
    "full_marker_detection_rate": 0.0,
    "per_camera_full_detection_rate": [0.0 for _ in range(EXPECTED_CAMERAS)],
    "avg_mapping_error_px": 0.0,
    "avg_model_fit_error_m": 0.0,
    "tracking_loop_hz": 0.0,
}
DEFAULT_BRIDGE_METRICS = {
    "serial_frames_rx": 0,
    "espnow_tx_attempts": 0,
    "espnow_tx_ok": 0,
    "espnow_tx_fail": 0,
    "oversize_drops": 0,
    "tx_rate_hz": 0.0,
    "last_tx_seq": 0,
    "last_tx_latency_us": 0,
    "counter_reset_sequence": 0,
}
DEFAULT_CONTROLLER_METRICS = {
    "rx_packets": 0,
    "rx_missing_packets": 0,
    "rx_duplicate_packets": 0,
    "rx_out_of_order_packets": 0,
    "last_rx_seq": 0,
    "loop_hz": 0.0,
    "control_hz": 0.0,
    "command_age_ms": 0,
    "apply_latency_us": 0,
    "motor_imbalance": 0.0,
    "motor_outputs": [0.0, 0.0, 0.0, 0.0],
    "counter_reset_sequence": 0,
}
IMU_LOG_COLUMNS = [
    "imu_ready",
    "imu_pitch_deg",
    "imu_roll_deg",
    "imu_pitch_rate_deg_s",
    "imu_roll_rate_deg_s",
    "mocap_valid",
    "mocap_x_error_m",
    "mocap_y_error_m",
    "mocap_z_error_m",
    "mocap_vx_m_s",
    "mocap_vy_m_s",
    "mocap_vz_m_s",
    "mocap_yaw_error_deg",
    "mocap_yaw_rate_deg_s",
]
PREVIEW_WIDTH = 240
PREVIEW_JPEG_QUALITY = 45
MAX_LED_REORDER_COST_PIXELS = 75
MAPPING_TEMPORAL_WEIGHT = 120.0
SEMANTIC_ROTATION_TEMPORAL_WEIGHT = 1e-4


def default_imu_telemetry():
    return copy.deepcopy(DEFAULT_IMU_TELEMETRY)


def default_telemetry():
    return copy.deepcopy(DEFAULT_TELEMETRY)


def default_vision_metrics():
    return copy.deepcopy(DEFAULT_VISION_METRICS)


def default_bridge_metrics():
    return copy.deepcopy(DEFAULT_BRIDGE_METRICS)


def default_controller_metrics():
    return copy.deepcopy(DEFAULT_CONTROLLER_METRICS)


def default_pid_config():
    return {
        axis: {term: float(value) for term, value in terms.items()}
        for axis, terms in CONTROL_PID_DEFAULTS.items()
    }


def default_target_config():
    return {axis: float(value) for axis, value in CONTROL_TARGET_DEFAULTS.items()}


def default_limit_config():
    return {axis: float(value) for axis, value in CONTROL_LIMIT_DEFAULTS.items()}


def coerce_float(value, default=0.0):
    try:
        if value is None:
            raise ValueError
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def coerce_int(value, default=0):
    try:
        if value is None:
            raise ValueError
        return int(value)
    except (TypeError, ValueError):
        return int(default)


def sanitize_imu_telemetry(payload):
    payload = payload if isinstance(payload, dict) else {}
    return {
        "ready": bool(payload.get("ready", False)),
        "pitch": round(coerce_float(payload.get("pitch"), 0.0), 2),
        "roll": round(coerce_float(payload.get("roll"), 0.0), 2),
        "pitch_rate": round(coerce_float(payload.get("pitch_rate"), 0.0), 2),
        "roll_rate": round(coerce_float(payload.get("roll_rate"), 0.0), 2),
    }


def sanitize_bridge_metrics(payload):
    payload = payload if isinstance(payload, dict) else {}
    return {
        "serial_frames_rx": coerce_int(payload.get("serial_frames_rx", payload.get("sr")), 0),
        "espnow_tx_attempts": coerce_int(
            payload.get("espnow_tx_attempts", payload.get("ea")), 0
        ),
        "espnow_tx_ok": coerce_int(payload.get("espnow_tx_ok", payload.get("eo")), 0),
        "espnow_tx_fail": coerce_int(payload.get("espnow_tx_fail", payload.get("ef")), 0),
        "oversize_drops": coerce_int(payload.get("oversize_drops", payload.get("od")), 0),
        "tx_rate_hz": round(
            coerce_float(payload.get("tx_rate_hz", payload.get("thz")), 0.0), 2
        ),
        "last_tx_seq": coerce_int(payload.get("last_tx_seq", payload.get("sq")), 0),
        "last_tx_latency_us": coerce_int(
            payload.get("last_tx_latency_us", payload.get("lu")), 0
        ),
        "counter_reset_sequence": coerce_int(
            payload.get("counter_reset_sequence", payload.get("rq")),
            0,
        ),
    }


def sanitize_controller_metrics(payload):
    payload = payload if isinstance(payload, dict) else {}
    raw_motor_outputs = payload.get("motor_outputs", payload.get("m", []))
    if not isinstance(raw_motor_outputs, list):
        raw_motor_outputs = []
    motor_outputs = [
        round(coerce_float(raw_motor_outputs[index], 0.0), 4)
        if index < len(raw_motor_outputs)
        else 0.0
        for index in range(4)
    ]
    return {
        "rx_packets": coerce_int(payload.get("rx_packets", payload.get("rp")), 0),
        "rx_missing_packets": coerce_int(
            payload.get("rx_missing_packets", payload.get("mg")), 0
        ),
        "rx_duplicate_packets": coerce_int(
            payload.get("rx_duplicate_packets", payload.get("dp")), 0
        ),
        "rx_out_of_order_packets": coerce_int(
            payload.get("rx_out_of_order_packets", payload.get("oo")), 0
        ),
        "last_rx_seq": coerce_int(payload.get("last_rx_seq", payload.get("sq")), 0),
        "loop_hz": round(coerce_float(payload.get("loop_hz", payload.get("lh")), 0.0), 2),
        "control_hz": round(
            coerce_float(payload.get("control_hz", payload.get("ch")), 0.0), 2
        ),
        "command_age_ms": coerce_int(
            payload.get("command_age_ms", payload.get("ca")), 0
        ),
        "apply_latency_us": coerce_int(
            payload.get("apply_latency_us", payload.get("au")), 0
        ),
        "motor_imbalance": round(
            coerce_float(payload.get("motor_imbalance", payload.get("mi")), 0.0), 4
        ),
        "motor_outputs": motor_outputs,
        "counter_reset_sequence": coerce_int(
            payload.get("counter_reset_sequence", payload.get("rq")),
            0,
        ),
    }


def merge_telemetry(base_telemetry=None, imu_telemetry=None):
    merged = default_telemetry()
    if isinstance(base_telemetry, dict):
        merged.update(copy.deepcopy(base_telemetry))
    merged["imu"] = sanitize_imu_telemetry(imu_telemetry)
    return merged


def build_imu_log_values(sample=None, mocap_sample=None):
    sample = sanitize_imu_telemetry(sample)
    mocap_sample = mocap_sample if isinstance(mocap_sample, dict) else {}
    return [
        int(bool(sample.get("ready", False))),
        f"{coerce_float(sample.get('pitch'), 0.0):.2f}",
        f"{coerce_float(sample.get('roll'), 0.0):.2f}",
        f"{coerce_float(sample.get('pitch_rate'), 0.0):.2f}",
        f"{coerce_float(sample.get('roll_rate'), 0.0):.2f}",
        int(bool(mocap_sample.get("valid", False))),
        f"{coerce_float(mocap_sample.get('x_error'), 0.0):.4f}",
        f"{coerce_float(mocap_sample.get('y_error'), 0.0):.4f}",
        f"{coerce_float(mocap_sample.get('z_error'), 0.0):.4f}",
        f"{coerce_float(mocap_sample.get('vx'), 0.0):.4f}",
        f"{coerce_float(mocap_sample.get('vy'), 0.0):.4f}",
        f"{coerce_float(mocap_sample.get('vz'), 0.0):.4f}",
        f"{coerce_float(mocap_sample.get('yaw_error'), 0.0):.2f}",
        f"{coerce_float(mocap_sample.get('yaw_rate'), 0.0):.2f}",
    ]


def build_vision_metrics_summary(
    frames_processed,
    valid_pose_frames,
    all_markers_detected_frames,
    per_camera_full_detection_frames,
    tracking_loop_hz=0.0,
    mapping_error_sum=0.0,
    model_fit_error_sum=0.0,
):
    frames_processed = max(0, coerce_int(frames_processed, 0))
    valid_pose_frames = max(0, coerce_int(valid_pose_frames, 0))
    all_markers_detected_frames = max(0, coerce_int(all_markers_detected_frames, 0))
    raw_per_camera = (
        per_camera_full_detection_frames
        if isinstance(per_camera_full_detection_frames, list)
        else []
    )
    per_camera_counts = [
        max(
            0,
            coerce_int(raw_per_camera[index] if index < len(raw_per_camera) else 0, 0),
        )
        for index in range(EXPECTED_CAMERAS)
    ]

    total_frames = max(1, frames_processed)
    valid_frames = max(1, valid_pose_frames)
    return {
        "frames_processed": frames_processed,
        "valid_pose_frames": valid_pose_frames,
        "all_markers_detected_frames": all_markers_detected_frames,
        "per_camera_full_detection_frames": per_camera_counts,
        "pose_success_rate": round(valid_pose_frames / total_frames, 5),
        "full_marker_detection_rate": round(
            all_markers_detected_frames / total_frames,
            5,
        ),
        "per_camera_full_detection_rate": [
            round(success_frames / total_frames, 5)
            for success_frames in per_camera_counts
        ],
        "avg_mapping_error_px": round(
            coerce_float(mapping_error_sum, 0.0) / valid_frames,
            5,
        ),
        "avg_model_fit_error_m": round(
            coerce_float(model_fit_error_sum, 0.0) / valid_frames,
            5,
        ),
        "tracking_loop_hz": round(coerce_float(tracking_loop_hz, 0.0), 2),
    }


def build_session_bridge_metrics(
    current_metrics,
    baseline_metrics=None,
    expected_reset_sequence=0,
):
    current = sanitize_bridge_metrics(current_metrics)
    baseline = sanitize_bridge_metrics(baseline_metrics)
    expected_reset_sequence = max(0, coerce_int(expected_reset_sequence, 0))
    if expected_reset_sequence > 0:
        if current["counter_reset_sequence"] < expected_reset_sequence:
            return default_bridge_metrics()
        return current
    return {
        "serial_frames_rx": max(
            0,
            current["serial_frames_rx"] - baseline["serial_frames_rx"],
        ),
        "espnow_tx_attempts": max(
            0,
            current["espnow_tx_attempts"] - baseline["espnow_tx_attempts"],
        ),
        "espnow_tx_ok": max(
            0,
            current["espnow_tx_ok"] - baseline["espnow_tx_ok"],
        ),
        "espnow_tx_fail": max(
            0,
            current["espnow_tx_fail"] - baseline["espnow_tx_fail"],
        ),
        "oversize_drops": max(
            0,
            current["oversize_drops"] - baseline["oversize_drops"],
        ),
        "tx_rate_hz": current["tx_rate_hz"],
        "last_tx_seq": max(0, current["last_tx_seq"] - baseline["last_tx_seq"]),
        "last_tx_latency_us": current["last_tx_latency_us"],
        "counter_reset_sequence": current["counter_reset_sequence"],
    }


def build_session_controller_metrics(
    current_metrics,
    baseline_metrics=None,
    expected_reset_sequence=0,
):
    current = sanitize_controller_metrics(current_metrics)
    baseline = sanitize_controller_metrics(baseline_metrics)
    expected_reset_sequence = max(0, coerce_int(expected_reset_sequence, 0))
    if expected_reset_sequence > 0:
        if current["counter_reset_sequence"] < expected_reset_sequence:
            return default_controller_metrics()
        return current
    return {
        "rx_packets": max(0, current["rx_packets"] - baseline["rx_packets"]),
        "rx_missing_packets": max(
            0,
            current["rx_missing_packets"] - baseline["rx_missing_packets"],
        ),
        "rx_duplicate_packets": max(
            0,
            current["rx_duplicate_packets"] - baseline["rx_duplicate_packets"],
        ),
        "rx_out_of_order_packets": max(
            0,
            current["rx_out_of_order_packets"] - baseline["rx_out_of_order_packets"],
        ),
        "last_rx_seq": max(0, current["last_rx_seq"] - baseline["last_rx_seq"]),
        "loop_hz": current["loop_hz"],
        "control_hz": current["control_hz"],
        "command_age_ms": current["command_age_ms"],
        "apply_latency_us": current["apply_latency_us"],
        "motor_imbalance": current["motor_imbalance"],
        "motor_outputs": copy.deepcopy(current["motor_outputs"]),
        "counter_reset_sequence": current["counter_reset_sequence"],
    }


def sanitize_pid_config(payload):
    payload = payload if isinstance(payload, dict) else {}
    sanitized = default_pid_config()
    for axis, defaults in CONTROL_PID_DEFAULTS.items():
        axis_payload = payload.get(axis, {})
        if not isinstance(axis_payload, dict):
            axis_payload = {}
        sanitized[axis] = {
            term: coerce_float(axis_payload.get(term), default)
            for term, default in defaults.items()
        }
    return sanitized


def sanitize_target_config(payload):
    payload = payload if isinstance(payload, dict) else {}
    return {
        axis: coerce_float(payload.get(axis), default)
        for axis, default in CONTROL_TARGET_DEFAULTS.items()
    }


def sanitize_limit_config(payload):
    payload = payload if isinstance(payload, dict) else {}
    sanitized = {
        axis: coerce_float(payload.get(axis), default)
        for axis, default in CONTROL_LIMIT_DEFAULTS.items()
    }
    sanitized["minThrottle"] = max(0.0, min(sanitized["minThrottle"], 1.0))
    sanitized["hoverThrottle"] = max(
        sanitized["minThrottle"],
        min(sanitized["hoverThrottle"], 1.0),
    )
    sanitized["maxThrottle"] = max(
        sanitized["hoverThrottle"],
        min(sanitized["maxThrottle"], 1.0),
    )
    sanitized["maxTiltDeg"] = max(1.0, sanitized["maxTiltDeg"])
    sanitized["maxYawRateDeg"] = max(1.0, sanitized["maxYawRateDeg"])
    return sanitized


def compact_numeric(value, digits=4):
    rounded = round(float(value), digits)
    if abs(rounded) < (10 ** -digits):
        return 0

    integer_value = int(rounded)
    if math.isclose(rounded, integer_value, abs_tol=10 ** -digits):
        return integer_value

    # Re-parse a fixed-width decimal string so json.dumps emits the shortest
    # numeric representation instead of preserving trailing float artifacts.
    return float(f"{rounded:.{digits}f}".rstrip("0").rstrip("."))


def compact_pid_triplet(pid_values, digits=4):
    return [
        compact_numeric(pid_values["kp"], digits),
        compact_numeric(pid_values["ki"], digits),
        compact_numeric(pid_values["kd"], digits),
    ]


def compact_pid_bundle(pid_config, digits=4):
    ordered_axes = (
        "xyPos",
        "zPos",
        "yawPos",
        "xyVel",
        "zVel",
        "roll",
        "pitch",
        "yawRate",
    )
    values = []
    for axis in ordered_axes:
        values.extend(compact_pid_triplet(pid_config[axis], digits))
    return values


def serialized_payload_size_bytes(payload):
    return len(
        json.dumps(
            payload,
            separators=(",", ":"),
            ensure_ascii=True,
            allow_nan=False,
        ).encode("utf-8")
    )


def encode_preview_frame(frame, leds=None, labels=None):
    preview = frame
    height, width = preview.shape[:2]
    if width > PREVIEW_WIDTH:
        scale = PREVIEW_WIDTH / width
        preview = cv.resize(
            preview,
            (PREVIEW_WIDTH, max(1, int(height * scale))),
            interpolation=cv.INTER_AREA,
        )
    else:
        preview = preview.copy()

    if len(preview.shape) == 2:
        preview = cv.cvtColor(preview, cv.COLOR_GRAY2BGR)

    scale_x = preview.shape[1] / width
    scale_y = preview.shape[0] / height
    for led_index, (x_coord, y_coord) in enumerate(leds or [], start=1):
        center = (int(round(x_coord * scale_x)), int(round(y_coord * scale_y)))
        cv.circle(preview, center, 8, (0, 220, 255), 2)
        label = str(led_index)
        if labels and (led_index - 1) < len(labels):
            label = str(labels[led_index - 1])
        cv.putText(
            preview,
            label,
            (center[0] + 10, center[1] - 10),
            cv.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 220, 255),
            1,
            cv.LINE_AA,
        )

    success, encoded = cv.imencode(
        ".jpg",
        preview,
        [int(cv.IMWRITE_JPEG_QUALITY), PREVIEW_JPEG_QUALITY],
    )
    if not success:
        return ""

    return base64.b64encode(encoded).decode("ascii")


def http_health_response(connection, request):
    headers = getattr(request, "headers", request)
    upgrade_header = headers.get("Upgrade", "")
    if upgrade_header.lower() == "websocket":
        return None

    body = (
        b"This endpoint expects a WebSocket connection.\n"
        b"Use the React frontend or a WebSocket client at ws://localhost:8765.\n"
    )
    response_headers = Headers()
    response_headers["Content-Type"] = "text/plain; charset=utf-8"
    response_headers["Content-Length"] = str(len(body))
    return Response(HTTPStatus.OK, "OK", response_headers, body)


def load_json(path: Path):
    with path.open("r", encoding="utf-8") as file:
        return json.load(file)


def triangulate_n_views(proj_mats, pts_2d):
    """Triangulate 3D point from multiple 2D observations using DLT."""
    a_matrix = []
    for proj_mat, (u_coord, v_coord) in zip(proj_mats, pts_2d):
        a_matrix.append(u_coord * proj_mat[2, :] - proj_mat[0, :])
        a_matrix.append(v_coord * proj_mat[2, :] - proj_mat[1, :])

    _, _, vt = np.linalg.svd(np.array(a_matrix))
    homogeneous = vt[-1]
    return homogeneous[:3] / homogeneous[3]


def project_world_point(proj_mat, point_world):
    """Project 3D world point to 2D image coordinates."""
    homogeneous_point = np.append(np.asarray(point_world, dtype=np.float32), 1.0)
    projected = proj_mat @ homogeneous_point
    if abs(projected[2]) < 1e-6:
        return None
    return projected[:2] / projected[2]


def rotation_matrix_to_euler_zyx(R):
    """
    Convert rotation matrix to Euler angles (ZYX intrinsic / yaw-pitch-roll).
    Returns (yaw, pitch, roll) in degrees.
    """
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
    
    singular = sy < 1e-6
    
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0
    
    return np.degrees([yaw, pitch, roll])


def wrap_angle_degrees(angle):
    """Wrap angle to [-180, 180] range."""
    return ((angle + 180.0) % 360.0) - 180.0


def unwrap_angle_degrees(previous_angle, next_angle):
    """Unwrap angle to prevent jumps at ±180 boundary."""
    return previous_angle + wrap_angle_degrees(next_angle - previous_angle)


class ScalarKalmanFilter:
    def __init__(self, process_variance, measurement_variance):
        self.process_variance = float(process_variance)
        self.measurement_variance = float(measurement_variance)
        self.value = 0.0
        self.error_covariance = 1.0
        self.initialized = False

    def reset(self):
        self.value = 0.0
        self.error_covariance = 1.0
        self.initialized = False

    def update(self, measurement):
        measurement = float(measurement)
        if not self.initialized:
            self.value = measurement
            self.error_covariance = 1.0
            self.initialized = True
            return self.value

        self.error_covariance += self.process_variance
        kalman_gain = self.error_covariance / (
            self.error_covariance + self.measurement_variance
        )
        self.value += kalman_gain * (measurement - self.value)
        self.error_covariance *= 1.0 - kalman_gain
        
        return self.value


class MotionStateKalmanFilter:
    """
    Adapted from Low-Cost-Mocap's object-state filter:
    state = [x, y, z, vx, vy, vz, ax, ay, az]
    measurement = [x, y, z, vx, vy, vz]
    """

    def __init__(self):
        self.kalman = cv.KalmanFilter(9, 6, 0, cv.CV_32F)
        self.kalman.processNoiseCov = (
            np.eye(9, dtype=np.float32) * MOTION_STATE_PROCESS_NOISE
        )
        self.kalman.measurementNoiseCov = (
            np.eye(6, dtype=np.float32) * MOTION_STATE_MEASUREMENT_NOISE
        )
        self.kalman.measurementMatrix = np.array(
            [
                [1, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 1, 0, 0, 0],
            ],
            dtype=np.float32,
        )
        self.reset()

    def _set_transition(self, dt_seconds):
        dt_seconds = float(dt_seconds)
        dt_squared = dt_seconds * dt_seconds
        self.kalman.transitionMatrix = np.array(
            [
                [1, 0, 0, dt_seconds, 0, 0, 0.5 * dt_squared, 0, 0],
                [0, 1, 0, 0, dt_seconds, 0, 0, 0.5 * dt_squared, 0],
                [0, 0, 1, 0, 0, dt_seconds, 0, 0, 0.5 * dt_squared],
                [0, 0, 0, 1, 0, 0, dt_seconds, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, dt_seconds, 0],
                [0, 0, 0, 0, 0, 1, 0, 0, dt_seconds],
                [0, 0, 0, 0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 1],
            ],
            dtype=np.float32,
        )

    def reset(self):
        zero_state = np.zeros((9, 1), dtype=np.float32)
        self.kalman.statePre = zero_state.copy()
        self.kalman.statePost = zero_state.copy()
        self.previous_position = None
        self.previous_timestamp = None
        self.initialized = False

    def update(self, position, timestamp=None):
        timestamp = time.time() if timestamp is None else float(timestamp)
        position = np.asarray(position, dtype=np.float32).reshape(3)

        if not self.initialized:
            self.kalman.statePre[:3, 0] = position
            self.kalman.statePost[:3, 0] = position
            self.previous_position = position.copy()
            self.previous_timestamp = timestamp
            self.initialized = True
            return position.copy(), np.zeros(3, dtype=np.float32)

        dt_seconds = timestamp - float(self.previous_timestamp)
        if dt_seconds <= 1e-3 or dt_seconds > MOTION_STATE_MAX_DT:
            self.kalman.statePre[:3, 0] = position
            self.kalman.statePost[:3, 0] = position
            self.kalman.statePre[3:9, 0] = 0.0
            self.kalman.statePost[3:9, 0] = 0.0
            self.previous_position = position.copy()
            self.previous_timestamp = timestamp
            return position.copy(), np.zeros(3, dtype=np.float32)

        raw_velocity = (position - self.previous_position) / dt_seconds
        raw_velocity = np.clip(
            raw_velocity,
            -MOTION_STATE_MAX_ABS_VELOCITY,
            MOTION_STATE_MAX_ABS_VELOCITY,
        ).astype(np.float32)

        self._set_transition(dt_seconds)
        measurement = np.concatenate((position, raw_velocity)).reshape(6, 1)
        self.kalman.predict()
        corrected_state = self.kalman.correct(measurement)[:, 0]

        self.previous_position = position.copy()
        self.previous_timestamp = timestamp

        filtered_position = corrected_state[:3].astype(np.float32)
        filtered_velocity = corrected_state[3:6].astype(np.float32)
        filtered_velocity[np.abs(filtered_velocity) < MOTION_STATE_ZERO_THRESHOLD] = 0.0
        return filtered_position, filtered_velocity


def detect_leds(frame):
    """Detect LED positions and sort them spatially (clockwise)."""
    _, mask = cv.threshold(frame, MIN_BRIGHTNESS, 255, cv.THRESH_BINARY)
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    led_candidates = []
    for contour in contours:
        moments = cv.moments(contour)
        if moments["m00"] > 2:
            led_candidates.append({
                "x": moments["m10"] / moments["m00"],
                "y": moments["m01"] / moments["m00"],
                "area": float(moments["m00"]),
            })

    if len(led_candidates) < MAX_LEDS:
        return [(l["x"], l["y"]) for l in led_candidates]

    # 1. Take top 3 by area/intensity if more than 3 found (optional filter)
    top_three = sorted(
        led_candidates,
        key=lambda candidate: candidate["area"],
        reverse=True,
    )[:MAX_LEDS]

    # 2. Calculate Centroid of the 3 points
    cx = sum(l["x"] for l in top_three) / MAX_LEDS
    cy = sum(l["y"] for l in top_three) / MAX_LEDS

    # 3. Sort by angle around centroid (atan2(y-cy, x-cx))
    # This creates a consistent clockwise or counter-clockwise sequence
    top_three.sort(key=lambda l: math.atan2(l["y"] - cy, l["x"] - cx))

    return [(l["x"], l["y"]) for l in top_three]


def undistort_led_points(led_points, camera_matrix, dist_coeff):
    """Undistort 2D pixel observations while keeping pixel coordinate output."""
    if not led_points:
        return []

    points = np.array(led_points, dtype=np.float32).reshape(-1, 1, 2)
    undistorted = cv.undistortPoints(
        points,
        np.asarray(camera_matrix, dtype=np.float32),
        np.asarray(dist_coeff, dtype=np.float32),
        P=np.asarray(camera_matrix, dtype=np.float32),
    )
    return [tuple(point[0]) for point in undistorted]


def solve_pose_procrustes(points_3d, model_points):
    """Solve for scale, rotation, and translation using similarity Procrustes."""
    points_3d = np.asarray(points_3d, dtype=np.float32)
    model_points = np.asarray(model_points, dtype=np.float32)
    
    centroid_data = np.mean(points_3d, axis=0)
    centroid_model = np.mean(model_points, axis=0)
    
    data_centered = points_3d - centroid_data
    model_centered = model_points - centroid_model
    
    H = model_centered.T @ data_centered
    U, _, Vt = np.linalg.svd(H)
    
    R = Vt.T @ U.T
    
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    aligned_model = model_centered @ R.T
    model_variance = float(np.sum(model_centered ** 2))
    if model_variance <= 1e-9:
        scale = 1.0
    else:
        scale = float(np.sum(data_centered * aligned_model) / model_variance)
    
    t = centroid_data - scale * (R @ centroid_model)
    
    return scale, R, t


def compute_reprojection_error(points_3d, model_points):
    """Compute RMS distance between points."""
    return np.sqrt(np.mean(np.sum((points_3d - model_points)**2, axis=1)))


def rotation_matrix_angular_distance_degrees(matrix_a, matrix_b):
    """Return the shortest rotation angle between two 3x3 rotation matrices."""
    relative_rotation = np.asarray(matrix_a, dtype=np.float32).T @ np.asarray(
        matrix_b, dtype=np.float32
    )
    cosine = max(-1.0, min(1.0, (np.trace(relative_rotation) - 1.0) * 0.5))
    return math.degrees(math.acos(cosine))


def cyclic_shift(points_2d, shift):
    """Rotate a clockwise point list while preserving clockwise order."""
    return [points_2d[(index + shift) % MAX_LEDS] for index in range(MAX_LEDS)]


def mapping_reprojection_cost(mapped_leds, proj_mats):
    """
    Score a cross-camera LED mapping.
    Lower is better: RMS reprojection error in pixels across all LEDs/cameras.
    """
    squared_error_sum = 0.0
    observations = 0

    for led_idx in range(MAX_LEDS):
        points_2d = [cam_leds[led_idx] for cam_leds in mapped_leds]
        point_3d = triangulate_n_views(proj_mats, points_2d)

        for proj_mat, observed in zip(proj_mats, points_2d):
            projected = project_world_point(proj_mat, point_3d)
            if projected is None:
                return float("inf")

            diff = projected - np.asarray(observed, dtype=np.float32)
            squared_error_sum += float(diff[0] ** 2 + diff[1] ** 2)
            observations += 1

    if observations == 0:
        return float("inf")

    return math.sqrt(squared_error_sum / observations)


def triangulate_mapped_leds(mapped_leds, proj_mats):
    """Triangulate one 3D point for each mapped LED index."""
    points = []
    for led_idx in range(MAX_LEDS):
        pts_2d = [cam_leds[led_idx] for cam_leds in mapped_leds]
        points.append(triangulate_n_views(proj_mats, pts_2d))
    return np.array(points, dtype=np.float32)


def mapping_temporal_penalty(candidate_points, previous_led_points):
    """
    Penalize sudden A/B/C label swaps between frames.
    Returns a penalty in pseudo-pixel units.
    """
    if previous_led_points is None:
        return 0.0
    previous_led_points = np.asarray(previous_led_points, dtype=np.float32)
    if previous_led_points.shape != candidate_points.shape:
        return 0.0

    rms_delta_m = float(
        np.sqrt(np.mean(np.sum((candidate_points - previous_led_points) ** 2, axis=1)))
    )
    return MAPPING_TEMPORAL_WEIGHT * rms_delta_m


def find_best_clockwise_mapping(all_cam_leds, proj_mats, previous_led_points=None):
    """
    Search cyclic LED orderings per camera and select the globally minimum
    cross-camera reprojection-cost mapping.
    """
    best_cost = float("inf")
    best_reprojection_cost = float("inf")
    best_leds = None
    best_shifts = [0 for _ in all_cam_leds]
    best_points = None

    shift_ranges = [range(MAX_LEDS) for _ in all_cam_leds]
    # 3 cameras x 3 LEDs -> 3^3 = 27 combinations.
    for shifts in np.ndindex(*[len(options) for options in shift_ranges]):
        mapped_leds = [
            cyclic_shift(cam_leds, shift)
            for cam_leds, shift in zip(all_cam_leds, shifts)
        ]
        candidate_points = triangulate_mapped_leds(mapped_leds, proj_mats)
        reprojection_cost = mapping_reprojection_cost(mapped_leds, proj_mats)
        temporal_penalty = mapping_temporal_penalty(candidate_points, previous_led_points)
        cost = reprojection_cost + temporal_penalty

        if cost < best_cost:
            best_cost = cost
            best_reprojection_cost = reprojection_cost
            best_leds = mapped_leds
            best_shifts = [int(value) for value in shifts]
            best_points = candidate_points

    return best_leds, best_reprojection_cost, best_shifts, best_points


def solve_led_positions(all_cam_leds, proj_mats, previous_led_points=None):
    """
    Solve LED 3D positions from multi-view geometry only (no drone model usage).
    Returns mapped observations, mapping quality, and unlabeled 3D coordinates.
    """
    mapped_leds, mapping_cost, mapping_shifts, candidate_points = find_best_clockwise_mapping(
        all_cam_leds,
        proj_mats,
        previous_led_points=previous_led_points,
    )
    if mapped_leds is None:
        return None

    return {
        "mapped_leds": mapped_leds,
        "mapping_cost": float(mapping_cost),
        "mapping_shifts": [int(value) for value in mapping_shifts],
        "candidate_points": candidate_points,
    }


def select_best_semantic_pose(candidate_points, previous_rotation=None):
    """
    Choose the F/R/L assignment that best matches the asymmetric LED model.
    A light temporal bias keeps the semantic labels from flipping between frames.
    """
    best_solution = None
    candidate_points = np.asarray(candidate_points, dtype=np.float32)

    for permutation in itertools.permutations(range(MAX_LEDS)):
        semantic_points = candidate_points[list(permutation)]
        scale_factor, rotation, translation = solve_pose_procrustes(
            semantic_points,
            DRONE_LED_MODEL,
        )
        transformed_model = scale_factor * (rotation @ DRONE_LED_MODEL.T).T + translation
        fit_error = float(compute_reprojection_error(semantic_points, transformed_model))
        rotation_penalty = 0.0
        if previous_rotation is not None:
            rotation_penalty = (
                SEMANTIC_ROTATION_TEMPORAL_WEIGHT
                * rotation_matrix_angular_distance_degrees(previous_rotation, rotation)
            )
        score = fit_error + rotation_penalty

        if best_solution is None or score < best_solution["score"]:
            best_solution = {
                "score": score,
                "fit_error": fit_error,
                "scale_factor": scale_factor,
                "rotation": rotation,
                "translation": translation,
                "semantic_permutation": [int(value) for value in permutation],
                "semantic_points": semantic_points,
            }

    return best_solution


def solve_pose(
    all_cam_leds,
    proj_mats,
    previous_rotation=None,
    previous_position=None,
    precomputed_led_solution=None,
):
    """
    Solve 6DOF pose with cyclic-only mapping search.
    We choose the mapping with minimum cross-camera reprojection error.
    """
    led_solution = precomputed_led_solution or solve_led_positions(all_cam_leds, proj_mats)
    if led_solution is None:
        return None

    mapped_leds = led_solution["mapped_leds"]
    mapping_cost = led_solution["mapping_cost"]
    mapping_shifts = led_solution["mapping_shifts"]
    candidate_points = led_solution["candidate_points"]

    if mapping_cost >= MAX_LED_REORDER_COST_PIXELS:
        return None

    semantic_pose = select_best_semantic_pose(
        candidate_points,
        previous_rotation=previous_rotation,
    )
    scale_factor = semantic_pose["scale_factor"]
    rotation = semantic_pose["rotation"]
    translation = semantic_pose["translation"]
    fit_error = semantic_pose["fit_error"]
    semantic_permutation = semantic_pose["semantic_permutation"]
    semantic_points = semantic_pose["semantic_points"]

    # Check if fit is acceptable
    if fit_error >= MAX_FIT_ERROR:
        return None

    solved_led_coordinates = [
        {
            "label": label,
            "x": round(float(point[0]), 4),
            "y": round(float(point[1]), 4),
            "z": round(float(point[2]), 4),
        }
        for label, point in zip(["F", "R", "L"], semantic_points)
    ]

    return {
        "position": {
            "x": round(float(translation[0]), 4),
            "y": round(float(translation[1]), 4),
            "z": round(float(translation[2]), 4),
        },
        "translation_vector": translation,
        "rotation_matrix": rotation,
        "error": round(float(max(fit_error, mapping_cost)), 5),
        "mapping_error_px": round(float(mapping_cost), 5),
        "model_fit_error_m": round(float(fit_error), 5),
        "scale_factor": round(float(scale_factor), 5),
        "solved_led_coordinates": solved_led_coordinates,
        "mapping_shifts": [int(value) for value in mapping_shifts],
        "semantic_permutation": [int(value) for value in semantic_permutation],
        "semantic_led_points": np.asarray(semantic_points, dtype=np.float32),
        "detected_leds_per_camera": [len(leds) for leds in all_cam_leds],
    }


@dataclass
class ControlState:
    active: bool = False
    armed: bool = False
    serial_port: str = ""
    baud_rate: int = DEFAULT_BAUD_RATE
    target: dict = field(default_factory=default_target_config)
    limits: dict = field(default_factory=default_limit_config)
    pid: dict = field(default_factory=default_pid_config)


class ImuDataLogger:
    def __init__(self, log_dir: Path):
        self.log_dir = Path(log_dir)
        self.file_handle = None
        self.csv_writer = None
        self.log_path = None
        self.started_at = 0.0

    def is_active(self):
        return self.file_handle is not None

    def start(self):
        if self.is_active():
            return self.log_path

        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.started_at = time.time()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = self.log_dir / f"imu_log_{timestamp}.csv"
        self.file_handle = self.log_path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.file_handle)
        self.csv_writer.writerow(
            [
                "iso_time",
                "elapsed_s",
                *IMU_LOG_COLUMNS,
            ]
        )
        self.file_handle.flush()
        print(f"IMU logging started: {self.log_path}")
        return self.log_path

    def log_sample(self, sample, sample_time=None, mocap_sample=None):
        if not self.is_active():
            return

        mocap_sample = mocap_sample if isinstance(mocap_sample, dict) else {}
        sample_time = float(sample_time or time.time())
        timestamp = datetime.fromtimestamp(sample_time).isoformat(timespec="milliseconds")
        elapsed = max(0.0, sample_time - self.started_at)
        self.csv_writer.writerow(
            [
                timestamp,
                f"{elapsed:.3f}",
                *build_imu_log_values(sample, mocap_sample),
            ]
        )
        self.file_handle.flush()

    def stop(self):
        if not self.is_active():
            return None

        log_path = self.log_path
        try:
            self.file_handle.close()
        finally:
            self.file_handle = None
            self.csv_writer = None
            self.log_path = None
            self.started_at = 0.0
        print(f"IMU logging stopped: {log_path}")
        return log_path

    def close(self):
        return self.stop()


class ExperimentMetricsLogger:
    def __init__(self, log_dir: Path):
        self.log_dir = Path(log_dir)
        self.file_handle = None
        self.csv_writer = None
        self.log_path = None
        self.started_at = 0.0

    def is_active(self):
        return self.file_handle is not None

    def start(self):
        if self.is_active():
            return self.log_path

        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.started_at = time.time()
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = self.log_dir / f"session_metrics_{timestamp}.csv"
        self.file_handle = self.log_path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.file_handle)
        self.csv_writer.writerow(
            [
                "iso_time",
                "elapsed_s",
                *IMU_LOG_COLUMNS,
                "backend_loop_hz",
                "tracking_loop_hz",
                "frames_processed",
                "valid_pose_frames",
                "pose_success_rate",
                "full_marker_detection_rate",
                "cam1_full_detection_rate",
                "cam2_full_detection_rate",
                "cam1_leds",
                "cam2_leds",
                "spatial_data_valid",
                "mapping_error_px",
                "model_fit_error_m",
                "scale_factor",
                "position_x_m",
                "position_y_m",
                "position_z_m",
                "velocity_x_m_s",
                "velocity_y_m_s",
                "velocity_z_m_s",
                "yaw_deg",
                "pitch_deg",
                "roll_deg",
                "serial_connected",
                "serial_send_enabled",
                "serial_send_ok",
                "serial_payload_seq",
                "serial_payload_size_bytes",
                "bridge_serial_frames_rx",
                "bridge_espnow_tx_attempts",
                "bridge_espnow_tx_ok",
                "bridge_espnow_tx_fail",
                "bridge_oversize_drops",
                "bridge_tx_rate_hz",
                "bridge_last_tx_seq",
                "bridge_last_tx_latency_us",
                "controller_rx_packets",
                "controller_rx_missing_packets",
                "controller_rx_duplicate_packets",
                "controller_rx_out_of_order_packets",
                "controller_last_rx_seq",
                "controller_loop_hz",
                "controller_control_hz",
                "controller_command_age_ms",
                "controller_apply_latency_us",
                "controller_motor_imbalance",
                "motor_1_output",
                "motor_2_output",
                "motor_3_output",
                "motor_4_output",
            ]
        )
        self.file_handle.flush()
        print(f"Session metrics logging started: {self.log_path}")
        return self.log_path

    def log_sample(
        self,
        sample_time,
        telemetry,
        vision_metrics,
        bridge_metrics,
        controller_metrics,
        backend_loop_hz,
        serial_connected,
        serial_send_enabled,
        serial_send_ok,
        serial_payload_seq,
        serial_payload_size_bytes,
        imu_sample=None,
        mocap_sample=None,
    ):
        if not self.is_active():
            return

        telemetry = telemetry if isinstance(telemetry, dict) else {}
        vision_metrics = (
            vision_metrics if isinstance(vision_metrics, dict) else default_vision_metrics()
        )
        bridge_metrics = (
            bridge_metrics if isinstance(bridge_metrics, dict) else default_bridge_metrics()
        )
        controller_metrics = (
            controller_metrics
            if isinstance(controller_metrics, dict)
            else default_controller_metrics()
        )
        position = telemetry.get("position", {})
        velocity = telemetry.get("velocity", {})
        rotation = telemetry.get("rotation", {})
        detected_leds = telemetry.get("detected_leds_per_camera", [])
        motor_outputs = controller_metrics.get("motor_outputs", [0.0, 0.0, 0.0, 0.0])
        imu_values = build_imu_log_values(imu_sample, mocap_sample)
        timestamp = datetime.fromtimestamp(sample_time).isoformat(timespec="milliseconds")
        elapsed = max(0.0, float(sample_time) - self.started_at)

        self.csv_writer.writerow(
            [
                timestamp,
                f"{elapsed:.3f}",
                *imu_values,
                f"{coerce_float(backend_loop_hz, 0.0):.2f}",
                f"{coerce_float(vision_metrics.get('tracking_loop_hz'), 0.0):.2f}",
                coerce_int(vision_metrics.get("frames_processed"), 0),
                coerce_int(vision_metrics.get("valid_pose_frames"), 0),
                f"{coerce_float(vision_metrics.get('pose_success_rate'), 0.0):.5f}",
                f"{coerce_float(vision_metrics.get('full_marker_detection_rate'), 0.0):.5f}",
                f"{coerce_float((vision_metrics.get('per_camera_full_detection_rate') or [0.0, 0.0])[0], 0.0):.5f}",
                f"{coerce_float((vision_metrics.get('per_camera_full_detection_rate') or [0.0, 0.0])[1], 0.0):.5f}",
                coerce_int(detected_leds[0] if len(detected_leds) > 0 else 0, 0),
                coerce_int(detected_leds[1] if len(detected_leds) > 1 else 0, 0),
                int(bool(telemetry.get("spatial_data_valid", False))),
                f"{coerce_float(telemetry.get('mapping_error_px'), 0.0):.5f}",
                f"{coerce_float(telemetry.get('model_fit_error_m'), 0.0):.5f}",
                f"{coerce_float(telemetry.get('scale_factor'), 1.0):.5f}",
                f"{coerce_float(position.get('x'), 0.0):.4f}",
                f"{coerce_float(position.get('y'), 0.0):.4f}",
                f"{coerce_float(position.get('z'), 0.0):.4f}",
                f"{coerce_float(velocity.get('x'), 0.0):.4f}",
                f"{coerce_float(velocity.get('y'), 0.0):.4f}",
                f"{coerce_float(velocity.get('z'), 0.0):.4f}",
                f"{coerce_float(rotation.get('yaw'), 0.0):.2f}",
                f"{coerce_float(rotation.get('pitch'), 0.0):.2f}",
                f"{coerce_float(rotation.get('roll'), 0.0):.2f}",
                int(bool(serial_connected)),
                int(bool(serial_send_enabled)),
                int(bool(serial_send_ok)),
                coerce_int(serial_payload_seq, 0),
                coerce_int(serial_payload_size_bytes, 0),
                coerce_int(bridge_metrics.get("serial_frames_rx"), 0),
                coerce_int(bridge_metrics.get("espnow_tx_attempts"), 0),
                coerce_int(bridge_metrics.get("espnow_tx_ok"), 0),
                coerce_int(bridge_metrics.get("espnow_tx_fail"), 0),
                coerce_int(bridge_metrics.get("oversize_drops"), 0),
                f"{coerce_float(bridge_metrics.get('tx_rate_hz'), 0.0):.2f}",
                coerce_int(bridge_metrics.get("last_tx_seq"), 0),
                coerce_int(bridge_metrics.get("last_tx_latency_us"), 0),
                coerce_int(controller_metrics.get("rx_packets"), 0),
                coerce_int(controller_metrics.get("rx_missing_packets"), 0),
                coerce_int(controller_metrics.get("rx_duplicate_packets"), 0),
                coerce_int(controller_metrics.get("rx_out_of_order_packets"), 0),
                coerce_int(controller_metrics.get("last_rx_seq"), 0),
                f"{coerce_float(controller_metrics.get('loop_hz'), 0.0):.2f}",
                f"{coerce_float(controller_metrics.get('control_hz'), 0.0):.2f}",
                coerce_int(controller_metrics.get("command_age_ms"), 0),
                coerce_int(controller_metrics.get("apply_latency_us"), 0),
                f"{coerce_float(controller_metrics.get('motor_imbalance'), 0.0):.4f}",
                f"{coerce_float(motor_outputs[0] if len(motor_outputs) > 0 else 0.0, 0.0):.4f}",
                f"{coerce_float(motor_outputs[1] if len(motor_outputs) > 1 else 0.0, 0.0):.4f}",
                f"{coerce_float(motor_outputs[2] if len(motor_outputs) > 2 else 0.0, 0.0):.4f}",
                f"{coerce_float(motor_outputs[3] if len(motor_outputs) > 3 else 0.0, 0.0):.4f}",
            ]
        )
        self.file_handle.flush()

    def stop(self):
        if not self.is_active():
            return None

        log_path = self.log_path
        try:
            self.file_handle.close()
        finally:
            self.file_handle = None
            self.csv_writer = None
            self.log_path = None
            self.started_at = 0.0
        print(f"Session metrics logging stopped: {log_path}")
        return log_path

    def close(self):
        return self.stop()


class SerialBridge:
    def __init__(self):
        self.connection = None
        self.port = ""
        self.baud_rate = DEFAULT_BAUD_RATE
        self.last_error = ""
        self.available_ports = []
        self._last_refresh = 0.0
        self._last_connect_attempt = 0.0
        self._connected_at = 0.0
        self._consecutive_send_failures = 0
        self._serial_buffer = bytearray()
        self.latest_imu = default_imu_telemetry()
        self.latest_bridge_metrics = default_bridge_metrics()
        self.latest_controller_metrics = default_controller_metrics()
        self._last_imu_received_at = 0.0
        self.imu_sample_callback = None

    def refresh_ports(self, force=False):
        now = time.time()
        if not force and (now - self._last_refresh) < SERIAL_REFRESH_SECONDS:
            return

        self._last_refresh = now
        if list_ports is None:
            self.available_ports = []
            self.last_error = "pyserial is not installed."
            return

        self.available_ports = [
            port.device for port in sorted(list_ports.comports(), key=lambda item: item.device)
        ]

    def connect(self, port, baud_rate):
        self._last_connect_attempt = time.time()
        self.refresh_ports(force=True)
        self.disconnect()

        if serial is None:
            self.last_error = "pyserial is not installed."
            return False

        if not port:
            self.last_error = ""
            self.port = ""
            return False

        try:
            connection = serial.Serial()
            connection.port = port
            connection.baudrate = int(baud_rate)
            connection.timeout = 0.1
            connection.write_timeout = 1.0
            connection.rtscts = False
            connection.dsrdtr = False
            # Keep modem-control lines low so reconnects do not reset the ESP32-S3.
            try:
                connection.rts = False
                connection.dtr = False
            except Exception:
                pass
            connection.open()
            connection.reset_input_buffer()
            connection.reset_output_buffer()
            self.connection = connection
            self.port = port
            self.baud_rate = int(baud_rate)
            self._connected_at = time.time()
            self._consecutive_send_failures = 0
            self._serial_buffer = bytearray()
            self.latest_imu = default_imu_telemetry()
            self.latest_bridge_metrics = default_bridge_metrics()
            self.latest_controller_metrics = default_controller_metrics()
            self._last_imu_received_at = 0.0
            self.last_error = ""
            return True
        except Exception as exc:
            self.connection = None
            self.port = ""
            self._connected_at = 0.0
            self._consecutive_send_failures = 0
            self._serial_buffer = bytearray()
            self.latest_imu = default_imu_telemetry()
            self.latest_bridge_metrics = default_bridge_metrics()
            self.latest_controller_metrics = default_controller_metrics()
            self._last_imu_received_at = 0.0
            self.last_error = f"Serial connection failed: {exc}"
            return False

    def ensure_connected(self, port, baud_rate):
        if not port:
            return False
        if self.is_connected() and self.port == port and self.baud_rate == int(baud_rate):
            return True

        now = time.time()
        if (now - self._last_connect_attempt) < SERIAL_RECONNECT_SECONDS:
            return False

        return self.connect(port, baud_rate)

    def disconnect(self):
        if self.connection:
            try:
                self.connection.close()
            except Exception:
                pass
        self.connection = None
        self.port = ""
        self._connected_at = 0.0
        self._consecutive_send_failures = 0
        self._serial_buffer = bytearray()
        self.latest_imu = default_imu_telemetry()
        self.latest_bridge_metrics = default_bridge_metrics()
        self.latest_controller_metrics = default_controller_metrics()
        self._last_imu_received_at = 0.0

    def is_connected(self):
        return bool(self.connection and self.connection.is_open)

    def _handle_incoming_line(self, line):
        if not line.startswith("!"):
            return

        try:
            payload = json.loads(line[1:])
        except json.JSONDecodeError:
            return

        payload_type = str(payload.get("t", "")).strip().lower()
        if payload_type == "imu":
            self.latest_imu = sanitize_imu_telemetry(
                {
                    "ready": payload.get("ready", payload.get("ok", False)),
                    "pitch": payload.get("pitch", payload.get("p", 0.0)),
                    "roll": payload.get("roll", payload.get("r", 0.0)),
                    "pitch_rate": payload.get("pitch_rate", payload.get("pr", 0.0)),
                    "roll_rate": payload.get("roll_rate", payload.get("rr", 0.0)),
                }
            )
            self._last_imu_received_at = time.time()
            if callable(self.imu_sample_callback):
                self.imu_sample_callback(
                    copy.deepcopy(self.latest_imu), self._last_imu_received_at
                )
            return

        if payload_type == "bridge":
            self.latest_bridge_metrics = sanitize_bridge_metrics(payload)
            return

        if payload_type == "ctrl":
            self.latest_controller_metrics = sanitize_controller_metrics(payload)

    def poll_incoming(self):
        if not self.is_connected():
            return

        try:
            waiting = getattr(self.connection, "in_waiting", 0)
            if not waiting:
                return

            chunk = self.connection.read(waiting)
            if not chunk:
                return

            self._serial_buffer.extend(chunk)
            while True:
                terminator_index = self._serial_buffer.find(SERIAL_FRAME_TERMINATOR)
                if terminator_index < 0:
                    if len(self._serial_buffer) > 4096:
                        self._serial_buffer.clear()
                    break

                raw_line = bytes(self._serial_buffer[:terminator_index])
                del self._serial_buffer[: terminator_index + len(SERIAL_FRAME_TERMINATOR)]
                line = raw_line.decode("utf-8", errors="ignore").strip()
                if line:
                    self._handle_incoming_line(line)
        except Exception as exc:
            self.last_error = f"Serial read failed: {exc}"
            self.disconnect()

    def get_latest_imu(self):
        if (
            not self.is_connected()
            or self._last_imu_received_at <= 0.0
            or (time.time() - self._last_imu_received_at) > SERIAL_IMU_STALE_SECONDS
        ):
            return default_imu_telemetry()
        return copy.deepcopy(self.latest_imu)

    def send(self, drone_index, payload):
        if not self.is_connected():
            return False
        if (time.time() - self._connected_at) < SERIAL_SETTLE_SECONDS:
            self.last_error = "Serial link settling after reconnect."
            return False

        try:
            serialized_payload = json.dumps(
                payload,
                separators=(",", ":"),
                allow_nan=False,
            )
            encoded_payload = serialized_payload.encode("utf-8")
            if len(encoded_payload) > MAX_ESP_NOW_PAYLOAD_BYTES:
                raise ValueError(
                    f"Payload exceeds ESP-NOW limit ({len(encoded_payload)}/{MAX_ESP_NOW_PAYLOAD_BYTES} bytes)."
                )

            frame = (
                str(int(drone_index)).encode("utf-8")
                + encoded_payload
                + SERIAL_FRAME_TERMINATOR
            )
            written = self.connection.write(frame)
            if written != len(frame):
                raise IOError(
                    f"Incomplete serial write ({written}/{len(frame)} bytes sent)."
                )
            self._consecutive_send_failures = 0
            self.last_error = ""
            return True
        except Exception as exc:
            self._consecutive_send_failures += 1
            self.last_error = f"Serial write failed: {exc}"
            try:
                if self.is_connected():
                    self.connection.reset_output_buffer()
            except Exception:
                pass
            if (
                not self.is_connected()
                or self._consecutive_send_failures >= SERIAL_MAX_CONSECUTIVE_FAILURES
            ):
                self.disconnect()
            return False


class MotionCaptureEngine:
    def __init__(self):
        self.intrinsics = load_json(INTRINSICS_PATH)
        self.extrinsics = load_json(EXTRINSICS_PATH)
        self.camera = None
        self.camera_ids = []
        self.camera_error = ""
        self._next_camera_retry_at = 0.0
        self.proj_mats = []
        self.camera_matrices = []
        self.dist_coeffs = []
        self.last_rotation = None
        self.last_translation = None
        self.last_led_points = None
        self.last_unwrapped_angles = None
        self.latest_preview_frames = []
        self.last_preview_update = 0.0
        self.camera_pose_summary = []
        
        self.motion_state_filter = MotionStateKalmanFilter()
        
        # Attitude filters
        self.attitude_filters = {
            "yaw": ScalarKalmanFilter(1.0, 2.0),
            "pitch": ScalarKalmanFilter(1.0, 2.0),
            "roll": ScalarKalmanFilter(1.0, 2.0),
        }
        self.metrics = default_vision_metrics()
        self._last_frame_at = 0.0
        self._mapping_error_sum = 0.0
        self._model_fit_error_sum = 0.0

    def reset_metrics(self):
        self.metrics = default_vision_metrics()
        self._last_frame_at = 0.0
        self._mapping_error_sum = 0.0
        self._model_fit_error_sum = 0.0

    def record_frame_metrics(
        self,
        detected_led_counts,
        spatial_valid,
        mapping_error_px=0.0,
        model_fit_error_m=0.0,
        sample_time=None,
    ):
        sample_time = float(sample_time or time.time())
        self.metrics["frames_processed"] += 1

        if self._last_frame_at > 0.0:
            dt_seconds = sample_time - self._last_frame_at
            if dt_seconds > 0.0:
                self.metrics["tracking_loop_hz"] = round(1.0 / dt_seconds, 2)
        self._last_frame_at = sample_time

        full_detection_flags = []
        for index in range(EXPECTED_CAMERAS):
            detected_count = coerce_int(
                detected_led_counts[index] if index < len(detected_led_counts) else 0,
                0,
            )
            has_full_detection = detected_count == MAX_LEDS
            full_detection_flags.append(has_full_detection)
            if has_full_detection:
                self.metrics["per_camera_full_detection_frames"][index] += 1

        if all(full_detection_flags):
            self.metrics["all_markers_detected_frames"] += 1

        if spatial_valid:
            self.metrics["valid_pose_frames"] += 1
            self._mapping_error_sum += float(mapping_error_px)
            self._model_fit_error_sum += float(model_fit_error_m)

        self.metrics.update(
            build_vision_metrics_summary(
                frames_processed=self.metrics["frames_processed"],
                valid_pose_frames=self.metrics["valid_pose_frames"],
                all_markers_detected_frames=self.metrics["all_markers_detected_frames"],
                per_camera_full_detection_frames=self.metrics[
                    "per_camera_full_detection_frames"
                ],
                tracking_loop_hz=self.metrics["tracking_loop_hz"],
                mapping_error_sum=self._mapping_error_sum,
                model_fit_error_sum=self._model_fit_error_sum,
            )
        )

    def get_metrics_summary(self):
        return copy.deepcopy(self.metrics)

    def get_metrics_state(self):
        state = copy.deepcopy(self.metrics)
        state["_mapping_error_sum"] = float(self._mapping_error_sum)
        state["_model_fit_error_sum"] = float(self._model_fit_error_sum)
        return state

    def get_metrics_summary_since(self, baseline_state=None):
        current_state = self.get_metrics_state()
        baseline_state = baseline_state if isinstance(baseline_state, dict) else {}
        baseline_per_camera = baseline_state.get("per_camera_full_detection_frames", [])
        current_per_camera = current_state.get("per_camera_full_detection_frames", [])

        return build_vision_metrics_summary(
            frames_processed=max(
                0,
                coerce_int(current_state.get("frames_processed"), 0)
                - coerce_int(baseline_state.get("frames_processed"), 0),
            ),
            valid_pose_frames=max(
                0,
                coerce_int(current_state.get("valid_pose_frames"), 0)
                - coerce_int(baseline_state.get("valid_pose_frames"), 0),
            ),
            all_markers_detected_frames=max(
                0,
                coerce_int(current_state.get("all_markers_detected_frames"), 0)
                - coerce_int(baseline_state.get("all_markers_detected_frames"), 0),
            ),
            per_camera_full_detection_frames=[
                max(
                    0,
                    coerce_int(
                        current_per_camera[index] if index < len(current_per_camera) else 0,
                        0,
                    )
                    - coerce_int(
                        baseline_per_camera[index] if index < len(baseline_per_camera) else 0,
                        0,
                    ),
                )
                for index in range(EXPECTED_CAMERAS)
            ],
            tracking_loop_hz=current_state.get("tracking_loop_hz", 0.0),
            mapping_error_sum=max(
                0.0,
                coerce_float(current_state.get("_mapping_error_sum"), 0.0)
                - coerce_float(baseline_state.get("_mapping_error_sum"), 0.0),
            ),
            model_fit_error_sum=max(
                0.0,
                coerce_float(current_state.get("_model_fit_error_sum"), 0.0)
                - coerce_float(baseline_state.get("_model_fit_error_sum"), 0.0),
            ),
        )

    def ensure_camera(self):
        if self.camera is not None:
            return True
        if time.time() < self._next_camera_retry_at:
            return False

        try:
            camera = Camera(fps=60, resolution=Camera.RES_LARGE, colour=False)
            self.camera_ids = list(camera.ids)
            if len(self.camera_ids) < EXPECTED_CAMERAS:
                self.camera_error = (
                    f"Expected {EXPECTED_CAMERAS} cameras but found {len(self.camera_ids)}."
                )
                self._next_camera_retry_at = time.time() + CAMERA_RETRY_SECONDS
                camera.end()
                return False

            self.camera = camera
            self.camera_error = ""
            self._next_camera_retry_at = 0.0
            self.proj_mats = []
            self.camera_matrices = []
            self.dist_coeffs = []
            self.camera_pose_summary = []
            
            for index in range(EXPECTED_CAMERAS):
                cam_key = f"cam{index + 1}"
                intrinsic = np.array(self.intrinsics[cam_key]["camera_matrix"], dtype=np.float32)
                dist_coeff = np.array(self.intrinsics[cam_key]["dist_coeff"], dtype=np.float32)
                pose_cam_to_world = np.array(self.extrinsics[cam_key]["pose_matrix"])
                pose_world_to_cam = np.linalg.inv(pose_cam_to_world)
                self.proj_mats.append(intrinsic @ pose_world_to_cam[:3, :])
                self.camera_matrices.append(intrinsic)
                self.dist_coeffs.append(dist_coeff)
                
                self.camera_pose_summary.append(
                    {
                        "camera": cam_key,
                        "position": {
                            "x": float(pose_cam_to_world[0, 3]),
                            "y": float(pose_cam_to_world[1, 3]),
                            "z": float(pose_cam_to_world[2, 3]),
                        },
                        "axes": {
                            "x": [float(value) for value in pose_cam_to_world[:3, 0]],
                            "y": [float(value) for value in pose_cam_to_world[:3, 1]],
                            "z": [float(value) for value in pose_cam_to_world[:3, 2]],
                        },
                    }
                )
            
            print("LED Model loaded:")
            print(f"  Front: {DRONE_LED_MODEL[0]}")
            print(f"  Back-Right: {DRONE_LED_MODEL[1]}")
            print(f"  Back-Left: {DRONE_LED_MODEL[2]}")
            
            return True
        except Exception as exc:
            self.camera = None
            self.camera_ids = []
            self.camera_error = f"Camera initialisation failed: {exc}"
            self._next_camera_retry_at = time.time() + CAMERA_RETRY_SECONDS
            return False

    def close(self):
        if self.camera is not None:
            try:
                self.camera.end()
            except Exception:
                pass
        self.camera = None
        self.camera_ids = []
        self._next_camera_retry_at = 0.0
        self.camera_matrices = []
        self.dist_coeffs = []
        self.last_rotation = None
        self.last_translation = None
        self.last_led_points = None
        self.last_unwrapped_angles = None
        self.latest_preview_frames = []
        self.last_preview_update = 0.0
        self.camera_pose_summary = []
        self.reset_metrics()
        
        self.motion_state_filter.reset()
        for filter_instance in self.attitude_filters.values():
            filter_instance.reset()

    def filter_pose(self, translation, rotation_matrix):
        """Filter translation with a pos/vel/accel state model and smooth attitude."""
        yaw_raw, pitch_raw, roll_raw = rotation_matrix_to_euler_zyx(rotation_matrix)
        
        # Unwrap angles
        if self.last_unwrapped_angles is None:
            unwrapped_angles = {
                "yaw": yaw_raw,
                "pitch": pitch_raw,
                "roll": roll_raw,
            }
        else:
            unwrapped_angles = {
                "yaw": unwrap_angle_degrees(self.last_unwrapped_angles["yaw"], yaw_raw),
                "pitch": unwrap_angle_degrees(self.last_unwrapped_angles["pitch"], pitch_raw),
                "roll": unwrap_angle_degrees(self.last_unwrapped_angles["roll"], roll_raw),
            }
        
        self.last_unwrapped_angles = unwrapped_angles.copy()
        
        filtered_position_vector, filtered_velocity_vector = self.motion_state_filter.update(
            translation
        )
        filtered_position = {
            axis: round(float(value), 4)
            for axis, value in zip(("x", "y", "z"), filtered_position_vector)
        }
        filtered_velocity = {
            axis: round(float(value), 4)
            for axis, value in zip(("x", "y", "z"), filtered_velocity_vector)
        }
        
        filtered_angles_unwrapped = {
            axis: self.attitude_filters[axis].update(unwrapped_angles[axis])
            for axis in ("yaw", "pitch", "roll")
        }
        
        filtered_angles = {
            axis: round(wrap_angle_degrees(value), 2)
            for axis, value in filtered_angles_unwrapped.items()
        }
        
        return filtered_position, filtered_angles, filtered_velocity

    def read_pose(self):
        """Main tracking loop."""
        if not self.ensure_camera():
            return None

        try:
            frames, _ = self.camera.read()
        except Exception as exc:
            self.camera_error = f"Camera read failed: {exc}"
            self.close()
            return None

        if isinstance(frames, np.ndarray):
            frames = [frames]

        if len(frames) < EXPECTED_CAMERAS:
            self.camera_error = (
                f"Expected {EXPECTED_CAMERAS} frame streams but received {len(frames)}."
            )
            return None

        selected_frames = frames[:EXPECTED_CAMERAS]
        all_cam_leds = [detect_leds(frame.copy()) for frame in selected_frames]
        all_cam_leds_undistorted = [
            undistort_led_points(leds, camera_matrix, dist_coeff)
            for leds, camera_matrix, dist_coeff in zip(
                all_cam_leds,
                self.camera_matrices,
                self.dist_coeffs,
            )
        ]
        
        # Update previews and frame timing
        now = time.time()
        detected_led_counts = [len(leds) for leds in all_cam_leds]
        if (
            not self.latest_preview_frames
            or (now - self.last_preview_update) >= (1.0 / PREVIEW_HZ)
        ):
            self.latest_preview_frames = [
                {
                    "camera": f"cam{index + 1}",
                    "image": encode_preview_frame(frame, leds),
                    "ledCount": len(leds),
                }
                for index, (frame, leds) in enumerate(zip(selected_frames, all_cam_leds))
            ]
            self.last_preview_update = now
        else:
            for preview, leds in zip(self.latest_preview_frames, all_cam_leds):
                preview["ledCount"] = len(leds)
        
        # Check LED detection
        if not all(len(leds) == MAX_LEDS for leds in all_cam_leds_undistorted):
            self.camera_error = "Waiting for each camera to detect exactly three LEDs."
            self.motion_state_filter.reset()
            self.record_frame_metrics(
                detected_led_counts,
                spatial_valid=False,
                sample_time=now,
            )
            return {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                "rotation": {"yaw": 0.0, "pitch": 0.0, "roll": 0.0},
                "error": 0.0,
                "mapping_error_px": 0.0,
                "model_fit_error_m": 0.0,
                "scale_factor": 1.0,
                "solved_led_coordinates": [],
                "detected_leds_per_camera": detected_led_counts,
                "spatial_data_valid": False,
            }

        led_solution = solve_led_positions(
            all_cam_leds_undistorted,
            self.proj_mats,
            previous_led_points=self.last_led_points,
        )

        # Solve pose
        pose = solve_pose(
            all_cam_leds_undistorted,
            self.proj_mats,
            self.last_rotation,
            self.last_translation,
            precomputed_led_solution=led_solution,
        )
        
        if pose is None:
            self.camera_error = "Pose solve failed or exceeded the fitting threshold."
            self.motion_state_filter.reset()
            mapping_error_px = (
                round(float(led_solution["mapping_cost"]), 5)
                if led_solution is not None
                else 0.0
            )
            self.record_frame_metrics(
                detected_led_counts,
                spatial_valid=False,
                mapping_error_px=mapping_error_px,
                sample_time=now,
            )
            return {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                "rotation": {"yaw": 0.0, "pitch": 0.0, "roll": 0.0},
                "error": mapping_error_px,
                "mapping_error_px": mapping_error_px,
                "model_fit_error_m": 0.0,
                "scale_factor": 1.0,
                "solved_led_coordinates": [],
                "detected_leds_per_camera": detected_led_counts,
                "spatial_data_valid": False,
            }

        # Extract and filter
        raw_rotation = pose.pop("rotation_matrix")
        translation = pose.pop("translation_vector")
        mapping_shifts = pose.pop("mapping_shifts", None)
        semantic_permutation = pose.pop("semantic_permutation", None)
        semantic_led_points = pose.pop("semantic_led_points", None)
        if mapping_shifts is None:
            mapping_shifts = [0 for _ in all_cam_leds]
        if semantic_permutation is None:
            semantic_permutation = list(range(MAX_LEDS))
        filtered_position, filtered_angles, filtered_velocity = self.filter_pose(
            translation,
            raw_rotation,
        )
        
        pose["position"] = filtered_position
        pose["velocity"] = filtered_velocity
        pose["rotation"] = filtered_angles

        # Relabel preview LEDs using solved semantic ordering: Front, Right, Left.
        semantic_labels = ["F", "R", "L"]
        mapped_raw_leds = [
            cyclic_shift(leds, int(shift))
            for leds, shift in zip(all_cam_leds, mapping_shifts)
        ]
        semantic_raw_leds = [
            [leds[index] for index in semantic_permutation]
            for leds in mapped_raw_leds
        ]
        self.latest_preview_frames = [
            {
                "camera": f"cam{index + 1}",
                "image": encode_preview_frame(frame, leds, labels=semantic_labels),
                "ledCount": len(leds),
            }
            for index, (frame, leds) in enumerate(zip(selected_frames, semantic_raw_leds))
        ]
        self.last_preview_update = now
        
        # Store for next iteration
        self.last_rotation = raw_rotation.copy()
        self.last_translation = translation.copy()
        if semantic_led_points is not None:
            self.last_led_points = np.asarray(semantic_led_points, dtype=np.float32).copy()
        
        self.camera_error = ""
        pose["spatial_data_valid"] = True
        self.record_frame_metrics(
            detected_led_counts,
            spatial_valid=True,
            mapping_error_px=pose.get("mapping_error_px", 0.0),
            model_fit_error_m=pose.get("model_fit_error_m", 0.0),
            sample_time=now,
        )
        
        return pose


class ControlServer:
    def __init__(self):
        self.clients = set()
        self.control = ControlState()
        self.imu_logger = ImuDataLogger(DATA_LOG_DIR)
        self.metrics_logger = ExperimentMetricsLogger(DATA_LOG_DIR)
        self.serial_bridge = SerialBridge()
        self.serial_bridge.imu_sample_callback = self.handle_imu_sample
        self.mocap = MotionCaptureEngine()
        self.telemetry = default_telemetry()
        self.last_serial_payload = None
        self.last_serial_attempt_payload = None
        self.last_serial_payload_seq = 0
        self.last_serial_payload_size_bytes = 0
        self.last_serial_send_ok = False
        self.last_serial_send_error = ""
        self.serial_payload_sequence = 0
        self.imu_level_calibration_sequence = 0
        self.imu_level_calibration_requested_at = 0.0
        self.imu_level_calibration_sent = False
        self.imu_level_calibration_status = ""
        self.packet_counter_reset_sequence = 0
        self.session_packet_counter_reset_sequence = 0
        self.latest_mocap_log_sample = self.build_mocap_log_sample()
        self.last_mocap_yaw_unwrapped = None
        self.last_mocap_yaw_timestamp = 0.0
        self.backend_loop_hz = 0.0
        self._last_backend_loop_at = 0.0
        self.logging_status = (
            "Idle. Start a logging session from the frontend when ready."
        )
        self.session_vision_metrics_baseline = self.mocap.get_metrics_state()
        self.session_bridge_metrics_baseline = copy.deepcopy(
            self.serial_bridge.latest_bridge_metrics
        )
        self.session_controller_metrics_baseline = copy.deepcopy(
            self.serial_bridge.latest_controller_metrics
        )
        self.session_serial_payload_seq_baseline = 0
        self.metrics_log_path = None
        self.imu_log_path = None
        self.state_lock = asyncio.Lock()

    def build_mocap_log_sample(self, telemetry=None, yaw_rate=0.0):
        telemetry = telemetry if isinstance(telemetry, dict) else {}
        position = telemetry.get("position", {})
        velocity = telemetry.get("velocity", {})
        rotation = telemetry.get("rotation", {})
        target = self.control.target if isinstance(self.control.target, dict) else {}
        x_value = coerce_float(position.get("x"), 0.0)
        y_value = coerce_float(position.get("y"), 0.0)
        z_value = coerce_float(position.get("z"), 0.0)
        yaw_value = coerce_float(rotation.get("yaw"), 0.0)
        return {
            "valid": bool(telemetry.get("spatial_data_valid", False)),
            "x_error": coerce_float(target.get("x"), 0.0) - x_value,
            "y_error": coerce_float(target.get("y"), 0.0) - y_value,
            "z_error": coerce_float(target.get("z"), 0.0) - z_value,
            "vx": coerce_float(velocity.get("x"), 0.0),
            "vy": coerce_float(velocity.get("y"), 0.0),
            "vz": coerce_float(velocity.get("z"), 0.0),
            "yaw_error": wrap_angle_degrees(
                coerce_float(target.get("yaw"), 0.0) - yaw_value
            ),
            "yaw_rate": coerce_float(yaw_rate, 0.0),
        }

    def update_mocap_log_sample(self, telemetry, sample_time):
        telemetry = telemetry if isinstance(telemetry, dict) else {}
        if not telemetry.get("spatial_data_valid", False):
            self.latest_mocap_log_sample = self.build_mocap_log_sample()
            self.last_mocap_yaw_unwrapped = None
            self.last_mocap_yaw_timestamp = 0.0
            return

        yaw = coerce_float(telemetry.get("rotation", {}).get("yaw"), 0.0)
        yaw_rate = 0.0
        if self.last_mocap_yaw_unwrapped is None:
            yaw_unwrapped = yaw
        else:
            yaw_unwrapped = unwrap_angle_degrees(self.last_mocap_yaw_unwrapped, yaw)
            dt = float(sample_time - self.last_mocap_yaw_timestamp)
            if 0.0 < dt <= MOTION_STATE_MAX_DT:
                yaw_rate = (yaw_unwrapped - self.last_mocap_yaw_unwrapped) / dt

        self.last_mocap_yaw_unwrapped = yaw_unwrapped
        self.last_mocap_yaw_timestamp = float(sample_time)
        self.latest_mocap_log_sample = self.build_mocap_log_sample(telemetry, yaw_rate)

    def handle_imu_sample(self, sample, sample_time):
        self.imu_logger.log_sample(
            sample,
            sample_time,
            mocap_sample=self.latest_mocap_log_sample,
        )

    def is_logging_active(self):
        return self.metrics_logger.is_active()

    def reset_session_metric_baselines(self):
        self.packet_counter_reset_sequence += 1
        self.session_packet_counter_reset_sequence = self.packet_counter_reset_sequence
        self.session_vision_metrics_baseline = self.mocap.get_metrics_state()
        self.session_bridge_metrics_baseline = copy.deepcopy(
            self.serial_bridge.latest_bridge_metrics
        )
        self.session_controller_metrics_baseline = copy.deepcopy(
            self.serial_bridge.latest_controller_metrics
        )
        self.session_serial_payload_seq_baseline = self.last_serial_payload_seq

    def get_session_vision_metrics(self):
        return self.mocap.get_metrics_summary_since(self.session_vision_metrics_baseline)

    def get_session_bridge_metrics(self):
        return build_session_bridge_metrics(
            self.serial_bridge.latest_bridge_metrics,
            self.session_bridge_metrics_baseline,
            expected_reset_sequence=self.session_packet_counter_reset_sequence,
        )

    def get_session_controller_metrics(self):
        return build_session_controller_metrics(
            self.serial_bridge.latest_controller_metrics,
            self.session_controller_metrics_baseline,
            expected_reset_sequence=self.session_packet_counter_reset_sequence,
        )

    def get_session_serial_payload_seq(self):
        return max(
            0,
            coerce_int(self.last_serial_payload_seq, 0)
            - coerce_int(self.session_serial_payload_seq_baseline, 0),
        )

    def start_logging_session(self, status_message=None):
        self.imu_logger.stop()
        self.imu_log_path = None
        self.reset_session_metric_baselines()
        metrics_log_path = self.metrics_logger.start()
        if metrics_log_path is not None:
            self.metrics_log_path = str(metrics_log_path)

        if self.metrics_logger.is_active():
            self.logging_status = status_message or (
                "Logging session active. Click the button again to stop and save the combined session_metrics CSV."
            )
            return True

        self.logging_status = "Logging session could not be started cleanly."
        return False

    def stop_logging_session(self, status_message=None):
        metrics_log_path = self.metrics_logger.stop()
        self.imu_logger.stop()
        if metrics_log_path is not None:
            self.metrics_log_path = str(metrics_log_path)
        self.imu_log_path = None

        if self.metrics_log_path:
            self.logging_status = status_message or (
                "Logging session stopped. Combined metrics and IMU data were saved under backend/data_logs/."
            )
        else:
            self.logging_status = "Logging session stopped."
        return True

    def toggle_logging_session(self):
        if self.is_logging_active():
            return self.stop_logging_session()
        return self.start_logging_session()

    def is_imu_level_calibration_pending(self):
        if self.imu_level_calibration_sequence <= 0:
            return False
        return (
            time.time() - self.imu_level_calibration_requested_at
        ) < IMU_LEVEL_CALIBRATION_RETRY_SECONDS

    def queue_imu_level_calibration(self):
        if self.control.armed:
            self.imu_level_calibration_sent = False
            self.imu_level_calibration_status = (
                "Disarm motors before calibrating the IMU level."
            )
            return False

        if not self.control.serial_port:
            self.imu_level_calibration_sent = False
            self.imu_level_calibration_status = (
                "Select a serial port before calibrating the IMU level."
            )
            return False

        self.imu_level_calibration_sequence += 1
        self.imu_level_calibration_requested_at = time.time()
        self.imu_level_calibration_sent = False
        self.imu_level_calibration_status = (
            "Queued. Hold the drone level and still."
        )
        return True

    def build_serial_payload(self):
        spatial_data_valid = self.telemetry["spatial_data_valid"]
        drone_index = DEFAULT_DRONE_INDEX
        position = self.telemetry.get("position", {})
        velocity = self.telemetry.get("velocity", {})
        rotation = self.telemetry.get("rotation", {})
        imu_level_pending = self.is_imu_level_calibration_pending()
        self.serial_payload_sequence += 1
        payload_sequence = int(self.serial_payload_sequence)

        def make_payload(
            pose_digits,
            rotation_digits,
            target_digits,
            target_yaw_digits,
            pid_digits,
            throttle_digits,
            angle_limit_digits,
            include_version=True,
        ):
            payload = {
                "a": int(self.control.armed),
                "o": int(spatial_data_valid),
                "p": [
                    compact_numeric(position.get("x", 0.0), pose_digits),
                    compact_numeric(position.get("y", 0.0), pose_digits),
                    compact_numeric(position.get("z", 0.0), pose_digits),
                ],
                "d": [
                    compact_numeric(velocity.get("x", 0.0), pose_digits),
                    compact_numeric(velocity.get("y", 0.0), pose_digits),
                    compact_numeric(velocity.get("z", 0.0), pose_digits),
                ],
                "r": [
                    compact_numeric(rotation.get("yaw", 0.0), rotation_digits),
                    compact_numeric(rotation.get("pitch", 0.0), rotation_digits),
                    compact_numeric(rotation.get("roll", 0.0), rotation_digits),
                ],
                "g": [
                    compact_numeric(self.control.target["x"], target_digits),
                    compact_numeric(self.control.target["y"], target_digits),
                    compact_numeric(self.control.target["z"], target_digits),
                    compact_numeric(self.control.target["yaw"], target_yaw_digits),
                ],
                "u": compact_pid_bundle(self.control.pid, pid_digits),
                "m": [
                    compact_numeric(self.control.limits["hoverThrottle"], throttle_digits),
                    compact_numeric(self.control.limits["minThrottle"], throttle_digits),
                    compact_numeric(self.control.limits["maxThrottle"], throttle_digits),
                    compact_numeric(self.control.limits["maxTiltDeg"], angle_limit_digits),
                    compact_numeric(self.control.limits["maxYawRateDeg"], angle_limit_digits),
                ],
                "s": payload_sequence,
            }
            if include_version:
                payload["v"] = 2
            if imu_level_pending:
                payload["l"] = int(self.imu_level_calibration_sequence)
            return payload

        # Start with the higher-precision payload and only compact further if a
        # tuned frame would otherwise overflow the ESP-NOW transport limit.
        serial_payload = make_payload(
            pose_digits=3,
            rotation_digits=2,
            target_digits=3,
            target_yaw_digits=2,
            pid_digits=4,
            throttle_digits=3,
            angle_limit_digits=1,
            include_version=True,
        )
        if serialized_payload_size_bytes(serial_payload) > MAX_ESP_NOW_PAYLOAD_BYTES:
            serial_payload = make_payload(
                pose_digits=2,
                rotation_digits=1,
                target_digits=2,
                target_yaw_digits=1,
                pid_digits=4,
                throttle_digits=2,
                angle_limit_digits=0,
                include_version=False,
            )
        if serialized_payload_size_bytes(serial_payload) > MAX_ESP_NOW_PAYLOAD_BYTES:
            serial_payload = make_payload(
                pose_digits=2,
                rotation_digits=1,
                target_digits=2,
                target_yaw_digits=1,
                pid_digits=3,
                throttle_digits=2,
                angle_limit_digits=0,
                include_version=False,
            )
        return drone_index, serial_payload

    def build_snapshot(self):
        cameras_connected = len(self.mocap.camera_ids)
        cameras_ready = (
            cameras_connected >= EXPECTED_CAMERAS
            and self.telemetry["spatial_data_valid"]
        )
        serial_ready = self.serial_bridge.is_connected()
        serial_send_enabled = self.control.active and serial_ready
        if self.is_logging_active():
            vision_metrics = self.get_session_vision_metrics()
            bridge_metrics = self.get_session_bridge_metrics()
            controller_metrics = self.get_session_controller_metrics()
            serial_payload_seq = self.get_session_serial_payload_seq()
        else:
            vision_metrics = self.mocap.get_metrics_summary()
            bridge_metrics = copy.deepcopy(self.serial_bridge.latest_bridge_metrics)
            controller_metrics = copy.deepcopy(self.serial_bridge.latest_controller_metrics)
            serial_payload_seq = self.last_serial_payload_seq
        return {
            "type": "state",
            "control": {
                "active": self.control.active,
                "armed": self.control.armed,
                "serialPort": self.control.serial_port,
                "baudRate": self.control.baud_rate,
                "target": self.control.target,
                "limits": self.control.limits,
                "pid": self.control.pid,
            },
            "telemetry": self.telemetry,
            "system": {
                "frontendClients": len(self.clients),
                "expectedCameras": EXPECTED_CAMERAS,
                "connectedCameras": cameras_connected,
                "cameraIds": self.mocap.camera_ids,
                "cameraPoses": self.mocap.camera_pose_summary,
                "cameraPreviews": self.mocap.latest_preview_frames,
                "camerasReady": cameras_ready,
                "cameraError": self.mocap.camera_error,
                "serialConnected": serial_ready,
                "serialPort": self.serial_bridge.port,
                "serialError": self.serial_bridge.last_error,
                "availableSerialPorts": self.serial_bridge.available_ports,
                "canSendToEsp32": serial_send_enabled,
                "serialForwarding": serial_send_enabled and self.last_serial_send_ok,
                "lastSerialSendOk": self.last_serial_send_ok,
                "lastSerialSendError": self.last_serial_send_error,
                "lastSerialAttemptPayload": self.last_serial_attempt_payload,
                "lastSerialPayload": self.last_serial_payload,
                "lastSerialPayloadSeq": serial_payload_seq,
                "lastSerialPayloadSizeBytes": self.last_serial_payload_size_bytes,
                "imuLevelCalibrationPending": self.is_imu_level_calibration_pending(),
                "imuLevelCalibrationSent": self.imu_level_calibration_sent,
                "imuLevelCalibrationSequence": self.imu_level_calibration_sequence,
                "imuLevelCalibrationStatus": self.imu_level_calibration_status,
                "loggingActive": self.is_logging_active(),
                "loggingStatus": self.logging_status,
                "metricsLogPath": self.metrics_log_path,
                "imuLogPath": self.imu_log_path,
                "metrics": {
                    "backendLoopHz": round(self.backend_loop_hz, 2),
                    "vision": vision_metrics,
                    "bridge": bridge_metrics,
                    "controller": controller_metrics,
                },
            },
        }

    async def broadcast_state(self):
        if not self.clients:
            return

        message = json.dumps(self.build_snapshot())
        disconnected = set()
        for client in self.clients:
            try:
                await client.send(message)
            except Exception:
                disconnected.add(client)

        self.clients -= disconnected

    async def handle_message(self, message):
        try:
            payload = json.loads(message)
            message_type = payload.get("type")

            async with self.state_lock:
                if message_type == "set_control":
                    control_payload = payload.get("control", {})
                    if not isinstance(control_payload, dict):
                        control_payload = {}
                    previous_port = self.control.serial_port
                    previous_baud_rate = self.control.baud_rate
                    previous_armed = self.control.armed
                    self.control.active = bool(control_payload.get("active", self.control.active))
                    self.control.armed = bool(control_payload.get("armed", self.control.armed))
                    self.control.serial_port = str(
                        control_payload.get("serialPort", self.control.serial_port)
                    ).strip()
                    self.control.baud_rate = int(
                        control_payload.get("baudRate", self.control.baud_rate)
                    )
                    if "pid" in control_payload:
                        self.control.pid = sanitize_pid_config(control_payload["pid"])
                    if "target" in control_payload:
                        self.control.target = sanitize_target_config(control_payload["target"])
                    if "limits" in control_payload:
                        self.control.limits = sanitize_limit_config(control_payload["limits"])
                    if not self.control.active:
                        self.control.armed = False

                    serial_settings_changed = (
                        self.control.serial_port != previous_port
                        or self.control.baud_rate != previous_baud_rate
                    )

                    if not self.control.serial_port:
                        self.serial_bridge.disconnect()
                    elif serial_settings_changed or not self.serial_bridge.is_connected():
                        self.serial_bridge.connect(self.control.serial_port, self.control.baud_rate)

                    if self.control.armed and not previous_armed and not self.is_logging_active():
                        self.start_logging_session(
                            status_message=(
                                "Logging session started automatically because the drone was armed."
                            )
                        )
                    elif (
                        previous_armed
                        and not self.control.armed
                        and self.is_logging_active()
                    ):
                        self.stop_logging_session(
                            status_message=(
                                "Logging session stopped automatically because the drone was disarmed."
                            )
                        )
                elif message_type == "refresh_serial_ports":
                    self.serial_bridge.refresh_ports(force=True)
                elif message_type == "calibrate_imu_level":
                    self.queue_imu_level_calibration()
                elif message_type == "toggle_logging_session":
                    self.toggle_logging_session()
        except Exception as exc:
            self.serial_bridge.last_error = f"Control message handling failed: {exc}"
            print(f"Control message handling failed: {exc}")
            traceback.print_exc()

    async def register(self, websocket):
        self.clients.add(websocket)
        self.serial_bridge.refresh_ports(force=True)
        await websocket.send(json.dumps(self.build_snapshot()))

    async def unregister(self, websocket):
        self.clients.discard(websocket)

    async def websocket_handler(self, websocket):
        await self.register(websocket)
        try:
            async for message in websocket:
                await self.handle_message(message)
                await self.broadcast_state()
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as exc:
            print(f"WebSocket handler failed: {exc}")
            traceback.print_exc()
        finally:
            await self.unregister(websocket)

    async def tracking_loop(self):
        interval = 1 / TRACKING_HZ
        
        while True:
            loop_start = time.time()
            if self._last_backend_loop_at > 0.0:
                dt_seconds = loop_start - self._last_backend_loop_at
                if dt_seconds > 0.0:
                    self.backend_loop_hz = 1.0 / dt_seconds
            self._last_backend_loop_at = loop_start

            try:
                async with self.state_lock:
                    self.serial_bridge.refresh_ports()
                    if self.control.serial_port:
                        self.serial_bridge.ensure_connected(
                            self.control.serial_port,
                            self.control.baud_rate,
                        )
                    self.serial_bridge.poll_incoming()
                    try:
                        pose = self.mocap.read_pose()
                    except Exception as exc:
                        pose = None
                        self.mocap.camera_error = f"Tracking input failed: {exc}"
                        print(f"Tracking input failed: {exc}")
                        traceback.print_exc()

                    if pose is not None:
                        self.telemetry = merge_telemetry(
                            pose,
                            self.serial_bridge.get_latest_imu(),
                        )
                        self.update_mocap_log_sample(self.telemetry, time.time())
                    else:
                        self.telemetry = merge_telemetry(
                            None,
                            self.serial_bridge.get_latest_imu(),
                        )
                        self.update_mocap_log_sample(self.telemetry, time.time())
                    imu_level_pending = self.is_imu_level_calibration_pending()
                    should_send_serial = (
                        (self.control.active and self.serial_bridge.is_connected())
                        or imu_level_pending
                    )

                    if should_send_serial and self.control.serial_port:
                        drone_index, serial_payload = self.build_serial_payload()
                        self.last_serial_attempt_payload = serial_payload
                        self.last_serial_payload_seq = coerce_int(
                            serial_payload.get("s"),
                            self.last_serial_payload_seq,
                        )
                        self.last_serial_payload_size_bytes = serialized_payload_size_bytes(
                            serial_payload
                        )
                        if self.serial_bridge.send(drone_index, serial_payload):
                            self.last_serial_payload = serial_payload
                            self.last_serial_send_ok = True
                            self.last_serial_send_error = ""
                            if imu_level_pending:
                                self.imu_level_calibration_sent = True
                                self.imu_level_calibration_status = (
                                    "Level request sent. Keep the drone flat and still."
                                )
                        else:
                            self.last_serial_send_ok = False
                            self.last_serial_send_error = (
                                self.serial_bridge.last_error or "Serial send failed."
                            )
                            if imu_level_pending:
                                self.imu_level_calibration_status = (
                                    self.serial_bridge.last_error
                                    or "Level request send failed."
                                )
                    else:
                        self.last_serial_send_ok = False
                        if (
                            self.imu_level_calibration_sequence > 0
                            and not imu_level_pending
                            and not self.imu_level_calibration_sent
                            and self.imu_level_calibration_status.startswith("Queued")
                        ):
                            self.imu_level_calibration_status = (
                                "Level request expired before the serial link was ready."
                            )
                        if not self.control.active and not imu_level_pending:
                            self.last_serial_send_error = ""
                            self.last_serial_attempt_payload = None
                        elif not self.serial_bridge.is_connected():
                            self.last_serial_send_error = (
                                self.serial_bridge.last_error or "Serial link not connected."
                            )
                    self.serial_bridge.poll_incoming()
                    self.telemetry["imu"] = self.serial_bridge.get_latest_imu()
                    vision_metrics = (
                        self.get_session_vision_metrics()
                        if self.is_logging_active()
                        else self.mocap.get_metrics_summary()
                    )
                    bridge_metrics = (
                        self.get_session_bridge_metrics()
                        if self.is_logging_active()
                        else copy.deepcopy(self.serial_bridge.latest_bridge_metrics)
                    )
                    controller_metrics = (
                        self.get_session_controller_metrics()
                        if self.is_logging_active()
                        else copy.deepcopy(self.serial_bridge.latest_controller_metrics)
                    )
                    serial_payload_seq = (
                        self.get_session_serial_payload_seq()
                        if self.is_logging_active()
                        else self.last_serial_payload_seq
                    )
                    self.metrics_logger.log_sample(
                        sample_time=time.time(),
                        telemetry=self.telemetry,
                        vision_metrics=vision_metrics,
                        bridge_metrics=bridge_metrics,
                        controller_metrics=controller_metrics,
                        backend_loop_hz=self.backend_loop_hz,
                        serial_connected=self.serial_bridge.is_connected(),
                        serial_send_enabled=should_send_serial
                        and bool(self.control.serial_port),
                        serial_send_ok=self.last_serial_send_ok,
                        serial_payload_seq=serial_payload_seq,
                        serial_payload_size_bytes=self.last_serial_payload_size_bytes,
                        imu_sample=self.telemetry.get("imu", {}),
                        mocap_sample=self.latest_mocap_log_sample,
                    )
            except Exception as exc:
                self.telemetry = merge_telemetry(
                    None,
                    self.serial_bridge.get_latest_imu(),
                )
                self.update_mocap_log_sample(self.telemetry, time.time())
                self.mocap.camera_error = f"Tracking loop failed: {exc}"
                print(f"Tracking loop failed: {exc}")
                traceback.print_exc()
                vision_metrics = (
                    self.get_session_vision_metrics()
                    if self.is_logging_active()
                    else self.mocap.get_metrics_summary()
                )
                bridge_metrics = (
                    self.get_session_bridge_metrics()
                    if self.is_logging_active()
                    else copy.deepcopy(self.serial_bridge.latest_bridge_metrics)
                )
                controller_metrics = (
                    self.get_session_controller_metrics()
                    if self.is_logging_active()
                    else copy.deepcopy(self.serial_bridge.latest_controller_metrics)
                )
                serial_payload_seq = (
                    self.get_session_serial_payload_seq()
                    if self.is_logging_active()
                    else self.last_serial_payload_seq
                )
                self.metrics_logger.log_sample(
                    sample_time=time.time(),
                    telemetry=self.telemetry,
                    vision_metrics=vision_metrics,
                    bridge_metrics=bridge_metrics,
                    controller_metrics=controller_metrics,
                    backend_loop_hz=self.backend_loop_hz,
                    serial_connected=self.serial_bridge.is_connected(),
                    serial_send_enabled=False,
                    serial_send_ok=self.last_serial_send_ok,
                    serial_payload_seq=serial_payload_seq,
                    serial_payload_size_bytes=self.last_serial_payload_size_bytes,
                    imu_sample=self.telemetry.get("imu", {}),
                    mocap_sample=self.latest_mocap_log_sample,
                )

            await self.broadcast_state()
            
            elapsed = time.time() - loop_start
            sleep_time = max(0, interval - elapsed)
            await asyncio.sleep(sleep_time)

    async def close(self):
        self.serial_bridge.disconnect()
        self.imu_logger.close()
        self.metrics_logger.close()
        self.mocap.close()


async def main():
    server = ControlServer()
    tracking_task = asyncio.create_task(server.tracking_loop())
    tracking_task.set_name("tracking_loop")

    def report_task_result(task):
        try:
            task.result()
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            print(f"Background task '{task.get_name()}' failed: {exc}", file=sys.stderr)
            traceback.print_exc()

    tracking_task.add_done_callback(report_task_result)

    try:
        async with websockets.serve(
            server.websocket_handler,
            HOST,
            PORT,
            process_request=http_health_response,
        ):
            print(f"Control server running on ws://{HOST}:{PORT}")
            await asyncio.Future()
    finally:
        tracking_task.cancel()
        try:
            await tracking_task
        except asyncio.CancelledError:
            pass
        await server.close()


if __name__ == "__main__":
    try:
        print("Python faulthandler enabled.", flush=True)
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Server stopped.")
    except Exception as exc:
        print(f"Fatal server error: {exc}", file=sys.stderr)
        traceback.print_exc()
