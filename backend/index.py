# ... (keeping all imports and other code the same)
import asyncio
import base64
import faulthandler
import json
import math
import sys
import time
import traceback
from dataclasses import dataclass, field
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
CAMERA_RETRY_SECONDS = 5.0
DEFAULT_BAUD_RATE = 1000000
DEFAULT_DRONE_INDEX = 0
POSITION_JUMP_WEIGHT = 2.0
PREVIEW_HZ = 5
MAX_ESP_NOW_PAYLOAD_BYTES = 250
IMU_LEVEL_CALIBRATION_RETRY_SECONDS = 1.0

CONTROL_PID_DEFAULTS = {
    "x": {"kp": 17.5, "ki": 0.0, "kd": 0.5},
    "y": {"kp": 17.5, "ki": 0.0, "kd": 0.5},
    "z": {"kp": 0.9, "ki": 0.35, "kd": 0.18},
    "yaw": {"kp": 4.5, "ki": 0.0, "kd": 0.12},
    "roll": {"kp": 0.022, "ki": 0.0, "kd": 0.0014},
    "pitch": {"kp": 0.022, "ki": 0.0, "kd": 0.0014},
    "yawRate": {"kp": 0.006, "ki": 0.0, "kd": 0.0},
}
# Mocap/body frame is +x front, +y left, +z up. A yaw target of 0 deg means
# the drone's nose should stay aligned with the world +x direction.
CONTROL_TARGET_DEFAULTS = {"x": 0.0, "y": 0.0, "z": 0.35, "yaw": 0.0}
CONTROL_LIMIT_DEFAULTS = {
    "hoverThrottle": 0.36,
    "minThrottle": 0.18,
    "maxThrottle": 0.82,
    "maxTiltDeg": 12.0,
    "maxYawRateDeg": 120.0,
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

DEFAULT_TELEMETRY = {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "rotation": {"yaw": 0.0, "pitch": 0.0, "roll": 0.0},
    "error": 0.0,
    "mapping_error_px": 0.0,
    "model_fit_error_m": 0.0,
    "scale_factor": 1.0,
    "solved_led_coordinates": [],
    "detected_leds_per_camera": [0, 0],
    "spatial_data_valid": False,
}
PREVIEW_WIDTH = 240
PREVIEW_JPEG_QUALITY = 45
MAX_LED_REORDER_COST_PIXELS = 50
MAPPING_TEMPORAL_WEIGHT = 120.0


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
    return round(float(value), digits)


def compact_pid_triplet(pid_values):
    return [
        compact_numeric(pid_values["kp"], 4),
        compact_numeric(pid_values["ki"], 4),
        compact_numeric(pid_values["kd"], 4),
    ]


def compact_pid_bundle(pid_config):
    ordered_axes = ("x", "y", "z", "yaw", "roll", "pitch", "yawRate")
    values = []
    for axis in ordered_axes:
        values.extend(compact_pid_triplet(pid_config[axis]))
    return values


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
                "y": moments["m01"] / moments["m00"]
            })

    if len(led_candidates) < MAX_LEDS:
        return [(l["x"], l["y"]) for l in led_candidates]

    # 1. Take top 3 by area/intensity if more than 3 found (optional filter)
    # For now, we assume the 3 brightest blobs are the LEDs
    top_three = led_candidates[:MAX_LEDS]

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
    Returns mapped observations, mapping quality, and F/R/L 3D coordinates.
    """
    mapped_leds, mapping_cost, mapping_shifts, candidate_points = find_best_clockwise_mapping(
        all_cam_leds,
        proj_mats,
        previous_led_points=previous_led_points,
    )
    if mapped_leds is None:
        return None

    solved_led_coordinates = [
        {
            "label": label,
            "x": round(float(point[0]), 4),
            "y": round(float(point[1]), 4),
            "z": round(float(point[2]), 4),
        }
        for label, point in zip(["F", "R", "L"], candidate_points)
    ]

    return {
        "mapped_leds": mapped_leds,
        "mapping_cost": float(mapping_cost),
        "mapping_shifts": [int(value) for value in mapping_shifts],
        "candidate_points": candidate_points,
        "solved_led_coordinates": solved_led_coordinates,
    }


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
    solved_led_coordinates = led_solution["solved_led_coordinates"]

    if mapping_cost >= MAX_LED_REORDER_COST_PIXELS:
        return None

    # Solve for pose using Procrustes alignment
    scale_factor, rotation, translation = solve_pose_procrustes(candidate_points, DRONE_LED_MODEL)

    # Compute fit error
    transformed_model = scale_factor * (rotation @ DRONE_LED_MODEL.T).T + translation
    fit_error = compute_reprojection_error(candidate_points, transformed_model)

    # Check if fit is acceptable
    if fit_error >= MAX_FIT_ERROR:
        return None

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
            self.last_error = ""
            return True
        except Exception as exc:
            self.connection = None
            self.port = ""
            self._connected_at = 0.0
            self._consecutive_send_failures = 0
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

    def is_connected(self):
        return bool(self.connection and self.connection.is_open)

    def drain_input(self):
        if not self.is_connected():
            return

        try:
            waiting = getattr(self.connection, "in_waiting", 0)
            if waiting:
                self.connection.read(waiting)
        except Exception:
            pass

    def send(self, drone_index, payload):
        if not self.is_connected():
            return False
        if (time.time() - self._connected_at) < SERIAL_SETTLE_SECONDS:
            self.last_error = "Serial link settling after reconnect."
            return False

        try:
            self.drain_input()
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
        
        # Position filters
        self.position_filters = {
            "x": ScalarKalmanFilter(1e-4, 2e-3),
            "y": ScalarKalmanFilter(1e-4, 2e-3),
            "z": ScalarKalmanFilter(1e-4, 2e-3),
        }
        
        # Attitude filters
        self.attitude_filters = {
            "yaw": ScalarKalmanFilter(1.0, 2.0),
            "pitch": ScalarKalmanFilter(1.0, 2.0),
            "roll": ScalarKalmanFilter(1.0, 2.0),
        }

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
        
        for filter_instance in self.position_filters.values():
            filter_instance.reset()
        for filter_instance in self.attitude_filters.values():
            filter_instance.reset()

    def filter_pose(self, translation, rotation_matrix):
        """Apply Kalman filtering."""
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
        
        # Filter
        filtered_position = {
            "x": round(self.position_filters["x"].update(float(translation[0])), 4),
            "y": round(self.position_filters["y"].update(float(translation[1])), 4),
            "z": round(self.position_filters["z"].update(float(translation[2])), 4),
        }
        
        filtered_angles_unwrapped = {
            axis: self.attitude_filters[axis].update(unwrapped_angles[axis])
            for axis in ("yaw", "pitch", "roll")
        }
        
        filtered_angles = {
            axis: round(wrap_angle_degrees(value), 2)
            for axis, value in filtered_angles_unwrapped.items()
        }
        
        return filtered_position, filtered_angles

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
        
        # Update previews
        now = time.time()
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
            self.camera_error = "Waiting for all three cameras to detect exactly three LEDs."
            return {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "rotation": {"yaw": 0.0, "pitch": 0.0, "roll": 0.0},
                "error": 0.0,
                "mapping_error_px": 0.0,
                "model_fit_error_m": 0.0,
                "scale_factor": 1.0,
                "solved_led_coordinates": [],
                "detected_leds_per_camera": [len(leds) for leds in all_cam_leds],
                "spatial_data_valid": False,
            }

        led_solution = solve_led_positions(
            all_cam_leds_undistorted,
            self.proj_mats,
            previous_led_points=self.last_led_points,
        )
        solved_led_coordinates = (
            led_solution["solved_led_coordinates"] if led_solution is not None else []
        )
        if led_solution is not None:
            self.last_led_points = np.asarray(
                led_solution["candidate_points"], dtype=np.float32
            ).copy()

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
            return {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "rotation": {"yaw": 0.0, "pitch": 0.0, "roll": 0.0},
                "error": (
                    round(float(led_solution["mapping_cost"]), 5)
                    if led_solution is not None
                    else 0.0
                ),
                "mapping_error_px": (
                    round(float(led_solution["mapping_cost"]), 5)
                    if led_solution is not None
                    else 0.0
                ),
                "model_fit_error_m": 0.0,
                "scale_factor": 1.0,
                "solved_led_coordinates": solved_led_coordinates,
                "detected_leds_per_camera": [len(leds) for leds in all_cam_leds],
                "spatial_data_valid": False,
            }

        # Extract and filter
        raw_rotation = pose.pop("rotation_matrix")
        translation = pose.pop("translation_vector")
        mapping_shifts = pose.pop("mapping_shifts", None)
        if mapping_shifts is None:
            mapping_shifts = [0 for _ in all_cam_leds]
        filtered_position, filtered_angles = self.filter_pose(translation, raw_rotation)
        
        pose["position"] = filtered_position
        pose["rotation"] = filtered_angles

        # Relabel preview LEDs using solved semantic ordering: Front, Right, Left.
        semantic_labels = ["F", "R", "L"]
        mapped_raw_leds = [
            cyclic_shift(leds, int(shift))
            for leds, shift in zip(all_cam_leds, mapping_shifts)
        ]
        self.latest_preview_frames = [
            {
                "camera": f"cam{index + 1}",
                "image": encode_preview_frame(frame, leds, labels=semantic_labels),
                "ledCount": len(leds),
            }
            for index, (frame, leds) in enumerate(zip(selected_frames, mapped_raw_leds))
        ]
        self.last_preview_update = now
        
        # Store for next iteration
        self.last_rotation = raw_rotation.copy()
        self.last_translation = translation.copy()
        
        self.camera_error = ""
        pose["spatial_data_valid"] = True
        
        return pose


class ControlServer:
    def __init__(self):
        self.clients = set()
        self.control = ControlState()
        self.serial_bridge = SerialBridge()
        self.mocap = MotionCaptureEngine()
        self.telemetry = dict(DEFAULT_TELEMETRY)
        self.last_serial_payload = None
        self.last_serial_attempt_payload = None
        self.last_serial_send_ok = False
        self.last_serial_send_error = ""
        self.imu_level_calibration_sequence = 0
        self.imu_level_calibration_requested_at = 0.0
        self.imu_level_calibration_sent = False
        self.imu_level_calibration_status = ""
        self.state_lock = asyncio.Lock()

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
        rotation = self.telemetry.get("rotation", {})
        # Keep keys short so the full controller frame stays comfortably within
        # the ESP-NOW payload limit even with non-zero pose and tuning values.
        serial_payload = {
            "v": 2,
            "a": int(self.control.armed),
            "o": int(spatial_data_valid),
            "p": [
                compact_numeric(position.get("x", 0.0), 4),
                compact_numeric(position.get("y", 0.0), 4),
                compact_numeric(position.get("z", 0.0), 4),
            ],
            "r": [
                compact_numeric(rotation.get("yaw", 0.0), 2),
                compact_numeric(rotation.get("pitch", 0.0), 2),
                compact_numeric(rotation.get("roll", 0.0), 2),
            ],
            "g": [
                compact_numeric(self.control.target["x"], 4),
                compact_numeric(self.control.target["y"], 4),
                compact_numeric(self.control.target["z"], 4),
                compact_numeric(self.control.target["yaw"], 2),
            ],
            "u": compact_pid_bundle(self.control.pid),
            "m": [
                compact_numeric(self.control.limits["hoverThrottle"], 3),
                compact_numeric(self.control.limits["minThrottle"], 3),
                compact_numeric(self.control.limits["maxThrottle"], 3),
                compact_numeric(self.control.limits["maxTiltDeg"], 1),
                compact_numeric(self.control.limits["maxYawRateDeg"], 1),
            ],
        }
        if self.is_imu_level_calibration_pending():
            serial_payload["l"] = int(self.imu_level_calibration_sequence)
        return drone_index, serial_payload

    def build_snapshot(self):
        cameras_connected = len(self.mocap.camera_ids)
        cameras_ready = (
            cameras_connected >= EXPECTED_CAMERAS
            and self.telemetry["spatial_data_valid"]
        )
        serial_ready = self.serial_bridge.is_connected()
        serial_send_enabled = self.control.active and serial_ready
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
                "imuLevelCalibrationPending": self.is_imu_level_calibration_pending(),
                "imuLevelCalibrationSent": self.imu_level_calibration_sent,
                "imuLevelCalibrationSequence": self.imu_level_calibration_sequence,
                "imuLevelCalibrationStatus": self.imu_level_calibration_status,
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
                elif message_type == "refresh_serial_ports":
                    self.serial_bridge.refresh_ports(force=True)
                elif message_type == "calibrate_imu_level":
                    self.queue_imu_level_calibration()
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

            try:
                async with self.state_lock:
                    self.serial_bridge.refresh_ports()
                    if self.control.serial_port:
                        self.serial_bridge.ensure_connected(
                            self.control.serial_port,
                            self.control.baud_rate,
                        )
                    try:
                        pose = self.mocap.read_pose()
                    except Exception as exc:
                        pose = None
                        self.mocap.camera_error = f"Tracking input failed: {exc}"
                        print(f"Tracking input failed: {exc}")
                        traceback.print_exc()

                    if pose is not None:
                        self.telemetry = pose
                    else:
                        self.telemetry = dict(DEFAULT_TELEMETRY)

                    snapshot = self.build_snapshot()
                    imu_level_pending = self.is_imu_level_calibration_pending()
                    should_send_serial = (
                        snapshot["system"]["canSendToEsp32"] or imu_level_pending
                    )

                    if should_send_serial and self.control.serial_port:
                        drone_index, serial_payload = self.build_serial_payload()
                        self.last_serial_attempt_payload = serial_payload
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
            except Exception as exc:
                self.telemetry = dict(DEFAULT_TELEMETRY)
                self.mocap.camera_error = f"Tracking loop failed: {exc}"
                print(f"Tracking loop failed: {exc}")
                traceback.print_exc()

            await self.broadcast_state()
            
            elapsed = time.time() - loop_start
            sleep_time = max(0, interval - elapsed)
            await asyncio.sleep(sleep_time)

    async def close(self):
        self.serial_bridge.disconnect()
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
