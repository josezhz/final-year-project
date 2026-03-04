import asyncio
import base64
import json
import math
import time
from dataclasses import dataclass, field
from http import HTTPStatus
from itertools import permutations
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
except ImportError:  # pragma: no cover - depends on local environment
    serial = None
    list_ports = None


BASE_DIR = Path(__file__).resolve().parent
INTRINSICS_PATH = BASE_DIR / "calibration" / "camera_intrinsics.json"
EXTRINSICS_PATH = BASE_DIR / "calibration" / "camera_extrinsics.json"

HOST = "localhost"
PORT = 8765
EXPECTED_CAMERAS = 3
MIN_BRIGHTNESS = 50
MAX_LEDS = 3
MAX_FIT_ERROR = 0.05
TRACKING_HZ = 20
SERIAL_REFRESH_SECONDS = 1.0
DEFAULT_BAUD_RATE = 115200
ALLOW_DEFAULT_TELEMETRY_SEND = True
POSITION_JUMP_WEIGHT = 15.0
PREVIEW_HZ = 5

# Front LED, back-left LED, back-right LED in meters.
DRONE_LED_MODEL = np.array(
    [
        [0.050, 0.000, 0.025],
        [0.000,-0.050, 0.025],
        [0.000, 0.050, 0.025],
    ],
    dtype=np.float32,
)

DEFAULT_TELEMETRY = {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "rotation": {"yaw": 0.0, "pitch": 0.0, "roll": 0.0},
    "error": 0.0,
    "detected_leds_per_camera": [0, 0, 0],
    "ready_to_send": False,
    "using_default_data": True,
}
PREVIEW_WIDTH = 240
PREVIEW_JPEG_QUALITY = 45
MAX_LED_REORDER_COST_PIXELS = 80.0


def encode_preview_frame(frame, leds=None):
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
        cv.putText(
            preview,
            str(led_index),
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
    a_matrix = []
    for proj_mat, (u_coord, v_coord) in zip(proj_mats, pts_2d):
        a_matrix.append(u_coord * proj_mat[2, :] - proj_mat[0, :])
        a_matrix.append(v_coord * proj_mat[2, :] - proj_mat[1, :])

    _, _, vt = np.linalg.svd(np.array(a_matrix))
    homogeneous = vt[-1]
    return homogeneous[:3] / homogeneous[3]


def project_world_point(proj_mat, point_world):
    homogeneous_point = np.append(np.asarray(point_world, dtype=np.float32), 1.0)
    projected = proj_mat @ homogeneous_point
    if abs(projected[2]) < 1e-6:
        return None
    return projected[:2] / projected[2]


def reorder_leds_by_prediction(all_cam_leds, proj_mats, rotation, translation):
    transformed_model = (rotation @ DRONE_LED_MODEL.T).T + translation
    reordered_leds = []

    for camera_leds, proj_mat in zip(all_cam_leds, proj_mats):
        predicted_leds = [
            project_world_point(proj_mat, model_point) for model_point in transformed_model
        ]
        if any(predicted is None for predicted in predicted_leds):
            return all_cam_leds, False

        best_perm = None
        best_cost = float("inf")
        for permutation_index in permutations(range(MAX_LEDS)):
            cost = 0.0
            for model_index, led_index in enumerate(permutation_index):
                detected = np.array(camera_leds[led_index], dtype=np.float32)
                predicted = np.array(predicted_leds[model_index], dtype=np.float32)
                cost += float(np.linalg.norm(detected - predicted))
            if cost < best_cost:
                best_cost = cost
                best_perm = permutation_index

        if best_perm is None or (best_cost / MAX_LEDS) > MAX_LED_REORDER_COST_PIXELS:
            return all_cam_leds, False

        reordered_leds.append([camera_leds[index] for index in best_perm])

    return reordered_leds, True


def get_euler_angles(rotation_matrix):
    sy = math.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)
    if sy > 1e-6:
        x_rot = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        y_rot = math.atan2(-rotation_matrix[2, 0], sy)
        z_rot = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        x_rot = math.atan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        y_rot = math.atan2(-rotation_matrix[2, 0], sy)
        z_rot = 0

    return np.degrees([z_rot, y_rot, x_rot])


def get_rotation_delta_degrees(reference_rotation, candidate_rotation):
    relative_rotation = reference_rotation.T @ candidate_rotation
    trace_value = np.trace(relative_rotation)
    cos_theta = max(-1.0, min(1.0, (trace_value - 1.0) / 2.0))
    return math.degrees(math.acos(cos_theta))


def wrap_angle_degrees(angle):
    return ((angle + 180.0) % 360.0) - 180.0


def unwrap_angle_degrees(previous_angle, next_angle):
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
    _, mask = cv.threshold(frame, MIN_BRIGHTNESS, 255, cv.THRESH_BINARY)
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    leds = []
    for contour in contours:
        moments = cv.moments(contour)
        if moments["m00"] > 2:
            leds.append(
                {
                    "x": moments["m10"] / moments["m00"],
                    "y": moments["m01"] / moments["m00"],
                    "area": moments["m00"],
                }
            )

    leds.sort(key=lambda led: led["area"], reverse=True)
    return [(led["x"], led["y"]) for led in leds[:MAX_LEDS]]


def solve_pose(
    all_cam_leds,
    proj_mats,
    previous_rotation=None,
    previous_position=None,
    permutations_to_try=None,
):
    best_error = float("inf")
    best_fit_error = float("inf")
    best_pose = None
    best_rotation_delta = float("inf")
    best_position_delta = float("inf")

    if permutations_to_try is None:
        permutations_to_try = [
            (perm_one, perm_two, perm_three)
            for perm_one in permutations(range(MAX_LEDS))
            for perm_two in permutations(range(MAX_LEDS))
            for perm_three in permutations(range(MAX_LEDS))
        ]

    for perm_one, perm_two, perm_three in permutations_to_try:
                candidate_points = []
                for index in range(MAX_LEDS):
                    pts_2d = [
                        all_cam_leds[0][perm_one[index]],
                        all_cam_leds[1][perm_two[index]],
                        all_cam_leds[2][perm_three[index]],
                    ]
                    candidate_points.append(triangulate_n_views(proj_mats, pts_2d))

                candidate_points = np.array(candidate_points, dtype=np.float32)
                centroid_model = np.mean(DRONE_LED_MODEL, axis=0)
                centroid_data = np.mean(candidate_points, axis=0)

                model_centered = DRONE_LED_MODEL - centroid_model
                data_centered = candidate_points - centroid_data

                covariance = model_centered.T @ data_centered
                u_mat, _, vt = np.linalg.svd(covariance)
                rotation = vt.T @ u_mat.T

                if np.linalg.det(rotation) < 0:
                    vt[-1, :] *= -1
                    rotation = vt.T @ u_mat.T

                translation = centroid_data - rotation @ centroid_model
                transformed = (rotation @ DRONE_LED_MODEL.T).T + translation
                fit_error = np.mean(np.linalg.norm(candidate_points - transformed, axis=1))
                rotation_delta = (
                    get_rotation_delta_degrees(previous_rotation, rotation)
                    if previous_rotation is not None
                    else 0.0
                )
                position_delta = (
                    float(np.linalg.norm(translation - previous_position))
                    if previous_position is not None
                    else 0.0
                )
                weighted_error = fit_error + POSITION_JUMP_WEIGHT * position_delta

                if weighted_error < best_error - 1e-6 or (
                    abs(weighted_error - best_error) <= 1e-6
                    and (
                        rotation_delta < best_rotation_delta
                        or (
                            abs(rotation_delta - best_rotation_delta) <= 1e-6
                            and position_delta < best_position_delta
                        )
                    )
                ):
                    best_error = weighted_error
                    best_fit_error = fit_error
                    best_rotation_delta = rotation_delta
                    best_position_delta = position_delta
                    best_pose = (rotation, translation)

    if not best_pose or best_fit_error >= MAX_FIT_ERROR:
        return None

    rotation, translation = best_pose
    return {
        "position": {
            "x": round(float(translation[0]), 4),
            "y": round(float(translation[1]), 4),
            "z": round(float(translation[2]), 4),
        },
        "translation_vector": translation,
        "rotation_matrix": rotation,
        "error": round(float(best_fit_error), 5),
        "detected_leds_per_camera": [len(leds) for leds in all_cam_leds],
    }


@dataclass
class ControlState:
    active: bool = False
    serial_port: str = ""
    baud_rate: int = DEFAULT_BAUD_RATE
    pid: dict = field(
        default_factory=lambda: {
            "x": {"kp": 0.0, "ki": 0.0, "kd": 0.0},
            "y": {"kp": 0.0, "ki": 0.0, "kd": 0.0},
            "z": {"kp": 0.0, "ki": 0.0, "kd": 0.0},
            "yaw": {"kp": 0.0, "ki": 0.0, "kd": 0.0},
        }
    )


class SerialBridge:
    def __init__(self):
        self.connection = None
        self.port = ""
        self.baud_rate = DEFAULT_BAUD_RATE
        self.last_error = ""
        self.available_ports = []
        self._last_refresh = 0.0

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
            self.connection = serial.Serial(port=port, baudrate=int(baud_rate), timeout=0.1)
            self.port = port
            self.baud_rate = int(baud_rate)
            self.last_error = ""
            return True
        except Exception as exc:  # pragma: no cover - depends on hardware
            self.connection = None
            self.port = ""
            self.last_error = f"Serial connection failed: {exc}"
            return False

    def disconnect(self):
        if self.connection:
            try:
                self.connection.close()
            except Exception:
                pass
        self.connection = None
        self.port = ""

    def is_connected(self):
        return bool(self.connection and self.connection.is_open)

    def send(self, payload):
        if not self.is_connected():
            return False

        try:
            encoded = (json.dumps(payload) + "\n").encode("utf-8")
            self.connection.write(encoded)
            self.connection.flush()
            self.last_error = ""
            return True
        except Exception as exc:  # pragma: no cover - depends on hardware
            self.last_error = f"Serial write failed: {exc}"
            self.disconnect()
            return False


class MotionCaptureEngine:
    def __init__(self):
        self.intrinsics = load_json(INTRINSICS_PATH)
        self.extrinsics = load_json(EXTRINSICS_PATH)
        self.camera = None
        self.camera_ids = []
        self.camera_error = ""
        self.proj_mats = []
        self.last_rotation = None
        self.last_translation = None
        self.last_relative_angles = None
        self.last_filtered_pose = None
        self.latest_preview_frames = []
        self.last_preview_update = 0.0
        self.camera_pose_summary = []
        self.position_filters = {
            "x": ScalarKalmanFilter(1e-4, 4e-4),
            "y": ScalarKalmanFilter(1e-4, 4e-4),
            "z": ScalarKalmanFilter(1e-4, 4e-4),
        }
        self.attitude_filters = {
            "yaw": ScalarKalmanFilter(0.8, 9.0),
            "pitch": ScalarKalmanFilter(0.5, 6.0),
            "roll": ScalarKalmanFilter(0.5, 6.0),
        }

    def ensure_camera(self):
        if self.camera is not None:
            return True

        try:
            camera = Camera(fps=60, resolution=Camera.RES_LARGE, colour=False)
            self.camera_ids = list(camera.ids)
            if len(self.camera_ids) < EXPECTED_CAMERAS:
                self.camera_error = (
                    f"Expected {EXPECTED_CAMERAS} cameras but found {len(self.camera_ids)}."
                )
                camera.end()
                return False

            self.camera = camera
            self.camera_error = ""
            self.proj_mats = []
            self.camera_pose_summary = []
            for index in range(EXPECTED_CAMERAS):
                cam_key = f"cam{index + 1}"
                intrinsic = np.array(self.intrinsics[cam_key]["camera_matrix"])
                pose_cam_to_world = np.array(self.extrinsics[cam_key]["pose_matrix"])
                pose_world_to_cam = np.linalg.inv(pose_cam_to_world)
                self.proj_mats.append(intrinsic @ pose_world_to_cam[:3, :])
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
            return True
        except Exception as exc:  # pragma: no cover - depends on hardware
            self.camera = None
            self.camera_ids = []
            self.camera_error = f"Camera initialisation failed: {exc}"
            return False

    def close(self):
        if self.camera is not None:
            try:
                self.camera.end()
            except Exception:
                pass
        self.camera = None
        self.camera_ids = []
        self.last_rotation = None
        self.last_translation = None
        self.last_relative_angles = None
        self.last_filtered_pose = None
        self.latest_preview_frames = []
        self.last_preview_update = 0.0
        self.camera_pose_summary = []
        for filter_instance in self.position_filters.values():
            filter_instance.reset()
        for filter_instance in self.attitude_filters.values():
            filter_instance.reset()

    def filter_pose(self, translation, relative_rotation):
        raw_yaw, raw_pitch, raw_roll = get_euler_angles(relative_rotation)
        angle_measurements = {
            "yaw": float(raw_yaw),
            "pitch": float(raw_pitch),
            "roll": float(raw_roll),
        }

        if self.last_relative_angles is None:
            unwrapped_angles = dict(angle_measurements)
        else:
            unwrapped_angles = {
                axis: unwrap_angle_degrees(self.last_relative_angles[axis], value)
                for axis, value in angle_measurements.items()
            }

        filtered_position = {
            axis: round(self.position_filters[axis].update(float(value)), 4)
            for axis, value in zip(("x", "y", "z"), translation)
        }
        filtered_angles = {
            axis: round(
                wrap_angle_degrees(self.attitude_filters[axis].update(unwrapped_angles[axis])),
                2,
            )
            for axis in ("yaw", "pitch", "roll")
        }
        self.last_relative_angles = unwrapped_angles
        self.last_filtered_pose = {
            "position": filtered_position,
            "rotation": filtered_angles,
        }
        return filtered_position, filtered_angles

    def read_pose(self):
        if not self.ensure_camera():
            return None

        try:
            frames, _ = self.camera.read()
        except Exception as exc:  # pragma: no cover - depends on hardware
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
        used_prediction_tracking = False
        if self.last_rotation is not None and self.last_translation is not None:
            all_cam_leds, used_prediction_tracking = reorder_leds_by_prediction(
                all_cam_leds,
                self.proj_mats,
                self.last_rotation,
                self.last_translation,
            )
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
        if not all(len(leds) == MAX_LEDS for leds in all_cam_leds):
            self.camera_error = "Waiting for all three cameras to detect exactly three LEDs."
            return {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "rotation": {"yaw": 0.0, "pitch": 0.0, "roll": 0.0},
                "error": 0.0,
                "detected_leds_per_camera": [len(leds) for leds in all_cam_leds],
                "ready_to_send": False,
            }

        identity_permutations = [(tuple(range(MAX_LEDS)),) * EXPECTED_CAMERAS]
        pose = solve_pose(
            all_cam_leds,
            self.proj_mats,
            previous_rotation=self.last_rotation,
            previous_position=self.last_translation,
            permutations_to_try=identity_permutations if used_prediction_tracking else None,
        )
        if pose is None and used_prediction_tracking:
            pose = solve_pose(
                all_cam_leds,
                self.proj_mats,
                previous_rotation=self.last_rotation,
                previous_position=self.last_translation,
            )
        if pose is None:
            self.camera_error = "Pose solve failed or exceeded the fitting threshold."
            return {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "rotation": {"yaw": 0.0, "pitch": 0.0, "roll": 0.0},
                "error": 0.0,
                "detected_leds_per_camera": [len(leds) for leds in all_cam_leds],
                "ready_to_send": False,
            }

        raw_rotation = pose.pop("rotation_matrix")
        translation = pose.pop("translation_vector")
        relative_rotation = raw_rotation
        filtered_position, filtered_angles = self.filter_pose(translation, relative_rotation)
        pose["position"] = filtered_position
        pose["rotation"] = filtered_angles
        pose["used_prediction_tracking"] = used_prediction_tracking
        self.last_rotation = raw_rotation.copy()
        self.last_translation = translation.copy()
        self.camera_error = ""
        pose["ready_to_send"] = True
        return pose


class ControlServer:
    def __init__(self):
        self.clients = set()
        self.control = ControlState()
        self.serial_bridge = SerialBridge()
        self.mocap = MotionCaptureEngine()
        self.telemetry = {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "rotation": {"yaw": 0.0, "pitch": 0.0, "roll": 0.0},
            "error": 0.0,
            "detected_leds_per_camera": [0, 0, 0],
            "ready_to_send": False,
            "using_default_data": True,
        }
        self.last_serial_payload = None
        self.state_lock = asyncio.Lock()

    def build_snapshot(self):
        cameras_connected = len(self.mocap.camera_ids)
        cameras_ready = cameras_connected >= EXPECTED_CAMERAS and self.telemetry["ready_to_send"]
        serial_ready = self.serial_bridge.is_connected()
        return {
            "type": "state",
            "control": {
                "active": self.control.active,
                "serialPort": self.control.serial_port,
                "baudRate": self.control.baud_rate,
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
                "canSendToEsp32": self.control.active and serial_ready,
                "sendingDefaultTelemetry": (
                    self.control.active
                    and serial_ready
                    and (ALLOW_DEFAULT_TELEMETRY_SEND and not cameras_ready)
                ),
                "lastSerialPayload": self.last_serial_payload,
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
        payload = json.loads(message)
        message_type = payload.get("type")

        async with self.state_lock:
            if message_type == "set_control":
                control_payload = payload.get("control", {})
                self.control.active = bool(control_payload.get("active", self.control.active))
                self.control.serial_port = str(
                    control_payload.get("serialPort", self.control.serial_port)
                ).strip()
                self.control.baud_rate = int(
                    control_payload.get("baudRate", self.control.baud_rate)
                )
                if "pid" in control_payload:
                    self.control.pid = control_payload["pid"]

                if self.control.serial_port:
                    self.serial_bridge.connect(self.control.serial_port, self.control.baud_rate)
                else:
                    self.serial_bridge.disconnect()
            elif message_type == "refresh_serial_ports":
                self.serial_bridge.refresh_ports(force=True)

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
        finally:
            await self.unregister(websocket)

    async def tracking_loop(self):
        interval = 1 / TRACKING_HZ
        while True:
            async with self.state_lock:
                self.serial_bridge.refresh_ports()
                pose = self.mocap.read_pose()
                if pose is not None:
                    pose["using_default_data"] = not pose.get("ready_to_send", False)
                    self.telemetry = pose
                else:
                    self.telemetry = dict(DEFAULT_TELEMETRY)

                snapshot = self.build_snapshot()
                if snapshot["system"]["canSendToEsp32"]:
                    serial_payload = {
                        "timestamp": time.time(),
                        "active": self.control.active,
                        "position": self.telemetry["position"],
                        "rotation": self.telemetry["rotation"],
                        "using_default_data": self.telemetry["using_default_data"],
                        "pid": self.control.pid,
                    }
                    if self.serial_bridge.send(serial_payload):
                        self.last_serial_payload = serial_payload

            await self.broadcast_state()
            await asyncio.sleep(interval)

    async def close(self):
        self.serial_bridge.disconnect()
        self.mocap.close()


async def main():
    server = ControlServer()
    tracking_task = asyncio.create_task(server.tracking_loop())

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
        await server.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Server stopped.")
