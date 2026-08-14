#!/usr/bin/env python3

from __future__ import annotations

import asyncio
import copy
import json
import math
import os
import sys
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

SYSTEM_DIST_PACKAGES = Path("/usr/lib/python3/dist-packages")

if (
    SYSTEM_DIST_PACKAGES.exists()
    and str(SYSTEM_DIST_PACKAGES) not in sys.path
):
    # Gazebo Python bindings are installed by apt, while the dashboard
    # runs from a virtualenv that does not include system site packages.
    sys.path.append(str(SYSTEM_DIST_PACKAGES))

# Gazebo Transport uses the default partition when GZ_PARTITION is unset.
# A named partition may still be supplied by the launcher, but the dashboard
# must not invent one because that isolates it from an already-running Gazebo.

import paho.mqtt.client as mqtt
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, StreamingResponse
from pydantic import BaseModel

from body_attitude_recenter import BodyAttitudeRecenterController
from tracking_web import TrackingManager

try:
    import cv2
    import numpy as np
    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.image_pb2 import Image as GzImage
    from gz.msgs10.imu_pb2 import IMU as GzImu
    from gz.transport13 import Node as GzNode

    try:
        from gz.msgs10.laserscan_pb2 import LaserScan
    except ImportError:
        LaserScan = None

    GAZEBO_IMPORT_ERROR = ""
except ImportError as error:
    cv2 = None
    np = None
    Double = None
    GzImage = None
    GzImu = None
    LaserScan = None
    GzNode = None
    GAZEBO_IMPORT_ERROR = str(error)


# ============================================================
# Configuration
# ============================================================

MQTT_HOST = "127.0.0.1"
MQTT_PORT = 1883
MQTT_CLIENT_ID = os.environ.get(
    "SWARM_DASHBOARD_MQTT_CLIENT_ID",
    f"swarm-dashboard-backend-{os.getpid()}",
)

TELEMETRY_TOPIC = "swarm/+/telemetry/state"
CONTROL_RESULT_TOPIC = "swarm/+/control/result"
CONTROL_TOPIC_TEMPLATE = "swarm/{drone_id}/control/command"

ALLOWED_DRONES = {
    "UAV-01",
    "UAV-02",
}

DRONE_MODELS = {
    "UAV-01": "x500_custom_0",
    "UAV-02": "x500_custom_1",
}

GIMBAL_LIMITS_DEG = {
    "roll": (-45.0, 45.0),
    "pitch": (-135.0, 45.0),
    "yaw": (-180.0, 180.0),
}
try:
    TRACKING_GIMBAL_YAW_LIMIT_DEG = max(
        5.0,
        min(
            90.0,
            float(
                os.environ.get(
                    "SWARM_TRACKING_GIMBAL_YAW_LIMIT_DEG",
                    "15",
                )
            ),
        ),
    )
except ValueError:
    TRACKING_GIMBAL_YAW_LIMIT_DEG = 15.0
try:
    TRACKING_GIMBAL_ROLL_RATE_DEG_S = max(
        1.0,
        min(
            180.0,
            float(
                os.environ.get(
                    "SWARM_TRACKING_GIMBAL_ROLL_RATE_DEG_S",
                    "90",
                )
            ),
        ),
    )
except ValueError:
    TRACKING_GIMBAL_ROLL_RATE_DEG_S = 90.0
try:
    TRACKING_BODY_YAW_MIN_ALTITUDE_M = max(
        0.0,
        min(
            5.0,
            float(
                os.environ.get(
                    "SWARM_TRACKING_BODY_YAW_MIN_ALTITUDE_M",
                    "0.5",
                )
            ),
        ),
    )
except ValueError:
    TRACKING_BODY_YAW_MIN_ALTITUDE_M = 0.5
TRACKING_OFFBOARD_ENABLED = os.environ.get(
    "SWARM_TRACKING_OFFBOARD_ENABLED",
    "true",
).strip().lower() not in {"0", "false", "no", "off"}
try:
    TRACKING_OFFBOARD_MAX_FORWARD_M_S = max(
        0.1,
        min(
            3.0,
            float(os.environ.get("SWARM_TRACKING_OFFBOARD_MAX_FORWARD_M_S", "3.0")),
        ),
    )
except ValueError:
    TRACKING_OFFBOARD_MAX_FORWARD_M_S = 3.0

CAMERA_MAX_WIDTH = 640
CAMERA_JPEG_QUALITY = 72
CAMERA_MAX_FPS = 10.0

ALLOWED_ACTIONS = {
    "position",
    "offboard_map",
    "takeoff",
    "arm",
    "disarm",
    "land",
    "hold",
    "rtl",
    "hold_current",
    # Giữ tương thích với giao diện cũ.
    "enable_offboard",
    "keyboard_off",
    "stop",
}

BASE_DIR = Path(__file__).resolve().parent
INDEX_FILE = BASE_DIR / "static" / "index.html"


# ============================================================
# Shared state
# ============================================================

latest_drones: dict[str, dict[str, Any]] = {}
latest_control_results: dict[str, dict[str, Any]] = {}
manual_control_deadline = {
    drone_id: 0.0
    for drone_id in ALLOWED_DRONES
}
gimbal_angles_deg: dict[str, dict[str, float]] = {
    drone_id: {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
    }
    for drone_id in ALLOWED_DRONES
}
body_attitude_controllers = {
    drone_id: BodyAttitudeRecenterController()
    for drone_id in ALLOWED_DRONES
}
body_attitude_last_update = {
    drone_id: 0.0
    for drone_id in ALLOWED_DRONES
}
tracking_motion_hold_z_down_m: dict[str, float | None] = {
    drone_id: None
    for drone_id in ALLOWED_DRONES
}

state_lock = threading.Lock()
gimbal_lock = threading.Lock()
mqtt_connected = threading.Event()


# ============================================================
# Helpers
# ============================================================

def now_ms() -> int:
    return int(time.time() * 1000)


def finite_float(
    value: Any,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
) -> float:
    number = float(value)

    if not math.isfinite(number):
        raise ValueError("Value is not finite")

    if minimum is not None and number < minimum:
        raise ValueError(
            f"Value must be >= {minimum}"
        )

    if maximum is not None and number > maximum:
        raise ValueError(
            f"Value must be <= {maximum}"
        )

    return number


class GazeboDashboardBridge:
    """Long-lived Gazebo publishers plus per-UAV camera subscriptions."""

    def __init__(self) -> None:
        self.node: Any = None
        self.publishers: dict[tuple[str, str], Any] = {}
        self.camera_topics: dict[str, str] = {}
        self.imu_topics: set[str] = set()
        self.imu_lock = threading.Lock()
        self.body_quaternions: dict[str, tuple[float, float, float, float]] = {}
        self.camera_quaternions: dict[str, tuple[float, float, float, float]] = {}
        self.gimbal_feedback_deg: dict[str, dict[str, float]] = {}
        self.gimbal_feedback_monotonic: dict[str, float] = {}
        self.lidar_topic = "/x500_custom/front_lidar"
        self.lidar_lock = threading.Lock()
        self.lidar_scans: dict[str, dict[str, Any]] = {}
        self.lidar_subscribed = False
        self.started = False
        self.error = GAZEBO_IMPORT_ERROR
        self.publish_lock = threading.Lock()
        self.frame_condition = threading.Condition()
        self.frames: dict[str, bytes] = {}
        self.frame_versions = {
            drone_id: 0
            for drone_id in ALLOWED_DRONES
        }
        self.raw_frames: dict[str, Any] = {}
        self.raw_frame_versions = {
            drone_id: 0
            for drone_id in ALLOWED_DRONES
        }
        self.camera_clients = {
            drone_id: 0
            for drone_id in ALLOWED_DRONES
        } 
        self.tracking_drone_id: str | None = None
        self.last_frame_monotonic: dict[str, float] = {}
        self.last_raw_frame_monotonic: dict[str, float] = {}
        self.last_raw_conversion_monotonic: dict[str, float] = {}
        self.last_encode_monotonic: dict[str, float] = {}
        self.source_fps = {
            drone_id: 0.0
            for drone_id in ALLOWED_DRONES
        }
        self.source_window_started: dict[str, float] = {}
        self.source_window_frames = {
            drone_id: 0
            for drone_id in ALLOWED_DRONES
        }
        self.conversion_ms = {
            drone_id: 0.0
            for drone_id in ALLOWED_DRONES
        }

    def start(self) -> None:
        if self.started:
            return

        if (
            GzNode is None
            or Double is None
            or GzImage is None
            or GzImu is None
            or cv2 is None
            or np is None
        ):
            self.error = (
                "Gazebo camera dependencies are unavailable: "
                f"{GAZEBO_IMPORT_ERROR or 'unknown import error'}"
            )
            return

        try:
            self.node = GzNode()

            for drone_id, model_name in DRONE_MODELS.items():
                for axis in GIMBAL_LIMITS_DEG:
                    topic = (
                        f"/model/{model_name}/command/"
                        f"gimbal_{axis}"
                    )
                    self.publishers[(drone_id, axis)] = (
                        self.node.advertise(topic, Double)
                    )

                camera_topic = (
                    f"/world/default/model/{model_name}/"
                    "link/camera_link/sensor/camera/image"
                )
                self.camera_topics[drone_id] = camera_topic

                def camera_callback(
                    message: Any,
                    selected_drone_id: str = drone_id,
                ) -> None:
                    self._handle_camera_image(
                        selected_drone_id,
                        message,
                    )

                subscribed = self.node.subscribe(
                    GzImage,
                    camera_topic,
                    camera_callback,
                )

                if not subscribed:
                    raise RuntimeError(
                        "Could not subscribe to "
                        f"{camera_topic}"
                    )

                imu_topic_prefix = (
                    f"/world/default/model/{model_name}/link"
                )
                body_imu_topic = (
                    f"{imu_topic_prefix}/base_link/sensor/imu_sensor/imu"
                )
                camera_imu_topic = (
                    f"{imu_topic_prefix}/camera_link/sensor/camera_imu/imu"
                )

                def body_imu_callback(
                    message: Any,
                    selected_drone_id: str = drone_id,
                ) -> None:
                    self._handle_gimbal_imu(selected_drone_id, "body", message)

                def camera_imu_callback(
                    message: Any,
                    selected_drone_id: str = drone_id,
                ) -> None:
                    self._handle_gimbal_imu(selected_drone_id, "camera", message)

                for topic, callback in (
                    (body_imu_topic, body_imu_callback),
                    (camera_imu_topic, camera_imu_callback),
                ):
                    if not self.node.subscribe(GzImu, topic, callback):
                        raise RuntimeError(f"Could not subscribe to {topic}")
                    self.imu_topics.add(topic)

            if LaserScan is not None:
                self.lidar_subscribed = bool(
                    self.node.subscribe(
                        LaserScan,
                        self.lidar_topic,
                        self._handle_lidar_scan,
                    )
                )

            self.started = True
            self.error = ""
            print(
                "Gazebo gimbal/camera bridge started for "
                f"{', '.join(sorted(ALLOWED_DRONES))}",
                flush=True,
            )
        except Exception as error:
            self.error = str(error)
            self.started = False
            print(
                f"Gazebo bridge unavailable: {error}",
                flush=True,
            )

    def stop(self) -> None:
        if self.node is not None:
            for topic in self.camera_topics.values():
                try:
                    self.node.unsubscribe(topic)
                except Exception:
                    pass
            for topic in self.imu_topics:
                try:
                    self.node.unsubscribe(topic)
                except Exception:
                    pass
            if self.lidar_subscribed:
                try:
                    self.node.unsubscribe(self.lidar_topic)
                except Exception:
                    pass

        self.started = False
        self.lidar_subscribed = False

    @staticmethod
    def _quaternion_multiply(
        first: tuple[float, float, float, float],
        second: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        x1, y1, z1, w1 = first
        x2, y2, z2, w2 = second
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    @staticmethod
    def _quaternion_to_euler_deg(
        quaternion: tuple[float, float, float, float],
    ) -> tuple[float, float, float]:
        x, y, z, w = quaternion
        roll = math.atan2(
            2.0 * (w * x + y * z),
            1.0 - 2.0 * (x * x + y * y),
        )
        pitch = math.asin(
            max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        )
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
        return tuple(math.degrees(value) for value in (roll, pitch, yaw))

    def _handle_gimbal_imu(
        self,
        drone_id: str,
        source: str,
        message: Any,
    ) -> None:
        orientation = message.orientation
        quaternion = (
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )
        if not all(math.isfinite(value) for value in quaternion):
            return
        with self.imu_lock:
            target = (
                self.body_quaternions
                if source == "body"
                else self.camera_quaternions
            )
            target[drone_id] = quaternion
            body = self.body_quaternions.get(drone_id)
            camera = self.camera_quaternions.get(drone_id)
            if body is None or camera is None:
                return
            body_inverse = (-body[0], -body[1], -body[2], body[3])
            relative = self._quaternion_multiply(body_inverse, camera)
            roll, pitch, yaw = self._quaternion_to_euler_deg(relative)
            # The model joints use -X for roll, +Y for pitch and -Z for yaw.
            self.gimbal_feedback_deg[drone_id] = {
                "roll": -roll,
                "pitch": -pitch,
                "yaw": -yaw,
            }
            self.gimbal_feedback_monotonic[drone_id] = time.monotonic()

    def gimbal_feedback(self, drone_id: str) -> dict[str, Any]:
        with self.imu_lock:
            angles = self.gimbal_feedback_deg.get(drone_id)
            updated = self.gimbal_feedback_monotonic.get(drone_id)
            age_ms = (
                round((time.monotonic() - updated) * 1000)
                if updated is not None
                else None
            )
            return {
                "available": angles is not None and age_ms is not None and age_ms < 200,
                "angles_deg": dict(angles) if angles is not None else None,
                "age_ms": age_ms,
            }

    def camera_orientation(self, drone_id: str) -> dict[str, Any]:
        with self.imu_lock:
            quaternion = self.camera_quaternions.get(drone_id)
            updated = self.gimbal_feedback_monotonic.get(drone_id)
            age_ms = (
                round((time.monotonic() - updated) * 1000)
                if updated is not None
                else None
            )
            return {
                "available": bool(
                    quaternion is not None
                    and age_ms is not None
                    and age_ms < 200
                ),
                "quaternion_xyzw": (
                    list(quaternion) if quaternion is not None else None
                ),
                "age_ms": age_ms,
            }

    def publish_gimbal(
        self,
        drone_id: str,
        axis: str,
        angle_deg: float,
    ) -> tuple[bool, str]:
        if not self.started:
            return False, self.error or "Gazebo bridge is not running"

        publisher = self.publishers.get((drone_id, axis))

        if publisher is None or not publisher.valid():
            return False, f"Gazebo publisher is invalid for {drone_id} {axis}"

        if not publisher.has_connections():
            return False, (
                "No Gazebo gimbal subscriber for "
                f"{drone_id} {axis}; check GZ_PARTITION and model name"
            )

        message = Double()
        message.data = math.radians(angle_deg)

        try:
            with self.publish_lock:
                published = publisher.publish(message)
        except Exception as error:
            return False, f"Gazebo publish failed: {error}"

        if published is False:
            return False, f"Gazebo rejected the {axis} command"

        return True, ""

    def _handle_camera_image(
        self,
        drone_id: str,
        message: Any,
    ) -> None:
        now = time.monotonic()

        with self.frame_condition:
            self._update_source_fps_locked(drone_id, now)
            camera_requested = self.camera_clients[drone_id] > 0
            tracking_requested = self.tracking_drone_id == drone_id

            if not camera_requested and not tracking_requested:
                return

            previous = self.last_raw_conversion_monotonic.get(
                drone_id,
                0.0,
            )

            if (
                not tracking_requested
                and now - previous < 1.0 / CAMERA_MAX_FPS
            ):
                return

            self.last_raw_conversion_monotonic[drone_id] = now
            encode_camera_frame = (
                camera_requested
                and now - self.last_encode_monotonic.get(drone_id, 0.0)
                >= 1.0 / CAMERA_MAX_FPS
            )

            if encode_camera_frame:
                self.last_encode_monotonic[drone_id] = now

        conversion_started = time.perf_counter()
        try:
            raw_frame = self._image_to_bgr(message)
        except Exception as error:
            self.error = f"Camera conversion failed for {drone_id}: {error}"
            return

        conversion_ms = (
            time.perf_counter() - conversion_started
        ) * 1000.0

        with self.frame_condition:
            previous_conversion_ms = self.conversion_ms[drone_id]
            self.conversion_ms[drone_id] = (
                conversion_ms
                if previous_conversion_ms <= 0.0
                else previous_conversion_ms * 0.8 + conversion_ms * 0.2
            )
            if self.tracking_drone_id == drone_id:
                self.raw_frames[drone_id] = raw_frame
                self.raw_frame_versions[drone_id] += 1
                self.last_raw_frame_monotonic[drone_id] = time.monotonic()
                self.frame_condition.notify_all()

        if not encode_camera_frame:
            return

        try:
            jpeg = self._bgr_to_jpeg(raw_frame)
        except Exception as error:
            self.error = f"Camera JPEG failed for {drone_id}: {error}"
            return

        with self.frame_condition:
            if self.camera_clients[drone_id] > 0:
                self.frames[drone_id] = jpeg
                self.frame_versions[drone_id] += 1
                self.last_frame_monotonic[drone_id] = time.monotonic()
            self.frame_condition.notify_all()

    def _handle_lidar_scan(self, message: Any) -> None:
        frame = str(getattr(message, "frame", ""))
        drone_id = next(
            (
                candidate_drone_id
                for candidate_drone_id, model_name in DRONE_MODELS.items()
                if frame.startswith(f"{model_name}::")
            ),
            None,
        )
        if drone_id is None:
            return
        ranges = np.asarray(message.ranges, dtype=np.float32)
        if ranges.size <= 0:
            return
        with self.lidar_lock:
            self.lidar_scans[drone_id] = {
                "ranges": ranges.copy(),
                "angle_min": float(message.angle_min),
                "angle_max": float(message.angle_max),
                "angle_step": float(message.angle_step),
                "range_min": float(message.range_min),
                "range_max": float(message.range_max),
                "updated_monotonic": time.monotonic(),
            }

    def lidar_distance(
        self,
        drone_id: str,
        bearing_deg: float,
        *,
        max_age_s: float = 0.4,
        window_deg: float = 2.0,
    ) -> float | None:
        with self.lidar_lock:
            scan = self.lidar_scans.get(drone_id)
            if scan is None:
                return None
            if time.monotonic() - scan["updated_monotonic"] > max_age_s:
                return None
            angle = math.radians(float(bearing_deg))
            if angle < scan["angle_min"] or angle > scan["angle_max"]:
                return None
            step = scan["angle_step"]
            if not math.isfinite(step) or abs(step) < 1e-9:
                return None
            center_index = int(round((angle - scan["angle_min"]) / step))
            half_window = max(
                1,
                int(round(math.radians(window_deg) / abs(step))),
            )
            ranges = scan["ranges"]
            start = max(0, center_index - half_window)
            stop = min(ranges.size, center_index + half_window + 1)
            values = ranges[start:stop]
            valid = values[
                np.isfinite(values)
                & (values >= scan["range_min"])
                & (values <= scan["range_max"])
            ]
            if valid.size <= 0:
                return None
            return float(np.median(valid))

    def _update_source_fps_locked(
        self,
        drone_id: str,
        now: float,
    ) -> None:
        started = self.source_window_started.get(drone_id)

        if started is None:
            self.source_window_started[drone_id] = now
            self.source_window_frames[drone_id] = 1
            return

        self.source_window_frames[drone_id] += 1
        elapsed = now - started

        if elapsed < 1.0:
            return

        measured_fps = self.source_window_frames[drone_id] / elapsed
        previous_fps = self.source_fps[drone_id]
        self.source_fps[drone_id] = (
            measured_fps
            if previous_fps <= 0.0
            else previous_fps * 0.7 + measured_fps * 0.3
        )
        self.source_window_started[drone_id] = now
        self.source_window_frames[drone_id] = 0

    @staticmethod
    def _image_to_bgr(message: Any) -> Any:
        width = int(message.width)
        height = int(message.height)
        step = int(message.step)
        pixel_format = int(message.pixel_format_type)
        raw_data = np.frombuffer(message.data, dtype=np.uint8)

        if width <= 0 or height <= 0 or raw_data.size <= 0:
            raise ValueError("empty Gazebo image")

        if pixel_format == 1:
            channels = 1
            conversion = cv2.COLOR_GRAY2BGR
        elif pixel_format == 3:
            channels = 3
            conversion = cv2.COLOR_RGB2BGR
        elif pixel_format == 4:
            channels = 4
            conversion = cv2.COLOR_RGBA2BGR
        elif pixel_format == 5:
            channels = 4
            conversion = cv2.COLOR_BGRA2BGR
        elif pixel_format == 8:
            channels = 3
            conversion = None
        else:
            raise ValueError(
                f"unsupported Gazebo pixel format {pixel_format}"
            )

        row_bytes = width * channels
        if step < row_bytes or raw_data.size < step * height:
            raise ValueError(
                f"invalid Gazebo image stride {step} for {width}x{height}"
            )

        rows = raw_data[: step * height].reshape(height, step)
        image = rows[:, :row_bytes].reshape(height, width, channels)

        if conversion is not None:
            image = cv2.cvtColor(image, conversion)
        else:
            image = image.copy()

        if width > CAMERA_MAX_WIDTH:
            resized_height = max(
                1,
                round(height * CAMERA_MAX_WIDTH / width),
            )
            image = cv2.resize(
                image,
                (CAMERA_MAX_WIDTH, resized_height),
                interpolation=cv2.INTER_LINEAR,
            )

        return np.ascontiguousarray(image)

    @staticmethod
    def _bgr_to_jpeg(image: Any) -> bytes:
        success, encoded = cv2.imencode(
            ".jpg",
            image,
            [cv2.IMWRITE_JPEG_QUALITY, CAMERA_JPEG_QUALITY],
        )

        if not success:
            raise RuntimeError("OpenCV could not encode camera frame")

        return encoded.tobytes()

    def set_tracking_drone(
        self,
        drone_id: str | None,
    ) -> None:
        with self.frame_condition:
            self.tracking_drone_id = drone_id
            if drone_id is None:
                self.raw_frames.clear()
            else:
                self.raw_frames.pop(drone_id, None)
                self.raw_frame_versions[drone_id] += 1
            self.frame_condition.notify_all()

    def mjpeg_frames(self, drone_id: str):
        last_version = -1

        with self.frame_condition:
            self.camera_clients[drone_id] += 1

        try:
            while True:
                with self.frame_condition:
                    self.frame_condition.wait_for(
                        lambda: (
                            self.frame_versions[drone_id]
                            != last_version
                        ),
                        timeout=2.0,
                    )
                    frame = self.frames.get(drone_id)
                    last_version = self.frame_versions[drone_id]

                if frame is None:
                    continue

                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii")
                    + frame
                    + b"\r\n"
                )
        finally:
            with self.frame_condition:
                self.camera_clients[drone_id] = max(
                    0,
                    self.camera_clients[drone_id] - 1,
                )

    def status(self) -> dict[str, Any]:
        now = time.monotonic()
        cameras: dict[str, dict[str, Any]] = {}

        with self.frame_condition:
            for drone_id in ALLOWED_DRONES:
                last_frame = self.last_frame_monotonic.get(drone_id)
                last_raw_frame = self.last_raw_frame_monotonic.get(drone_id)
                cameras[drone_id] = {
                    "topic": self.camera_topics.get(drone_id, ""),
                    "has_frame": drone_id in self.frames,
                    "clients": self.camera_clients[drone_id],
                    "tracking": self.tracking_drone_id == drone_id,
                    "source_fps": round(self.source_fps[drone_id], 1),
                    "conversion_ms": round(self.conversion_ms[drone_id], 2),
                    "has_raw_frame": drone_id in self.raw_frames,
                    "last_raw_frame_age_ms": (
                        round((now - last_raw_frame) * 1000)
                        if last_raw_frame is not None
                        else None
                    ),
                    "last_frame_age_ms": (
                        round((now - last_frame) * 1000)
                        if last_frame is not None
                        else None
                    ),
                    "lidar_front_distance_m": self.lidar_distance(
                        drone_id,
                        0.0,
                    ),
                    "gimbal_feedback": self.gimbal_feedback(drone_id),
                }

        return {
            "started": self.started,
            "error": self.error,
            "partition": os.environ.get("GZ_PARTITION") or "default",
            "cameras": cameras,
        }


gazebo_bridge = GazeboDashboardBridge()


def publish_gimbal_home(drone_id: str) -> dict[str, Any]:
    if drone_id not in ALLOWED_DRONES:
        return {"ok": False, "error": "Invalid drone ID"}

    errors: list[str] = []
    with gimbal_lock:
        for axis in GIMBAL_LIMITS_DEG:
            ok, error = gazebo_bridge.publish_gimbal(drone_id, axis, 0.0)
            if ok:
                gimbal_angles_deg[drone_id][axis] = 0.0
            else:
                errors.append(f"{axis}: {error}")

    return {
        "ok": not errors,
        "error": "; ".join(errors),
    }


def synced_gimbal_angles_deg(drone_id: str) -> dict[str, float]:
    current = dict(gimbal_angles_deg[drone_id])
    feedback = gazebo_bridge.gimbal_feedback(drone_id)
    if not feedback.get("available"):
        return current

    angles = feedback.get("angles_deg")
    if not isinstance(angles, dict):
        return current

    for axis, (minimum, maximum) in GIMBAL_LIMITS_DEG.items():
        try:
            value = float(angles[axis])
        except (KeyError, TypeError, ValueError):
            continue
        if not math.isfinite(value):
            continue
        # IMU-to-Euler can become ambiguous close to gimbal singularities.
        # Trust feedback only when it is close to this joint's command range.
        if minimum - 5.0 <= value <= maximum + 5.0:
            current[axis] = max(minimum, min(maximum, value))

    gimbal_angles_deg[drone_id].update(current)
    return current


def command_tracking_gimbal(
    drone_id: str,
    pan_rate_rad_s: float,
    tilt_rate_rad_s: float,
    dt: float,
) -> dict[str, Any]:
    dt = max(0.001, min(0.1, float(dt)))

    with gimbal_lock:
        current = synced_gimbal_angles_deg(drone_id)
        roll_delta_limit = TRACKING_GIMBAL_ROLL_RATE_DEG_S * dt
        roll_delta = max(
            -roll_delta_limit,
            min(roll_delta_limit, -current["roll"]),
        )
        requested_rates = {
            "roll": roll_delta / dt,
            "pitch": math.degrees(tilt_rate_rad_s),
            "yaw": math.degrees(pan_rate_rad_s),
        }
        requested_targets = {
            "roll": current["roll"] + roll_delta,
            "pitch": (
                current["pitch"]
                + requested_rates["pitch"] * dt
            ),
            "yaw": (
                current["yaw"]
                + requested_rates["yaw"] * dt
            ),
        }
        targets = dict(requested_targets)

        for axis, (minimum, maximum) in GIMBAL_LIMITS_DEG.items():
            targets[axis] = max(
                minimum,
                min(maximum, targets[axis]),
            )
        tracking_yaw_minimum = max(
            GIMBAL_LIMITS_DEG["yaw"][0],
            -TRACKING_GIMBAL_YAW_LIMIT_DEG,
        )
        tracking_yaw_maximum = min(
            GIMBAL_LIMITS_DEG["yaw"][1],
            TRACKING_GIMBAL_YAW_LIMIT_DEG,
        )

        yaw_return_rate_deg_s = max(
            15.0,
            abs(requested_rates["yaw"]),
        )
        if current["yaw"] < tracking_yaw_minimum:
            targets["yaw"] = min(
                tracking_yaw_minimum,
                current["yaw"] + yaw_return_rate_deg_s * dt,
            )
        elif current["yaw"] > tracking_yaw_maximum:
            targets["yaw"] = max(
                tracking_yaw_maximum,
                current["yaw"] - yaw_return_rate_deg_s * dt,
            )

        yaw_saturated_outward = bool(
            requested_targets["yaw"] < tracking_yaw_minimum
            or requested_targets["yaw"] > tracking_yaw_maximum
        )
        if tracking_yaw_minimum <= current["yaw"] <= tracking_yaw_maximum:
            targets["yaw"] = max(
                tracking_yaw_minimum,
                min(tracking_yaw_maximum, targets["yaw"]),
            )

        applied_rates = {
            axis: (targets[axis] - current[axis]) / dt
            for axis in GIMBAL_LIMITS_DEG
        }
        errors: list[str] = []

        for axis in ("roll", "pitch", "yaw"):
            ok, error = gazebo_bridge.publish_gimbal(
                drone_id,
                axis,
                targets[axis],
            )

            if ok:
                gimbal_angles_deg[drone_id][axis] = targets[axis]
            else:
                applied_rates[axis] = 0.0
                errors.append(f"{axis}: {error}")

        angles = dict(gimbal_angles_deg[drone_id])

    return {
        "ok": not errors,
        "angles_deg": angles,
        "rates_deg_s": applied_rates,
        "yaw_limit_deg": TRACKING_GIMBAL_YAW_LIMIT_DEG,
        "yaw_saturated_outward": yaw_saturated_outward,
        "yaw_requested_rate_deg_s": requested_rates["yaw"],
        "error": "; ".join(errors),
    }


def tracking_body_yaw_safety_error(drone_id: str) -> str:
    with state_lock:
        drone = copy.deepcopy(latest_drones.get(drone_id))
        manual_override = (
            time.monotonic()
            <= manual_control_deadline.get(drone_id, 0.0)
        )

    if manual_override:
        return "Body yaw paused: keyboard control has priority"

    if not isinstance(drone, dict) or not drone.get("online", False):
        return "Body yaw blocked: vehicle telemetry is offline"

    status = drone.get("status", {})
    if not isinstance(status, dict):
        status = {}
    if not bool(status.get("armed", False)):
        return "Body yaw blocked: vehicle is not armed"
    if bool(status.get("failsafe", False)):
        return "Body yaw blocked: PX4 failsafe is active"
    try:
        nav_state = int(status.get("nav_state", -1))
    except (TypeError, ValueError):
        nav_state = -1
    allowed_nav_states = {2, 4, 14} if TRACKING_OFFBOARD_ENABLED else {2}
    if nav_state not in allowed_nav_states:
        return (
            "Tracking motion blocked: PX4 must be in "
            "Position, Hold or Offboard mode"
        )

    failsafe_flags = drone.get("failsafe_flags", {})
    if not isinstance(failsafe_flags, dict):
        failsafe_flags = {}
    if bool(failsafe_flags.get("manual_control_signal_lost", False)):
        return "Body yaw blocked: PX4 manual-control signal is lost"

    local_position = drone.get("local_position", {})
    if not isinstance(local_position, dict):
        local_position = {}
    try:
        altitude_m = -float(local_position.get("z_down_m"))
    except (TypeError, ValueError):
        return "Body yaw blocked: local altitude is unavailable"
    if altitude_m < TRACKING_BODY_YAW_MIN_ALTITUDE_M:
        return (
            "Body yaw blocked: vehicle altitude is below "
            f"{TRACKING_BODY_YAW_MIN_ALTITUDE_M:.1f} m"
        )
    return ""


def command_tracking_body_yaw(
    drone_id: str,
    enabled: bool,
    yaw: float,
    target_yaw_error_deg: float,
    track_state: str,
) -> dict[str, Any]:
    yaw = max(-0.55, min(0.55, float(yaw)))
    requested_enabled = bool(enabled and abs(yaw) > 1e-6)
    safety_error = (
        tracking_body_yaw_safety_error(drone_id)
        if requested_enabled
        else ""
    )
    effective_enabled = requested_enabled and not safety_error
    effective_yaw = yaw if effective_enabled else 0.0
    if TRACKING_OFFBOARD_ENABLED:
        return {
            "ok": not safety_error,
            "enabled": effective_enabled,
            "yaw": effective_yaw,
            "error": safety_error,
        }
    ok, publish_error = publish_control_message(
        {
            "type": "tracking_yaw",
            "drone_id": drone_id,
            "enabled": effective_enabled,
            "yaw": effective_yaw,
            "target_yaw_error_deg": round(
                float(target_yaw_error_deg),
                3,
            ),
            "track_state": str(track_state),
        }
    )
    error = safety_error or publish_error
    return {
        "ok": bool(ok and not error),
        "enabled": effective_enabled,
        "yaw": effective_yaw,
        "error": error,
    }


def command_tracking_follow(
    drone_id: str,
    enabled: bool,
    forward: float,
    measured_distance_m: float | None,
    target_distance_m: float,
    distance_source: str,
    track_state: str,
) -> dict[str, Any]:
    forward = max(-0.35, min(0.35, float(forward)))
    requested_enabled = bool(enabled and abs(forward) > 1e-6)
    safety_error = (
        tracking_body_yaw_safety_error(drone_id)
        if requested_enabled
        else ""
    )
    effective_enabled = requested_enabled and not safety_error
    effective_forward = forward if effective_enabled else 0.0
    if TRACKING_OFFBOARD_ENABLED:
        return {
            "ok": not safety_error,
            "enabled": effective_enabled,
            "forward": effective_forward,
            "error": safety_error,
        }
    ok, publish_error = publish_control_message(
        {
            "type": "tracking_follow",
            "drone_id": drone_id,
            "enabled": effective_enabled,
            "forward": effective_forward,
            "right": 0.0,
            "up": 0.0,
            "measured_distance_m": measured_distance_m,
            "target_distance_m": round(float(target_distance_m), 3),
            "distance_source": str(distance_source),
            "track_state": str(track_state),
        }
    )
    error = safety_error or publish_error
    return {
        "ok": bool(ok and not error),
        "enabled": effective_enabled,
        "forward": effective_forward,
        "error": error,
    }


def command_tracking_motion(
    drone_id: str,
    enabled: bool,
    forward_velocity_m_s: float,
    yaw_rate_deg_s: float,
    down_velocity_m_s: float,
    measured_distance_m: float | None,
    target_distance_m: float,
    distance_source: str,
    track_state: str,
) -> dict[str, Any]:
    forward_velocity_m_s = max(
        -TRACKING_OFFBOARD_MAX_FORWARD_M_S,
        min(TRACKING_OFFBOARD_MAX_FORWARD_M_S, float(forward_velocity_m_s)),
    )
    yaw_rate_deg_s = max(
        -45.0,
        min(45.0, float(yaw_rate_deg_s)),
    )
    down_velocity_m_s = max(
        -0.8,
        min(0.8, float(down_velocity_m_s)),
    )
    requested_enabled = bool(enabled and TRACKING_OFFBOARD_ENABLED)
    safety_error = (
        tracking_body_yaw_safety_error(drone_id)
        if requested_enabled
        else ""
    )
    effective_enabled = requested_enabled and not safety_error
    north_velocity_m_s = 0.0
    east_velocity_m_s = 0.0
    hold_z_down_m = tracking_motion_hold_z_down_m[drone_id]
    if effective_enabled:
        with state_lock:
            drone = copy.deepcopy(latest_drones.get(drone_id, {}))
        local_position = (
            drone.get("local_position", {})
            if isinstance(drone, dict)
            else {}
        )
        try:
            current_z_down_m = float(local_position["z_down_m"])
            heading_rad = float(local_position["heading_rad"])
            if not all(
                math.isfinite(value)
                for value in (current_z_down_m, heading_rad)
            ):
                raise ValueError("position is not finite")
            if hold_z_down_m is None:
                hold_z_down_m = current_z_down_m
                tracking_motion_hold_z_down_m[drone_id] = hold_z_down_m
            north_velocity_m_s = forward_velocity_m_s * math.cos(heading_rad)
            east_velocity_m_s = forward_velocity_m_s * math.sin(heading_rad)
        except (KeyError, TypeError, ValueError) as error:
            safety_error = f"Tracking altitude hold unavailable: {error}"
            effective_enabled = False
    if not effective_enabled:
        tracking_motion_hold_z_down_m[drone_id] = None
        hold_z_down_m = None
    ok, publish_error = publish_control_message(
        {
            "type": "offboard_follow",
            "drone_id": drone_id,
            "enabled": effective_enabled,
            "velocity_frame": "local_ned_altitude_hold",
            "north_velocity_m_s": (
                north_velocity_m_s if effective_enabled else 0.0
            ),
            "east_velocity_m_s": (
                east_velocity_m_s if effective_enabled else 0.0
            ),
            "hold_z_down_m": (
                hold_z_down_m if effective_enabled else 0.0
            ),
            "forward_velocity_m_s": 0.0,
            "right_velocity_m_s": 0.0,
            "down_velocity_m_s": 0.0,
            "yaw_rate_deg_s": (
                yaw_rate_deg_s if effective_enabled else 0.0
            ),
            "measured_distance_m": measured_distance_m,
            "target_distance_m": round(float(target_distance_m), 3),
            "distance_source": str(distance_source),
            "track_state": str(track_state),
        }
    )
    error = safety_error or publish_error
    return {
        "ok": bool(ok and not error),
        "enabled": effective_enabled,
        "forward_velocity_m_s": (
            forward_velocity_m_s if effective_enabled else 0.0
        ),
        "yaw_rate_deg_s": yaw_rate_deg_s if effective_enabled else 0.0,
        "down_velocity_m_s": 0.0,
        "hold_z_down_m": hold_z_down_m if effective_enabled else None,
        "error": error,
    }


def command_tracking_attitude(
    drone_id: str,
    enabled: bool,
    normalized_forward_command: float,
    track_state: str,
    neutral: bool = False,
    bbox_image_error_deg: dict[str, Any] | None = None,
) -> dict[str, Any]:
    controller = body_attitude_controllers[drone_id]
    if not controller.config.enabled:
        controller.reset()
        return controller.output(False, 0.0, "disabled")

    requested_enabled = bool(enabled and TRACKING_OFFBOARD_ENABLED)
    safety_error = (
        tracking_body_yaw_safety_error(drone_id)
        if requested_enabled
        else ""
    )
    feedback = gazebo_bridge.gimbal_feedback(drone_id)
    if requested_enabled and not feedback.get("available"):
        safety_error = "Body attitude blocked: gimbal IMU feedback is stale"

    with state_lock:
        drone = copy.deepcopy(latest_drones.get(drone_id, {}))
    attitude = drone.get("attitude", {}) if isinstance(drone, dict) else {}
    local_position = (
        drone.get("local_position", {}) if isinstance(drone, dict) else {}
    )
    now = time.monotonic()
    previous = body_attitude_last_update[drone_id]
    dt = 0.05 if previous <= 0.0 else now - previous
    body_attitude_last_update[drone_id] = now

    effective_enabled = requested_enabled and not safety_error
    heading_rad = 0.0
    try:
        heading_rad = float(local_position.get("heading_rad"))
        north_velocity = float(local_position.get("vx_m_s"))
        east_velocity = float(local_position.get("vy_m_s"))
        if not all(
            math.isfinite(value)
            for value in (heading_rad, north_velocity, east_velocity)
        ):
            raise ValueError("horizontal velocity feedback is not finite")
        forward_velocity = (
            north_velocity * math.cos(heading_rad)
            + east_velocity * math.sin(heading_rad)
        )
        right_velocity = (
            -north_velocity * math.sin(heading_rad)
            + east_velocity * math.cos(heading_rad)
        )
        result = controller.update(
            enabled=effective_enabled,
            gimbal_angles_deg=feedback.get("angles_deg") or {},
            body_attitude_deg={
                "roll": attitude.get("roll_deg"),
                "pitch": attitude.get("pitch_deg"),
            },
            altitude_m=-float(local_position.get("z_down_m")),
            vertical_velocity_down_m_s=float(
                local_position.get("vz_m_s")
            ),
            horizontal_velocity_body_m_s={
                "forward": forward_velocity,
                "right": right_velocity,
            },
            bbox_image_error_deg=bbox_image_error_deg,
            normalized_forward_command=normalized_forward_command,
            dt=dt,
        )
    except (KeyError, TypeError, ValueError) as error:
        controller.reset()
        safety_error = f"Body attitude feedback invalid: {error}"
        result = controller.output(False, 0.0, "blocked")

    rates = result["body_rates_deg_s"]
    acceleration = result.get("body_acceleration_m_s2", {})
    forward_acceleration = float(acceleration.get("forward", 0.0))
    right_acceleration = float(acceleration.get("right", 0.0))
    yaw_rate = float(rates["yaw"])
    if neutral:
        forward_acceleration = 0.0
        right_acceleration = 0.0
        yaw_rate = 0.0
    north_acceleration = (
        forward_acceleration * math.cos(heading_rad)
        - right_acceleration * math.sin(heading_rad)
    )
    east_acceleration = (
        forward_acceleration * math.sin(heading_rad)
        + right_acceleration * math.cos(heading_rad)
    )
    hold_z_down_m = -float(
        result.get(
            "altitude_target_m",
            -float(local_position.get("z_down_m")),
        )
    )
    ok, publish_error = publish_control_message(
        {
            "type": "offboard_follow",
            "drone_id": drone_id,
            "enabled": bool(result["active"] and not safety_error),
            "velocity_frame": "local_ned_acceleration_altitude_hold",
            "north_acceleration_m_s2": (
                north_acceleration if result["active"] else 0.0
            ),
            "east_acceleration_m_s2": (
                east_acceleration if result["active"] else 0.0
            ),
            "north_velocity_m_s": 0.0,
            "east_velocity_m_s": 0.0,
            "hold_z_down_m": hold_z_down_m,
            "forward_velocity_m_s": 0.0,
            "right_velocity_m_s": 0.0,
            "down_velocity_m_s": 0.0,
            "yaw_rate_deg_s": yaw_rate if result["active"] else 0.0,
            "gimbal_feedback_deg": feedback.get("angles_deg"),
            "track_state": str(track_state),
        }
    )
    error = safety_error or publish_error
    result.update(
        {
            "ok": bool(ok and not error and result["active"]),
            "error": error,
            "gimbal_feedback": feedback,
            "local_acceleration_m_s2": {
                "north": round(north_acceleration, 4),
                "east": round(east_acceleration, 4),
            },
            "hold_z_down_m": round(hold_z_down_m, 3),
        }
    )
    return result


def tracking_visual_pose(drone_id: str) -> dict[str, Any]:
    with state_lock:
        drone = copy.deepcopy(latest_drones.get(drone_id, {}))
    camera = gazebo_bridge.camera_orientation(drone_id)
    if not isinstance(drone, dict) or not drone.get("online", False):
        return {"available": False, "error": "Vehicle telemetry is offline"}
    received_ms = drone.get("dashboard_received_ms")
    try:
        telemetry_age_ms = now_ms() - int(received_ms)
    except (TypeError, ValueError):
        return {"available": False, "error": "Telemetry timestamp is unavailable"}
    if telemetry_age_ms > 500:
        return {"available": False, "error": "Vehicle telemetry is stale"}
    if not camera.get("available"):
        return {"available": False, "error": "Camera orientation is stale"}
    local_position = drone.get("local_position")
    global_position = drone.get("global_position")
    if not isinstance(local_position, dict) or not isinstance(global_position, dict):
        return {"available": False, "error": "Vehicle position is unavailable"}
    required_local = ("x_north_m", "y_east_m", "z_down_m", "heading_rad")
    required_global = ("latitude_deg", "longitude_deg", "altitude_msl_m")
    try:
        values = [float(local_position[key]) for key in required_local]
        values.extend(float(global_position[key]) for key in required_global)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("position is not finite")
    except (KeyError, TypeError, ValueError) as error:
        return {"available": False, "error": f"Vehicle pose invalid: {error}"}
    return {
        "available": True,
        "local_position": local_position,
        "global_position": global_position,
        "camera_quaternion_xyzw": camera["quaternion_xyzw"],
        "telemetry_age_ms": telemetry_age_ms,
        "camera_age_ms": camera.get("age_ms"),
    }


def tracking_visual_follow_safety_error(drone_id: str) -> str:
    with state_lock:
        drone = copy.deepcopy(latest_drones.get(drone_id, {}))
        manual_override = time.monotonic() <= manual_control_deadline.get(
            drone_id, 0.0
        )
    if manual_override:
        return "Visual Follow paused: manual control has priority"
    if not isinstance(drone, dict) or not drone.get("online", False):
        return "Visual Follow blocked: telemetry is offline"
    status = drone.get("status", {})
    if not isinstance(status, dict) or not bool(status.get("armed", False)):
        return "Visual Follow blocked: vehicle is not armed"
    if bool(status.get("failsafe", False)):
        return "Visual Follow blocked: PX4 failsafe is active"
    try:
        nav_state = int(status.get("nav_state", -1))
    except (TypeError, ValueError):
        nav_state = -1
    if nav_state not in {2, 4, 19}:
        return "Visual Follow blocked: PX4 must be in Position, Hold or Follow"
    failsafe_flags = drone.get("failsafe_flags", {})
    if isinstance(failsafe_flags, dict):
        if bool(failsafe_flags.get("local_position_invalid", False)):
            return "Visual Follow blocked: local position is invalid"
        if bool(failsafe_flags.get("global_position_invalid", False)):
            return "Visual Follow blocked: global position is invalid"
    pose = tracking_visual_pose(drone_id)
    if not pose.get("available"):
        return f"Visual Follow blocked: {pose.get('error', 'pose unavailable')}"
    try:
        altitude_m = -float(pose["local_position"]["z_down_m"])
    except (KeyError, TypeError, ValueError):
        return "Visual Follow blocked: local altitude is unavailable"
    if altitude_m < max(1.0, TRACKING_BODY_YAW_MIN_ALTITUDE_M):
        return "Visual Follow blocked: take off above 1 m first"
    return ""


def command_visual_follow_target(
    drone_id: str,
    enabled: bool,
    target: dict[str, Any] | None,
    track_state: str,
) -> dict[str, Any]:
    safety_error = tracking_visual_follow_safety_error(drone_id) if enabled else ""
    effective_enabled = bool(enabled and not safety_error and target is not None)
    payload: dict[str, Any] = {
        "type": "visual_follow_target",
        "drone_id": drone_id,
        "enabled": effective_enabled,
        "track_state": str(track_state),
    }
    if effective_enabled and target is not None:
        try:
            payload.update(
                {
                    "latitude_deg": finite_float(
                        target["latitude_deg"], minimum=-90.0, maximum=90.0
                    ),
                    "longitude_deg": finite_float(
                        target["longitude_deg"], minimum=-180.0, maximum=180.0
                    ),
                    "altitude_msl_m": finite_float(target["altitude_msl_m"]),
                    "velocity_north_m_s": finite_float(
                        target.get("velocity_north_m_s", 0.0),
                        minimum=-20.0,
                        maximum=20.0,
                    ),
                    "velocity_east_m_s": finite_float(
                        target.get("velocity_east_m_s", 0.0),
                        minimum=-20.0,
                        maximum=20.0,
                    ),
                    "velocity_down_m_s": finite_float(
                        target.get("velocity_down_m_s", 0.0),
                        minimum=-10.0,
                        maximum=10.0,
                    ),
                    "quality": finite_float(
                        target.get("quality", 0.0), minimum=0.0, maximum=1.0
                    ),
                    "follow_distance_m": finite_float(
                        target.get("follow_distance_m", 10.0),
                        minimum=3.0,
                        maximum=20.0,
                    ),
                    "follow_height_m": finite_float(
                        target.get("follow_height_m", 3.0),
                        minimum=1.0,
                        maximum=30.0,
                    ),
                }
            )
        except (KeyError, TypeError, ValueError) as error:
            safety_error = f"Visual Follow target invalid: {error}"
            payload["enabled"] = False
            effective_enabled = False
    ok, publish_error = publish_control_message(payload)
    error = safety_error or publish_error
    return {
        "ok": bool(ok and effective_enabled and not error),
        "enabled": effective_enabled,
        "error": error,
    }


tracking_manager = TrackingManager(
    gazebo_bridge,
    ALLOWED_DRONES,
    gimbal_command=command_tracking_gimbal,
    body_yaw_command=command_tracking_body_yaw,
    follow_command=command_tracking_follow,
    motion_command=command_tracking_motion,
    attitude_command=command_tracking_attitude,
    visual_target_command=command_visual_follow_target,
    pose_provider=tracking_visual_pose,
)


class TrackingStartRequest(BaseModel):
    drone_id: str
    desired_distance_m: float = 10.0


class TrackingBBoxRequest(BaseModel):
    x: float
    y: float
    width: float
    height: float


class TrackingVisualFollowRequest(BaseModel):
    enabled: bool


def arm_recovery_reason(
    drone: dict[str, Any] | None,
) -> str:
    if not drone:
        return ""

    status = drone.get("status", {})

    if not isinstance(status, dict):
        status = {}

    nav_state = status.get(
        "nav_state",
        drone.get("nav_state"),
    )

    preflight_ok = bool(
        status.get(
            "preflight_checks_pass",
            drone.get(
                "preflight_checks_pass",
                False,
            ),
        )
    )

    if not preflight_ok:
        return "preflight checks are not ready"

    try:
        nav_state_number = int(nav_state)
    except (TypeError, ValueError):
        return ""

    blocked_states = {
        5: "RTL",
        14: "Offboard",
        17: "Takeoff",
        18: "Land",
    }

    if nav_state_number in blocked_states:
        return (
            "vehicle is still in "
            f"{blocked_states[nav_state_number]} mode"
        )

    return ""


def publish_control_message(
    payload: dict[str, Any],
) -> tuple[bool, str]:
    drone_id = str(
        payload.get("drone_id", "")
    ).strip()

    if drone_id not in ALLOWED_DRONES:
        return False, "Invalid drone ID"

    if not mqtt_connected.is_set():
        return False, "MQTT is not connected"

    message = dict(payload)
    message["server_timestamp_ms"] = now_ms()

    topic = CONTROL_TOPIC_TEMPLATE.format(
        drone_id=drone_id
    )

    result = mqtt_client.publish(
        topic=topic,
        payload=json.dumps(
            message,
            ensure_ascii=False,
            separators=(",", ":"),
        ),
        qos=0,
        retain=False,
    )

    if result.rc != mqtt.MQTT_ERR_SUCCESS:
        return (
            False,
            f"MQTT publish failed, rc={result.rc}",
        )

    return True, ""


# ============================================================
# MQTT callbacks
# ============================================================

def on_mqtt_connect(
    client: mqtt.Client,
    userdata: Any,
    flags: Any,
    reason_code: Any,
    properties: Any = None,
) -> None:
    print(
        f"MQTT connected: {reason_code}",
        flush=True,
    )

    mqtt_connected.set()

    client.subscribe(
        [
            (TELEMETRY_TOPIC, 0),
            (CONTROL_RESULT_TOPIC, 0),
        ]
    )


def on_mqtt_disconnect(
    client: mqtt.Client,
    userdata: Any,
    disconnect_flags: Any = None,
    reason_code: Any = None,
    properties: Any = None,
) -> None:
    mqtt_connected.clear()

    print(
        f"MQTT disconnected: {reason_code}",
        flush=True,
    )


def on_mqtt_message(
    client: mqtt.Client,
    userdata: Any,
    message: mqtt.MQTTMessage,
) -> None:
    try:
        decoded = message.payload.decode(
            "utf-8"
        )

        payload = json.loads(decoded)

    except (
        UnicodeDecodeError,
        json.JSONDecodeError,
    ) as error:
        print(
            f"Invalid MQTT JSON on "
            f"{message.topic}: {error}",
            flush=True,
        )
        return

    if not isinstance(payload, dict):
        return

    drone_id = str(
        payload.get("drone_id", "")
    ).strip()

    if drone_id not in ALLOWED_DRONES:
        # Một số bridge có thể không chèn drone_id.
        # Trong trường hợp đó lấy từ tên topic.
        topic_parts = str(
            message.topic
        ).split("/")

        if (
            len(topic_parts) >= 2
            and topic_parts[1]
            in ALLOWED_DRONES
        ):
            drone_id = topic_parts[1]
            payload["drone_id"] = drone_id
        else:
            return

    payload["dashboard_received_ms"] = (
        now_ms()
    )

    topic = str(message.topic)

    with state_lock:
        if topic.endswith(
            "/telemetry/state"
        ):
            latest_drones[drone_id] = (
                payload
            )

        elif topic.endswith(
            "/control/result"
        ):
            latest_control_results[
                drone_id
            ] = payload


# ============================================================
# MQTT client
# ============================================================

try:
    mqtt_client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        client_id=MQTT_CLIENT_ID,
    )
except AttributeError:
    mqtt_client = mqtt.Client(
        client_id=MQTT_CLIENT_ID,
    )

mqtt_client.on_connect = on_mqtt_connect
mqtt_client.on_disconnect = (
    on_mqtt_disconnect
)
mqtt_client.on_message = on_mqtt_message

mqtt_client.reconnect_delay_set(
    min_delay=1,
    max_delay=10,
)


# ============================================================
# FastAPI lifecycle
# ============================================================

@asynccontextmanager
async def lifespan(
    app: FastAPI,
):
    if tracking_manager.metric_depth_estimator.enabled:
        await asyncio.to_thread(tracking_manager.preload_metric_depth)

    mqtt_client.connect(
        MQTT_HOST,
        MQTT_PORT,
        keepalive=30,
    )

    mqtt_client.loop_start()
    gazebo_bridge.start()

    try:
        yield
    finally:
        mqtt_connected.clear()
        tracking_manager.shutdown()
        gazebo_bridge.stop()

        mqtt_client.loop_stop()
        mqtt_client.disconnect()


app = FastAPI(
    title="Swarm UAV Dashboard",
    lifespan=lifespan,
)


# ============================================================
# HTTP routes
# ============================================================

@app.get("/")
async def index() -> FileResponse:
    if not INDEX_FILE.exists():
        raise RuntimeError(
            f"Missing frontend file: "
            f"{INDEX_FILE}"
        )

    return FileResponse(
        INDEX_FILE,
        headers={
            "Cache-Control": (
                "no-store, no-cache, "
                "must-revalidate, max-age=0"
            ),
            "Pragma": "no-cache",
            "Expires": "0",
        },
    )


@app.get("/api/drones")
async def get_drones() -> dict[str, Any]:
    with state_lock:
        drones = copy.deepcopy(
            latest_drones
        )

        results = copy.deepcopy(
            latest_control_results
        )

    with gimbal_lock:
        gimbal_state = copy.deepcopy(
            gimbal_angles_deg
        )

    return {
        "drones": drones,
        "control_results": results,
        "gimbal_angles_deg": gimbal_state,
        "gazebo": gazebo_bridge.status(),
        "tracking": tracking_manager.status(),
        "count": len(drones),
        "mqtt_connected": (
            mqtt_connected.is_set()
        ),
        "server_timestamp_ms": now_ms(),
    }


@app.get("/health")
async def health() -> dict[str, Any]:
    return {
        "ok": True,
        "mqtt_connected": (
            mqtt_connected.is_set()
        ),
        "index_exists": (
            INDEX_FILE.exists()
        ),
        "gazebo": gazebo_bridge.status(),
        "tracking": tracking_manager.status(),
        "server_timestamp_ms": now_ms(),
    }


@app.get("/api/camera/{drone_id}/stream")
def camera_stream(
    drone_id: str,
) -> StreamingResponse:
    if drone_id not in ALLOWED_DRONES:
        raise HTTPException(
            status_code=404,
            detail="Invalid drone ID",
        )

    if not gazebo_bridge.started:
        raise HTTPException(
            status_code=503,
            detail=(
                gazebo_bridge.error
                or "Gazebo bridge is not running"
            ),
        )

    return StreamingResponse(
        gazebo_bridge.mjpeg_frames(drone_id),
        media_type=(
            "multipart/x-mixed-replace; "
            "boundary=frame"
        ),
        headers={
            "Cache-Control": "no-store, no-cache, must-revalidate",
            "Pragma": "no-cache",
            "X-Accel-Buffering": "no",
        },
    )


@app.post("/api/tracking/start")
def start_tracking(
    request: TrackingStartRequest,
) -> dict[str, Any]:
    drone_id = request.drone_id.strip()

    if drone_id not in ALLOWED_DRONES:
        raise HTTPException(
            status_code=404,
            detail="Invalid drone ID",
        )

    if not gazebo_bridge.started:
        raise HTTPException(
            status_code=503,
            detail=(
                gazebo_bridge.error
                or "Gazebo bridge is not running"
            ),
        )

    gimbal_home = publish_gimbal_home(drone_id)

    try:
        status = tracking_manager.start(
            drone_id,
            request.desired_distance_m,
        )
    except (RuntimeError, ValueError) as error:
        raise HTTPException(
            status_code=503,
            detail=str(error),
        ) from error

    return {
        "ok": True,
        "gimbal_home": gimbal_home,
        "tracking": status,
    }


@app.post("/api/tracking/bbox")
def initialize_tracking_bbox(
    request: TrackingBBoxRequest,
) -> dict[str, Any]:
    try:
        status = tracking_manager.set_bbox(
            request.x,
            request.y,
            request.width,
            request.height,
        )
    except ValueError as error:
        raise HTTPException(
            status_code=400,
            detail=str(error),
        ) from error
    except RuntimeError as error:
        raise HTTPException(
            status_code=409,
            detail=str(error),
        ) from error

    return {
        "ok": True,
        "tracking": status,
    }


@app.post("/api/tracking/follow")
def set_visual_tracking_follow(
    request: TrackingVisualFollowRequest,
) -> dict[str, Any]:
    try:
        status = tracking_manager.set_visual_follow_requested(request.enabled)
    except RuntimeError as error:
        raise HTTPException(status_code=409, detail=str(error)) from error
    return {"ok": True, "tracking": status}


@app.post("/api/tracking/stop")
def stop_tracking() -> dict[str, Any]:
    active_drone_id = tracking_manager.status().get("drone_id", "")
    gimbal_home = (
        publish_gimbal_home(active_drone_id)
        if active_drone_id in ALLOWED_DRONES
        else {"ok": True, "error": ""}
    )
    return {
        "ok": True,
        "gimbal_home": gimbal_home,
        "tracking": tracking_manager.stop(),
    }


@app.get("/api/tracking/stream")
async def tracking_stream() -> StreamingResponse:
    status = tracking_manager.status()

    if not status["active"]:
        raise HTTPException(
            status_code=409,
            detail="Tracking is not active",
        )

    return StreamingResponse(
        tracking_manager.mjpeg_frames(),
        media_type=(
            "multipart/x-mixed-replace; "
            "boundary=frame"
        ),
        headers={
            "Cache-Control": "no-store, no-cache, must-revalidate",
            "Pragma": "no-cache",
            "X-Accel-Buffering": "no",
        },
    )


# ============================================================
# WebSocket helpers
# ============================================================

async def safe_send_json(
    websocket: WebSocket,
    send_lock: asyncio.Lock,
    payload: dict[str, Any],
) -> None:
    async with send_lock:
        await websocket.send_json(payload)


async def send_publish_result(
    websocket: WebSocket,
    send_lock: asyncio.Lock,
    *,
    ok: bool,
    drone_id: str = "",
    action: str = "",
    error: str = "",
) -> None:
    payload: dict[str, Any] = {
        "type": "control_publish_result",
        "ok": ok,
        "drone_id": drone_id,
        "server_timestamp_ms": now_ms(),
    }

    if action:
        payload["action"] = action

    if error:
        payload["error"] = error

    await safe_send_json(
        websocket,
        send_lock,
        payload,
    )


async def send_snapshots(
    websocket: WebSocket,
    send_lock: asyncio.Lock,
) -> None:
    while True:
        with state_lock:
            drones = copy.deepcopy(
                latest_drones
            )

            control_results = (
                copy.deepcopy(
                    latest_control_results
                )
            )

        with gimbal_lock:
            gimbal_state = copy.deepcopy(
                gimbal_angles_deg
            )

        await safe_send_json(
            websocket,
            send_lock,
            {
                "type": (
                    "telemetry_snapshot"
                ),
                "drones": drones,
                "control_results": (
                    control_results
                ),
                "gimbal_angles_deg": (
                    gimbal_state
                ),
                "gazebo": gazebo_bridge.status(),
                "tracking": tracking_manager.status(),
                "mqtt_connected": (
                    mqtt_connected.is_set()
                ),
                "server_timestamp_ms": (
                    now_ms()
                ),
            },
        )

        await asyncio.sleep(0.2)


# ============================================================
# WebSocket command handlers
# ============================================================

async def handle_control_action(
    websocket: WebSocket,
    send_lock: asyncio.Lock,
    message: dict[str, Any],
    drone_id: str,
) -> None:
    action = str(
        message.get("action", "")
    ).strip()

    if action not in ALLOWED_ACTIONS:
        await send_publish_result(
            websocket,
            send_lock,
            ok=False,
            drone_id=drone_id,
            action=action,
            error=(
                f"Unsupported action: "
                f"{action}"
            ),
        )
        return

    payload: dict[str, Any] = {
        "type": "action",
        "drone_id": drone_id,
        "action": action,
    }

    if action == "arm":
        with state_lock:
            drone = copy.deepcopy(
                latest_drones.get(drone_id)
            )

        recovery_reason = arm_recovery_reason(
            drone
        )

        if recovery_reason:
            ok, error = publish_control_message(
                {
                    "type": "action",
                    "drone_id": drone_id,
                    "action": "position",
                }
            )

            recovery_message = (
                "ARM blocked: requested POSITION first "
                f"because {recovery_reason}. "
                "Wait for Position/Preflight OK, then ARM again."
            )

            if not ok:
                recovery_message = (
                    f"{recovery_message} "
                    f"Recovery publish failed: {error}"
                )

            await send_publish_result(
                websocket,
                send_lock,
                ok=False,
                drone_id=drone_id,
                action=action,
                error=recovery_message,
            )
            return

    # TAKEOFF cần giữ lại altitude_m từ frontend.
    # Frontend giới hạn 2.5-10 m, nhưng backend vẫn phải kiểm tra lại.
    if action == "takeoff":
        try:
            altitude_m = finite_float(
                message.get("altitude_m"),
                minimum=2.5,
                maximum=10.0,
            )
        except (TypeError, ValueError) as error:
            await send_publish_result(
                websocket,
                send_lock,
                ok=False,
                drone_id=drone_id,
                action=action,
                error=(
                    "Takeoff altitude must be "
                    f"between 2.5 and 10.0 m: {error}"
                ),
            )
            return

        payload["altitude_m"] = altitude_m

    ok, error = publish_control_message(
        payload
    )

    await send_publish_result(
        websocket,
        send_lock,
        ok=ok,
        drone_id=drone_id,
        action=action,
        error=error,
    )


async def handle_manual_control(
    websocket: WebSocket,
    send_lock: asyncio.Lock,
    message: dict[str, Any],
    drone_id: str,
) -> None:
    enabled = message.get(
        "enabled",
        False,
    )

    if not isinstance(enabled, bool):
        await send_publish_result(
            websocket,
            send_lock,
            ok=False,
            drone_id=drone_id,
            action="manual_control",
            error=(
                "enabled must be true "
                "or false"
            ),
        )
        return

    try:
        forward = finite_float(
            message.get("forward", 0.0),
            minimum=-1.0,
            maximum=1.0,
        )

        right = finite_float(
            message.get("right", 0.0),
            minimum=-1.0,
            maximum=1.0,
        )

        up = finite_float(
            message.get("up", 0.0),
            minimum=-1.0,
            maximum=1.0,
        )

        yaw = finite_float(
            message.get("yaw", 0.0),
            minimum=-1.0,
            maximum=1.0,
        )

    except (
        TypeError,
        ValueError,
    ) as error:
        await send_publish_result(
            websocket,
            send_lock,
            ok=False,
            drone_id=drone_id,
            action="manual_control",
            error=str(error),
        )
        return

    ok, error = publish_control_message(
        {
            "type": "manual_control",
            "drone_id": drone_id,
            "enabled": enabled,
            "forward": forward,
            "right": right,
            "up": up,
            "yaw": yaw,
        }
    )

    if ok:
        with state_lock:
            manual_control_deadline[drone_id] = (
                time.monotonic() + 0.4
                if enabled
                else 0.0
            )

    # manual_control được gửi ở 20 Hz.
    # Chỉ gửi phản hồi khi có lỗi để tránh
    # làm đầy WebSocket.
    if not ok:
        await send_publish_result(
            websocket,
            send_lock,
            ok=False,
            drone_id=drone_id,
            action="manual_control",
            error=error,
        )


async def handle_goto_global(
    websocket: WebSocket,
    send_lock: asyncio.Lock,
    message: dict[str, Any],
    drone_id: str,
) -> None:
    try:
        latitude_deg = finite_float(
            message["latitude_deg"],
            minimum=-90.0,
            maximum=90.0,
        )

        longitude_deg = finite_float(
            message["longitude_deg"],
            minimum=-180.0,
            maximum=180.0,
        )

        altitude_m = finite_float(
            message.get(
                "altitude_m",
                5.0,
            ),
            minimum=1.0,
            maximum=30.0,
        )

    except (
        KeyError,
        TypeError,
        ValueError,
    ) as error:
        await send_publish_result(
            websocket,
            send_lock,
            ok=False,
            drone_id=drone_id,
            action="goto_global",
            error=(
                f"Invalid map target: "
                f"{error}"
            ),
        )
        return

    ok, error = publish_control_message(
        {
            "type": "goto_global",
            "drone_id": drone_id,
            "latitude_deg": latitude_deg,
            "longitude_deg": (
                longitude_deg
            ),
            "altitude_m": altitude_m,
        }
    )

    await send_publish_result(
        websocket,
        send_lock,
        ok=ok,
        drone_id=drone_id,
        action="goto_global",
        error=error,
    )


async def send_gimbal_result(
    websocket: WebSocket,
    send_lock: asyncio.Lock,
    *,
    ok: bool,
    drone_id: str,
    axis: str = "",
    error: str = "",
) -> None:
    with gimbal_lock:
        angles = copy.deepcopy(
            gimbal_angles_deg[drone_id]
        )

    await safe_send_json(
        websocket,
        send_lock,
        {
            "type": "gimbal_control_result",
            "ok": ok,
            "drone_id": drone_id,
            "axis": axis,
            "angles_deg": angles,
            "error": error,
            "server_timestamp_ms": now_ms(),
        },
    )


async def handle_gimbal_control(
    websocket: WebSocket,
    send_lock: asyncio.Lock,
    message: dict[str, Any],
    drone_id: str,
) -> None:
    axis = str(message.get("axis", "")).strip().lower()

    if axis not in GIMBAL_LIMITS_DEG:
        await send_gimbal_result(
            websocket,
            send_lock,
            ok=False,
            drone_id=drone_id,
            axis=axis,
            error="Axis must be roll, pitch, or yaw",
        )
        return

    if tracking_manager.controls_gimbal(drone_id):
        await send_gimbal_result(
            websocket,
            send_lock,
            ok=False,
            drone_id=drone_id,
            axis=axis,
            error=(
                "Gimbal is controlled automatically by tracking; "
                "stop tracking before manual control"
            ),
        )
        return

    try:
        delta_deg = finite_float(
            message.get("delta_deg"),
            minimum=-15.0,
            maximum=15.0,
        )
    except (TypeError, ValueError) as error:
        await send_gimbal_result(
            websocket,
            send_lock,
            ok=False,
            drone_id=drone_id,
            axis=axis,
            error=f"Invalid gimbal step: {error}",
        )
        return

    if abs(delta_deg) < 1e-9:
        await send_gimbal_result(
            websocket,
            send_lock,
            ok=False,
            drone_id=drone_id,
            axis=axis,
            error="Gimbal step must not be zero",
        )
        return

    minimum, maximum = GIMBAL_LIMITS_DEG[axis]

    with gimbal_lock:
        current = gimbal_angles_deg[drone_id][axis]
        target = max(
            minimum,
            min(maximum, current + delta_deg),
        )
        ok, error = gazebo_bridge.publish_gimbal(
            drone_id,
            axis,
            target,
        )

        if ok:
            gimbal_angles_deg[drone_id][axis] = target

    await send_gimbal_result(
        websocket,
        send_lock,
        ok=ok,
        drone_id=drone_id,
        axis=axis,
        error=error,
    )


async def handle_gimbal_home(
    websocket: WebSocket,
    send_lock: asyncio.Lock,
    drone_id: str,
) -> None:
    if tracking_manager.controls_gimbal(drone_id):
        await send_gimbal_result(
            websocket,
            send_lock,
            ok=False,
            drone_id=drone_id,
            axis="home",
            error=(
                "Gimbal is controlled automatically by tracking; "
                "stop tracking before HOME"
            ),
        )
        return

    errors: list[str] = []

    with gimbal_lock:
        for axis in GIMBAL_LIMITS_DEG:
            ok, error = gazebo_bridge.publish_gimbal(
                drone_id,
                axis,
                0.0,
            )

            if ok:
                gimbal_angles_deg[drone_id][axis] = 0.0
            else:
                errors.append(f"{axis}: {error}")

    await send_gimbal_result(
        websocket,
        send_lock,
        ok=not errors,
        drone_id=drone_id,
        axis="home",
        error="; ".join(errors),
    )


async def receive_web_commands(
    websocket: WebSocket,
    send_lock: asyncio.Lock,
) -> None:
    while True:
        raw_message = (
            await websocket.receive_text()
        )

        try:
            message = json.loads(
                raw_message
            )
        except json.JSONDecodeError:
            await send_publish_result(
                websocket,
                send_lock,
                ok=False,
                error=(
                    "Invalid WebSocket JSON"
                ),
            )
            continue

        if not isinstance(message, dict):
            await send_publish_result(
                websocket,
                send_lock,
                ok=False,
                error=(
                    "WebSocket message "
                    "must be an object"
                ),
            )
            continue

        message_type = str(
            message.get("type", "")
        ).strip()

        drone_id = str(
            message.get("drone_id", "")
        ).strip()

        if drone_id not in ALLOWED_DRONES:
            await send_publish_result(
                websocket,
                send_lock,
                ok=False,
                drone_id=drone_id,
                error="Invalid drone ID",
            )
            continue

        if message_type == "control_action":
            await handle_control_action(
                websocket,
                send_lock,
                message,
                drone_id,
            )
            continue

        if message_type == "manual_control":
            await handle_manual_control(
                websocket,
                send_lock,
                message,
                drone_id,
            )
            continue

        if message_type == "goto_global":
            await handle_goto_global(
                websocket,
                send_lock,
                message,
                drone_id,
            )
            continue

        if message_type == "gimbal_control":
            await handle_gimbal_control(
                websocket,
                send_lock,
                message,
                drone_id,
            )
            continue

        if message_type == "gimbal_home":
            await handle_gimbal_home(
                websocket,
                send_lock,
                drone_id,
            )
            continue

        await send_publish_result(
            websocket,
            send_lock,
            ok=False,
            drone_id=drone_id,
            error=(
                f"Unsupported message type: "
                f"{message_type}"
            ),
        )


# ============================================================
# WebSocket route
# ============================================================

@app.websocket("/ws")
async def dashboard_websocket(
    websocket: WebSocket,
) -> None:
    await websocket.accept()

    send_lock = asyncio.Lock()

    sender_task = asyncio.create_task(
        send_snapshots(
            websocket,
            send_lock,
        )
    )

    receiver_task = asyncio.create_task(
        receive_web_commands(
            websocket,
            send_lock,
        )
    )

    try:
        done, pending = await asyncio.wait(
            {
                sender_task,
                receiver_task,
            },
            return_when=(
                asyncio.FIRST_COMPLETED
            ),
        )

        for task in pending:
            task.cancel()

        await asyncio.gather(
            *pending,
            return_exceptions=True,
        )

        for task in done:
            if task.cancelled():
                continue

            exception = task.exception()

            if exception is not None:
                raise exception

    except (
        WebSocketDisconnect,
        RuntimeError,
        asyncio.CancelledError,
    ):
        pass

    finally:
        sender_task.cancel()
        receiver_task.cancel()

        await asyncio.gather(
            sender_task,
            receiver_task,
            return_exceptions=True,
        )
