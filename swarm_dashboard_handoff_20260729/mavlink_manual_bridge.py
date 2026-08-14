#!/usr/bin/env python3

from __future__ import annotations

import json
import logging
import math
import os
import signal
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Any

os.environ.setdefault("MAVLINK20", "1")

import paho.mqtt.client as mqtt
from pymavlink import mavutil


MQTT_HOST = "127.0.0.1"
MQTT_PORT = 1883
MQTT_TOPIC = "swarm/+/control/command"

SEND_RATE_HZ = 20.0
SEND_PERIOD_S = 1.0 / SEND_RATE_HZ

# Khi không nhận lệnh mới trong khoảng thời gian này,
# bridge tự đưa toàn bộ cần điều khiển về giữa.
COMMAND_TIMEOUT_S = 0.35
TRACKING_YAW_TIMEOUT_S = 0.35
TRACKING_YAW_MAX = 0.55
TRACKING_FOLLOW_TIMEOUT_S = 0.35
TRACKING_FOLLOW_HORIZONTAL_MAX = 0.35
TRACKING_FOLLOW_VERTICAL_MAX = 0.20
OFFBOARD_FOLLOW_TIMEOUT_S = 0.35
OFFBOARD_ATTITUDE_TIMEOUT_S = 0.35
OFFBOARD_ATTITUDE_RELEASE_TIMEOUT_S = 2.0
OFFBOARD_FORWARD_MAX_M_S = 1.0
OFFBOARD_RIGHT_MAX_M_S = 0.5
OFFBOARD_DOWN_MAX_M_S = 0.3
try:
    OFFBOARD_YAW_RATE_MAX_DEG_S = max(
        5.0,
        min(
            180.0,
            float(os.environ.get("SWARM_OFFBOARD_YAW_RATE_MAX_DEG_S", "90")),
        ),
    )
except ValueError:
    OFFBOARD_YAW_RATE_MAX_DEG_S = 90.0
OFFBOARD_PRESTREAM_FRAMES = 10
OFFBOARD_MODE_RETRY_S = 1.0
OFFBOARD_MODE_MAX_ATTEMPTS = 3
PX4_MAIN_MODE_POSCTL = 3
PX4_MAIN_MODE_AUTO = 4
PX4_MAIN_MODE_OFFBOARD = 6
PX4_SUB_MODE_AUTO_FOLLOW_TARGET = 8
VISUAL_FOLLOW_TIMEOUT_S = 0.45
VISUAL_FOLLOW_PRESTREAM_FRAMES = 5
OFFBOARD_VELOCITY_TYPE_MASK = 1479
# Ignore x/y position and vz, acceleration and yaw. Use absolute local-NED z,
# north/east velocity and yaw-rate so PX4 holds altitude without raw thrust.
OFFBOARD_LOCAL_ALTITUDE_HOLD_TYPE_MASK = 1507
# Ignore x/y position, all velocity, az and yaw. Use absolute z, horizontal
# acceleration and yaw-rate. PX4 converts acceleration to roll/pitch.
OFFBOARD_LOCAL_ACCEL_ALTITUDE_HOLD_TYPE_MASK = 1339
OFFBOARD_ATTITUDE_TYPE_MASK = 128  # Ignore quaternion; use body rates + thrust.
OFFBOARD_ROLL_RATE_MAX_DEG_S = 10.0
OFFBOARD_PITCH_RATE_MAX_DEG_S = 10.0
OFFBOARD_ATTITUDE_YAW_RATE_MAX_DEG_S = 30.0
ALLOW_RAW_ATTITUDE_OFFBOARD = os.environ.get(
    "SWARM_ALLOW_RAW_ATTITUDE_OFFBOARD",
    "false",
).strip().lower() in {"1", "true", "yes", "on"}

# PX4 SITL enables three EKF2 IMU instances by default. The uXRCE-DDS
# bridge used by this project exposes all of those uORB instances on the
# same ROS 2 topic, which makes position, attitude and vehicle status appear
# to jump between estimators. Keep one estimator for this SITL bridge. Set
# this to false when connecting the bridge to hardware that intentionally
# uses multi-EKF redundancy.
SINGLE_EKF_SITL_ENABLED = os.environ.get(
    "SWARM_PX4_SINGLE_EKF_ENABLED",
    "true",
).strip().lower() in {"1", "true", "yes", "on"}
PX4_RECONNECT_HEARTBEAT_GAP_S = 2.0
QGC_MAVLINK_PROXY_ENABLED = os.environ.get(
    "SWARM_QGC_MAVLINK_PROXY_ENABLED",
    "true",
).strip().lower() in {"1", "true", "yes", "on"}
QGC_MAVLINK_HOST = os.environ.get(
    "SWARM_QGC_MAVLINK_HOST",
    "127.0.0.1",
)
QGC_MAVLINK_PORT = int(
    os.environ.get("SWARM_QGC_MAVLINK_PORT", "14550")
)
QGC_PROXY_BASE_PORT = int(
    os.environ.get("SWARM_QGC_PROXY_BASE_PORT", "14650")
)

# Giới hạn stick để UAV không tăng tốc quá mạnh khi thử nghiệm.
HORIZONTAL_SCALE = 500
VERTICAL_SCALE = 500
YAW_SCALE = 400
TRACKING_YAW_SCALE = 600
TRACKING_FOLLOW_HORIZONTAL_SCALE = 1000
# MAVLink MANUAL_CONTROL uses z as throttle in the 0..1000 range.
# 500 is centered/hold; 0 is minimum throttle, not neutral.
THROTTLE_CENTER = 500

SOURCE_SYSTEM_ID = 250
SOURCE_COMPONENT_ID = 191

VEHICLES = {
    "UAV-01": {
        "port": 14540,
        "system_id": 1,
    },
    "UAV-02": {
        "port": 14541,
        "system_id": 2,
    },
}


logging.basicConfig(
    level=logging.INFO,
    format=(
        "%(asctime)s | %(levelname)s | %(message)s"
    ),
)

LOGGER = logging.getLogger("mavlink-manual-bridge")


def clamp(value: Any, minimum: float, maximum: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0

    if not math.isfinite(number):
        return 0.0

    return max(minimum, min(maximum, number))


@dataclass
class ManualState:
    enabled: bool = False
    forward: float = 0.0
    right: float = 0.0
    up: float = 0.0
    yaw: float = 0.0
    last_command_time: float = 0.0
    lock: threading.Lock = field(
        default_factory=threading.Lock
    )

    def update(self, payload: dict[str, Any]) -> None:
        with self.lock:
            self.enabled = bool(
                payload.get("enabled", False)
            )

            self.forward = clamp(
                payload.get("forward", 0.0),
                -1.0,
                1.0,
            )

            self.right = clamp(
                payload.get("right", 0.0),
                -1.0,
                1.0,
            )

            self.up = clamp(
                payload.get("up", 0.0),
                -1.0,
                1.0,
            )

            self.yaw = clamp(
                payload.get("yaw", 0.0),
                -1.0,
                1.0,
            )

            self.last_command_time = time.monotonic()

    def disable(self) -> None:
        with self.lock:
            self.enabled = False
            self.forward = 0.0
            self.right = 0.0
            self.up = 0.0
            self.yaw = 0.0
            self.last_command_time = time.monotonic()

    def output(self) -> tuple[int, int, int, int, bool]:
        with self.lock:
            age = (
                time.monotonic()
                - self.last_command_time
            )

            active = (
                self.enabled
                and age <= COMMAND_TIMEOUT_S
            )

            if not active:
                return 0, 0, THROTTLE_CENTER, 0, False

            x = int(
                round(
                    self.forward
                    * HORIZONTAL_SCALE
                )
            )

            y = int(
                round(
                    self.right
                    * HORIZONTAL_SCALE
                )
            )

            z = int(
                round(
                    THROTTLE_CENTER
                    + self.up
                    * VERTICAL_SCALE
                )
            )
            z = max(0, min(1000, z))

            r = int(
                round(
                    self.yaw
                    * YAW_SCALE
                )
            )

            return x, y, z, r, True


@dataclass
class TrackingYawState:
    enabled: bool = False
    yaw: float = 0.0
    last_command_time: float = 0.0
    lock: threading.Lock = field(
        default_factory=threading.Lock
    )

    def update(self, payload: dict[str, Any]) -> None:
        with self.lock:
            self.enabled = bool(
                payload.get("enabled", False)
            )
            self.yaw = clamp(
                payload.get("yaw", 0.0),
                -TRACKING_YAW_MAX,
                TRACKING_YAW_MAX,
            )
            self.last_command_time = time.monotonic()

    def disable(self) -> None:
        with self.lock:
            self.enabled = False
            self.yaw = 0.0
            self.last_command_time = time.monotonic()

    def output(self) -> tuple[int, bool]:
        with self.lock:
            age = time.monotonic() - self.last_command_time
            active = (
                self.enabled
                and age <= TRACKING_YAW_TIMEOUT_S
                and abs(self.yaw) > 1e-6
            )
            if not active:
                return 0, False
            return int(round(self.yaw * TRACKING_YAW_SCALE)), True


@dataclass
class TrackingFollowState:
    enabled: bool = False
    forward: float = 0.0
    right: float = 0.0
    up: float = 0.0
    last_command_time: float = 0.0
    lock: threading.Lock = field(
        default_factory=threading.Lock
    )

    def update(self, payload: dict[str, Any]) -> None:
        with self.lock:
            self.enabled = bool(payload.get("enabled", False))
            self.forward = clamp(
                payload.get("forward", 0.0),
                -TRACKING_FOLLOW_HORIZONTAL_MAX,
                TRACKING_FOLLOW_HORIZONTAL_MAX,
            )
            self.right = clamp(
                payload.get("right", 0.0),
                -TRACKING_FOLLOW_HORIZONTAL_MAX,
                TRACKING_FOLLOW_HORIZONTAL_MAX,
            )
            self.up = clamp(
                payload.get("up", 0.0),
                -TRACKING_FOLLOW_VERTICAL_MAX,
                TRACKING_FOLLOW_VERTICAL_MAX,
            )
            self.last_command_time = time.monotonic()

    def disable(self) -> None:
        with self.lock:
            self.enabled = False
            self.forward = 0.0
            self.right = 0.0
            self.up = 0.0
            self.last_command_time = time.monotonic()

    def output(self) -> tuple[int, int, int, bool]:
        with self.lock:
            age = time.monotonic() - self.last_command_time
            active = (
                self.enabled
                and age <= TRACKING_FOLLOW_TIMEOUT_S
                and max(
                    abs(self.forward),
                    abs(self.right),
                    abs(self.up),
                ) > 1e-6
            )
            if not active:
                return 0, 0, THROTTLE_CENTER, False
            x = int(
                round(
                    self.forward
                    * TRACKING_FOLLOW_HORIZONTAL_SCALE
                )
            )
            y = int(
                round(
                    self.right
                    * TRACKING_FOLLOW_HORIZONTAL_SCALE
                )
            )
            z = int(round(THROTTLE_CENTER + self.up * VERTICAL_SCALE))
            return x, y, max(0, min(1000, z)), True


@dataclass
class VisualFollowTargetState:
    enabled: bool = False
    valid: bool = False
    latitude_deg: float = 0.0
    longitude_deg: float = 0.0
    altitude_msl_m: float = 0.0
    velocity_north_m_s: float = 0.0
    velocity_east_m_s: float = 0.0
    velocity_down_m_s: float = 0.0
    quality: float = 0.0
    follow_distance_m: float = 10.0
    follow_height_m: float = 8.0
    last_command_time: float = 0.0
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update(self, payload: dict[str, Any]) -> None:
        with self.lock:
            self.enabled = bool(payload.get("enabled", False))
            try:
                values = (
                    float(payload["latitude_deg"]),
                    float(payload["longitude_deg"]),
                    float(payload["altitude_msl_m"]),
                    float(payload.get("velocity_north_m_s", 0.0)),
                    float(payload.get("velocity_east_m_s", 0.0)),
                    float(payload.get("velocity_down_m_s", 0.0)),
                    float(payload.get("quality", 0.0)),
                    float(payload.get("follow_distance_m", 10.0)),
                    float(payload.get("follow_height_m", 8.0)),
                )
                self.valid = bool(
                    self.enabled
                    and all(math.isfinite(value) for value in values)
                    and -90.0 <= values[0] <= 90.0
                    and -180.0 <= values[1] <= 180.0
                    and 0.0 <= values[6] <= 1.0
                    and 3.0 <= values[7] <= 20.0
                    and 1.0 <= values[8] <= 30.0
                )
                if self.valid:
                    (
                        self.latitude_deg,
                        self.longitude_deg,
                        self.altitude_msl_m,
                        self.velocity_north_m_s,
                        self.velocity_east_m_s,
                        self.velocity_down_m_s,
                        self.quality,
                        self.follow_distance_m,
                        self.follow_height_m,
                    ) = values
            except (KeyError, TypeError, ValueError):
                self.valid = False
            self.last_command_time = time.monotonic()

    def disable(self) -> None:
        with self.lock:
            self.enabled = False
            self.valid = False
            self.last_command_time = time.monotonic()

    def output(
        self,
    ) -> tuple[float, float, float, float, float, float, float, float, float, bool]:
        with self.lock:
            active = bool(
                self.enabled
                and self.valid
                and time.monotonic() - self.last_command_time
                <= VISUAL_FOLLOW_TIMEOUT_S
            )
            return (
                self.latitude_deg,
                self.longitude_deg,
                self.altitude_msl_m,
                self.velocity_north_m_s,
                self.velocity_east_m_s,
                self.velocity_down_m_s,
                self.quality,
                self.follow_distance_m,
                self.follow_height_m,
                active,
            )


@dataclass
class OffboardFollowState:
    enabled: bool = False
    forward_velocity_m_s: float = 0.0
    right_velocity_m_s: float = 0.0
    down_velocity_m_s: float = 0.0
    yaw_rate_deg_s: float = 0.0
    local_altitude_hold: bool = False
    local_acceleration_hold: bool = False
    north_velocity_m_s: float = 0.0
    east_velocity_m_s: float = 0.0
    hold_z_down_m: float = 0.0
    north_acceleration_m_s2: float = 0.0
    east_acceleration_m_s2: float = 0.0
    last_command_time: float = 0.0
    lock: threading.Lock = field(
        default_factory=threading.Lock
    )

    def update(self, payload: dict[str, Any]) -> None:
        with self.lock:
            self.enabled = bool(payload.get("enabled", False))
            self.forward_velocity_m_s = clamp(
                payload.get("forward_velocity_m_s", 0.0),
                -OFFBOARD_FORWARD_MAX_M_S,
                OFFBOARD_FORWARD_MAX_M_S,
            )
            self.right_velocity_m_s = clamp(
                payload.get("right_velocity_m_s", 0.0),
                -OFFBOARD_RIGHT_MAX_M_S,
                OFFBOARD_RIGHT_MAX_M_S,
            )
            self.down_velocity_m_s = clamp(
                payload.get("down_velocity_m_s", 0.0),
                -OFFBOARD_DOWN_MAX_M_S,
                OFFBOARD_DOWN_MAX_M_S,
            )
            self.yaw_rate_deg_s = clamp(
                payload.get("yaw_rate_deg_s", 0.0),
                -OFFBOARD_YAW_RATE_MAX_DEG_S,
                OFFBOARD_YAW_RATE_MAX_DEG_S,
            )
            frame = payload.get("velocity_frame")
            self.local_altitude_hold = frame in {
                "local_ned_altitude_hold",
                "local_ned_acceleration_altitude_hold",
            }
            self.local_acceleration_hold = (
                frame == "local_ned_acceleration_altitude_hold"
            )
            self.north_velocity_m_s = clamp(
                payload.get("north_velocity_m_s", 0.0),
                -OFFBOARD_FORWARD_MAX_M_S,
                OFFBOARD_FORWARD_MAX_M_S,
            )
            self.east_velocity_m_s = clamp(
                payload.get("east_velocity_m_s", 0.0),
                -OFFBOARD_RIGHT_MAX_M_S,
                OFFBOARD_RIGHT_MAX_M_S,
            )
            self.hold_z_down_m = clamp(
                payload.get("hold_z_down_m", 0.0),
                -100.0,
                10.0,
            )
            self.north_acceleration_m_s2 = clamp(
                payload.get("north_acceleration_m_s2", 0.0),
                -3.0,
                3.0,
            )
            self.east_acceleration_m_s2 = clamp(
                payload.get("east_acceleration_m_s2", 0.0),
                -3.0,
                3.0,
            )
            self.last_command_time = time.monotonic()

    def disable(self) -> None:
        with self.lock:
            self.enabled = False
            self.forward_velocity_m_s = 0.0
            self.right_velocity_m_s = 0.0
            self.down_velocity_m_s = 0.0
            self.yaw_rate_deg_s = 0.0
            self.local_altitude_hold = False
            self.local_acceleration_hold = False
            self.north_velocity_m_s = 0.0
            self.east_velocity_m_s = 0.0
            self.hold_z_down_m = 0.0
            self.north_acceleration_m_s2 = 0.0
            self.east_acceleration_m_s2 = 0.0
            self.last_command_time = time.monotonic()

    def output(
        self,
    ) -> tuple[
        float, float, float, float, bool, float, float, float, bool,
        float, float, bool,
    ]:
        with self.lock:
            age = time.monotonic() - self.last_command_time
            active = self.enabled and age <= OFFBOARD_FOLLOW_TIMEOUT_S
            if not active:
                return (
                    0.0, 0.0, 0.0, 0.0, False, 0.0, 0.0, 0.0, False,
                    0.0, 0.0, False,
                )
            return (
                self.forward_velocity_m_s,
                self.right_velocity_m_s,
                self.down_velocity_m_s,
                self.yaw_rate_deg_s,
                True,
                self.north_velocity_m_s,
                self.east_velocity_m_s,
                self.hold_z_down_m,
                self.local_altitude_hold,
                self.north_acceleration_m_s2,
                self.east_acceleration_m_s2,
                self.local_acceleration_hold,
            )


@dataclass
class OffboardAttitudeState:
    enabled: bool = False
    roll_rate_deg_s: float = 0.0
    pitch_rate_deg_s: float = 0.0
    yaw_rate_deg_s: float = 0.0
    thrust: float = 0.0
    last_command_time: float = 0.0
    releasing: bool = False
    release_started_time: float = 0.0
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update(self, payload: dict[str, Any]) -> None:
        with self.lock:
            requested_enabled = bool(payload.get("enabled", False))
            was_active = self.enabled or self.releasing
            self.enabled = requested_enabled
            if requested_enabled:
                self.releasing = False
                self.release_started_time = 0.0
            elif was_active:
                self.releasing = True
                self.release_started_time = time.monotonic()
            self.roll_rate_deg_s = clamp(
                payload.get("roll_rate_deg_s", 0.0),
                -OFFBOARD_ROLL_RATE_MAX_DEG_S,
                OFFBOARD_ROLL_RATE_MAX_DEG_S,
            )
            self.pitch_rate_deg_s = clamp(
                payload.get("pitch_rate_deg_s", 0.0),
                -OFFBOARD_PITCH_RATE_MAX_DEG_S,
                OFFBOARD_PITCH_RATE_MAX_DEG_S,
            )
            self.yaw_rate_deg_s = clamp(
                payload.get("yaw_rate_deg_s", 0.0),
                -OFFBOARD_ATTITUDE_YAW_RATE_MAX_DEG_S,
                OFFBOARD_ATTITUDE_YAW_RATE_MAX_DEG_S,
            )
            requested_thrust = clamp(payload.get("thrust", 0.0), 0.0, 1.0)
            if requested_enabled:
                self.thrust = requested_thrust
            elif self.releasing:
                self.roll_rate_deg_s = 0.0
                self.pitch_rate_deg_s = 0.0
                self.yaw_rate_deg_s = 0.0
            self.last_command_time = time.monotonic()

    def disable(self) -> None:
        with self.lock:
            self.enabled = False
            self.roll_rate_deg_s = 0.0
            self.pitch_rate_deg_s = 0.0
            self.yaw_rate_deg_s = 0.0
            self.thrust = 0.0
            self.releasing = False
            self.release_started_time = 0.0
            self.last_command_time = time.monotonic()

    def finish_release(self) -> None:
        with self.lock:
            self.releasing = False
            self.release_started_time = 0.0
            if not self.enabled:
                self.thrust = 0.0

    def output(self) -> tuple[float, float, float, float, bool, bool]:
        with self.lock:
            age = time.monotonic() - self.last_command_time
            if self.enabled and age > OFFBOARD_ATTITUDE_TIMEOUT_S:
                self.enabled = False
                self.releasing = True
                self.release_started_time = time.monotonic()
                self.roll_rate_deg_s = 0.0
                self.pitch_rate_deg_s = 0.0
                self.yaw_rate_deg_s = 0.0
            release_age = time.monotonic() - self.release_started_time
            releasing = bool(
                self.releasing
                and release_age <= OFFBOARD_ATTITUDE_RELEASE_TIMEOUT_S
                and 0.05 <= self.thrust <= 0.95
            )
            active = releasing or (
                self.enabled
                and age <= OFFBOARD_ATTITUDE_TIMEOUT_S
                and 0.05 <= self.thrust <= 0.95
            )
            if not active:
                return 0.0, 0.0, 0.0, 0.0, False, False
            return (
                self.roll_rate_deg_s,
                self.pitch_rate_deg_s,
                self.yaw_rate_deg_s,
                self.thrust,
                True,
                releasing,
            )


class MavlinkWorker(threading.Thread):
    def __init__(
        self,
        drone_id: str,
        port: int,
        expected_system_id: int,
        state: ManualState,
        tracking_yaw_state: TrackingYawState,
        tracking_follow_state: TrackingFollowState,
        visual_follow_target_state: VisualFollowTargetState,
        offboard_follow_state: OffboardFollowState,
        offboard_attitude_state: OffboardAttitudeState,
        stop_event: threading.Event,
    ) -> None:
        super().__init__(
            name=f"mavlink-{drone_id}",
            daemon=True,
        )

        self.drone_id = drone_id
        self.port = port
        self.expected_system_id = expected_system_id
        self.state = state
        self.tracking_yaw_state = tracking_yaw_state
        self.tracking_follow_state = tracking_follow_state
        self.visual_follow_target_state = visual_follow_target_state
        self.offboard_follow_state = offboard_follow_state
        self.offboard_attitude_state = offboard_attitude_state
        self.stop_event = stop_event
        self.connection: Any = None
        self.offboard_mode_requested = False
        self.offboard_stream_frames = 0
        self.last_mode_request_monotonic = 0.0
        self.last_px4_main_mode: int | None = None
        self.last_px4_sub_mode: int | None = None
        self.offboard_request_attempts = 0
        self.offboard_entry_blocked = False
        self.offboard_input_active = False
        self.visual_follow_mode_requested = False
        self.visual_follow_stream_frames = 0
        self.visual_follow_request_attempts = 0
        self.visual_follow_entry_blocked = False
        self.visual_follow_input_active = False
        self.visual_follow_params_configured = False
        self.last_vehicle_heartbeat_monotonic = 0.0
        self.single_ekf_configured = False
        self.qgc_socket: socket.socket | None = None

    def connect(self) -> Any:
        connection_string = (
            f"udpin:0.0.0.0:{self.port}"
        )

        LOGGER.info(
            "%s: listening for PX4 on %s",
            self.drone_id,
            connection_string,
        )

        connection = mavutil.mavlink_connection(
            connection_string,
            source_system=SOURCE_SYSTEM_ID,
            source_component=SOURCE_COMPONENT_ID,
            dialect="common",
            robust_parsing=True,
        )

        deadline = time.monotonic() + 15.0
        heartbeat = None
        while time.monotonic() < deadline:
            candidate = connection.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=max(0.1, deadline - time.monotonic()),
            )
            if candidate is None:
                continue
            if candidate.get_srcSystem() == self.expected_system_id:
                heartbeat = candidate
                break

        if heartbeat is None:
            connection.close()
            raise TimeoutError(
                f"{self.drone_id}: heartbeat timeout for system "
                f"{self.expected_system_id} on UDP {self.port}"
            )

        received_system_id = heartbeat.get_srcSystem()
        received_component_id = heartbeat.get_srcComponent()

        LOGGER.info(
            "%s connected: system=%d component=%d port=%d",
            self.drone_id,
            received_system_id,
            received_component_id,
            self.port,
        )

        return connection

    def send_centered_controls(
        self,
        count: int = 5,
    ) -> None:
        if self.connection is None:
            return

        for _ in range(count):
            try:
                self.connection.mav.manual_control_send(
                    self.expected_system_id,
                    0,
                    0,
                    THROTTLE_CENTER,
                    0,
                    0,
                )
            except Exception:
                break

            time.sleep(0.03)

    def configure_single_ekf_sitl(self) -> None:
        if (
            not SINGLE_EKF_SITL_ENABLED
            or self.connection is None
            or self.single_ekf_configured
        ):
            return

        for name in (b"EKF2_MULTI_IMU", b"EKF2_MULTI_MAG"):
            self.connection.mav.param_set_send(
                self.expected_system_id,
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                name,
                0.0,
                mavutil.mavlink.MAV_PARAM_TYPE_INT32,
            )

        self.single_ekf_configured = True
        LOGGER.info(
            "%s configured PX4 SITL single-EKF output",
            self.drone_id,
        )

    def open_qgc_proxy(self) -> None:
        if not QGC_MAVLINK_PROXY_ENABLED or self.qgc_socket is not None:
            return
        proxy_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        proxy_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        proxy_port = QGC_PROXY_BASE_PORT + self.expected_system_id - 1
        proxy_socket.bind(("127.0.0.1", proxy_port))
        proxy_socket.setblocking(False)
        self.qgc_socket = proxy_socket
        LOGGER.info(
            "%s QGC proxy: 127.0.0.1:%d -> %s:%d",
            self.drone_id,
            proxy_port,
            QGC_MAVLINK_HOST,
            QGC_MAVLINK_PORT,
        )

    def forward_qgc_commands(self) -> None:
        if self.qgc_socket is None or self.connection is None:
            return
        for _ in range(30):
            try:
                payload, _address = self.qgc_socket.recvfrom(65535)
            except BlockingIOError:
                break
            if payload:
                self.connection.write(payload)

    def forward_px4_message_to_qgc(self, message: Any) -> None:
        if self.qgc_socket is None:
            return
        payload = message.get_msgbuf()
        if payload:
            self.qgc_socket.sendto(
                payload,
                (QGC_MAVLINK_HOST, QGC_MAVLINK_PORT),
            )

    def request_px4_main_mode(
        self,
        main_mode: int,
        label: str,
        sub_mode: int = 0,
    ) -> None:
        if self.connection is None:
            return
        self.connection.mav.command_long_send(
            self.expected_system_id,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            float(main_mode),
            float(sub_mode),
            0.0,
            0.0,
            0.0,
            0.0,
        )
        self.last_mode_request_monotonic = time.monotonic()
        LOGGER.info(
            "%s requested PX4 mode: %s",
            self.drone_id,
            label,
        )

    def request_position_mode(self) -> None:
        offboard_observed = self.last_px4_main_mode == PX4_MAIN_MODE_OFFBOARD
        visual_follow_observed = bool(
            self.last_px4_main_mode == PX4_MAIN_MODE_AUTO
            and self.last_px4_sub_mode == PX4_SUB_MODE_AUTO_FOLLOW_TARGET
        )
        if (
            (
                self.offboard_mode_requested
                or offboard_observed
                or self.visual_follow_mode_requested
                or visual_follow_observed
            )
            and (
                self.offboard_mode_requested
                or time.monotonic() - self.last_mode_request_monotonic
                >= OFFBOARD_MODE_RETRY_S
            )
        ):
            self.request_px4_main_mode(
                PX4_MAIN_MODE_POSCTL,
                "POSITION",
            )
            self.offboard_mode_requested = False
            self.visual_follow_mode_requested = False
        if self.last_px4_main_mode == PX4_MAIN_MODE_POSCTL:
            self.offboard_mode_requested = False
            self.offboard_stream_frames = 0
            self.visual_follow_mode_requested = False
            self.visual_follow_stream_frames = 0

    def send_visual_follow_target(
        self,
        latitude_deg: float,
        longitude_deg: float,
        altitude_msl_m: float,
        velocity_north_m_s: float,
        velocity_east_m_s: float,
        velocity_down_m_s: float,
        quality: float,
    ) -> None:
        assert self.connection is not None
        uncertainty = max(0.5, 8.0 * (1.0 - float(quality)))
        self.connection.mav.follow_target_send(
            int(time.monotonic() * 1000.0),
            3,  # Position and velocity estimates are valid.
            int(round(float(latitude_deg) * 1e7)),
            int(round(float(longitude_deg) * 1e7)),
            float(altitude_msl_m),
            [
                float(velocity_north_m_s),
                float(velocity_east_m_s),
                float(velocity_down_m_s),
            ],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [uncertainty, uncertainty, uncertainty],
            0,
        )

    def configure_visual_follow_parameters(
        self,
        follow_distance_m: float,
        follow_height_m: float,
    ) -> None:
        assert self.connection is not None
        try:
            follow_response_s = max(
                0.1,
                min(
                    1.0,
                    float(os.environ.get("SWARM_VISUAL_FOLLOW_RESPONSE_S", "0.75")),
                ),
            )
            follow_max_velocity = max(
                0.4,
                min(
                    1.5,
                    float(os.environ.get("SWARM_VISUAL_FOLLOW_MAX_VEL_M_S", "0.4")),
                ),
            )
        except ValueError:
            follow_response_s = 0.75
            follow_max_velocity = 0.4
        parameters = (
            (b"FLW_TGT_ALT_M", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
            (b"FLW_TGT_DST", float(follow_distance_m), mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            # PX4 documents 8 m as the minimum native Follow height.
            (b"FLW_TGT_HT", max(8.0, float(follow_height_m)), mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (b"FLW_TGT_RS", follow_response_s, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
            (b"FLW_TGT_MAX_VEL", follow_max_velocity, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
        )
        for name, value, parameter_type in parameters:
            self.connection.mav.param_set_send(
                self.expected_system_id,
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                name,
                value,
                parameter_type,
            )

    def send_offboard_setpoint(
        self,
        forward_velocity_m_s: float,
        right_velocity_m_s: float,
        down_velocity_m_s: float,
        yaw_rate_deg_s: float,
        north_velocity_m_s: float = 0.0,
        east_velocity_m_s: float = 0.0,
        hold_z_down_m: float = 0.0,
        local_altitude_hold: bool = False,
        north_acceleration_m_s2: float = 0.0,
        east_acceleration_m_s2: float = 0.0,
        local_acceleration_hold: bool = False,
    ) -> None:
        assert self.connection is not None
        frame = (
            mavutil.mavlink.MAV_FRAME_LOCAL_NED
            if local_altitude_hold
            else mavutil.mavlink.MAV_FRAME_BODY_NED
        )
        type_mask = (
            OFFBOARD_LOCAL_ACCEL_ALTITUDE_HOLD_TYPE_MASK
            if local_acceleration_hold
            else OFFBOARD_LOCAL_ALTITUDE_HOLD_TYPE_MASK
            if local_altitude_hold
            else OFFBOARD_VELOCITY_TYPE_MASK
        )
        self.connection.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000.0) & 0xFFFFFFFF,
            self.expected_system_id,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            frame,
            type_mask,
            0.0,
            0.0,
            float(hold_z_down_m) if local_altitude_hold else 0.0,
            (
                float(north_velocity_m_s)
                if local_altitude_hold and not local_acceleration_hold
                else float(forward_velocity_m_s)
                if not local_acceleration_hold
                else 0.0
            ),
            (
                float(east_velocity_m_s)
                if local_altitude_hold and not local_acceleration_hold
                else float(right_velocity_m_s)
                if not local_acceleration_hold
                else 0.0
            ),
            0.0 if local_altitude_hold else float(down_velocity_m_s),
            float(north_acceleration_m_s2) if local_acceleration_hold else 0.0,
            float(east_acceleration_m_s2) if local_acceleration_hold else 0.0,
            0.0,
            0.0,
            math.radians(float(yaw_rate_deg_s)),
        )

    def send_offboard_attitude_setpoint(
        self,
        roll_rate_deg_s: float,
        pitch_rate_deg_s: float,
        yaw_rate_deg_s: float,
        thrust: float,
    ) -> None:
        assert self.connection is not None
        self.connection.mav.set_attitude_target_send(
            int(time.monotonic() * 1000.0) & 0xFFFFFFFF,
            self.expected_system_id,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            OFFBOARD_ATTITUDE_TYPE_MASK,
            [1.0, 0.0, 0.0, 0.0],
            math.radians(float(roll_rate_deg_s)),
            math.radians(float(pitch_rate_deg_s)),
            math.radians(float(yaw_rate_deg_s)),
            float(thrust),
        )

    def run_connected(self) -> None:
        assert self.connection is not None

        last_heartbeat_sent = 0.0
        last_control_source: str | None = None
        self.single_ekf_configured = False
        self.last_vehicle_heartbeat_monotonic = time.monotonic()
        self.configure_single_ekf_sitl()
        self.open_qgc_proxy()

        while not self.stop_event.is_set():
            loop_started = time.monotonic()

            # Đọc và bỏ các gói telemetry đang chờ để duy trì socket.
            for _ in range(30):
                message = self.connection.recv_match(
                    blocking=False
                )

                if message is None:
                    break
                self.forward_px4_message_to_qgc(message)
                if (
                    message.get_type() == "HEARTBEAT"
                    and message.get_srcSystem() == self.expected_system_id
                ):
                    heartbeat_now = time.monotonic()
                    if (
                        self.last_vehicle_heartbeat_monotonic > 0.0
                        and heartbeat_now
                        - self.last_vehicle_heartbeat_monotonic
                        >= PX4_RECONNECT_HEARTBEAT_GAP_S
                    ):
                        self.single_ekf_configured = False
                    self.last_vehicle_heartbeat_monotonic = heartbeat_now
                    self.configure_single_ekf_sitl()
                    custom_mode = int(getattr(message, "custom_mode", 0))
                    self.last_px4_main_mode = (custom_mode >> 16) & 0xFF
                    self.last_px4_sub_mode = (custom_mode >> 24) & 0xFF
                    if self.last_px4_main_mode == PX4_MAIN_MODE_POSCTL:
                        self.offboard_attitude_state.finish_release()

            self.forward_qgc_commands()

            x, y, z, r, manual_active = self.state.output()
            tracking_r, tracking_active = self.tracking_yaw_state.output()
            follow_x, follow_y, follow_z, follow_active = (
                self.tracking_follow_state.output()
            )
            (
                visual_lat,
                visual_lon,
                visual_alt,
                visual_vn,
                visual_ve,
                visual_vd,
                visual_quality,
                visual_follow_distance,
                visual_follow_height,
                visual_follow_active,
            ) = self.visual_follow_target_state.output()
            (
                offboard_forward,
                offboard_right,
                offboard_down,
                offboard_yaw_rate,
                offboard_active,
                offboard_north,
                offboard_east,
                offboard_hold_z,
                offboard_local_altitude_hold,
                offboard_north_acceleration,
                offboard_east_acceleration,
                offboard_local_acceleration_hold,
            ) = self.offboard_follow_state.output()
            (
                attitude_roll_rate,
                attitude_pitch_rate,
                attitude_yaw_rate,
                attitude_thrust,
                attitude_active,
                attitude_releasing,
            ) = self.offboard_attitude_state.output()

            automation_active = (
                attitude_active or offboard_active or visual_follow_active
            )
            if automation_active and not self.offboard_input_active:
                self.offboard_request_attempts = 0
                self.offboard_entry_blocked = False
                self.offboard_stream_frames = 0
            self.offboard_input_active = automation_active

            if visual_follow_active and not self.visual_follow_input_active:
                self.visual_follow_stream_frames = 0
                self.visual_follow_request_attempts = 0
                self.visual_follow_entry_blocked = False
                self.visual_follow_params_configured = False
            self.visual_follow_input_active = visual_follow_active

            if manual_active:
                self.request_position_mode()
                control_source = "MANUAL"
                self.connection.mav.manual_control_send(
                    self.expected_system_id,
                    x,
                    y,
                    z,
                    r,
                    0,
                )
            elif visual_follow_active and not self.visual_follow_entry_blocked:
                if not self.visual_follow_params_configured:
                    self.configure_visual_follow_parameters(
                        visual_follow_distance,
                        visual_follow_height,
                    )
                    self.visual_follow_params_configured = True
                self.send_visual_follow_target(
                    visual_lat,
                    visual_lon,
                    visual_alt,
                    visual_vn,
                    visual_ve,
                    visual_vd,
                    visual_quality,
                )
                self.visual_follow_stream_frames += 1
                now = time.monotonic()
                follow_mode_observed = bool(
                    self.last_px4_main_mode == PX4_MAIN_MODE_AUTO
                    and self.last_px4_sub_mode
                    == PX4_SUB_MODE_AUTO_FOLLOW_TARGET
                )
                if (
                    not follow_mode_observed
                    and self.visual_follow_stream_frames
                    >= VISUAL_FOLLOW_PRESTREAM_FRAMES
                    and self.visual_follow_request_attempts
                    < OFFBOARD_MODE_MAX_ATTEMPTS
                    and (
                        not self.visual_follow_mode_requested
                        or now - self.last_mode_request_monotonic
                        >= OFFBOARD_MODE_RETRY_S
                    )
                ):
                    self.request_px4_main_mode(
                        PX4_MAIN_MODE_AUTO,
                        "AUTO FOLLOW TARGET",
                        PX4_SUB_MODE_AUTO_FOLLOW_TARGET,
                    )
                    self.visual_follow_mode_requested = True
                    self.visual_follow_request_attempts += 1
                if (
                    not follow_mode_observed
                    and self.visual_follow_request_attempts
                    >= OFFBOARD_MODE_MAX_ATTEMPTS
                    and now - self.last_mode_request_monotonic
                    >= OFFBOARD_MODE_RETRY_S
                ):
                    self.visual_follow_entry_blocked = True
                    LOGGER.error(
                        "%s: PX4 did not confirm Follow Target after %d attempts; "
                        "blocking visual follow",
                        self.drone_id,
                        self.visual_follow_request_attempts,
                    )
                control_source = "VISUAL_FOLLOW_TARGET"
            elif visual_follow_active:
                self.request_position_mode()
                control_source = "VISUAL_FOLLOW_ENTRY_BLOCKED"
            elif attitude_active:
                self.send_offboard_attitude_setpoint(
                    attitude_roll_rate,
                    attitude_pitch_rate,
                    attitude_yaw_rate,
                    attitude_thrust,
                )
                self.connection.mav.manual_control_send(
                    self.expected_system_id,
                    0,
                    0,
                    THROTTLE_CENTER,
                    0,
                    0,
                )
                if attitude_releasing:
                    self.request_position_mode()
                    control_source = "OFFBOARD_ATTITUDE_RELEASE"
                else:
                    self.offboard_stream_frames += 1
                    now = time.monotonic()
                    if (
                        self.offboard_stream_frames >= OFFBOARD_PRESTREAM_FRAMES
                        and (
                            not self.offboard_mode_requested
                            or now - self.last_mode_request_monotonic
                            >= OFFBOARD_MODE_RETRY_S
                        )
                    ):
                        self.request_px4_main_mode(
                            PX4_MAIN_MODE_OFFBOARD,
                            "OFFBOARD",
                        )
                        self.offboard_mode_requested = True
                    control_source = "OFFBOARD_ATTITUDE_FOLLOW"
            elif offboard_active and not self.offboard_entry_blocked:
                self.send_offboard_setpoint(
                    offboard_forward,
                    offboard_right,
                    offboard_down,
                    offboard_yaw_rate,
                    offboard_north,
                    offboard_east,
                    offboard_hold_z,
                    offboard_local_altitude_hold,
                    offboard_north_acceleration,
                    offboard_east_acceleration,
                    offboard_local_acceleration_hold,
                )
                self.connection.mav.manual_control_send(
                    self.expected_system_id,
                    0,
                    0,
                    THROTTLE_CENTER,
                    0,
                    0,
                )
                self.offboard_stream_frames += 1
                now = time.monotonic()
                if (
                    self.last_px4_main_mode != PX4_MAIN_MODE_OFFBOARD
                    and
                    self.offboard_stream_frames >= OFFBOARD_PRESTREAM_FRAMES
                    and self.offboard_request_attempts < OFFBOARD_MODE_MAX_ATTEMPTS
                    and (
                        not self.offboard_mode_requested
                        or now - self.last_mode_request_monotonic
                        >= OFFBOARD_MODE_RETRY_S
                    )
                ):
                    self.request_px4_main_mode(
                        PX4_MAIN_MODE_OFFBOARD,
                        "OFFBOARD",
                    )
                    self.offboard_mode_requested = True
                    self.offboard_request_attempts += 1
                if (
                    self.last_px4_main_mode != PX4_MAIN_MODE_OFFBOARD
                    and self.offboard_request_attempts >= OFFBOARD_MODE_MAX_ATTEMPTS
                    and now - self.last_mode_request_monotonic
                    >= OFFBOARD_MODE_RETRY_S
                ):
                    self.offboard_entry_blocked = True
                    LOGGER.error(
                        "%s: PX4 did not confirm OFFBOARD after %d attempts; "
                        "blocking automation until tracking stops",
                        self.drone_id,
                        self.offboard_request_attempts,
                    )
                control_source = "OFFBOARD_FOLLOW"
            elif offboard_active:
                self.request_position_mode()
                control_source = "OFFBOARD_ENTRY_BLOCKED"
                self.connection.mav.manual_control_send(
                    self.expected_system_id,
                    0,
                    0,
                    THROTTLE_CENTER,
                    0,
                    0,
                )
            elif follow_active or tracking_active:
                self.request_position_mode()
                x = follow_x if follow_active else 0
                y = follow_y if follow_active else 0
                z = follow_z if follow_active else THROTTLE_CENTER
                r = tracking_r
                control_source = (
                    "TRACKING_FOLLOW"
                    if follow_active
                    else "TRACKING_YAW"
                )
                self.connection.mav.manual_control_send(
                    self.expected_system_id,
                    x,
                    y,
                    z,
                    r,
                    0,
                )
            else:
                self.request_position_mode()
                control_source = "CENTERED"
                self.connection.mav.manual_control_send(
                    self.expected_system_id,
                    x,
                    y,
                    z,
                    r,
                    0,
                )

            now = time.monotonic()

            if now - last_heartbeat_sent >= 1.0:
                self.connection.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )

                last_heartbeat_sent = now

            if control_source != last_control_source:
                if control_source in {
                    "OFFBOARD_ATTITUDE_FOLLOW",
                    "OFFBOARD_ATTITUDE_RELEASE",
                }:
                    LOGGER.info(
                        "%s control output: %s "
                        "p=%.1f q=%.1f r=%.1fdeg/s thrust=%.3f",
                        self.drone_id,
                        control_source,
                        attitude_roll_rate,
                        attitude_pitch_rate,
                        attitude_yaw_rate,
                        attitude_thrust,
                    )
                elif control_source == "VISUAL_FOLLOW_TARGET":
                    LOGGER.info(
                        "%s control output: %s lat=%.7f lon=%.7f "
                        "alt=%.1f vn=%.2f ve=%.2f q=%.2f",
                        self.drone_id,
                        control_source,
                        visual_lat,
                        visual_lon,
                        visual_alt,
                        visual_vn,
                        visual_ve,
                        visual_quality,
                    )
                elif control_source == "OFFBOARD_FOLLOW":
                    LOGGER.info(
                        "%s control output: %s "
                        "vx=%.2f vy=%.2f vz=%.2f yaw_rate=%.1fdeg/s "
                        "local_hold=%s accel_hold=%s z=%.2f ax=%.2f ay=%.2f",
                        self.drone_id,
                        control_source,
                        offboard_forward,
                        offboard_right,
                        offboard_down,
                        offboard_yaw_rate,
                        offboard_local_altitude_hold,
                        offboard_local_acceleration_hold,
                        offboard_hold_z,
                        offboard_north_acceleration,
                        offboard_east_acceleration,
                    )
                else:
                    LOGGER.info(
                        "%s control output: %s "
                        "x=%d y=%d z=%d r=%d",
                        self.drone_id,
                        control_source,
                        x,
                        y,
                        z,
                        r,
                    )

                last_control_source = control_source

            elapsed = (
                time.monotonic()
                - loop_started
            )

            sleep_time = (
                SEND_PERIOD_S - elapsed
            )

            if sleep_time > 0:
                self.stop_event.wait(
                    sleep_time
                )

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                self.connection = self.connect()
                self.send_centered_controls()
                self.run_connected()

            except Exception as error:
                LOGGER.error(
                    "%s MAVLink error: %s",
                    self.drone_id,
                    error,
                )

                self.stop_event.wait(2.0)

            finally:
                try:
                    self.request_position_mode()
                except Exception:
                    pass
                self.send_centered_controls()

                if self.connection is not None:
                    try:
                        self.connection.close()
                    except Exception:
                        pass

                self.connection = None
                if self.qgc_socket is not None:
                    try:
                        self.qgc_socket.close()
                    except Exception:
                        pass
                    self.qgc_socket = None


states = {
    drone_id: ManualState()
    for drone_id in VEHICLES
}

tracking_yaw_states = {
    drone_id: TrackingYawState()
    for drone_id in VEHICLES
}

tracking_follow_states = {
    drone_id: TrackingFollowState()
    for drone_id in VEHICLES
}

visual_follow_target_states = {
    drone_id: VisualFollowTargetState()
    for drone_id in VEHICLES
}

offboard_follow_states = {
    drone_id: OffboardFollowState()
    for drone_id in VEHICLES
}

offboard_attitude_states = {
    drone_id: OffboardAttitudeState()
    for drone_id in VEHICLES
}

stop_event = threading.Event()


def on_mqtt_connect(
    client: mqtt.Client,
    userdata: Any,
    flags: Any,
    reason_code: Any,
    properties: Any = None,
) -> None:
    LOGGER.info(
        "MQTT connected: %s",
        reason_code,
    )

    client.subscribe(
        MQTT_TOPIC,
        qos=0,
    )


def on_mqtt_disconnect(
    client: mqtt.Client,
    userdata: Any,
    disconnect_flags: Any = None,
    reason_code: Any = None,
    properties: Any = None,
) -> None:
    LOGGER.warning(
        "MQTT disconnected: %s",
        reason_code,
    )

    for state in states.values():
        state.disable()
    for state in tracking_yaw_states.values():
        state.disable()
    for state in tracking_follow_states.values():
        state.disable()
    for state in visual_follow_target_states.values():
        state.disable()
    for state in offboard_follow_states.values():
        state.disable()
    for state in offboard_attitude_states.values():
        state.disable()


def on_mqtt_message(
    client: mqtt.Client,
    userdata: Any,
    message: mqtt.MQTTMessage,
) -> None:
    try:
        payload = json.loads(
            message.payload.decode("utf-8")
        )
    except (
        UnicodeDecodeError,
        json.JSONDecodeError,
    ) as error:
        LOGGER.warning(
            "Invalid MQTT JSON: %s",
            error,
        )
        return

    if not isinstance(payload, dict):
        return

    message_type = payload.get("type")
    if message_type not in {
        "manual_control",
        "tracking_yaw",
        "tracking_follow",
        "visual_follow_target",
        "offboard_follow",
        "offboard_attitude_follow",
    }:
        return
    if message_type == "offboard_attitude_follow" and not ALLOW_RAW_ATTITUDE_OFFBOARD:
        LOGGER.warning("Rejected raw attitude/thrust Offboard command")
        return

    drone_id = str(
        payload.get("drone_id", "")
    )

    if message_type == "manual_control":
        state = states.get(drone_id)
    elif message_type == "tracking_yaw":
        state = tracking_yaw_states.get(drone_id)
    elif message_type == "tracking_follow":
        state = tracking_follow_states.get(drone_id)
    elif message_type == "visual_follow_target":
        state = visual_follow_target_states.get(drone_id)
    elif message_type == "offboard_follow":
        state = offboard_follow_states.get(drone_id)
    else:
        state = offboard_attitude_states.get(drone_id)

    if state is None:
        LOGGER.warning(
            "Unknown drone ID: %s",
            drone_id,
        )
        return

    # A human input always wins and must latch automation off. Otherwise the
    # previous attitude command could resume as soon as MANUAL_CONTROL expires.
    if message_type == "manual_control" and bool(payload.get("enabled", False)):
        tracking_yaw_states[drone_id].disable()
        tracking_follow_states[drone_id].disable()
        offboard_follow_states[drone_id].disable()
        offboard_attitude_states[drone_id].disable()
        visual_follow_target_states[drone_id].disable()
    elif message_type == "visual_follow_target":
        # Native PX4 Follow Target is mutually exclusive with all legacy
        # manual-stick and Offboard tracking controllers.
        tracking_yaw_states[drone_id].disable()
        tracking_follow_states[drone_id].disable()
        offboard_follow_states[drone_id].disable()
        offboard_attitude_states[drone_id].disable()
    elif message_type == "offboard_attitude_follow":
        # Prevent a stale velocity setpoint from becoming active during the
        # attitude-to-Position handover.
        offboard_follow_states[drone_id].disable()
        visual_follow_target_states[drone_id].disable()
    elif message_type == "offboard_follow":
        offboard_attitude_states[drone_id].disable()
        visual_follow_target_states[drone_id].disable()
    elif message_type in {"tracking_follow", "tracking_yaw"}:
        visual_follow_target_states[drone_id].disable()

    state.update(payload)


def create_mqtt_client() -> mqtt.Client:
    try:
        client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id="mavlink-manual-bridge",
        )
    except AttributeError:
        client = mqtt.Client(
            client_id="mavlink-manual-bridge",
        )

    client.on_connect = on_mqtt_connect
    client.on_disconnect = on_mqtt_disconnect
    client.on_message = on_mqtt_message

    client.reconnect_delay_set(
        min_delay=1,
        max_delay=10,
    )

    return client


def request_shutdown(
    signum: int,
    frame: Any,
) -> None:
    LOGGER.info(
        "Shutdown requested"
    )

    stop_event.set()


def main() -> None:
    signal.signal(
        signal.SIGINT,
        request_shutdown,
    )

    signal.signal(
        signal.SIGTERM,
        request_shutdown,
    )

    mqtt_client = create_mqtt_client()

    mqtt_client.connect(
        MQTT_HOST,
        MQTT_PORT,
        keepalive=30,
    )

    mqtt_client.loop_start()

    workers = []

    for drone_id, config in VEHICLES.items():
        worker = MavlinkWorker(
            drone_id=drone_id,
            port=int(config["port"]),
            expected_system_id=int(
                config["system_id"]
            ),
            state=states[drone_id],
            tracking_yaw_state=tracking_yaw_states[drone_id],
            tracking_follow_state=tracking_follow_states[drone_id],
            visual_follow_target_state=visual_follow_target_states[drone_id],
            offboard_follow_state=offboard_follow_states[drone_id],
            offboard_attitude_state=offboard_attitude_states[drone_id],
            stop_event=stop_event,
        )

        workers.append(worker)
        worker.start()

    LOGGER.info(
        "Manual bridge started at %.1f Hz",
        SEND_RATE_HZ,
    )

    LOGGER.info(
        "MQTT command topic: %s",
        MQTT_TOPIC,
    )

    try:
        while not stop_event.wait(1.0):
            pass

    finally:
        stop_event.set()

        for state in states.values():
            state.disable()
        for state in tracking_yaw_states.values():
            state.disable()
        for state in tracking_follow_states.values():
            state.disable()
        for state in visual_follow_target_states.values():
            state.disable()
        for state in offboard_follow_states.values():
            state.disable()
        for state in offboard_attitude_states.values():
            state.disable()

        for worker in workers:
            worker.join(timeout=3.0)

        mqtt_client.loop_stop()
        mqtt_client.disconnect()

        LOGGER.info(
            "Manual bridge stopped"
        )


if __name__ == "__main__":
    main()
