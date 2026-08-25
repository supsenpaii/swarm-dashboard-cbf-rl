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
from collections import deque
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
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from body_attitude_recenter import BodyAttitudeRecenterController
from camera_source_trace import record as _trace_camera_hop
from cbf_command_gate import (
    CbfCommand,
    CbfCommandGate,
    CbfConfig,
    cbf_covariance_sigma,
    cbf_position_covariance_required,
)
from formation_controller import (
    DeterministicFormationController,
    FormationConfig,
    FormationSlot,
    TargetRelativeFormationController,
)
from gazebo_subscription_phase_trace import get_recorder
from pose_time_sync import PoseSample, TimestampedPoseBuffer
from range_ground_truth import RangeGroundTruthRouter
from simulation_ground_truth import (
    GT_CONTRACT_SHA256,
    GT_CONTRACT_VERSION,
    TARGET_REFERENCE,
    interpolate_pose,
    optical_center_distance,
)
from mission_plan import (
    MissionLimits,
    MissionRejected,
    review_missions,
    validate_mission,
)
from trajectory_controller import mission_speed_preview
from swarm_state import GeodeticOrigin, SwarmStateStore
from tracking_web import TrackingManager

try:
    import cv2
    import numpy as np
    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.image_pb2 import Image as GzImage
    from gz.msgs10.imu_pb2 import IMU as GzImu
    from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseVector
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
    GzPoseVector = None
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
TRACKING_POSE_TOPIC = "swarm/+/tracking/pose"
CONTROL_TOPIC_TEMPLATE = "swarm/{drone_id}/control/command"
try:
    TRACKING_STATUS_MAX_AGE_MS = max(
        500,
        min(
            5000,
            int(os.environ.get("SWARM_TRACKING_STATUS_MAX_AGE_MS", "750")),
        ),
    )
except ValueError:
    TRACKING_STATUS_MAX_AGE_MS = 750

ALLOWED_DRONES = {
    "UAV-01",
    "UAV-02",
}
# SITL/test-only fault-injection hook. Inert unless
# SWARM_TEST_TELEMETRY_BLOCK_FILE is set to a path. While set, a test harness
# can write a drone_id (e.g. "UAV-01") into that file to withhold that
# drone's telemetry from SwarmState ingestion without restarting the stack:
# the drone's raw telemetry keeps arriving over MQTT (still visible in
# "drones"), but message_age_ms ages naturally toward
# SWARM_STATE_MAX_MESSAGE_AGE_MS instead of being reset on every message.
# Clearing the file (or writing anything else) resumes ingestion immediately.
SWARM_TEST_TELEMETRY_BLOCK_FILE = os.environ.get(
    "SWARM_TEST_TELEMETRY_BLOCK_FILE", ""
).strip()


def _test_telemetry_ingest_blocked(drone_id: str) -> bool:
    if not SWARM_TEST_TELEMETRY_BLOCK_FILE:
        return False
    try:
        with open(SWARM_TEST_TELEMETRY_BLOCK_FILE, "r", encoding="utf-8") as f:
            return f.read().strip() == drone_id
    except OSError:
        return False
FORMATION_SHADOW_ENABLED = os.environ.get(
    "SWARM_FORMATION_SHADOW_ENABLED", "false"
).strip().lower() in {"1", "true", "yes", "on"}
CBF_SHADOW_ENABLED = os.environ.get(
    "SWARM_CBF_SHADOW_ENABLED", "false"
).strip().lower() in {"1", "true", "yes", "on"}
FORMATION_LEADER_ID = os.environ.get(
    "SWARM_FORMATION_LEADER_ID", "UAV-01"
).strip()
try:
    FORMATION_POSITION_GAIN_S_INV = max(
        0.01,
        min(5.0, float(os.environ.get("SWARM_FORMATION_POSITION_GAIN_S_INV", "0.6"))),
    )
    FORMATION_MAXIMUM_VELOCITY_M_S = max(
        0.1,
        min(20.0, float(os.environ.get("SWARM_FORMATION_MAXIMUM_VELOCITY_M_S", "2.0"))),
    )
    FORMATION_ARRIVAL_RADIUS_M = max(
        0.05,
        min(5.0, float(os.environ.get("SWARM_FORMATION_ARRIVAL_RADIUS_M", "0.25"))),
    )
except ValueError:
    FORMATION_POSITION_GAIN_S_INV = 0.6
    FORMATION_MAXIMUM_VELOCITY_M_S = 2.0
    FORMATION_ARRIVAL_RADIUS_M = 0.25
try:
    CBF_MINIMUM_SEPARATION_M = max(
        0.5,
        min(50.0, float(os.environ.get("SWARM_CBF_MINIMUM_SEPARATION_M", "4.0"))),
    )
    CBF_MAXIMUM_VELOCITY_M_S = max(
        0.1,
        min(20.0, float(os.environ.get("SWARM_CBF_MAXIMUM_VELOCITY_M_S", "2.0"))),
    )
    CBF_BARRIER_GAIN_S_INV = max(
        0.1,
        min(10.0, float(os.environ.get("SWARM_CBF_BARRIER_GAIN_S_INV", "2.0"))),
    )
    CBF_COMMAND_LATENCY_S = max(
        0.0,
        min(2.0, float(os.environ.get("SWARM_CBF_COMMAND_LATENCY_S", "0.10"))),
    )
    CBF_DESIGN_MARGIN_BUFFER_M = max(
        0.0,
        min(5.0, float(os.environ.get("SWARM_CBF_DESIGN_MARGIN_BUFFER_M", "0.0"))),
    )
    CBF_RELATIVE_BRAKING_ACCELERATION_M_S2 = max(
        0.0,
        min(
            30.0,
            float(
                os.environ.get(
                    "SWARM_CBF_RELATIVE_BRAKING_ACCELERATION_M_S2", "0.0"
                )
            ),
        ),
    )
    CBF_TRACKING_RESERVE_M = max(
        0.0,
        min(20.0, float(os.environ.get("SWARM_CBF_TRACKING_RESERVE_M", "0.0"))),
    )
except ValueError:
    CBF_MINIMUM_SEPARATION_M = 4.0
    CBF_MAXIMUM_VELOCITY_M_S = 2.0
    CBF_BARRIER_GAIN_S_INV = 2.0
    CBF_COMMAND_LATENCY_S = 0.10
    CBF_DESIGN_MARGIN_BUFFER_M = 0.0
    CBF_RELATIVE_BRAKING_ACCELERATION_M_S2 = 0.0
    CBF_TRACKING_RESERVE_M = 0.0
try:
    SWARM_STATE_MAX_MESSAGE_AGE_S = max(
        0.05,
        min(
            5.0,
            float(os.environ.get("SWARM_STATE_MAX_MESSAGE_AGE_MS", "500"))
            / 1000.0,
        ),
    )
except ValueError:
    SWARM_STATE_MAX_MESSAGE_AGE_S = 0.5

try:
    SWARM_ENU_ORIGIN = GeodeticOrigin(
        float(os.environ["SWARM_ENU_ORIGIN_LAT_DEG"]),
        float(os.environ["SWARM_ENU_ORIGIN_LON_DEG"]),
        float(os.environ["SWARM_ENU_ORIGIN_ALT_MSL_M"]),
    )
except (KeyError, ValueError):
    # An explicit reference is mandatory: deriving it from one PX4's local
    # origin would make inter-vehicle separation frame-dependent.
    SWARM_ENU_ORIGIN = None
try:
    TRACKING_CAMERA_OFFSET_BODY_FRD_M = (
        float(os.environ.get("SWARM_CAMERA_OFFSET_BODY_FORWARD_M", "0.0")),
        float(os.environ.get("SWARM_CAMERA_OFFSET_BODY_RIGHT_M", "0.0")),
        float(os.environ.get("SWARM_CAMERA_OFFSET_BODY_DOWN_M", "0.0")),
    )
    if not all(
        math.isfinite(value) for value in TRACKING_CAMERA_OFFSET_BODY_FRD_M
    ):
        raise ValueError("camera offset is not finite")
except ValueError:
    TRACKING_CAMERA_OFFSET_BODY_FRD_M = (0.0, 0.0, 0.0)

DRONE_MODELS = {
    "UAV-01": os.environ.get(
        "SWARM_GAZEBO_MODEL_UAV_01", "sparrow_gimbal_0"
    ).strip(),
    "UAV-02": os.environ.get(
        "SWARM_GAZEBO_MODEL_UAV_02", "sparrow_gimbal_1"
    ).strip(),
}


def _expected_gazebo_rate(name: str, default: float) -> float:
    try:
        return max(0.1, float(os.environ.get(name, str(default))))
    except ValueError:
        return default


GAZEBO_CAMERA_EXPECTED_RATE_HZ = _expected_gazebo_rate(
    "SWARM_GAZEBO_CAMERA_EXPECTED_RATE_HZ", 30.0
)
GAZEBO_CAMERA_IMU_EXPECTED_RATE_HZ = _expected_gazebo_rate(
    "SWARM_GAZEBO_CAMERA_IMU_EXPECTED_RATE_HZ", 250.0
)

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
                    "10",
                )
            ),
        ),
    )
except ValueError:
    TRACKING_GIMBAL_YAW_LIMIT_DEG = 10.0

try:
    TRACKING_GIMBAL_PITCH_MIN_DEG = max(
        GIMBAL_LIMITS_DEG["pitch"][0],
        min(
            0.0,
            float(
                os.environ.get(
                    "SWARM_TRACKING_GIMBAL_PITCH_MIN_DEG",
                    "-45",
                )
            ),
        ),
    )
    TRACKING_GIMBAL_PITCH_MAX_DEG = min(
        GIMBAL_LIMITS_DEG["pitch"][1],
        max(
            0.0,
            float(
                os.environ.get(
                    "SWARM_TRACKING_GIMBAL_PITCH_MAX_DEG",
                    "45",
                )
            ),
        ),
    )
except ValueError:
    TRACKING_GIMBAL_PITCH_MIN_DEG = -45.0
    TRACKING_GIMBAL_PITCH_MAX_DEG = 45.0


def clamp_tracking_gimbal_yaw_deg(
    requested_yaw_deg: float,
    limit_deg: float = TRACKING_GIMBAL_YAW_LIMIT_DEG,
) -> tuple[float, bool]:
    """Clamp tracking yaw and report an outward saturation request."""
    requested = float(requested_yaw_deg)
    limit = max(0.1, abs(float(limit_deg)))
    if not math.isfinite(requested):
        return 0.0, True
    return (
        max(-limit, min(limit, requested)),
        bool(requested < -limit or requested > limit),
    )


def clamp_tracking_gimbal_pitch_deg(
    requested_pitch_deg: float,
    minimum_deg: float = TRACKING_GIMBAL_PITCH_MIN_DEG,
    maximum_deg: float = TRACKING_GIMBAL_PITCH_MAX_DEG,
) -> tuple[float, bool]:
    """Clamp pointing pitch away from model singularities."""

    requested = float(requested_pitch_deg)
    minimum = float(minimum_deg)
    maximum = float(maximum_deg)
    if not math.isfinite(requested) or minimum > maximum:
        return 0.0, True
    return (
        max(minimum, min(maximum, requested)),
        bool(requested < minimum or requested > maximum),
    )


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
    TRACKING_GIMBAL_MAX_PITCH_RATE_DEG_S = max(
        1.0,
        min(
            180.0,
            float(
                os.environ.get(
                    "SWARM_TRACKING_GIMBAL_MAX_PITCH_RATE_DEG_S",
                    "45",
                )
            ),
        ),
    )
except ValueError:
    TRACKING_GIMBAL_MAX_PITCH_RATE_DEG_S = 45.0
try:
    TRACKING_GIMBAL_MAX_YAW_RATE_DEG_S = max(
        1.0,
        min(
            180.0,
            float(
                os.environ.get(
                    "SWARM_TRACKING_GIMBAL_MAX_YAW_RATE_DEG_S",
                    "60",
                )
            ),
        ),
    )
except ValueError:
    TRACKING_GIMBAL_MAX_YAW_RATE_DEG_S = 60.0
try:
    TRACKING_GIMBAL_MAX_ACCEL_DEG_S2 = max(
        1.0,
        min(
            2000.0,
            float(
                os.environ.get(
                    "SWARM_TRACKING_GIMBAL_MAX_ACCEL_DEG_S2",
                    "360",
                )
            ),
        ),
    )
except ValueError:
    TRACKING_GIMBAL_MAX_ACCEL_DEG_S2 = 360.0
try:
    TRACKING_GIMBAL_MAX_BRAKE_ACCEL_DEG_S2 = max(
        TRACKING_GIMBAL_MAX_ACCEL_DEG_S2,
        min(
            2000.0,
            float(
                os.environ.get(
                    "SWARM_TRACKING_GIMBAL_MAX_BRAKE_ACCEL_DEG_S2",
                    "720",
                )
            ),
        ),
    )
except ValueError:
    TRACKING_GIMBAL_MAX_BRAKE_ACCEL_DEG_S2 = 720.0
try:
    TRACKING_GIMBAL_COMMAND_STALE_RESET_S = max(
        0.1,
        min(
            2.0,
            float(
                os.environ.get(
                    "SWARM_TRACKING_GIMBAL_COMMAND_STALE_RESET_S",
                    "0.25",
                )
            ),
        ),
    )
except ValueError:
    TRACKING_GIMBAL_COMMAND_STALE_RESET_S = 0.25
try:
    TRACKING_GIMBAL_MAX_COMMAND_LEAD_DEG = max(
        0.5,
        min(
            15.0,
            float(
                os.environ.get(
                    "SWARM_TRACKING_GIMBAL_MAX_COMMAND_LEAD_DEG",
                    "5.0",
                )
            ),
        ),
    )
except ValueError:
    TRACKING_GIMBAL_MAX_COMMAND_LEAD_DEG = 5.0


def limit_tracking_gimbal_rate_deg_s(
    requested_rate_deg_s: float,
    previous_rate_deg_s: float,
    maximum_rate_deg_s: float,
    maximum_acceleration_deg_s2: float,
    dt_s: float,
    maximum_braking_acceleration_deg_s2: float | None = None,
) -> float:
    """Apply rate/slew limits and stop at zero before reversing."""
    requested = float(requested_rate_deg_s)
    previous = float(previous_rate_deg_s)
    if not math.isfinite(requested):
        requested = 0.0
    if not math.isfinite(previous):
        previous = 0.0
    maximum_rate = max(0.0, abs(float(maximum_rate_deg_s)))
    acceleration = max(
        0.0,
        abs(float(maximum_acceleration_deg_s2)),
    )
    braking_acceleration = (
        acceleration
        if maximum_braking_acceleration_deg_s2 is None
        else max(
            acceleration,
            abs(float(maximum_braking_acceleration_deg_s2)),
        )
    )
    target = max(-maximum_rate, min(maximum_rate, requested))
    braking = bool(
        previous != 0.0
        and (
            target == 0.0
            or target * previous < 0.0
            or abs(target) < abs(previous)
        )
    )
    maximum_delta = (
        braking_acceleration if braking else acceleration
    ) * max(0.001, float(dt_s))
    if target * previous < 0.0:
        # Do not cross zero during a reversal. This prevents one noisy bbox
        # sample from immediately kicking the gimbal in the opposite direction.
        delta = max(
            -maximum_delta,
            min(maximum_delta, -previous),
        )
        return max(-maximum_rate, min(maximum_rate, previous + delta))
    delta = max(
        -maximum_delta,
        min(maximum_delta, target - previous),
    )
    return max(-maximum_rate, min(maximum_rate, previous + delta))


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
    "false",
).strip().lower() in {"1", "true", "yes", "on"}
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

try:
    CAMERA_MAX_WIDTH = max(
        160,
        min(
            1920,
            int(os.environ.get("SWARM_CAMERA_PREVIEW_MAX_WIDTH", "640")),
        ),
    )
except ValueError:
    CAMERA_MAX_WIDTH = 640
CAMERA_JPEG_QUALITY = 72
CAMERA_MAX_FPS = 10.0

ALLOWED_ACTIONS = {
    "position",
    "takeoff",
    "arm",
    "disarm",
    "land",
    "hold",
    "rtl",
    "hold_current",
    # Giữ tương thích với giao diện cũ.
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
swarm_state_store = SwarmStateStore(
    SWARM_ENU_ORIGIN,
    max_message_age_s=SWARM_STATE_MAX_MESSAGE_AGE_S,
)
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
tracking_gimbal_rates_deg_s: dict[str, dict[str, float]] = {
    drone_id: {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
    }
    for drone_id in ALLOWED_DRONES
}
tracking_gimbal_last_command_monotonic: dict[str, float] = {
    drone_id: 0.0
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
tracking_vehicle_pose_buffers = {
    drone_id: TimestampedPoseBuffer(
        maxlen=300,
        max_interpolation_gap_s=0.25,
        max_nearest_age_s=0.15,
    )
    for drone_id in ALLOWED_DRONES
}
tracking_fast_pose_received_monotonic = {
    drone_id: 0.0
    for drone_id in ALLOWED_DRONES
}
tracking_visual_follow_bridge_status: dict[str, dict[str, Any]] = {
    drone_id: {}
    for drone_id in ALLOWED_DRONES
}
tracking_peer_state_status: dict[str, dict[str, Any]] = {
    drone_id: {}
    for drone_id in ALLOWED_DRONES
}
# The companion runs its own nominal + CBF from the peer-to-peer link. This is
# a report of a decision already taken on the drone, not an input to anything
# here: the server's own shadow gate below is computed independently.
tracking_companion_safety: dict[str, dict[str, Any]] = {
    drone_id: {}
    for drone_id in ALLOWED_DRONES
}
# How often the barrier had to correct the command it was handed. A flight
# can hold a positive margin and still be one configuration change away from
# not holding it, and the intervention rate is what says so first: the
# Sparrow 10 m/s rung passed on 2026-08-17 with 13 mm of margin while the
# barrier was acting on 3.5% of frames, and widening the coordinator's lane
# change took that to 5.4 m and 0.4%. Margin alone showed a pass either way.
# 200 frames is about 10 s at the 20 Hz peer-state rate.
CBF_INTERVENTION_WINDOW = 200
tracking_cbf_intervention: dict[str, deque[bool]] = {
    drone_id: deque(maxlen=CBF_INTERVENTION_WINDOW)
    for drone_id in ALLOWED_DRONES
}


def cbf_status(drone_id: str) -> dict[str, Any]:
    """Live barrier health for one vehicle, for the dashboard and drivers."""
    window = tracking_cbf_intervention[drone_id]
    safety = tracking_companion_safety[drone_id] or {}
    return {
        "minimum_margin_m": (safety.get("cbf") or {}).get("minimum_margin_m"),
        "intervention_rate": (
            round(sum(window) / len(window), 3) if window else None
        ),
        "intervention_samples": len(window),
    }


def with_cbf_status(drones: dict[str, Any]) -> dict[str, Any]:
    for drone_id in ALLOWED_DRONES:
        if isinstance(drones.get(drone_id), dict):
            drones[drone_id]["cbf_status"] = cbf_status(drone_id)
    return drones
# Last mission this server accepted per drone, kept only to answer "does the
# path being sent now come near one already out there?" at the moment the
# operator presses send. Advisory, like the rest of this validator: the bridge
# owns the authoritative copy and may have cleared or replaced a mission
# without telling us, so this warns and never refuses.
accepted_mission_paths: dict[str, Any] = {}
try:
    if FORMATION_LEADER_ID not in ALLOWED_DRONES:
        raise ValueError("leader is not an allowed drone")
    formation_slots = []
    for formation_drone_id in sorted(ALLOWED_DRONES - {FORMATION_LEADER_ID}):
        raw_slot = os.environ.get(
            "SWARM_FORMATION_SLOT_"
            f"{formation_drone_id.replace('-', '_')}_ENU_M",
            "-10,0,0",
        )
        components = tuple(float(value.strip()) for value in raw_slot.split(","))
        if len(components) != 3:
            raise ValueError("formation slot requires three ENU components")
        formation_slots.append(FormationSlot(formation_drone_id, components))
    formation_shadow_controller = DeterministicFormationController(
        FORMATION_LEADER_ID,
        tuple(formation_slots),
        FormationConfig(
            position_gain_s_inv=FORMATION_POSITION_GAIN_S_INV,
            maximum_velocity_m_s=FORMATION_MAXIMUM_VELOCITY_M_S,
            arrival_radius_m=FORMATION_ARRIVAL_RADIUS_M,
        ),
    )
    formation_shadow_config_error = ""
except (TypeError, ValueError):
    formation_shadow_controller = None
    formation_shadow_config_error = "formation configuration is invalid"

# Target-relative dynamic formation (architecture doc §17's "formation slots
# around the target"), shadow-only and independent of the leader-relative
# formation above. Target fusion is a server responsibility (§3.1), unlike
# CBF, which had to move to the companion -- so this stays here.
#
# SWARM_TARGET_FORMATION_STAND_IN_DRONE_ID names an ALLOWED_DRONES member
# whose own real, GPS-derived common-ENU state is used as the "target" input.
# This is a validation stand-in, not vision integration: it exists so the
# dynamic-slot math can be exercised against real, live ENU numbers without
# activating the camera/tracker pipeline. Swapping in a genuine vision
# estimate later means feeding swarm_state.target_wgs84_to_state()'s output
# here instead -- the controller itself does not know the difference, since
# both arrive in the same {valid, position_enu_m, velocity_enu_m_s} shape.
TARGET_FORMATION_SHADOW_ENABLED = os.environ.get(
    "SWARM_TARGET_FORMATION_SHADOW_ENABLED", "false"
).strip().lower() in {"1", "true", "yes", "on"}
TARGET_FORMATION_STAND_IN_DRONE_ID = os.environ.get(
    "SWARM_TARGET_FORMATION_STAND_IN_DRONE_ID", ""
).strip()
try:
    if not TARGET_FORMATION_STAND_IN_DRONE_ID:
        raise ValueError("no stand-in target drone configured")
    if TARGET_FORMATION_STAND_IN_DRONE_ID not in ALLOWED_DRONES:
        raise ValueError("stand-in target drone is not an allowed drone")
    target_formation_slots = []
    for formation_drone_id in sorted(
        ALLOWED_DRONES - {TARGET_FORMATION_STAND_IN_DRONE_ID}
    ):
        raw_slot = os.environ.get(
            "SWARM_TARGET_FORMATION_SLOT_"
            f"{formation_drone_id.replace('-', '_')}_ENU_M",
            "-5,0,2",
        )
        components = tuple(float(value.strip()) for value in raw_slot.split(","))
        if len(components) != 3:
            raise ValueError("target formation slot requires three ENU components")
        target_formation_slots.append(FormationSlot(formation_drone_id, components))
    target_formation_controller = TargetRelativeFormationController(
        tuple(target_formation_slots),
        FormationConfig(
            position_gain_s_inv=FORMATION_POSITION_GAIN_S_INV,
            maximum_velocity_m_s=FORMATION_MAXIMUM_VELOCITY_M_S,
            arrival_radius_m=FORMATION_ARRIVAL_RADIUS_M,
        ),
    )
    target_formation_config_error = ""
except (TypeError, ValueError):
    target_formation_controller = None
    target_formation_config_error = "target formation configuration is invalid"

try:
    def _cbf_enu_vector(name: str, default: str) -> tuple[float, float, float]:
        values = tuple(
            float(value.strip())
            for value in os.environ.get(name, default).split(",")
        )
        if len(values) != 3:
            raise ValueError(f"{name} requires three ENU components")
        return values

    cbf_config = CbfConfig(
        minimum_separation_m=CBF_MINIMUM_SEPARATION_M,
        barrier_gain_s_inv=CBF_BARRIER_GAIN_S_INV,
        maximum_velocity_m_s=CBF_MAXIMUM_VELOCITY_M_S,
        command_latency_s=CBF_COMMAND_LATENCY_S,
        relative_braking_acceleration_m_s2=CBF_RELATIVE_BRAKING_ACCELERATION_M_S2,
        tracking_reserve_m=CBF_TRACKING_RESERVE_M,
        design_margin_buffer_m=CBF_DESIGN_MARGIN_BUFFER_M,
        covariance_sigma=cbf_covariance_sigma(),
        require_position_covariance=cbf_position_covariance_required(),
        geofence_min_enu_m=_cbf_enu_vector(
            "SWARM_CBF_GEOFENCE_MIN_ENU_M", "-100,-100,0"
        ),
        geofence_max_enu_m=_cbf_enu_vector(
            "SWARM_CBF_GEOFENCE_MAX_ENU_M", "100,100,50"
        ),
    )
    cbf_shadow_gates = {
        drone_id: CbfCommandGate(
            drone_id,
            tuple(sorted(ALLOWED_DRONES - {drone_id})),
            cbf_config,
        )
        for drone_id in ALLOWED_DRONES
    }
    cbf_shadow_config_error = ""
except (TypeError, ValueError):
    cbf_shadow_gates = {}
    cbf_shadow_config_error = "CBF configuration is invalid"
TRACKING_VISUAL_FOLLOW_MANUAL_LOSS_GRACE_S = 0.75
tracking_visual_follow_manual_loss_started_monotonic: dict[
    str, float | None
] = {
    drone_id: None
    for drone_id in ALLOWED_DRONES
}


# ============================================================
# Helpers
# ============================================================

def now_ms() -> int:
    return int(time.time() * 1000)


def _euler_to_quaternion_xyzw(
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> tuple[float, float, float, float]:
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def record_tracking_vehicle_pose(
    drone_id: str,
    payload: dict[str, Any],
    timestamp_s: float,
) -> None:
    """Record PX4 pose in the camera frame's monotonic clock domain."""

    try:
        local = payload["local_position"]
        attitude = payload.get("attitude", {})
        position = (
            float(local["x_north_m"]),
            float(local["y_east_m"]),
            float(local["z_down_m"]),
        )
        velocity = (
            float(local.get("vx_m_s", 0.0)),
            float(local.get("vy_m_s", 0.0)),
            float(local.get("vz_m_s", 0.0)),
        )
        direct_quaternion = payload.get("quaternion_xyzw")
        if (
            isinstance(direct_quaternion, (list, tuple))
            and len(direct_quaternion) == 4
        ):
            quaternion_xyzw = tuple(
                float(value) for value in direct_quaternion
            )
        else:
            roll_rad = math.radians(float(attitude.get("roll_deg", 0.0)))
            pitch_rad = math.radians(float(attitude.get("pitch_deg", 0.0)))
            yaw_rad = float(local.get("heading_rad", 0.0))
            quaternion_xyzw = _euler_to_quaternion_xyzw(
                roll_rad,
                pitch_rad,
                yaw_rad,
            )
        tracking_vehicle_pose_buffers[drone_id].append(
            PoseSample(
                timestamp_s=timestamp_s,
                position_ned_m=position,
                velocity_ned_m_s=velocity,
                quaternion_xyzw=quaternion_xyzw,
            )
        )
    except (KeyError, TypeError, ValueError):
        # The pose provider exposes the missing/stale reason to tracking. A
        # malformed telemetry packet must not poison the synchronization buffer.
        return


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


# Ground-truth pose-bracket race fix (CORE_RANGE_GT_BRACKET_RACE_FIX).
#
# simulation_target_ground_truth() looks up the two pose_history snapshots
# bracketing a camera frame's sim-time. pose_history is appended to from a
# separate Gazebo pose-subscription callback, so a camera frame can
# occasionally be processed a physics step or two before the pose sample
# that would complete its bracket has arrived -- observed at ~0.5-0.6% of
# frames under the sim_time_plugin trajectory driver (see
# docs/CORE_RANGE_SIM_TIME_DYNAMIC_RETRAIN_REPORT.md), not correlated with
# RTF. Only that specific case (upper_source missing: the camera timestamp
# is newer than every buffered pose sample so far) is worth waiting for --
# a newer sample will arrive shortly and resolve it. A camera timestamp
# OLDER than the oldest buffered sample (lower_source missing) can never
# resolve by waiting, since time only moves forward, so that case still
# fails immediately, unchanged from before this fix.
GT_BRACKET_WAIT_TIMEOUT_S = 0.12
GT_BRACKET_MAX_PENDING_WAITERS = 4


class GazeboDashboardBridge:
    """Long-lived Gazebo publishers plus per-UAV camera subscriptions."""

    def __init__(self) -> None:
        self.node: Any = None
        self.publishers: dict[tuple[str, str], Any] = {}
        self.camera_topics: dict[str, str] = {}
        self.camera_callbacks: dict[str, Any] = {}
        self.camera_subscribed: set[str] = set()
        self.subscription_lock = threading.RLock()
        self.imu_topics: set[str] = set()
        self.imu_lock = threading.Lock()
        self.body_quaternions: dict[str, tuple[float, float, float, float]] = {}
        self.camera_quaternions: dict[str, tuple[float, float, float, float]] = {}
        self.camera_pose_buffers = {
            drone_id: TimestampedPoseBuffer(
                maxlen=600,
                max_interpolation_gap_s=0.20,
                max_nearest_age_s=0.12,
            )
            for drone_id in ALLOWED_DRONES
        }
        self.gimbal_feedback_deg: dict[str, dict[str, float]] = {}
        self.gimbal_feedback_monotonic: dict[str, float] = {}
        self.lidar_topic = os.environ.get(
            "SWARM_GAZEBO_LIDAR_TOPIC", "/sparrow_gimbal/front_lidar"
        ).strip()
        self.lidar_lock = threading.Lock()
        self.lidar_scans: dict[str, dict[str, Any]] = {}
        self.lidar_subscribed = False
        self.pose_topic = "/world/default/pose/info"
        self.pose_subscribed = False
        self.pose_lock = threading.Lock()
        self.pose_history: deque[dict[str, Any]] = deque(maxlen=600)
        self.pose_history_condition = threading.Condition(self.pose_lock)
        self._gt_bracket_generation = 0
        self._gt_bracket_waiting_count = 0
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
        self.last_raw_frame_sim_timestamp_s: dict[str, float | None] = {}
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
        self.conversion_samples_ms = {
            drone_id: deque(maxlen=240)
            for drone_id in ALLOWED_DRONES
        }
        self.camera_encode_samples_ms = {
            drone_id: deque(maxlen=240)
            for drone_id in ALLOWED_DRONES
        }
        self.camera_encode_times_s = {
            drone_id: deque(maxlen=240)
            for drone_id in ALLOWED_DRONES
        }
        self.camera_trace_seq = {
            drone_id: 0
            for drone_id in ALLOWED_DRONES
        }
        self.subscription_profile = os.environ.get(
            "SWARM_GAZEBO_SUBSCRIPTION_PROFILE", "production"
        ).strip().lower()
        if self.subscription_profile not in {
            "production", "eager", "zero", "single", "count", "copy"
        }:
            self.subscription_profile = "production"
        self.subscription_topic_ids = {
            value.strip()
            for value in os.environ.get(
                "SWARM_GAZEBO_SUBSCRIPTION_TOPICS", ""
            ).replace(",", ";").split(";")
            if value.strip()
        }
        self.subscription_trace = get_recorder()

    def _subscription_enabled(self, topic_id: str) -> bool:
        if self.subscription_profile == "zero":
            return False
        if self.subscription_profile == "single":
            return topic_id in self.subscription_topic_ids
        return True

    def _subscribe_camera(self, drone_id: str) -> bool:
        """Subscribe only while a preview or tracker consumes this camera.

        Gazebo image messages are large and are deserialized before Python's
        callback can early-return.  Keeping both cameras subscribed while the
        dashboard is idle therefore creates avoidable transport/scheduler
        pressure even though no pixels are processed by the application.
        """
        with self.subscription_lock:
            if drone_id in self.camera_subscribed:
                return True
            if self.node is None:
                return False
            topic = self.camera_topics.get(drone_id)
            callback = self.camera_callbacks.get(drone_id)
            if not topic or callback is None:
                return False
            if not self.node.subscribe(GzImage, topic, callback):
                return False
            self.camera_subscribed.add(drone_id)
            return True

    def _unsubscribe_camera_if_idle(self, drone_id: str) -> None:
        if self.subscription_profile != "production":
            return
        with self.subscription_lock:
            with self.frame_condition:
                needed = (
                    self.camera_clients[drone_id] > 0
                    or self.tracking_drone_id == drone_id
                )
            if needed or drone_id not in self.camera_subscribed:
                return
            try:
                self.node.unsubscribe(self.camera_topics[drone_id])
            finally:
                self.camera_subscribed.discard(drone_id)

    @staticmethod
    def _copy_image_message(message: Any) -> bytes:
        return bytes(message.data)

    @staticmethod
    def _copy_imu_message(message: Any) -> tuple[float, ...]:
        orientation = message.orientation
        angular_velocity = message.angular_velocity
        linear_acceleration = message.linear_acceleration
        return tuple(
            float(value)
            for value in (
                orientation.x, orientation.y, orientation.z, orientation.w,
                angular_velocity.x, angular_velocity.y, angular_velocity.z,
                linear_acceleration.x, linear_acceleration.y,
                linear_acceleration.z,
            )
        )

    @staticmethod
    def _copy_lidar_message(message: Any) -> tuple[tuple[Any, ...], tuple[Any, ...]]:
        return tuple(message.ranges), tuple(message.intensities)

    def _run_subscription_callback(
        self,
        *,
        topic_id: str,
        expected_rate_hz: float,
        message: Any,
        production_callback: Any,
        copy_callback: Any,
    ) -> None:
        copy_action = (
            (lambda: copy_callback(message))
            if self.subscription_profile == "copy"
            else None
        )
        processing_action = (
            (lambda: production_callback(message))
            if self.subscription_profile in {"production", "eager", "single"}
            else None
        )
        self.subscription_trace.execute(
            topic_id=topic_id,
            expected_rate_hz=expected_rate_hz,
            message=message,
            copy_action=copy_action,
            processing_action=processing_action,
            copy_observable=self.subscription_profile == "copy",
        )

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

                camera_topic_id = f"camera_{drone_id.lower().replace('-', '')}"

                def camera_callback(
                    message: Any,
                    selected_drone_id: str = drone_id,
                    selected_topic_id: str = camera_topic_id,
                ) -> None:
                    if (
                        self.subscription_profile == "production"
                        and not self.subscription_trace.enabled
                    ):
                        self._handle_camera_image(selected_drone_id, message)
                        return
                    self._run_subscription_callback(
                        topic_id=selected_topic_id,
                        expected_rate_hz=GAZEBO_CAMERA_EXPECTED_RATE_HZ,
                        message=message,
                        production_callback=lambda item: self._handle_camera_image(
                            selected_drone_id, item
                        ),
                        copy_callback=self._copy_image_message,
                    )

                self.camera_callbacks[drone_id] = camera_callback
                if (
                    self._subscription_enabled(camera_topic_id)
                    and self.subscription_profile != "production"
                ):
                    if not self._subscribe_camera(drone_id):
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
                    if (
                        self.subscription_profile == "production"
                        and not self.subscription_trace.enabled
                    ):
                        self._handle_gimbal_imu(selected_drone_id, "body", message)
                        return
                    self._run_subscription_callback(
                        topic_id=f"body_imu_{selected_drone_id.lower().replace('-', '')}",
                        expected_rate_hz=250.0,
                        message=message,
                        production_callback=lambda item: self._handle_gimbal_imu(
                            selected_drone_id, "body", item
                        ),
                        copy_callback=self._copy_imu_message,
                    )

                def camera_imu_callback(
                    message: Any,
                    selected_drone_id: str = drone_id,
                ) -> None:
                    if (
                        self.subscription_profile == "production"
                        and not self.subscription_trace.enabled
                    ):
                        self._handle_gimbal_imu(selected_drone_id, "camera", message)
                        return
                    self._run_subscription_callback(
                        topic_id=f"camera_imu_{selected_drone_id.lower().replace('-', '')}",
                        expected_rate_hz=GAZEBO_CAMERA_IMU_EXPECTED_RATE_HZ,
                        message=message,
                        production_callback=lambda item: self._handle_gimbal_imu(
                            selected_drone_id, "camera", item
                        ),
                        copy_callback=self._copy_imu_message,
                    )

                for topic_id, topic, callback in (
                    (f"body_imu_{drone_id.lower().replace('-', '')}", body_imu_topic, body_imu_callback),
                    (f"camera_imu_{drone_id.lower().replace('-', '')}", camera_imu_topic, camera_imu_callback),
                ):
                    if not self._subscription_enabled(topic_id):
                        continue
                    if not self.node.subscribe(GzImu, topic, callback):
                        raise RuntimeError(f"Could not subscribe to {topic}")
                    self.imu_topics.add(topic)

            def lidar_callback(message: Any) -> None:
                if (
                    self.subscription_profile == "production"
                    and not self.subscription_trace.enabled
                ):
                    self._handle_lidar_scan(message)
                    return
                self._run_subscription_callback(
                    topic_id="front_lidar",
                    expected_rate_hz=10.0,
                    message=message,
                    production_callback=self._handle_lidar_scan,
                    copy_callback=self._copy_lidar_message,
                )

            if LaserScan is not None and self._subscription_enabled("front_lidar"):
                self.lidar_subscribed = bool(
                    self.node.subscribe(
                        LaserScan,
                        self.lidar_topic,
                        lidar_callback,
                    )
                )
            if os.environ.get("SWARM_RANGE_DATASET_DIR", "").strip():
                if GzPoseVector is None:
                    raise RuntimeError(
                        "Gazebo pose message type is unavailable"
                    )
                self.pose_subscribed = bool(
                    self.node.subscribe(
                        GzPoseVector,
                        self.pose_topic,
                        self._handle_world_poses,
                    )
                )
                if not self.pose_subscribed:
                    raise RuntimeError(
                        f"Could not subscribe to {self.pose_topic}"
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
            for drone_id in tuple(self.camera_subscribed):
                try:
                    self.node.unsubscribe(self.camera_topics[drone_id])
                except Exception:
                    pass
            self.camera_subscribed.clear()
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
            if self.pose_subscribed:
                try:
                    self.node.unsubscribe(self.pose_topic)
                except Exception:
                    pass
        self.started = False
        self.lidar_subscribed = False
        self.pose_subscribed = False
        self.subscription_trace.close()

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
        received_monotonic_s = time.monotonic()
        orientation = message.orientation
        quaternion = (
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )
        if not all(math.isfinite(value) for value in quaternion):
            return
        if source == "camera":
            self.camera_pose_buffers[drone_id].append(
                PoseSample(
                    timestamp_s=received_monotonic_s,
                    position_ned_m=(0.0, 0.0, 0.0),
                    velocity_ned_m_s=(0.0, 0.0, 0.0),
                    quaternion_xyzw=quaternion,
                )
            )
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
            self.gimbal_feedback_monotonic[drone_id] = received_monotonic_s

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

    @staticmethod
    def _message_timestamp_s(message: Any) -> float | None:
        try:
            stamp = message.header.stamp
            timestamp = float(stamp.sec) + float(stamp.nsec) * 1e-9
            return timestamp if math.isfinite(timestamp) else None
        except (AttributeError, TypeError, ValueError):
            return None

    @staticmethod
    def _rotate_vector(
        quaternion_xyzw: tuple[float, float, float, float],
        vector: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        x, y, z, w = quaternion_xyzw
        vx, vy, vz = vector
        tx = 2.0 * (y * vz - z * vy)
        ty = 2.0 * (z * vx - x * vz)
        tz = 2.0 * (x * vy - y * vx)
        return (
            vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx),
        )

    def _handle_world_poses(self, message: Any) -> None:
        sim_timestamp_s = self._message_timestamp_s(message)
        if sim_timestamp_s is None:
            return
        records: list[dict[str, Any]] = []
        model_records: dict[str, dict[str, Any]] = {}
        model_names = set(DRONE_MODELS.values())
        for pose in message.pose:
            name = str(getattr(pose, "name", ""))
            position = pose.position
            orientation = pose.orientation
            values = (
                float(position.x),
                float(position.y),
                float(position.z),
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            )
            if not all(math.isfinite(value) for value in values):
                continue
            record = {
                "name": name,
                "entity_id": int(getattr(pose, "id", 0)),
                "position_xyz": values[:3],
                "quaternion_xyzw": values[3:],
            }
            records.append(record)
            if name in model_names:
                model_records[name] = record

        poses: dict[str, dict[str, Any]] = {}
        for model_name, record in model_records.items():
            poses[model_name] = {
                "position_xyz": record["position_xyz"],
                "quaternion_xyzw": record["quaternion_xyzw"],
            }
        model_by_entity_id = sorted(
            (
                int(record["entity_id"]),
                model_name,
                record,
            )
            for model_name, record in model_records.items()
            if int(record["entity_id"]) > 0
        )
        for record in records:
            name = str(record["name"])
            if name in model_names:
                continue
            explicitly_scoped = next(
                (
                    model_name
                    for model_name in model_names
                    if model_name in name
                ),
                None,
            )
            if explicitly_scoped is not None:
                poses[name] = {
                    "position_xyz": record["position_xyz"],
                    "quaternion_xyzw": record["quaternion_xyzw"],
                }
                continue
            entity_id = int(record["entity_id"])
            owners = [
                item
                for item in model_by_entity_id
                if item[0] < entity_id
            ]
            if not owners:
                continue
            _, owner_name, owner = max(
                owners,
                key=lambda item: item[0],
            )
            rotated_position = self._rotate_vector(
                tuple(owner["quaternion_xyzw"]),
                tuple(record["position_xyz"]),
            )
            world_position = tuple(
                float(owner["position_xyz"][index])
                + rotated_position[index]
                for index in range(3)
            )
            world_orientation = self._quaternion_multiply(
                tuple(owner["quaternion_xyzw"]),
                tuple(record["quaternion_xyzw"]),
            )
            poses[f"{owner_name}::{name}"] = {
                "position_xyz": world_position,
                "quaternion_xyzw": world_orientation,
            }
        if poses:
            with self.pose_lock:
                self.pose_history.append(
                    {
                        "sim_timestamp_s": sim_timestamp_s,
                        "poses": poses,
                    }
                )
                self.pose_history_condition.notify_all()

    @staticmethod
    def _matching_pose(
        poses: dict[str, dict[str, Any]],
        model_name: str,
        *,
        camera: bool,
    ) -> tuple[str, dict[str, Any]] | None:
        candidates = [
            (name, pose)
            for name, pose in poses.items()
            if model_name in name
            and (
                name.endswith("camera_link")
                if camera
                else name == model_name
            )
        ]
        if not candidates and not camera:
            candidates = [
                (name, pose)
                for name, pose in poses.items()
                if name.endswith(model_name)
                and "::" not in name
            ]
        return min(candidates, key=lambda item: len(item[0])) if candidates else None

    def _locate_pose_bracket_locked(
        self, source_sim_timestamp_s: float
    ) -> tuple[dict[str, Any] | None, dict[str, Any] | None]:
        """Must be called with self.pose_lock held. Returns the (lower,
        upper) pose_history records (not copies) bracketing
        source_sim_timestamp_s, or None for either side not found."""
        lower_source = None
        upper_source = None
        for item in self.pose_history:
            item_timestamp_s = float(item["sim_timestamp_s"])
            if item_timestamp_s <= source_sim_timestamp_s:
                lower_source = item
            if item_timestamp_s >= source_sim_timestamp_s:
                upper_source = item
                break
        return lower_source, upper_source

    def reset_ground_truth_pending(self) -> None:
        """Abandon any in-flight pose-bracket wait immediately instead of
        letting it run out its own timeout. Call this at tracking-session
        boundaries (session start, bbox/target-session change) so a wait
        that was started for a frame from the old session never resolves
        against -- or blocks return of -- data belonging to a new one."""
        with self.pose_lock:
            self._gt_bracket_generation += 1
            self.pose_history_condition.notify_all()

    def simulation_target_ground_truth(
        self,
        camera_drone_id: str,
        source_sim_timestamp_s: float | None,
        target_drone_id: str | None = None,
    ) -> dict[str, Any]:
        if source_sim_timestamp_s is None:
            return {
                "available": False,
                "error": "source simulation timestamp unavailable",
            }
        target_drone_id = (
            str(target_drone_id).strip()
            if target_drone_id is not None
            else next(
                (
                    drone_id
                    for drone_id in sorted(ALLOWED_DRONES)
                    if drone_id != camera_drone_id
                ),
                "",
            )
        )
        if (
            target_drone_id not in ALLOWED_DRONES
            or target_drone_id == camera_drone_id
        ):
            return {
                "available": False,
                "error": "explicit ground-truth target is invalid",
            }
        camera_model = DRONE_MODELS.get(camera_drone_id)
        target_model = DRONE_MODELS.get(target_drone_id)
        if not camera_model or not target_model:
            return {"available": False, "error": "drone model mapping unavailable"}
        with self.pose_lock:
            # Pose callbacks can retain 600 snapshots containing every Gazebo
            # model/link.  Copying that entire deque for every tracking frame
            # held the Python GIL for 150-250 ms and starved both tracking and
            # the MiDaS worker.  The history is timestamp ordered, so select
            # the two immutable-by-convention bracket records while holding
            # the lock and copy only those records before releasing it.
            lower_source, upper_source = self._locate_pose_bracket_locked(
                source_sim_timestamp_s
            )
            bracket_retry_count = 0
            bracket_wait_started_monotonic_s: float | None = None
            if upper_source is None:
                if self._gt_bracket_waiting_count >= GT_BRACKET_MAX_PENDING_WAITERS:
                    return {
                        "available": False,
                        "error": "GT_BRACKET_PENDING_LIMIT_EXCEEDED",
                        "source_sim_timestamp_s": source_sim_timestamp_s,
                        "clock_domain": "gazebo_sim_time",
                    }
                # The camera frame's sim-time is newer than every pose
                # sample received so far -- the pose callback (a separate
                # subscription/thread) just hasn't caught up yet. Wait
                # (bounded) for it to notify a new sample instead of
                # failing this frame outright; a lower_source-missing
                # frame (camera timestamp older than the oldest buffered
                # sample) is not retried below since no future sample can
                # ever fill that in.
                self._gt_bracket_waiting_count += 1
                start_generation = self._gt_bracket_generation
                bracket_wait_started_monotonic_s = time.monotonic()
                deadline = bracket_wait_started_monotonic_s + GT_BRACKET_WAIT_TIMEOUT_S
                try:
                    while upper_source is None:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            break
                        notified = self.pose_history_condition.wait(
                            timeout=remaining
                        )
                        if self._gt_bracket_generation != start_generation:
                            break
                        if not notified:
                            break
                        bracket_retry_count += 1
                        lower_source, upper_source = (
                            self._locate_pose_bracket_locked(
                                source_sim_timestamp_s
                            )
                        )
                finally:
                    self._gt_bracket_waiting_count -= 1
            if lower_source is None or upper_source is None:
                timed_out = bracket_wait_started_monotonic_s is not None
                return {
                    "available": False,
                    "error": (
                        "GT_BRACKET_TIMEOUT"
                        if timed_out
                        else "Gazebo pose bracket unavailable"
                    ),
                    "source_sim_timestamp_s": source_sim_timestamp_s,
                    "clock_domain": "gazebo_sim_time",
                    "bracket_retry_count": bracket_retry_count,
                    "bracket_wait_s": (
                        round(
                            time.monotonic() - bracket_wait_started_monotonic_s,
                            4,
                        )
                        if bracket_wait_started_monotonic_s is not None
                        else 0.0
                    ),
                }
            lower = copy.deepcopy(lower_source)
            upper = (
                lower
                if upper_source is lower_source
                else copy.deepcopy(upper_source)
            )
        lower_time = float(lower["sim_timestamp_s"])
        upper_time = float(upper["sim_timestamp_s"])
        interpolation_span_s = upper_time - lower_time
        exact_lower = abs(float(source_sim_timestamp_s) - lower_time) <= 1e-9
        exact_upper = abs(float(source_sim_timestamp_s) - upper_time) <= 1e-9
        if exact_lower or exact_upper:
            snapshot = lower if exact_lower else upper
            pose_age_s = 0.0
            interpolation_span_s = 0.0
        elif (
            lower_time < float(source_sim_timestamp_s) < upper_time
            and 0.0 < interpolation_span_s <= 0.20
        ):
            poses: dict[str, dict[str, Any]] = {}
            for name in set(lower["poses"]) & set(upper["poses"]):
                poses[name] = interpolate_pose(
                    {
                        "timestamp_s": lower_time,
                        **lower["poses"][name],
                    },
                    {
                        "timestamp_s": upper_time,
                        **upper["poses"][name],
                    },
                    float(source_sim_timestamp_s),
                )
            snapshot = {
                "sim_timestamp_s": float(source_sim_timestamp_s),
                "poses": poses,
            }
            pose_age_s = 0.0
        else:
            return {
                "available": False,
                "error": "Gazebo pose bracket span invalid",
                "source_sim_timestamp_s": source_sim_timestamp_s,
                "pose_bracket_lower_sim_timestamp_s": lower_time,
                "pose_bracket_upper_sim_timestamp_s": upper_time,
                "interpolation_span_ms": round(interpolation_span_s * 1000.0, 2),
                "clock_domain": "gazebo_sim_time",
            }
        poses = snapshot["poses"]
        camera_pose_match = self._matching_pose(
            poses,
            camera_model,
            camera=True,
        )
        camera_center_match = self._matching_pose(
            poses,
            camera_model,
            camera=False,
        )
        target_center_match = self._matching_pose(
            poses,
            target_model,
            camera=False,
        )
        if camera_pose_match is None or target_center_match is None:
            return {
                "available": False,
                "error": "camera link or target model pose unavailable",
                "pose_age_ms": round(pose_age_s * 1000.0, 2),
            }
        camera_pose_name, camera_pose = camera_pose_match
        _, target_pose = target_center_match
        camera_link_position = tuple(camera_pose["position_xyz"])
        target_position = tuple(target_pose["position_xyz"])
        geometry = optical_center_distance(camera_pose, target_pose)
        camera_position = tuple(geometry["camera_optical_center_xyz"])
        vector = tuple(
            target_position[index] - camera_position[index]
            for index in range(3)
        )
        camera_to_target = float(geometry["distance_m"])
        camera_forward = self._rotate_vector(
            tuple(geometry["camera_optical_quaternion_xyzw"]),
            # Gazebo camera sensor optical forward axis after its SDF pose.
            (1.0, 0.0, 0.0),
        )
        cosine = (
            sum(
                vector[index] * camera_forward[index]
                for index in range(3)
            )
            / max(1e-9, camera_to_target)
        )
        target_angle_deg = math.degrees(
            math.acos(max(-1.0, min(1.0, cosine)))
        )
        drone_center_distance = None
        camera_center_position = None
        if camera_center_match is not None:
            _, camera_center_pose = camera_center_match
            camera_center_position = tuple(camera_center_pose["position_xyz"])
            drone_center_distance = math.sqrt(
                sum(
                    (
                        target_position[index]
                        - camera_center_position[index]
                    ) ** 2
                    for index in range(3)
                )
            )
        return {
            "available": True,
            "camera_drone_id": camera_drone_id,
            "target_drone_id": target_drone_id,
            "source_sim_timestamp_s": source_sim_timestamp_s,
            "pose_sim_timestamp_s": snapshot["sim_timestamp_s"],
            "camera_pose_sim_timestamp_s": snapshot["sim_timestamp_s"],
            "target_pose_sim_timestamp_s": snapshot["sim_timestamp_s"],
            "pose_bracket_lower_sim_timestamp_s": lower_time,
            "pose_bracket_upper_sim_timestamp_s": upper_time,
            "bracket_retry_count": bracket_retry_count,
            "bracket_wait_s": (
                round(
                    time.monotonic() - bracket_wait_started_monotonic_s, 4
                )
                if bracket_wait_started_monotonic_s is not None
                else 0.0
            ),
            "pose_age_ms": round(pose_age_s * 1000.0, 2),
            "interpolation_span_ms": (
                round(interpolation_span_s * 1000.0, 2)
                if interpolation_span_s is not None
                else None
            ),
            "camera_pose_source": camera_pose_name,
            "camera_position_xyz": camera_position,
            "camera_optical_center_xyz": camera_position,
            "camera_link_origin_xyz": camera_link_position,
            "camera_optical_quaternion_xyzw": geometry[
                "camera_optical_quaternion_xyzw"
            ],
            "camera_drone_center_xyz": camera_center_position,
            "target_position_xyz": target_position,
            "target_reference_xyz": target_position,
            "target_reference": TARGET_REFERENCE,
            "camera_to_target_center_m": camera_to_target,
            "drone_center_to_center_m": drone_center_distance,
            "target_angle_deg": target_angle_deg,
            "clock_domain": "gazebo_sim_time",
            "ground_truth_contract_version": GT_CONTRACT_VERSION,
            "ground_truth_contract_sha256": GT_CONTRACT_SHA256,
            "trajectory_driver_version": os.environ.get(
                "SWARM_SIM_TIME_TRAJECTORY_DRIVER_VERSION",
                "legacy_wall_clock_set_pose",
            ),
            "trajectory_driver_checksum": os.environ.get(
                "SWARM_SIM_TIME_TRAJECTORY_CONTRACT_SHA256",
            ),
        }

    def _handle_camera_image(
        self,
        drone_id: str,
        message: Any,
    ) -> None:
        now = time.monotonic()
        sim_timestamp_s = self._message_timestamp_s(message)

        with self.frame_condition:
            self._update_source_fps_locked(drone_id, now)
            self.camera_trace_seq[drone_id] += 1
            trace_frame_seq = self.camera_trace_seq[drone_id]
            _trace_camera_hop(
                "camera_source_receipt",
                drone_id=drone_id,
                frame_seq=trace_frame_seq,
                sim_timestamp_s=sim_timestamp_s,
                monotonic_receipt_s=now,
            )
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
            self.conversion_samples_ms[drone_id].append(conversion_ms)
            if self.tracking_drone_id == drone_id:
                mailbox_store_monotonic_s = time.monotonic()
                self.raw_frames[drone_id] = raw_frame
                self.raw_frame_versions[drone_id] += 1
                self.last_raw_frame_monotonic[drone_id] = mailbox_store_monotonic_s
                self.last_raw_frame_sim_timestamp_s[drone_id] = sim_timestamp_s
                _trace_camera_hop(
                    "camera_mailbox_store",
                    drone_id=drone_id,
                    frame_seq=trace_frame_seq,
                    sim_timestamp_s=sim_timestamp_s,
                    monotonic_store_s=mailbox_store_monotonic_s,
                    raw_frame_version=self.raw_frame_versions[drone_id],
                    conversion_ms=conversion_ms,
                )
                self.frame_condition.notify_all()

        if not encode_camera_frame:
            return

        encode_started = time.perf_counter()
        try:
            jpeg = self._bgr_to_jpeg(raw_frame)
        except Exception as error:
            self.error = f"Camera JPEG failed for {drone_id}: {error}"
            return
        encode_ms = (time.perf_counter() - encode_started) * 1000.0

        with self.frame_condition:
            if self.camera_clients[drone_id] > 0:
                self.frames[drone_id] = jpeg
                self.frame_versions[drone_id] += 1
                self.last_frame_monotonic[drone_id] = time.monotonic()
                self.camera_encode_samples_ms[drone_id].append(encode_ms)
                self.camera_encode_times_s[drone_id].append(time.monotonic())
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

        return np.ascontiguousarray(image)

    @staticmethod
    def _bgr_to_jpeg(image: Any) -> bytes:
        encode_image = image
        height, width = image.shape[:2]
        if width > CAMERA_MAX_WIDTH:
            resized_height = max(
                1,
                round(height * CAMERA_MAX_WIDTH / width),
            )
            encode_image = cv2.resize(
                image,
                (CAMERA_MAX_WIDTH, resized_height),
                interpolation=cv2.INTER_AREA,
            )
        success, encoded = cv2.imencode(
            ".jpg",
            encode_image,
            [cv2.IMWRITE_JPEG_QUALITY, CAMERA_JPEG_QUALITY],
        )

        if not success:
            raise RuntimeError("OpenCV could not encode camera frame")

        return encoded.tobytes()

    def set_tracking_drone(
        self,
        drone_id: str | None,
    ) -> None:
        previous_drone_id = self.tracking_drone_id
        if drone_id is not None and not self._subscribe_camera(drone_id):
            raise RuntimeError(f"Could not subscribe to camera for {drone_id}")
        with self.frame_condition:
            self.tracking_drone_id = drone_id
            if drone_id is None:
                self.raw_frames.clear()
            else:
                self.raw_frames.pop(drone_id, None)
                self.raw_frame_versions[drone_id] += 1
            self.frame_condition.notify_all()
        if previous_drone_id is not None and previous_drone_id != drone_id:
            self._unsubscribe_camera_if_idle(previous_drone_id)

    def mjpeg_frames(self, drone_id: str):
        last_version = -1

        with self.frame_condition:
            self.camera_clients[drone_id] += 1
        if not self._subscribe_camera(drone_id):
            with self.frame_condition:
                self.camera_clients[drone_id] -= 1
            raise RuntimeError(f"Could not subscribe to camera for {drone_id}")

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
            self._unsubscribe_camera_if_idle(drone_id)

    def status(self) -> dict[str, Any]:
        now = time.monotonic()
        cameras: dict[str, dict[str, Any]] = {}
        with self.pose_lock:
            pose_samples = len(self.pose_history)
            latest_pose_sim_timestamp_s = (
                float(self.pose_history[-1]["sim_timestamp_s"])
                if self.pose_history
                else None
            )

        with self.frame_condition:
            for drone_id in ALLOWED_DRONES:
                last_frame = self.last_frame_monotonic.get(drone_id)
                last_raw_frame = self.last_raw_frame_monotonic.get(drone_id)
                conversion_timing = self._timing_summary(
                    self.conversion_samples_ms[drone_id]
                )
                encode_timing = self._timing_summary(
                    self.camera_encode_samples_ms[drone_id]
                )
                recent_encodes = [
                    timestamp
                    for timestamp in self.camera_encode_times_s[drone_id]
                    if now - timestamp <= 2.0
                ]
                encode_fps = (
                    (len(recent_encodes) - 1)
                    / max(
                        1e-6,
                        recent_encodes[-1] - recent_encodes[0],
                    )
                    if len(recent_encodes) >= 2
                    else 0.0
                )
                cameras[drone_id] = {
                    "topic": self.camera_topics.get(drone_id, ""),
                    "has_frame": drone_id in self.frames,
                    "clients": self.camera_clients[drone_id],
                    "tracking": self.tracking_drone_id == drone_id,
                    "subscribed": drone_id in self.camera_subscribed,
                    "source_fps": round(self.source_fps[drone_id], 1),
                    "conversion_ms": round(self.conversion_ms[drone_id], 2),
                    "conversion_average_ms": conversion_timing["average_ms"],
                    "conversion_p95_ms": conversion_timing["p95_ms"],
                    "jpeg_encode_fps": round(encode_fps, 1),
                    "jpeg_encode_average_ms": encode_timing["average_ms"],
                    "jpeg_encode_p95_ms": encode_timing["p95_ms"],
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
            "pose_subscribed": self.pose_subscribed,
            "pose_samples": pose_samples,
            "latest_pose_sim_timestamp_s": latest_pose_sim_timestamp_s,
            "cameras": cameras,
        }

    @staticmethod
    def _timing_summary(samples: Any) -> dict[str, float]:
        ordered = sorted(float(value) for value in samples)
        if not ordered:
            return {"average_ms": 0.0, "p95_ms": 0.0}
        p95_index = max(0, math.ceil(0.95 * len(ordered)) - 1)
        return {
            "average_ms": round(sum(ordered) / len(ordered), 2),
            "p95_ms": round(ordered[p95_index], 2),
        }


gazebo_bridge = GazeboDashboardBridge()


def publish_gimbal_home(drone_id: str) -> dict[str, Any]:
    if drone_id not in ALLOWED_DRONES:
        return {"ok": False, "error": "Invalid drone ID"}

    errors: list[str] = []
    with gimbal_lock:
        tracking_gimbal_rates_deg_s[drone_id] = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
        tracking_gimbal_last_command_monotonic[drone_id] = 0.0
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

    return current


def command_tracking_gimbal(
    drone_id: str,
    pan_rate_rad_s: float,
    tilt_rate_rad_s: float,
    dt: float,
) -> dict[str, Any]:
    dt = max(0.001, min(0.1, float(dt)))

    with gimbal_lock:
        now = time.monotonic()
        measured = synced_gimbal_angles_deg(drone_id)
        commanded = dict(gimbal_angles_deg[drone_id])
        previous_rates = dict(tracking_gimbal_rates_deg_s[drone_id])
        command_stale = (
            now - tracking_gimbal_last_command_monotonic[drone_id]
            > TRACKING_GIMBAL_COMMAND_STALE_RESET_S
        )
        if command_stale:
            previous_rates = {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            }
            commanded = dict(measured)
            gimbal_angles_deg[drone_id].update(commanded)
        raw_requested_rates = {
            "roll": -measured["roll"] / dt,
            "pitch": math.degrees(tilt_rate_rad_s),
            "yaw": math.degrees(pan_rate_rad_s),
        }
        maximum_rates = {
            "roll": min(
                TRACKING_GIMBAL_ROLL_RATE_DEG_S,
                TRACKING_GIMBAL_MAX_PITCH_RATE_DEG_S,
            ),
            "pitch": TRACKING_GIMBAL_MAX_PITCH_RATE_DEG_S,
            "yaw": TRACKING_GIMBAL_MAX_YAW_RATE_DEG_S,
        }
        limited_rates = {
            axis: limit_tracking_gimbal_rate_deg_s(
                raw_requested_rates[axis],
                previous_rates[axis],
                maximum_rates[axis],
                TRACKING_GIMBAL_MAX_ACCEL_DEG_S2,
                dt,
                TRACKING_GIMBAL_MAX_BRAKE_ACCEL_DEG_S2,
            )
            for axis in GIMBAL_LIMITS_DEG
        }
        requested_targets = {
            axis: commanded[axis] + limited_rates[axis] * dt
            for axis in GIMBAL_LIMITS_DEG
        }
        targets = dict(requested_targets)

        for axis, (minimum, maximum) in GIMBAL_LIMITS_DEG.items():
            targets[axis] = max(
                measured[axis] - TRACKING_GIMBAL_MAX_COMMAND_LEAD_DEG,
                min(
                    measured[axis] + TRACKING_GIMBAL_MAX_COMMAND_LEAD_DEG,
                    targets[axis],
                ),
            )
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

        targets["yaw"], yaw_saturated_outward = (
            clamp_tracking_gimbal_yaw_deg(
                requested_targets["yaw"],
                TRACKING_GIMBAL_YAW_LIMIT_DEG,
            )
        )
        targets["yaw"] = max(
            tracking_yaw_minimum,
            min(tracking_yaw_maximum, targets["yaw"]),
        )
        targets["yaw"] = max(
            measured["yaw"] - TRACKING_GIMBAL_MAX_COMMAND_LEAD_DEG,
            min(
                measured["yaw"] + TRACKING_GIMBAL_MAX_COMMAND_LEAD_DEG,
                targets["yaw"],
            ),
        )
        targets["pitch"], pitch_saturated_outward = (
            clamp_tracking_gimbal_pitch_deg(targets["pitch"])
        )

        applied_rates = {
            axis: (targets[axis] - commanded[axis]) / dt
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
                tracking_gimbal_rates_deg_s[drone_id][axis] = (
                    applied_rates[axis]
                )
            else:
                applied_rates[axis] = 0.0
                tracking_gimbal_rates_deg_s[drone_id][axis] = 0.0
                errors.append(f"{axis}: {error}")

        tracking_gimbal_last_command_monotonic[drone_id] = now
        feedback = gazebo_bridge.gimbal_feedback(drone_id)
        angles = (
            dict(feedback.get("angles_deg"))
            if (
                feedback.get("available")
                and isinstance(feedback.get("angles_deg"), dict)
            )
            else dict(gimbal_angles_deg[drone_id])
        )
        commanded_angles = dict(gimbal_angles_deg[drone_id])

    return {
        "ok": not errors,
        "angles_deg": angles,
        "commanded_angles_deg": commanded_angles,
        "rates_deg_s": applied_rates,
        "raw_requested_rates_deg_s": raw_requested_rates,
        "limited_rates_deg_s": limited_rates,
        "maximum_rates_deg_s": maximum_rates,
        "maximum_acceleration_deg_s2": (
            TRACKING_GIMBAL_MAX_ACCEL_DEG_S2
        ),
        "maximum_braking_acceleration_deg_s2": (
            TRACKING_GIMBAL_MAX_BRAKE_ACCEL_DEG_S2
        ),
        "maximum_command_lead_deg": (
            TRACKING_GIMBAL_MAX_COMMAND_LEAD_DEG
        ),
        "yaw_limit_deg": TRACKING_GIMBAL_YAW_LIMIT_DEG,
        "yaw_minimum_deg": tracking_yaw_minimum,
        "yaw_maximum_deg": tracking_yaw_maximum,
        "yaw_saturated_outward": yaw_saturated_outward,
        "pitch_limits_deg": {
            "minimum": TRACKING_GIMBAL_PITCH_MIN_DEG,
            "maximum": TRACKING_GIMBAL_PITCH_MAX_DEG,
        },
        "pitch_saturated_outward": pitch_saturated_outward,
        "yaw_requested_rate_deg_s": raw_requested_rates["yaw"],
        "feedback_available": bool(feedback.get("available")),
        "feedback_age_ms": feedback.get("age_ms"),
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


def command_rgb_bootstrap_motion(
    drone_id: str,
    enabled: bool,
    north_velocity_m_s: float,
    east_velocity_m_s: float,
    hold_z_down_m: float,
    yaw_rate_deg_s: float,
    workflow_state: str,
) -> dict[str, Any]:
    maximum_speed_m_s = 0.75
    north_velocity_m_s = max(
        -maximum_speed_m_s,
        min(maximum_speed_m_s, float(north_velocity_m_s)),
    )
    east_velocity_m_s = max(
        -maximum_speed_m_s,
        min(maximum_speed_m_s, float(east_velocity_m_s)),
    )
    yaw_rate_deg_s = max(-25.0, min(25.0, float(yaw_rate_deg_s)))
    hold_z_down_m = float(hold_z_down_m)
    values = (
        north_velocity_m_s,
        east_velocity_m_s,
        yaw_rate_deg_s,
        hold_z_down_m,
    )
    requested_enabled = bool(
        enabled
        and TRACKING_OFFBOARD_ENABLED
        and all(math.isfinite(value) for value in values)
    )
    safety_error = (
        tracking_body_yaw_safety_error(drone_id)
        if requested_enabled
        else ""
    )
    effective_enabled = requested_enabled and not safety_error
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
            "motion_authority": "rgb_bootstrap",
            "track_state": str(workflow_state),
        }
    )
    error = safety_error or publish_error
    return {
        "ok": bool(ok and effective_enabled and not error),
        "enabled": effective_enabled,
        "north_velocity_m_s": (
            north_velocity_m_s if effective_enabled else 0.0
        ),
        "east_velocity_m_s": (
            east_velocity_m_s if effective_enabled else 0.0
        ),
        "yaw_rate_deg_s": yaw_rate_deg_s if effective_enabled else 0.0,
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


def tracking_visual_pose(
    drone_id: str,
    frame_timestamp_s: float | None = None,
) -> dict[str, Any]:
    with state_lock:
        drone = copy.deepcopy(latest_drones.get(drone_id, {}))
        visual_follow_bridge = copy.deepcopy(
            tracking_visual_follow_bridge_status.get(drone_id, {})
        )
        manual_override = time.monotonic() <= manual_control_deadline.get(
            drone_id, 0.0
        )
        manual_loss_started = (
            tracking_visual_follow_manual_loss_started_monotonic.get(
                drone_id
            )
        )
    if not isinstance(drone, dict) or not drone.get("online", False):
        return {"available": False, "error": "Vehicle telemetry is offline"}
    received_ms = drone.get("dashboard_received_ms")
    try:
        telemetry_age_ms = now_ms() - int(received_ms)
    except (TypeError, ValueError):
        return {"available": False, "error": "Telemetry timestamp is unavailable"}
    if telemetry_age_ms > TRACKING_STATUS_MAX_AGE_MS:
        return {"available": False, "error": "Vehicle telemetry is stale"}
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
    query_timestamp_s = (
        time.monotonic()
        if frame_timestamp_s is None
        else float(frame_timestamp_s)
    )
    vehicle_pose = tracking_vehicle_pose_buffers[drone_id].sample_at(
        query_timestamp_s
    )
    if not vehicle_pose.valid:
        return {
            "available": False,
            "error": f"Vehicle pose synchronization failed: {vehicle_pose.reason}",
            "frame_timestamp_s": query_timestamp_s,
            "pose_sync_reason": vehicle_pose.reason,
            "pose_age_ms": (
                round(vehicle_pose.sample_age_s * 1000.0, 2)
                if vehicle_pose.sample_age_s is not None
                else None
            ),
            "pose_sample_offset_ms": (
                round(vehicle_pose.sample_offset_s * 1000.0, 2)
                if vehicle_pose.sample_offset_s is not None
                else None
            ),
            "pose_interpolation_span_ms": (
                round(vehicle_pose.interpolation_span_s * 1000.0, 2)
                if vehicle_pose.interpolation_span_s is not None
                else None
            ),
            "pose_lower_delta_ms": (
                round(vehicle_pose.lower_sample_delta_s * 1000.0, 2)
                if vehicle_pose.lower_sample_delta_s is not None
                else None
            ),
            "pose_upper_delta_ms": (
                round(vehicle_pose.upper_sample_delta_s * 1000.0, 2)
                if vehicle_pose.upper_sample_delta_s is not None
                else None
            ),
        }
    camera_pose = gazebo_bridge.camera_pose_buffers[drone_id].sample_at(
        query_timestamp_s
    )
    if not camera_pose.valid:
        return {
            "available": False,
            "error": f"Gimbal pose synchronization failed: {camera_pose.reason}",
            "frame_timestamp_s": query_timestamp_s,
            "gimbal_sync_reason": camera_pose.reason,
            "gimbal_age_ms": (
                round(camera_pose.sample_age_s * 1000.0, 2)
                if camera_pose.sample_age_s is not None
                else None
            ),
            "gimbal_sample_offset_ms": (
                round(camera_pose.sample_offset_s * 1000.0, 2)
                if camera_pose.sample_offset_s is not None
                else None
            ),
            "gimbal_interpolation_span_ms": (
                round(camera_pose.interpolation_span_s * 1000.0, 2)
                if camera_pose.interpolation_span_s is not None
                else None
            ),
            "gimbal_lower_delta_ms": (
                round(camera_pose.lower_sample_delta_s * 1000.0, 2)
                if camera_pose.lower_sample_delta_s is not None
                else None
            ),
            "gimbal_upper_delta_ms": (
                round(camera_pose.upper_sample_delta_s * 1000.0, 2)
                if camera_pose.upper_sample_delta_s is not None
                else None
            ),
        }
    assert vehicle_pose.position_ned_m is not None
    assert vehicle_pose.quaternion_xyzw is not None
    assert camera_pose.quaternion_xyzw is not None
    camera_offset_ned = gazebo_bridge._rotate_vector(
        vehicle_pose.quaternion_xyzw,
        TRACKING_CAMERA_OFFSET_BODY_FRD_M,
    )
    camera_position_ned = tuple(
        vehicle_pose.position_ned_m[index] + camera_offset_ned[index]
        for index in range(3)
    )
    latest_vehicle_ned = (
        float(local_position["x_north_m"]),
        float(local_position["y_east_m"]),
        float(local_position["z_down_m"]),
    )
    north_delta = vehicle_pose.position_ned_m[0] - latest_vehicle_ned[0]
    east_delta = vehicle_pose.position_ned_m[1] - latest_vehicle_ned[1]
    down_delta = vehicle_pose.position_ned_m[2] - latest_vehicle_ned[2]
    earth_radius_m = 6_378_137.0
    latest_latitude_rad = math.radians(
        float(global_position["latitude_deg"])
    )
    longitude_scale = max(0.01, abs(math.cos(latest_latitude_rad)))
    synchronized_global_position = (
        float(global_position["latitude_deg"])
        + math.degrees(north_delta / earth_radius_m),
        float(global_position["longitude_deg"])
        + math.degrees(east_delta / (earth_radius_m * longitude_scale)),
        float(global_position["altitude_msl_m"]) - down_delta,
    )
    vehicle_x, vehicle_y, vehicle_z, vehicle_w = (
        float(value) for value in vehicle_pose.quaternion_xyzw
    )
    synchronized_heading_rad = math.atan2(
        2.0 * (
            vehicle_w * vehicle_z
            + vehicle_x * vehicle_y
        ),
        1.0 - 2.0 * (
            vehicle_y * vehicle_y
            + vehicle_z * vehicle_z
        ),
    )
    vehicle_status = drone.get("status", {})
    failsafe_flags = drone.get("failsafe_flags", {})
    return {
        "available": True,
        "local_position": local_position,
        "global_position": global_position,
        "vehicle_position_ned_m": vehicle_pose.position_ned_m,
        "vehicle_velocity_ned_m_s": vehicle_pose.velocity_ned_m_s,
        "vehicle_quaternion_xyzw": vehicle_pose.quaternion_xyzw,
        "synchronized_heading_rad": synchronized_heading_rad,
        "synchronized_global_position": synchronized_global_position,
        "camera_position_ned_m": camera_position_ned,
        "camera_offset_body_frd_m": TRACKING_CAMERA_OFFSET_BODY_FRD_M,
        "camera_quaternion_xyzw": camera_pose.quaternion_xyzw,
        "armed": bool(
            vehicle_status.get("armed", False)
            if isinstance(vehicle_status, dict)
            else False
        ),
        "failsafe": bool(
            vehicle_status.get("failsafe", False)
            if isinstance(vehicle_status, dict)
            else True
        ),
        "local_position_valid": not bool(
            failsafe_flags.get("local_position_invalid", False)
            if isinstance(failsafe_flags, dict)
            else True
        ),
        "global_position_valid": not bool(
            failsafe_flags.get("global_position_invalid", False)
            if isinstance(failsafe_flags, dict)
            else True
        ),
        "manual_override": manual_override,
        "px4_nav_state": (
            int(vehicle_status.get("nav_state", -1))
            if isinstance(vehicle_status, dict)
            else -1
        ),
        "visual_follow_bridge": visual_follow_bridge,
        "manual_loss_grace_state": (
            "GRACE"
            if manual_loss_started is not None
            else "INACTIVE"
        ),
        "manual_loss_grace_age_ms": (
            round(
                max(0.0, time.monotonic() - manual_loss_started)
                * 1000.0,
                2,
            )
            if manual_loss_started is not None
            else None
        ),
        "frame_timestamp_s": query_timestamp_s,
        "pose_sync_reason": vehicle_pose.reason,
        "gimbal_sync_reason": camera_pose.reason,
        "telemetry_age_ms": round(
            float(vehicle_pose.sample_age_s or 0.0) * 1000.0,
            2,
        ),
        "camera_age_ms": round(
            float(camera_pose.sample_age_s or 0.0) * 1000.0,
            2,
        ),
        "pose_interpolation_span_ms": round(
            float(vehicle_pose.interpolation_span_s or 0.0) * 1000.0,
            2,
        ),
        "gimbal_interpolation_span_ms": round(
            float(camera_pose.interpolation_span_s or 0.0) * 1000.0,
            2,
        ),
        "pose_interpolated": vehicle_pose.interpolated,
        "gimbal_interpolated": camera_pose.interpolated,
        "pose_sample_offset_ms": round(
            float(vehicle_pose.sample_offset_s or 0.0) * 1000.0,
            2,
        ),
        "gimbal_sample_offset_ms": round(
            float(camera_pose.sample_offset_s or 0.0) * 1000.0,
            2,
        ),
        "pose_lower_delta_ms": (
            round(vehicle_pose.lower_sample_delta_s * 1000.0, 2)
            if vehicle_pose.lower_sample_delta_s is not None
            else None
        ),
        "pose_upper_delta_ms": (
            round(vehicle_pose.upper_sample_delta_s * 1000.0, 2)
            if vehicle_pose.upper_sample_delta_s is not None
            else None
        ),
        "gimbal_lower_delta_ms": (
            round(camera_pose.lower_sample_delta_s * 1000.0, 2)
            if camera_pose.lower_sample_delta_s is not None
            else None
        ),
        "gimbal_upper_delta_ms": (
            round(camera_pose.upper_sample_delta_s * 1000.0, 2)
            if camera_pose.upper_sample_delta_s is not None
            else None
        ),
    }


def tracking_visual_follow_safety_error(drone_id: str) -> str:
    now = time.monotonic()
    with state_lock:
        drone = copy.deepcopy(latest_drones.get(drone_id, {}))
        bridge_status = copy.deepcopy(
            tracking_visual_follow_bridge_status.get(drone_id, {})
        )
        manual_override = now <= manual_control_deadline.get(
            drone_id, 0.0
        )
    if manual_override:
        return "Visual Follow paused: manual control has priority"
    if not isinstance(drone, dict) or not drone.get("online", False):
        return "Visual Follow blocked: telemetry is offline"
    status = drone.get("status", {})
    if not isinstance(status, dict) or not bool(status.get("armed", False)):
        return "Visual Follow blocked: vehicle is not armed"
    try:
        nav_state = int(status.get("nav_state", -1))
    except (TypeError, ValueError):
        nav_state = -1
    failsafe_flags = drone.get("failsafe_flags", {})
    if not isinstance(failsafe_flags, dict):
        failsafe_flags = {}
    if bool(status.get("failsafe", False)):
        manual_signal_lost = bool(
            failsafe_flags.get("manual_control_signal_lost", False)
        )
        # offboard_control_signal_lost is expected after native Follow takes
        # authority. Every other asserted flag remains fail-closed.
        benign_during_native_follow = {
            "manual_control_signal_lost",
            "offboard_control_signal_lost",
        }
        other_asserted_flags = {
            str(name)
            for name, value in failsafe_flags.items()
            if bool(value) and str(name) not in benign_during_native_follow
        }
        try:
            prestream_ready = int(
                bridge_status.get("prestream_count", 0)
            ) >= int(bridge_status.get("prestream_required", 10))
        except (TypeError, ValueError):
            prestream_ready = False
        bridge_handover_active = bool(
            bridge_status.get("enabled", False)
            and bridge_status.get("valid", False)
            and str(bridge_status.get("timeout_state", "INACTIVE"))
            in {"FRESH", "DEGRADED"}
            and (prestream_ready or nav_state == 19)
        )
        allow_manual_loss_grace = bool(
            manual_signal_lost
            and not other_asserted_flags
            and bridge_handover_active
        )
        if allow_manual_loss_grace:
            with state_lock:
                started = (
                    tracking_visual_follow_manual_loss_started_monotonic.get(
                        drone_id
                    )
                )
                if started is None:
                    started = now
                    tracking_visual_follow_manual_loss_started_monotonic[
                        drone_id
                    ] = started
            if now - started > TRACKING_VISUAL_FOLLOW_MANUAL_LOSS_GRACE_S:
                return (
                    "Visual Follow blocked: manual-control loss persisted "
                    "beyond handover grace"
                )
        else:
            with state_lock:
                tracking_visual_follow_manual_loss_started_monotonic[
                    drone_id
                ] = None
            return "Visual Follow blocked: PX4 failsafe is active"
    else:
        with state_lock:
            tracking_visual_follow_manual_loss_started_monotonic[
                drone_id
            ] = None
    # Stage-1 relative visual servo runs in Offboard (nav_state 14). Once a
    # real metric target is ready, the bridge atomically replaces that source
    # with FOLLOW_TARGET prestream before requesting AUTO Follow. Therefore
    # Offboard is a valid handover origin, not a reason to reject the first
    # real target sample.
    if nav_state not in {2, 4, 14, 19}:
        return (
            "Visual Follow blocked: PX4 must be in Position, Hold, "
            "Offboard or Follow"
        )
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
            source_measurement_timestamp_s = finite_float(
                target.get("measurement_timestamp_s", time.monotonic())
            )
            estimate_age_ms = finite_float(
                target.get("estimate_age_ms", 0.0),
                minimum=0.0,
                maximum=1500.0,
            )
            # Keep bridge freshness tied to the newest accepted metric
            # correction. Publishing an EKF prediction must not disguise an
            # old range sample as a new measurement.
            measurement_timestamp_s = (
                time.monotonic() - estimate_age_ms / 1000.0
            )
            covariance_source = target.get(
                "position_covariance",
                (100.0, 100.0, 100.0),
            )
            if (
                not isinstance(covariance_source, (list, tuple))
                or len(covariance_source) != 3
            ):
                raise ValueError(
                    "position_covariance must contain three values"
                )
            position_covariance = [
                finite_float(value, minimum=0.0, maximum=1.0e6)
                for value in covariance_source
            ]
            estimate_capabilities = int(
                target.get("est_capabilities", 1)
            )
            if estimate_capabilities < 1 or estimate_capabilities > 255:
                raise ValueError("est_capabilities is outside MAVLink range")
            session_id = int(target["session_id"])
            if session_id <= 0:
                raise ValueError("session_id must be positive")
            source_kind = str(target["source_kind"]).strip().lower()
            target_semantics = str(
                target["target_semantics"]
            ).strip().lower()
            if not source_kind:
                raise ValueError("source_kind is required")
            if target_semantics != "measured_target":
                raise ValueError("target semantics are not measured")
            latitude_deg = finite_float(
                target["latitude_deg"], minimum=-90.0, maximum=90.0
            )
            longitude_deg = finite_float(
                target["longitude_deg"], minimum=-180.0, maximum=180.0
            )
            if latitude_deg == 0.0 and longitude_deg == 0.0:
                raise ValueError("zero latitude/longitude target is forbidden")
            payload.update(
                {
                    "latitude_deg": latitude_deg,
                    "longitude_deg": longitude_deg,
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
                    "follow_angle_deg": finite_float(
                        target.get("follow_angle_deg", 180.0),
                        minimum=-180.0,
                        maximum=180.0,
                    ),
                    "position_covariance": position_covariance,
                    "est_capabilities": estimate_capabilities,
                    "measurement_timestamp_s": measurement_timestamp_s,
                    "source_measurement_timestamp_s": (
                        source_measurement_timestamp_s
                    ),
                    "estimate_age_ms": estimate_age_ms,
                    "session_id": session_id,
                    "source_kind": source_kind,
                    "target_semantics": target_semantics,
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


range_dataset_collection_enabled = bool(
    os.environ.get("SWARM_RANGE_DATASET_DIR", "").strip()
)
if range_dataset_collection_enabled:
    # Offline/SITL collection is intentionally passive and can trade latency
    # for complete CPU inference results plus better ground-anchor coverage.
    os.environ["SWARM_METRIC_TARGET_DEPTH_RATE_HZ"] = os.environ.get(
        "SWARM_RANGE_DATASET_DEPTH_RATE_HZ",
        "5.0",
    )
    os.environ["SWARM_METRIC_TARGET_MAX_DEPTH_AGE_S"] = os.environ.get(
        "SWARM_RANGE_DATASET_MAX_DEPTH_AGE_S",
        "3.0",
    )
    os.environ["SWARM_TRACKING_GIMBAL_ENABLED"] = os.environ.get(
        "SWARM_RANGE_DATASET_GIMBAL_TRACKING_ENABLED",
        "false",
    )


def range_ground_truth_telemetry(
    drone_id: str,
) -> dict[str, Any] | None:
    """Return an immutable telemetry snapshot for label-only RTK evaluation."""
    with state_lock:
        telemetry = latest_drones.get(str(drone_id))
        return None if telemetry is None else copy.deepcopy(telemetry)


range_ground_truth_provider = RangeGroundTruthRouter.from_environment(
    gazebo_bridge.simulation_target_ground_truth,
    range_ground_truth_telemetry,
)

tracking_manager = TrackingManager(
    gazebo_bridge,
    ALLOWED_DRONES,
    gimbal_command=command_tracking_gimbal,
    body_yaw_command=(
        None
        if range_dataset_collection_enabled
        else command_tracking_body_yaw
    ),
    follow_command=(
        None
        if range_dataset_collection_enabled
        else command_tracking_follow
    ),
    motion_command=(
        None
        if range_dataset_collection_enabled
        else command_tracking_motion
    ),
    attitude_command=(
        None
        if range_dataset_collection_enabled
        else command_tracking_attitude
    ),
    bootstrap_command=(
        None
        if range_dataset_collection_enabled
        else command_rgb_bootstrap_motion
    ),
    visual_target_command=(
        None
        if range_dataset_collection_enabled
        else command_visual_follow_target
    ),
    pose_provider=tracking_visual_pose,
    ground_truth_provider=range_ground_truth_provider,
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
    bootstrap_direction: str | None = None
    corridor_confirmed: bool = False


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

    with state_lock:
        swarm_state_store.record_command(drone_id, message)

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
            (TRACKING_POSE_TOPIC, 0),
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
    received_monotonic_s = time.monotonic()
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
    payload["dashboard_received_monotonic_s"] = received_monotonic_s

    topic = str(message.topic)

    with state_lock:
        if topic.endswith("/tracking/pose"):
            tracking_fast_pose_received_monotonic[drone_id] = (
                received_monotonic_s
            )
            record_tracking_vehicle_pose(
                drone_id,
                payload,
                received_monotonic_s,
            )
            bridge_status = payload.get("visual_follow_bridge")
            if isinstance(bridge_status, dict):
                tracking_visual_follow_bridge_status[drone_id] = copy.deepcopy(
                    bridge_status
                )
            peer_state_status = payload.get("peer_state_status")
            if isinstance(peer_state_status, dict):
                tracking_peer_state_status[drone_id] = copy.deepcopy(
                    peer_state_status
                )
            companion_safety = payload.get("companion_safety")
            if isinstance(companion_safety, dict):
                tracking_companion_safety[drone_id] = copy.deepcopy(
                    companion_safety
                )
                # Only frames with a live nominal, i.e. where there was a
                # real command for the barrier to correct. Parked with no
                # mission the nominal is zero and inactive, yet the gate
                # still reports intervened: the geofence floor adds ~0.4 m/s
                # upward to a vehicle sitting at z=0. Counting those put a
                # parked drone at 100%, which is exactly the alarm this
                # number exists to make meaningful. No mission, no samples,
                # and the panel reads N/A rather than crying wolf.
                if companion_safety.get("nominal_active"):
                    tracking_cbf_intervention[drone_id].append(
                        bool(companion_safety.get("intervened"))
                    )

        elif topic.endswith(
            "/telemetry/state"
        ):
            latest_drones[drone_id] = (
                payload
            )
            if not _test_telemetry_ingest_blocked(drone_id):
                swarm_state_store.ingest(
                    drone_id,
                    payload,
                    received_monotonic_s,
                )
            if (
                received_monotonic_s
                - tracking_fast_pose_received_monotonic[drone_id]
                > 0.25
            ):
                record_tracking_vehicle_pose(
                    drone_id,
                    payload,
                    received_monotonic_s,
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

app.mount(
    "/static",
    StaticFiles(directory=str(BASE_DIR / "static")),
    name="static",
)


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
    monotonic_now = time.monotonic()
    with state_lock:
        drones = copy.deepcopy(
            latest_drones
        )

        results = copy.deepcopy(
            latest_control_results
        )
        tracking_pose_streams = tracking_pose_stream_snapshot(
            monotonic_now
        )
        swarm_state = swarm_state_store.snapshot(monotonic_now)
        formation_shadow = (
            {
                drone_id: formation_shadow_controller.command(
                    drone_id,
                    swarm_state,
                ).as_dict()
                for drone_id in sorted(ALLOWED_DRONES)
            }
            if FORMATION_SHADOW_ENABLED and formation_shadow_controller is not None
            else {}
        )
        cbf_shadow = {}
        if CBF_SHADOW_ENABLED and cbf_shadow_gates:
            for drone_id in sorted(ALLOWED_DRONES):
                nominal = formation_shadow.get(drone_id, {})
                if nominal.get("active", False):
                    cbf_shadow[drone_id] = cbf_shadow_gates[drone_id].filter(
                        nominal.get("velocity_enu_m_s"),
                        swarm_state,
                    ).as_dict()
                else:
                    cbf_shadow[drone_id] = CbfCommand(
                        drone_id, (0.0, 0.0, 0.0), False,
                        "nominal_command_inactive", None, 0.0,
                    ).as_dict()
        target_formation_shadow = (
            {
                drone_id: target_formation_controller.command(
                    drone_id,
                    swarm_state.get(drone_id),
                    swarm_state.get(TARGET_FORMATION_STAND_IN_DRONE_ID),
                ).as_dict()
                for drone_id in sorted(
                    ALLOWED_DRONES - {TARGET_FORMATION_STAND_IN_DRONE_ID}
                )
            }
            if TARGET_FORMATION_SHADOW_ENABLED
            and target_formation_controller is not None
            else {}
        )

    with gimbal_lock:
        gimbal_state = copy.deepcopy(
            gimbal_angles_deg
        )

    return {
        "drones": with_cbf_status(drones),
        "mission_config": mission_runtime_config(),
        "control_results": results,
        "gimbal_angles_deg": gimbal_state,
        "gazebo": gazebo_bridge.status(),
        "tracking": tracking_manager.status(),
        "tracking_pose_streams": tracking_pose_streams,
        "swarm_state": swarm_state,
        "swarm_state_frame": "ENU",
        "swarm_state_origin_configured": SWARM_ENU_ORIGIN is not None,
        "formation": {
            "enabled": FORMATION_SHADOW_ENABLED,
            "authority": "shadow_nominal_only",
            "leader_id": FORMATION_LEADER_ID,
            "config_error": formation_shadow_config_error,
            "commands": formation_shadow,
        },
        "cbf": {
            "enabled": CBF_SHADOW_ENABLED,
            "authority": "shadow_safety_gate_only",
            "config_error": cbf_shadow_config_error,
            "commands": cbf_shadow,
        },
        "target_formation": {
            "enabled": TARGET_FORMATION_SHADOW_ENABLED,
            "authority": "shadow_nominal_only",
            "stand_in_drone_id": TARGET_FORMATION_STAND_IN_DRONE_ID,
            "note": (
                "stand_in_drone_id's own real ENU state is used as the "
                "target input for validation; this is not vision "
                "integration -- see main.py's target_formation_controller "
                "comment."
            ),
            "config_error": target_formation_config_error,
            "commands": target_formation_shadow,
        },
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
        "swarm_state_frame": "ENU",
        "swarm_state_origin_configured": SWARM_ENU_ORIGIN is not None,
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
        status = tracking_manager.set_visual_follow_requested(
            request.enabled,
            bootstrap_direction=request.bootstrap_direction,
            corridor_confirmed=request.corridor_confirmed,
        )
    except (RuntimeError, ValueError) as error:
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
    detail: dict[str, Any] | None = None,
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

    if detail:
        payload["detail"] = detail

    await safe_send_json(
        websocket,
        send_lock,
        payload,
    )


def mission_verdicts() -> dict[str, Any]:
    """Per-drone verdict on the last operator mission, for the snapshot.

    Just the verdict, not the whole companion-safety payload: the operator
    needs to see WHY a drawn path was refused, and the rest of that blob has
    no business on a per-frame snapshot. Call under `state_lock`.

    This exists as its own function because the first version read the verdict
    from a place the websocket snapshot does not carry, so the dashboard
    silently showed nothing while the bridge was refusing every mission.
    """
    return {
        drone_id: copy.deepcopy(
            (
                tracking_companion_safety[drone_id] or {}
            ).get("mission")
        )
        for drone_id in ALLOWED_DRONES
    }


def mission_runtime_config() -> dict[str, Any]:
    """Resolved limits shown by the dashboard and enforced by both validators."""
    limits = MissionLimits.from_environment()
    try:
        default_speed = float(
            os.environ.get(
                "SWARM_MISSION_DEFAULT_SPEED_M_S",
                str(limits.maximum_speed_m_s),
            )
        )
    except ValueError:
        default_speed = limits.maximum_speed_m_s
    if not math.isfinite(default_speed):
        default_speed = limits.maximum_speed_m_s
    default_speed = max(0.2, min(limits.maximum_speed_m_s, default_speed))
    return {
        "minimum_speed_m_s": 0.2,
        "maximum_speed_m_s": limits.maximum_speed_m_s,
        "default_speed_m_s": default_speed,
        "command_limit_m_s": FORMATION_MAXIMUM_VELOCITY_M_S,
        "cbf_maximum_velocity_m_s": CBF_MAXIMUM_VELOCITY_M_S,
        "minimum_separation_m": limits.minimum_separation_m,
        "cbf_rl_mode": os.environ.get("SWARM_CBF_RL_MODE", "off").strip().lower(),
    }


def tracking_pose_stream_snapshot(
    monotonic_now: float,
) -> dict[str, dict[str, Any]]:
    """Per-drone fast-pose stream state, including the companion safety frame.

    The dashboard's safety layer reads `companion_safety` from here. It used to
    be built inline in /api/drones only, so the WebSocket -- the payload the
    page actually consumes -- never carried it and the safety strip could not
    render a single number. One builder, both transports.
    """
    return {
        drone_id: {
            "source": "px4_mavlink",
            "age_ms": (
                round(
                    max(
                        0.0,
                        monotonic_now
                        - tracking_fast_pose_received_monotonic[drone_id],
                    )
                    * 1000.0,
                    2,
                )
                if tracking_fast_pose_received_monotonic[drone_id] > 0.0
                else None
            ),
            "fresh": bool(
                tracking_fast_pose_received_monotonic[drone_id] > 0.0
                and monotonic_now
                - tracking_fast_pose_received_monotonic[drone_id]
                <= 0.25
            ),
            "visual_follow_bridge": copy.deepcopy(
                tracking_visual_follow_bridge_status[drone_id]
            ),
            "peer_state": copy.deepcopy(
                tracking_peer_state_status[drone_id]
            ),
            "companion_safety": copy.deepcopy(
                tracking_companion_safety[drone_id]
            ),
        }
        for drone_id in ALLOWED_DRONES
    }


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

            missions = mission_verdicts()

            pose_streams = (
                tracking_pose_stream_snapshot(
                    time.monotonic()
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
                "drones": with_cbf_status(drones),
                "missions": missions,
                "mission_config": mission_runtime_config(),
                "control_results": (
                    control_results
                ),
                "gimbal_angles_deg": (
                    gimbal_state
                ),
                "gazebo": gazebo_bridge.status(),
                "tracking": tracking_manager.status(),
                "tracking_pose_streams": (
                    pose_streams
                ),
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

        if message_type == "mission_path":
            await handle_mission_path(
                websocket,
                send_lock,
                message,
                drone_id,
            )
            continue

        if message_type in {"mission_start", "mission_stop"}:
            await handle_mission_action(
                websocket,
                send_lock,
                message_type,
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


def mission_origin() -> GeodeticOrigin | None:
    """The shared ENU origin, read from the same environment the bridge uses."""
    try:
        return GeodeticOrigin(
            float(os.environ["SWARM_ENU_ORIGIN_LAT_DEG"]),
            float(os.environ["SWARM_ENU_ORIGIN_LON_DEG"]),
            float(os.environ["SWARM_ENU_ORIGIN_ALT_MSL_M"]),
        )
    except (KeyError, ValueError):
        return None


def _float_environment(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        return default


async def handle_mission_path(
    websocket: WebSocket,
    send_lock: asyncio.Lock,
    message: dict[str, Any],
    drone_id: str,
) -> None:
    """Validate a drawn mission and hand it to the drone's bridge.

    The same validation runs again on the bridge, which is the authoritative
    side. This copy exists so an operator who draws something unflyable is
    told immediately and in the browser, rather than watching a path vanish
    into MQTT and having to read a worker log to find out why.
    """
    origin = mission_origin()

    if origin is None:
        await send_publish_result(
            websocket,
            send_lock,
            ok=False,
            drone_id=drone_id,
            action="mission_path",
            error=(
                "Shared ENU origin is not configured"
            ),
        )
        return

    try:
        trajectory = validate_mission(
            message,
            origin,
        )
    except MissionRejected as error:
        await send_publish_result(
            websocket,
            send_lock,
            ok=False,
            drone_id=drone_id,
            action="mission_path",
            error=str(error),
        )
        return

    with state_lock:
        pending = dict(accepted_mission_paths)
    pending[drone_id] = trajectory
    review = (
        review_missions(pending)
        if len(pending) > 1
        else {"geometrically_separated": True, "conflicts": []}
    )

    ok, error = publish_control_message(
        {
            "type": "mission_path",
            "drone_id": drone_id,
            "altitude_m": message.get(
                "altitude_m"
            ),
            "speed_m_s": message.get(
                "speed_m_s"
            ),
            "waypoints": message.get(
                "waypoints"
            ),
        }
    )

    await send_publish_result(
        websocket,
        send_lock,
        ok=ok,
        drone_id=drone_id,
        action="mission_path",
        error=error,
        detail={
            "waypoints": len(
                trajectory.waypoints_enu_m
            ),
            "perimeter_m": round(
                trajectory.perimeter_m(),
                2,
            ),
            "lap_duration_s": round(
                trajectory.lap_duration_s(),
                1,
            ),
            "conflict_review": review,
            "speed_preview": mission_speed_preview(
                trajectory,
                maximum_acceleration_m_s2=_float_environment(
                    "SWARM_MISSION_MAXIMUM_ACCELERATION_M_S2", 0.5
                ),
                corner_tracking_tolerance_m=_float_environment(
                    "SWARM_MISSION_CORNER_TRACKING_TOLERANCE_M", 1.0
                ),
            ),
        },
    )

    if ok:
        with state_lock:
            accepted_mission_paths[drone_id] = trajectory


async def handle_mission_action(
    websocket: WebSocket,
    send_lock: asyncio.Lock,
    action: str,
    drone_id: str,
) -> None:
    """Route mission execution to the companion that owns OFFBOARD.

    Mission start/stop deliberately do not use the legacy ROS action path.
    The companion worker performs the authoritative armed/mode/setpoint
    readiness checks immediately before it asks PX4 to change mode.
    """
    ok, error = publish_control_message(
        {
            "type": action,
            "drone_id": drone_id,
        }
    )
    await send_publish_result(
        websocket,
        send_lock,
        ok=ok,
        drone_id=drone_id,
        action=action,
        error=error,
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
