#!/usr/bin/env python3

from __future__ import annotations

import gc
import json
import logging
import math
import os
import signal
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Iterable

os.environ.setdefault("MAVLINK20", "1")

import paho.mqtt.client as mqtt
from pymavlink import mavutil

from active_offboard_setpoint_sender import ActiveOffboardSetpointSender
from cbf_command_gate import cbf_uncertainty_source_enabled
from companion_safety import CompanionSafetyMonitor
from mission_plan import MissionRejected, validate_mission
from offboard_abort_conditions import conditions_from_status
from offboard_authority import (
    companion_active_transmit_authorized,
    legacy_writer_permitted,
    resolve_authority,
)
from offboard_setpoint_sender import ShadowOffboardSetpointSender
from peer_state import DirectPeerStateTransport, PeerStateRegistry, make_peer_state, parse_endpoints
from swarm_state import GeodeticOrigin, geodetic_to_enu, ned_to_enu, ned_variance_to_enu
from two_uav_active_readiness import is_authorized_for_two_uav_active_flight
from worker_liveness_watchdog import WorkerLivenessSample, WorkerLivenessWatchdog


MQTT_HOST = "127.0.0.1"
MQTT_PORT = 1883
MQTT_TOPIC = "swarm/+/control/command"
FAST_POSE_TOPIC_TEMPLATE = "swarm/{drone_id}/tracking/pose"
WORKER_LIVENESS_TOPIC_TEMPLATE = "swarm/{drone_id}/system/worker_liveness"

SEND_RATE_HZ = 20.0
SEND_PERIOD_S = 1.0 / SEND_RATE_HZ
FAST_POSE_MAX_SOURCE_AGE_S = 0.20
try:
    PEER_STATE_RATE_HZ = max(
        10.0,
        min(20.0, float(os.environ.get("SWARM_PEER_STATE_RATE_HZ", "20"))),
    )
except ValueError:
    PEER_STATE_RATE_HZ = 20.0
PEER_STATE_PERIOD_S = 1.0 / PEER_STATE_RATE_HZ
try:
    PEER_STATE_MAX_AGE_S = max(
        0.1,
        min(5.0, float(os.environ.get("SWARM_PEER_STATE_MAX_AGE_MS", "100")) / 1000.0),
    )
except ValueError:
    PEER_STATE_MAX_AGE_S = 0.1

# Own state gets its own budget because it is a different measurement.
# PEER_STATE_MAX_AGE_S bounds a *peer* sample: network plus processing, an age
# that varies continuously. Own state is read from the local MAVLink stream at
# PEER_STATE_RATE_HZ, so its age is quantised to whole sample periods -- 0, one
# period, two periods, nothing in between. Reusing the peer budget unchanged
# therefore gave own state no allowance for its own sampling at all, and at
# 20 Hz the limit landed exactly two periods out: one dropped message reads as
# 100.1 ms against 100 ms and latches the flight.
#
# Measured 2026-08-19 on the 20 m/s corridor: 100.1/100.2 ms peak own-state age
# with the Gazebo GUI running (flight aborted twice), 99.8 ms headless (flight
# passed). 0.2 ms of headroom decided whether a four-minute flight completed.
# One period of allowance absorbs a single dropped message and still fails on
# two consecutive ones, which is a real loss of the stream rather than a hiccup.
#
# This does not spend the safety argument, because the barrier already prices
# whatever age it is handed: cbf_command_gate's `age_latency` term multiplies
# max(own_age, peer_age) by the reserve speed inside required_margin, so a
# state that is one period older simply demands proportionally more separation.
# The latch is the cruder second guard, not the reasoning.
OWN_STATE_MAX_AGE_S = PEER_STATE_MAX_AGE_S + PEER_STATE_PERIOD_S

# Liveness is a different question from freshness and needs its own budget.
# PEER_STATE_MAX_AGE_S bounds how old a *position sample* may be. The validated
# crossing is strict-feasible only through 165 ms and first hard-fails at
# 239 ms, so 100 ms leaves offline margin. It is still far shorter than PX4's
# 1 Hz HEARTBEAT, so reusing it here declared the
# vehicle dead for roughly half of every second. Three missed heartbeats is the
# usual MAVLink convention.
try:
    VEHICLE_HEARTBEAT_TIMEOUT_S = max(
        1.0,
        min(
            30.0,
            float(os.environ.get("SWARM_VEHICLE_HEARTBEAT_TIMEOUT_S", "3.0")),
        ),
    )
except ValueError:
    VEHICLE_HEARTBEAT_TIMEOUT_S = 3.0

# How long evaluate_companion_safety() may go without producing a new frame
# before the outside watchdog (main(), not this worker) calls it stalled.
# Independent of VEHICLE_HEARTBEAT_TIMEOUT_S: that bounds PX4 HEARTBEAT age as
# observed *by* the loop; this bounds the loop's own cadence as observed *from
# outside* it, which is the one signal a frozen loop cannot report about
# itself. See worker_liveness_watchdog.py.
try:
    WORKER_LIVENESS_STALL_S = max(
        0.5,
        min(
            30.0,
            float(os.environ.get("SWARM_WORKER_LIVENESS_STALL_S", "1.5")),
        ),
    )
except ValueError:
    WORKER_LIVENESS_STALL_S = 1.5

PEER_STATE_ENABLED = os.environ.get("SWARM_PEER_STATE_ENABLED", "false").strip().lower() in {"1", "true", "yes", "on"}

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
MISSION_ACTIVE_EXECUTION_STATES = frozenset({"entering", "running", "stopping"})
PX4_MAIN_MODE_POSCTL = 3
PX4_MAIN_MODE_AUTO = 4
PX4_MAIN_MODE_OFFBOARD = 6
PX4_SUB_MODE_AUTO_LOITER = 3
PX4_SUB_MODE_AUTO_FOLLOW_TARGET = 8
# COMMAND_ACK results that count as PX4 having accepted a mode request.
# Taken from pymavlink here rather than mirrored, because this module already
# depends on it; offboard_abort_conditions keeps its own plain-int default so
# it stays testable without the transport library.
OFFBOARD_ACCEPTED_ACK_RESULTS = frozenset(
    {
        int(mavutil.mavlink.MAV_RESULT_ACCEPTED),
        int(mavutil.mavlink.MAV_RESULT_IN_PROGRESS),
    }
)
VISUAL_FOLLOW_TIMEOUT_S = 0.45
try:
    VISUAL_FOLLOW_HOLD_TIMEOUT_S = max(
        VISUAL_FOLLOW_TIMEOUT_S,
        min(
            3.0,
            float(os.environ.get("SWARM_VISUAL_FOLLOW_HOLD_TIMEOUT_S", "1.50")),
        ),
    )
    VISUAL_FOLLOW_VELOCITY_RAMP_S = max(
        0.25,
        min(
            5.0,
            float(os.environ.get("SWARM_VISUAL_FOLLOW_VELOCITY_RAMP_S", "1.50")),
        ),
    )
    VISUAL_FOLLOW_MAX_TARGET_SPEED_M_S = max(
        0.1,
        min(
            0.9,
            float(
                os.environ.get(
                    "SWARM_VISUAL_FOLLOW_MAX_TARGET_SPEED_M_S",
                    "0.50",
                )
            ),
        ),
    )
    VISUAL_FOLLOW_MAX_TARGET_VERTICAL_SPEED_M_S = max(
        0.05,
        min(
            0.5,
            float(
                os.environ.get(
                    "SWARM_VISUAL_FOLLOW_MAX_TARGET_VERTICAL_SPEED_M_S",
                    "0.20",
                )
            ),
        ),
    )
except ValueError:
    VISUAL_FOLLOW_HOLD_TIMEOUT_S = 1.50
    VISUAL_FOLLOW_VELOCITY_RAMP_S = 1.50
    VISUAL_FOLLOW_MAX_TARGET_SPEED_M_S = 0.50
    VISUAL_FOLLOW_MAX_TARGET_VERTICAL_SPEED_M_S = 0.20
VISUAL_FOLLOW_PRESTREAM_FRAMES = 10
VISUAL_FOLLOW_SETTLE_FRAMES = 6
VISUAL_FOLLOW_SETTLE_MAX_XY_SPEED_M_S = 0.15
SAFE_NATIVE_FOLLOW_MODE_REQUEST_ENABLED = os.environ.get(
    "SWARM_SAFE_NATIVE_FOLLOW_MODE_REQUEST_ENABLED",
    "false",
).strip().lower() in {"1", "true", "yes", "on"}
LEGACY_AUTOMATION_ENABLED = os.environ.get(
    "SWARM_LEGACY_AUTOMATION_ENABLED",
    "false",
).strip().lower() in {"1", "true", "yes", "on"}
# Operator-drawn missions may be installed into the live nominal path only
# when this is explicitly on. Default off matches every other runtime
# capability in this stack: the dashboard can draw, validate and preview a
# mission with the flag off, and only turning it on lets one reach a vehicle.
MISSION_RUNTIME_ENABLED = os.environ.get(
    "SWARM_MISSION_RUNTIME_ENABLED",
    "false",
).strip().lower() in {"1", "true", "yes", "on"}
if LEGACY_AUTOMATION_ENABLED and SAFE_NATIVE_FOLLOW_MODE_REQUEST_ENABLED:
    raise RuntimeError(
        "legacy automation and safe native Follow entry are mutually exclusive"
    )
VISUAL_FOLLOW_PARAMETER_NAMES = (
    "FLW_TGT_DST",
    "FLW_TGT_HT",
    "FLW_TGT_FA",
    "FLW_TGT_ALT_M",
    "FLW_TGT_MAX_VEL",
    "FLW_TGT_RS",
)
VISUAL_FOLLOW_PARAMETER_REQUEST_RETRY_S = 1.0
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

try:
    PEER_STATE_ORIGIN = GeodeticOrigin(
        float(os.environ["SWARM_ENU_ORIGIN_LAT_DEG"]),
        float(os.environ["SWARM_ENU_ORIGIN_LON_DEG"]),
        float(os.environ["SWARM_ENU_ORIGIN_ALT_MSL_M"]),
    )
except (KeyError, ValueError):
    PEER_STATE_ORIGIN = None

try:
    PEER_STATE_BIND_HOST = os.environ.get("SWARM_PEER_STATE_BIND_HOST", "0.0.0.0")
    PEER_STATE_BIND_PORT = int(os.environ.get("SWARM_PEER_STATE_BIND_PORT", "14670"))
    if not 1 <= PEER_STATE_BIND_PORT <= 65535:
        raise ValueError("bind port is out of range")
    PEER_STATE_ENDPOINTS = parse_endpoints(os.environ.get("SWARM_PEER_STATE_ENDPOINTS", ""))
except ValueError as error:
    PEER_STATE_BIND_HOST = "0.0.0.0"
    PEER_STATE_BIND_PORT = 14670
    PEER_STATE_ENDPOINTS = ()
    PEER_STATE_CONFIG_ERROR = str(error)
else:
    PEER_STATE_CONFIG_ERROR = "" if PEER_STATE_ENDPOINTS else "no peer endpoints configured"


COMPANION_SAFETY_ENABLED = os.environ.get(
    "SWARM_COMPANION_SAFETY_ENABLED", "false"
).strip().lower() in {"1", "true", "yes", "on"}

# SITL/test-only fault-injection hook, companion side. Inert unless
# SWARM_TEST_COMPANION_TELEMETRY_BLOCK_FILE is set to a path. Mirrors
# main.py's SWARM_TEST_TELEMETRY_BLOCK_FILE (which withholds a drone's
# telemetry from the SERVER's SwarmState), but that hook has no effect here:
# the companion's FastPoseCache is fed directly from this worker's own
# MAVLink socket, never through the server or MQTT. While a drone_id is
# written to this file, that worker stops feeding new MAVLink messages into
# its own FastPoseCache -- position/attitude/global-position all age
# naturally toward staleness -- while HEARTBEAT handling (a few lines below,
# outside this hook) keeps running, so the vehicle still reads as "healthy"
# while its position telemetry goes stale. Clearing the file resumes
# ingestion immediately, no restart required.
SWARM_TEST_COMPANION_TELEMETRY_BLOCK_FILE = os.environ.get(
    "SWARM_TEST_COMPANION_TELEMETRY_BLOCK_FILE", ""
).strip()


def companion_telemetry_ingest_blocked(drone_id: str) -> bool:
    if not SWARM_TEST_COMPANION_TELEMETRY_BLOCK_FILE:
        return False
    try:
        with open(SWARM_TEST_COMPANION_TELEMETRY_BLOCK_FILE, "r", encoding="utf-8") as f:
            return f.read().strip() == drone_id
    except OSError:
        return False

# Optional JSONL trace of the companion safety stack. It exists because the
# normal way to observe this bridge is the server's /api/drones, and the whole
# point of running CBF here is to keep working when that server is gone.
COMPANION_SAFETY_LOG_PATH = os.environ.get("SWARM_COMPANION_SAFETY_LOG", "").strip()

# An iteration slower than this gets one WARNING naming where the time went.
# Five real flights on 2026-08-11 hit 0.25-0.57 s iterations, which is past
# WarmupContract.maximum_gap_s (0.25 s), so the active sender latched
# `setpoint_stream_gap` and aborted the flight. The trace those flights left
# behind can only bound the stall, not locate it: `evaluated_monotonic_s` is
# stamped at the top of the loop and `wall_clock_s` in the middle, and every
# stall showed a normal interval between the two -- so the time went somewhere
# in the second half of the body, which nothing was measuring. Both workers
# stalled within 3 ms of each other every time, so whatever it is holds the
# GIL or a lock they share. Default-on: the cost is one perf_counter per
# section on a loop that already does file and network I/O per frame, and the
# alternative is another armed flight spent narrowing it down.
SLOW_ITERATION_S = float(os.environ.get("SWARM_BRIDGE_SLOW_ITERATION_S", "0.15"))


class GcPauseProbe:
    """Records stop-the-world cyclic-GC pauses, the leading suspect for a
    stall that hits every thread at once. Costs nothing when GC is quiet:
    two callbacks per collection, and only the last few are kept."""

    def __init__(self, keep: int = 8) -> None:
        self.keep = keep
        self.started_at = 0.0
        self.pauses: list[tuple[float, float, int]] = []  # (end, seconds, gen)

    def __call__(self, phase: str, info: dict[str, Any]) -> None:
        if phase == "start":
            self.started_at = time.monotonic()
            return
        now = time.monotonic()
        self.pauses.append((now, now - self.started_at, int(info.get("generation", -1))))
        del self.pauses[: -self.keep]

    def during(self, since: float) -> list[tuple[float, float, int]]:
        return [entry for entry in self.pauses if entry[0] >= since]


GC_PAUSE_PROBE = GcPauseProbe()


def build_companion_safety(drone_id: str) -> CompanionSafetyMonitor | None:
    if not COMPANION_SAFETY_ENABLED:
        return None
    try:
        return CompanionSafetyMonitor.from_environment(drone_id, tuple(VEHICLES))
    except (TypeError, ValueError) as error:
        LOGGER.error("Companion safety disabled for %s: %s", drone_id, error)
        return None


def build_offboard_setpoint_sender(
    drone_id: str, px4_system_id: int
) -> ShadowOffboardSetpointSender | None:
    """Shadow setpoint preview for a companion that already has a safety stack.

    Velocity limit and command-age budget are taken from the same environment
    variables the safety stack itself reads, so the preview cannot disagree
    with the command it is previewing. No new threshold is introduced: the
    age budget reuses SWARM_PEER_STATE_MAX_AGE_MS, the existing freshness
    contract for companion-local state.
    """
    try:
        return ShadowOffboardSetpointSender(
            drone_id=drone_id,
            px4_system_id=px4_system_id,
            maximum_velocity_m_s=float(
                os.environ.get("SWARM_CBF_MAXIMUM_VELOCITY_M_S", "2.0")
            ),
            maximum_command_age_s=PEER_STATE_MAX_AGE_S,
        )
    except (TypeError, ValueError) as error:
        LOGGER.error("Offboard setpoint preview disabled for %s: %s", drone_id, error)
        return None


def build_active_offboard_setpoint_sender(
    drone_id: str,
    px4_system_id: int,
    transmit: Any = None,
    request_position_mode: Any = None,
) -> ActiveOffboardSetpointSender | None:
    """Build the active sender, with a sink only for the authorized vehicle.

    The sink is withheld unless BOTH gates `ActiveOffboardSetpointSender`
    itself re-checks on every frame also clear here: the authority-level
    `companion_active_transmit_authorized` (names which vehicles are ever
    eligible, via `offboard_authority.ACTIVE_FLIGHT_AUTHORIZED_VEHICLES`) and
    the readiness-level `is_authorized_for_two_uav_active_flight` (names
    which vehicle the TWO_UAV_ACTIVE_SITL_FLIGHT contract has cleared --
    UAV-01 and UAV-02, both requiring their own system id and opt-in;
    ONE_UAV_ACTIVE_SITL_FLIGHT's narrower one-vehicle gate in
    one_uav_active_readiness is unchanged but no longer the one consulted
    here). A sender that is not authorized by both therefore cannot transmit
    even if every later gate were somehow bypassed: it has nothing to call.

    Requiring both here, not just the first, matters beyond "checked twice":
    `holds_offboard_authority` (consulted by the legacy recovery path) goes
    true as soon as a sink is attached, before any frame is ever stepped. A
    vehicle cleared by only one gate would report holding OFFBOARD while its
    own per-frame check permanently withholds every setpoint -- silencing the
    legacy recovery path for a vehicle nothing is actually driving.

    Both checks are duplicated -- the sender re-runs them on every frame --
    and deliberately so. Here they decide whether the capability exists at
    all; there they decide whether this particular frame may use it. The
    second catches an authority or readiness state that changes while the
    process runs; the first ensures an unauthorized vehicle never holds a
    transmit path in the first place.

    `explicit_opt_in` follows the same gates rather than an environment
    variable, so authorizing a vehicle for active flight stays a committed
    source change, not a runtime flag flip.
    """
    permitted, reason = companion_active_transmit_authorized(drone_id, px4_system_id)
    if permitted:
        # Same `explicit_opt_in` value the readiness gate will see at
        # runtime: `ActiveOffboardSetpointSender` is constructed below with
        # `explicit_opt_in=permitted`, so probing it with that same value
        # here reproduces exactly what step()'s own check will decide.
        cleared, readiness_reason = is_authorized_for_two_uav_active_flight(
            drone_id, px4_system_id, permitted
        )
        if not cleared:
            permitted = False
            reason = f"readiness_gate_refused:{readiness_reason}"
    if not permitted:
        LOGGER.info(
            "Active offboard sender for %s has no transmit sink: %s", drone_id, reason
        )
    try:
        return ActiveOffboardSetpointSender(
            drone_id=drone_id,
            px4_system_id=px4_system_id,
            explicit_opt_in=permitted,
            transmit=transmit if permitted else None,
            # Wired unconditionally: handing control back to PX4's Position
            # mode is the abort action, and an unauthorized sender that
            # somehow reached an abort should still be able to ask for it.
            request_position_mode=request_position_mode,
        )
    except (TypeError, ValueError) as error:
        LOGGER.error("Active offboard sender disabled for %s: %s", drone_id, error)
        return None


def peer_state_env_suffix(drone_id: str) -> str:
    """Environment-variable suffix for a drone id, e.g. UAV-01 -> UAV_01."""
    return "".join(
        character if character.isalnum() else "_" for character in drone_id.strip().upper()
    )


def peer_state_config_for(drone_id: str) -> tuple[str, int, tuple[tuple[str, int], ...], str]:
    """Resolve one drone's own peer-state socket.

    On real hardware each companion is a separate host, so a single bind port
    and one endpoint list suffice -- that is the shared configuration above.
    In SITL every companion shares one host, so each needs its own port. When
    the per-drone variables are absent this falls back to the shared values,
    which keeps the previous single-socket behaviour byte-for-byte.
    """
    suffix = peer_state_env_suffix(drone_id)
    raw_port = os.environ.get(f"SWARM_PEER_STATE_BIND_PORT_{suffix}", "").strip()
    raw_endpoints = os.environ.get(f"SWARM_PEER_STATE_ENDPOINTS_{suffix}", "").strip()
    if not raw_port and not raw_endpoints:
        return (
            PEER_STATE_BIND_HOST,
            PEER_STATE_BIND_PORT,
            PEER_STATE_ENDPOINTS,
            PEER_STATE_CONFIG_ERROR,
        )
    host = os.environ.get(
        f"SWARM_PEER_STATE_BIND_HOST_{suffix}", PEER_STATE_BIND_HOST
    )
    try:
        port = int(raw_port) if raw_port else PEER_STATE_BIND_PORT
        if not 1 <= port <= 65535:
            raise ValueError("bind port is out of range")
        endpoints = (
            parse_endpoints(raw_endpoints) if raw_endpoints else PEER_STATE_ENDPOINTS
        )
    except ValueError as error:
        return PEER_STATE_BIND_HOST, PEER_STATE_BIND_PORT, (), str(error)
    return (
        host,
        port,
        endpoints,
        "" if endpoints else "no peer endpoints configured",
    )


def build_peer_state_transports(drone_ids: Iterable[str]) -> dict[str, DirectPeerStateTransport]:
    """One transport per drone, so each worker keeps its own neighbour table.

    A drone whose socket cannot be opened is simply absent from the result:
    its worker then runs without peer state rather than taking the whole
    bridge down, matching how the rest of this bridge degrades.
    """
    transports: dict[str, DirectPeerStateTransport] = {}
    for drone_id in drone_ids:
        host, port, endpoints, error = peer_state_config_for(drone_id)
        if PEER_STATE_ORIGIN is None:
            LOGGER.error(
                "Peer state disabled for %s: common ENU origin is not configured",
                drone_id,
            )
            continue
        if error:
            LOGGER.error("Peer state disabled for %s: %s", drone_id, error)
            continue
        try:
            transport = DirectPeerStateTransport(
                host,
                port,
                endpoints,
                PeerStateRegistry(max_age_s=PEER_STATE_MAX_AGE_S),
            )
        except (OSError, ValueError) as setup_error:
            LOGGER.error(
                "Peer state disabled for %s: UDP setup failed: %s",
                drone_id,
                setup_error,
            )
            continue
        transport.start()
        transports[drone_id] = transport
        LOGGER.info(
            "Direct peer state enabled for %s at %.1f Hz; bind=%s:%d peers=%s",
            drone_id,
            PEER_STATE_RATE_HZ,
            host,
            port,
            endpoints,
        )
    return transports


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
class FastPoseCache:
    """Latest PX4-local pose samples received on the control MAVLink link."""

    position_ned_m: tuple[float, float, float] | None = None
    velocity_ned_m_s: tuple[float, float, float] | None = None
    quaternion_xyzw: tuple[float, float, float, float] | None = None
    local_received_monotonic_s: float = 0.0
    attitude_received_monotonic_s: float = 0.0
    direct_quaternion_received_monotonic_s: float = 0.0
    home_z_down_m: float | None = None
    global_position: tuple[float, float, float] | None = None
    global_received_monotonic_s: float = 0.0
    position_variance_ned_m2: tuple[float, float, float] | None = None
    odometry_received_monotonic_s: float = 0.0
    odometry_reset_counter: int | None = None
    odometry_sample_count: int = 0
    odometry_covariance_sample_count: int = 0

    def update(self, message: Any, timestamp_s: float) -> None:
        message_type = str(message.get_type())
        if message_type == "LOCAL_POSITION_NED":
            position = (
                float(message.x),
                float(message.y),
                float(message.z),
            )
            velocity = (
                float(message.vx),
                float(message.vy),
                float(message.vz),
            )
            if all(math.isfinite(value) for value in position + velocity):
                self.position_ned_m = position
                self.velocity_ned_m_s = velocity
                self.local_received_monotonic_s = float(timestamp_s)
            return
        if message_type == "ODOMETRY":
            self._update_odometry(message, timestamp_s)
            return
        if message_type == "HOME_POSITION":
            home_z_down_m = float(message.z)
            if math.isfinite(home_z_down_m):
                self.home_z_down_m = home_z_down_m
            return
        if message_type == "GLOBAL_POSITION_INT":
            position = (
                float(message.lat) / 1.0e7,
                float(message.lon) / 1.0e7,
                float(message.alt) / 1000.0,
            )
            if (
                -90.0 <= position[0] <= 90.0
                and -180.0 <= position[1] <= 180.0
                and all(math.isfinite(value) for value in position)
            ):
                self.global_position = position
                self.global_received_monotonic_s = float(timestamp_s)
            return
        if message_type == "ATTITUDE_QUATERNION":
            raw = (
                float(message.q2),
                float(message.q3),
                float(message.q4),
                float(message.q1),
            )
            norm = math.sqrt(sum(value * value for value in raw))
            if norm > 1e-9 and all(math.isfinite(value) for value in raw):
                self.quaternion_xyzw = tuple(
                    value / norm for value in raw
                )
                self.attitude_received_monotonic_s = float(timestamp_s)
                self.direct_quaternion_received_monotonic_s = float(timestamp_s)
            return
        if message_type == "ATTITUDE":
            # Exact Euler-to-quaternion conversion is only a fallback when PX4
            # does not stream ATTITUDE_QUATERNION; downstream composition stays
            # quaternion-based and never linearly adds Euler angles.
            if (
                float(timestamp_s)
                - self.direct_quaternion_received_monotonic_s
                <= FAST_POSE_MAX_SOURCE_AGE_S
            ):
                return
            roll = float(message.roll)
            pitch = float(message.pitch)
            yaw = float(message.yaw)
            values = (roll, pitch, yaw)
            if not all(math.isfinite(value) for value in values):
                return
            cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
            cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
            cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
            self.quaternion_xyzw = (
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy,
            )
            self.attitude_received_monotonic_s = float(timestamp_s)

    def _update_odometry(self, message: Any, timestamp_s: float) -> None:
        """Cache PX4's own EKF position variance (m^2), straight from ODOMETRY.

        Only the diagonal is read. PX4 fills the off-diagonal entries of the
        packed upper-triangular 6x6 `pose_covariance` with NaN, so summing the
        array would poison it; indices 0/6/11 are the x/y/z diagonal. The frame
        check keeps the value in LOCAL_NED, the frame the position the CBF uses
        actually lives in. No unit conversion: the field is already variance in
        m^2, which is what `position_covariance_m2` means downstream.
        """
        self.odometry_sample_count += 1
        try:
            if int(message.frame_id) != mavutil.mavlink.MAV_FRAME_LOCAL_NED:
                return
            reset_counter = int(getattr(message, "reset_counter", 0))
            variance = tuple(
                float(message.pose_covariance[index]) for index in (0, 6, 11)
            )
        except (AttributeError, IndexError, TypeError, ValueError):
            return
        if reset_counter != self.odometry_reset_counter:
            # Drop only the pre-reset cache. The covariance parsed above belongs
            # to this ODOMETRY sample and can replace it immediately when valid.
            self.odometry_reset_counter = reset_counter
            self.position_variance_ned_m2 = None
        if not all(math.isfinite(value) and value >= 0.0 for value in variance):
            return
        self.position_variance_ned_m2 = variance
        self.odometry_received_monotonic_s = float(timestamp_s)
        self.odometry_covariance_sample_count += 1

    def position_covariance_age_ms(self, timestamp_s: float) -> float | None:
        if (
            self.position_variance_ned_m2 is None
            or self.odometry_received_monotonic_s <= 0.0
        ):
            return None
        return round(
            max(0.0, float(timestamp_s) - self.odometry_received_monotonic_s)
            * 1000.0,
            2,
        )

    def position_covariance_enu_m2(
        self, timestamp_s: float
    ) -> tuple[float, float, float] | None:
        """ENU position variance for the CBF, or None when it must not be used.

        None whenever the feature is off, no ODOMETRY has been accepted since
        the last EKF reset, or the sample is older than the existing
        peer-state freshness budget (no new threshold is introduced). The
        consumer decides what None means: with the feature on,
        `CbfConfig.require_position_covariance` turns it into a hold instead
        of a silent "perfectly certain" zero.
        """
        if (
            not cbf_uncertainty_source_enabled()
            or self.position_variance_ned_m2 is None
        ):
            return None
        age_s = max(0.0, timestamp_s - self.odometry_received_monotonic_s)
        if age_s > PEER_STATE_MAX_AGE_S:
            return None
        return ned_variance_to_enu(self.position_variance_ned_m2)

    def payload(
        self,
        drone_id: str,
        timestamp_s: float,
    ) -> dict[str, Any] | None:
        if (
            self.position_ned_m is None
            or self.velocity_ned_m_s is None
            or self.quaternion_xyzw is None
        ):
            return None
        local_age_s = timestamp_s - self.local_received_monotonic_s
        attitude_age_s = timestamp_s - self.attitude_received_monotonic_s
        if (
            local_age_s < 0.0
            or attitude_age_s < 0.0
            or local_age_s > FAST_POSE_MAX_SOURCE_AGE_S
            or attitude_age_s > FAST_POSE_MAX_SOURCE_AGE_S
        ):
            return None
        x, y, z = self.position_ned_m
        vx, vy, vz = self.velocity_ned_m_s
        qx, qy, qz, qw = self.quaternion_xyzw
        heading_rad = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        return {
            "type": "tracking_pose",
            "drone_id": drone_id,
            "source": "px4_mavlink",
            "source_timestamp_us": int(time.time() * 1_000_000.0),
            "local_position": {
                "x_north_m": x,
                "y_east_m": y,
                "z_down_m": z,
                "vx_m_s": vx,
                "vy_m_s": vy,
                "vz_m_s": vz,
                "heading_rad": heading_rad,
            },
            "quaternion_xyzw": [qx, qy, qz, qw],
            "local_source_age_ms": round(local_age_s * 1000.0, 2),
            "attitude_source_age_ms": round(attitude_age_s * 1000.0, 2),
        }

    def peer_payload(
        self,
        drone_id: str,
        timestamp_s: float,
        origin: GeodeticOrigin | None,
        sequence: int,
        healthy: bool,
    ) -> dict[str, Any] | None:
        """Build a common-ENU packet only when both input streams are fresh."""
        if (
            origin is None
            or self.global_position is None
            or self.velocity_ned_m_s is None
        ):
            return None
        # Clamped at zero because the worker takes `timestamp_s` at the top of
        # its loop and refreshes the cache further down the same iteration, so
        # a fresh sample reads as marginally negative age. Rejecting that made
        # this method return None on every call, which silently prevented the
        # peer-to-peer link from ever publishing.
        local_age_s = max(0.0, timestamp_s - self.local_received_monotonic_s)
        global_age_s = max(0.0, timestamp_s - self.global_received_monotonic_s)
        if (
            local_age_s > PEER_STATE_MAX_AGE_S
            or global_age_s > PEER_STATE_MAX_AGE_S
        ):
            return None
        latitude_deg, longitude_deg, altitude_msl_m = self.global_position
        try:
            position_enu_m = geodetic_to_enu(
                latitude_deg,
                longitude_deg,
                altitude_msl_m,
                origin,
            )
            velocity_enu_m_s = ned_to_enu(self.velocity_ned_m_s)
            return make_peer_state(
                drone_id=drone_id,
                sequence=sequence,
                position_enu_m=position_enu_m,
                velocity_enu_m_s=velocity_enu_m_s,
                healthy=healthy,
                position_covariance_m2=self.position_covariance_enu_m2(timestamp_s),
            )
        except ValueError:
            return None

    def own_swarm_state(
        self,
        timestamp_s: float,
        origin: GeodeticOrigin | None,
        healthy: bool,
    ) -> dict[str, Any]:
        """This drone's own state in the shape the CBF gate consumes.

        Mirrors `peer_payload` but always returns a dict: an unusable state has
        to reach the filter as an explicit invalid entry, because a missing key
        and a stale key must both fail closed rather than look like "no peer".
        """
        if (
            origin is None
            or self.global_position is None
            or self.velocity_ned_m_s is None
        ):
            return {"valid": False, "reason": "no_local_state", "message_age_ms": None}
        # Callers pass the timestamp taken at the top of their loop, while the
        # cache is refreshed later in the same iteration, so a sample can look
        # very slightly newer than "now". Clamp at zero, as PeerStateRegistry
        # does, instead of treating a few milliseconds as a clock fault.
        local_age_s = max(0.0, timestamp_s - self.local_received_monotonic_s)
        global_age_s = max(0.0, timestamp_s - self.global_received_monotonic_s)
        age_ms = max(local_age_s, global_age_s) * 1000.0
        latitude_deg, longitude_deg, altitude_msl_m = self.global_position
        try:
            position_enu_m = geodetic_to_enu(
                latitude_deg, longitude_deg, altitude_msl_m, origin
            )
            velocity_enu_m_s = ned_to_enu(self.velocity_ned_m_s)
        except ValueError:
            return {"valid": False, "reason": "enu_transform_failed", "message_age_ms": round(age_ms, 2)}
        stale = age_ms > OWN_STATE_MAX_AGE_S * 1000.0
        return {
            "drone_id": None,
            "frame": "ENU",
            "position_enu_m": list(position_enu_m),
            "velocity_enu_m_s": list(velocity_enu_m_s),
            "position_covariance_m2": self.position_covariance_enu_m2(timestamp_s),
            "message_age_ms": round(age_ms, 2),
            "valid": bool(healthy and not stale),
            "reason": (
                "ok"
                if healthy and not stale
                else "telemetry_stale"
                if stale
                else "vehicle_unhealthy"
            ),
        }


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
    follow_angle_deg: float = 180.0
    position_covariance: tuple[float, float, float] = (
        100.0,
        100.0,
        100.0,
    )
    est_capabilities: int = 1
    measurement_timestamp_s: float = 0.0
    session_id: int = 0
    source_kind: str = ""
    target_semantics: str = ""
    last_rejection_reason: str = "not_initialized"
    last_command_time: float = 0.0
    activated_monotonic_s: float = 0.0
    lock: threading.Lock = field(default_factory=threading.Lock)

    def _reject_locked(self, reason: str, now: float) -> None:
        self.enabled = False
        self.valid = False
        self.activated_monotonic_s = 0.0
        self.last_command_time = now
        self.last_rejection_reason = reason

    def update(self, payload: dict[str, Any]) -> None:
        with self.lock:
            requested_enabled = bool(payload.get("enabled", False))
            if not requested_enabled:
                self._reject_locked("disabled", time.monotonic())
                return
            try:
                now = time.monotonic()
                session_id = int(payload["session_id"])
                source_kind = str(payload["source_kind"]).strip().lower()
                target_semantics = str(
                    payload["target_semantics"]
                ).strip().lower()
                measurement_timestamp_s = float(
                    payload["measurement_timestamp_s"]
                )
                measurement_age_s = now - measurement_timestamp_s
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
                    float(payload.get("follow_angle_deg", 180.0)),
                )
                covariance = tuple(
                    float(value)
                    for value in payload.get(
                        "position_covariance",
                        (100.0, 100.0, 100.0),
                    )
                )
                capabilities = int(payload.get("est_capabilities", 1))
                forbidden_source_tokens = (
                    "selection",
                    "provisional",
                    "apparent",
                    "bootstrap",
                    "vehicle",
                    "drone",
                )
                candidate_valid = bool(
                    requested_enabled
                    and session_id > 0
                    and target_semantics == "measured_target"
                    and bool(source_kind)
                    and not any(
                        token in source_kind
                        for token in forbidden_source_tokens
                    )
                    and all(math.isfinite(value) for value in values)
                    and -90.0 <= values[0] <= 90.0
                    and -180.0 <= values[1] <= 180.0
                    and 0.0 <= values[6] <= 1.0
                    and 3.0 <= values[7] <= 20.0
                    and 1.0 <= values[8] <= 30.0
                    and -180.0 <= values[9] <= 180.0
                    and len(covariance) == 3
                    and all(
                        math.isfinite(value) and value >= 0.0
                        for value in covariance
                    )
                    and 1 <= capabilities <= 255
                    and -0.10
                    <= measurement_age_s
                    <= VISUAL_FOLLOW_TIMEOUT_S
                    and not (values[0] == 0.0 and values[1] == 0.0)
                    and (
                        session_id > self.session_id
                        or (
                            session_id == self.session_id
                            and measurement_timestamp_s
                            > self.measurement_timestamp_s
                        )
                    )
                )
                if candidate_valid:
                    if not self.enabled or not self.valid:
                        self.activated_monotonic_s = now
                    self.enabled = True
                    self.valid = True
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
                        self.follow_angle_deg,
                    ) = values
                    self.position_covariance = covariance
                    self.est_capabilities = capabilities
                    self.measurement_timestamp_s = measurement_timestamp_s
                    self.session_id = session_id
                    self.source_kind = source_kind
                    self.target_semantics = target_semantics
                    self.last_rejection_reason = ""
                    self.last_command_time = now
                    return
                self._reject_locked("candidate_contract_invalid", now)
            except KeyError as error:
                self._reject_locked(
                    f"missing_field:{error.args[0]}",
                    time.monotonic(),
                )
            except (TypeError, ValueError) as error:
                self._reject_locked(
                    f"invalid_field:{type(error).__name__}",
                    time.monotonic(),
                )

    def disable(self) -> None:
        with self.lock:
            self._reject_locked("disabled", time.monotonic())

    def output(
        self,
    ) -> tuple[
        float, float, float, float, float, float, float, float, float, float,
        tuple[float, float, float], int, bool
    ]:
        with self.lock:
            now = time.monotonic()
            command_age_s = now - self.last_command_time
            measurement_age_s = now - self.measurement_timestamp_s
            active = bool(
                self.enabled
                and self.valid
                and command_age_s <= VISUAL_FOLLOW_TIMEOUT_S
                and -0.10
                <= measurement_age_s
                <= VISUAL_FOLLOW_TIMEOUT_S
            )
            fresh = bool(
                active
                and command_age_s <= VISUAL_FOLLOW_TIMEOUT_S
                and measurement_age_s <= VISUAL_FOLLOW_TIMEOUT_S
            )
            velocity_north = self.velocity_north_m_s if fresh else 0.0
            velocity_east = self.velocity_east_m_s if fresh else 0.0
            velocity_down = self.velocity_down_m_s if fresh else 0.0
            capabilities = (
                self.est_capabilities
                if fresh
                else self.est_capabilities & ~2
            )
            horizontal_speed = math.hypot(velocity_north, velocity_east)
            if horizontal_speed > VISUAL_FOLLOW_MAX_TARGET_SPEED_M_S:
                scale = VISUAL_FOLLOW_MAX_TARGET_SPEED_M_S / horizontal_speed
                velocity_north *= scale
                velocity_east *= scale
            velocity_down = max(
                -VISUAL_FOLLOW_MAX_TARGET_VERTICAL_SPEED_M_S,
                min(
                    VISUAL_FOLLOW_MAX_TARGET_VERTICAL_SPEED_M_S,
                    velocity_down,
                ),
            )
            ramp = min(
                1.0,
                max(
                    0.0,
                    (now - self.activated_monotonic_s)
                    / VISUAL_FOLLOW_VELOCITY_RAMP_S,
                ),
            )
            return (
                self.latitude_deg,
                self.longitude_deg,
                self.altitude_msl_m,
                velocity_north * ramp,
                velocity_east * ramp,
                velocity_down * ramp,
                self.quality,
                self.follow_distance_m,
                self.follow_height_m,
                self.follow_angle_deg,
                self.position_covariance,
                capabilities,
                active,
            )

    def status(self) -> dict[str, Any]:
        with self.lock:
            now = time.monotonic()
            command_age_s = max(0.0, now - self.last_command_time)
            measurement_age_s = max(
                0.0,
                now - self.measurement_timestamp_s,
            )
            if not self.enabled or not self.valid:
                timeout_state = "INACTIVE"
            elif (
                command_age_s <= VISUAL_FOLLOW_TIMEOUT_S
                and measurement_age_s <= VISUAL_FOLLOW_TIMEOUT_S
            ):
                timeout_state = "FRESH"
            else:
                timeout_state = "STALE"
            return {
                "enabled": self.enabled,
                "valid": self.valid,
                "command_age_ms": command_age_s * 1000.0,
                "measurement_age_ms": measurement_age_s * 1000.0,
                "timeout_state": timeout_state,
                "est_capabilities": self.est_capabilities,
                "safe_distance_m": self.follow_distance_m,
                "session_id": self.session_id,
                "source_kind": self.source_kind,
                "target_semantics": self.target_semantics,
                "rejection_reason": self.last_rejection_reason,
            }


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
        mqtt_client: mqtt.Client,
        stop_event: threading.Event,
        peer_state_transport: DirectPeerStateTransport | None = None,
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
        self.mqtt_client = mqtt_client
        self.stop_event = stop_event
        self.peer_state_transport = peer_state_transport
        # Local safety stack. It needs peer state, so it only exists when the
        # direct peer-to-peer link is up; without neighbours there is nothing
        # for a collision-avoidance filter to reason about.
        self.companion_safety = (
            build_companion_safety(drone_id)
            if peer_state_transport is not None
            else None
        )
        self.companion_safety_status: dict[str, Any] | None = None
        # Verdict on the last operator mission, published with the companion
        # safety payload so the dashboard can show WHY a drawn path was
        # refused instead of silently ignoring it.
        self.mission_status: dict[str, Any] = {
            "runtime_enabled": MISSION_RUNTIME_ENABLED,
            "state": "none",
            "reason": "",
            "execution_state": "idle",
            "execution_reason": "",
            "start_ready": False,
            "start_block_reason": "mission is not installed",
        }
        self.mission_start_pending = False
        self.mission_offboard_expected = False
        self.mission_mode_ack_pending = False
        self.mission_start_attempts = 0
        self.mission_start_requested_monotonic = 0.0
        # Shadow-only: builds the would-be PX4 setpoint and never transmits.
        # Bound to the same worker as the safety stack it previews, and only
        # when that stack exists -- there is nothing to preview otherwise.
        self.offboard_setpoint_sender = (
            build_offboard_setpoint_sender(drone_id, expected_system_id)
            if self.companion_safety is not None
            else None
        )
        # Sink attached only for the authorized vehicle; see the builder.
        self.active_offboard_transmit_count = 0
        self.active_offboard_sender = (
            build_active_offboard_setpoint_sender(
                drone_id,
                expected_system_id,
                self.transmit_active_offboard_setpoint,
                self.request_position_mode,
            )
            if self.companion_safety is not None
            else None
        )
        # An abort latch is scoped to one flight. While the vehicle is
        # disarmed there is no flight to abort, so a latched sender is
        # replaced rather than left frozen -- otherwise a single transient
        # would silence the shadow output for the rest of the session.
        # Never done while armed: see reset_latched_active_sender.
        self.active_sender_reset_count = 0
        self.last_companion_evaluation_monotonic: float | None = None
        # Result of the most recent mission OFFBOARD request. Cleared after
        # entry, timeout, or abort so an old ACK cannot poison a later flight.
        self.last_offboard_mode_ack_result: int | None = None
        self.connection: Any = None
        self.offboard_mode_requested = False
        self.offboard_stream_frames = 0
        self.last_mode_request_monotonic = 0.0
        self.last_px4_main_mode: int | None = None
        self.last_px4_sub_mode: int | None = None
        # None until the first HEARTBEAT is parsed. Downstream must treat
        # None as "not known to be safe", never as "disarmed".
        self.last_px4_armed: bool | None = None
        # Counts transmits refused by the single-writer interlock.
        self.legacy_offboard_refused_count = 0
        self.offboard_request_attempts = 0
        self.offboard_entry_blocked = False
        self.offboard_input_active = False
        self.visual_follow_mode_requested = False
        self.visual_follow_stream_frames = 0
        self.visual_follow_request_attempts = 0
        self.visual_follow_entry_blocked = False
        self.visual_follow_input_active = False
        self.visual_follow_params_configured = False
        self.visual_follow_settle_frames = 0
        self.visual_follow_parameter_ack = "not_requested"
        self.visual_follow_parameter_ack_reason = ""
        self.visual_follow_parameter_ack_monotonic = 0.0
        self.visual_follow_mode_ack = "not_requested"
        self.visual_follow_mode_ack_result: int | None = None
        self.visual_follow_mode_ack_monotonic = 0.0
        self.visual_follow_exit_ack = "not_requested"
        self.visual_follow_exit_ack_result: int | None = None
        self.visual_follow_exit_ack_monotonic = 0.0
        self.visual_follow_last_publish_monotonic = 0.0
        self.visual_follow_publish_rate_hz = 0.0
        self.visual_follow_publish_count = 0
        self.last_vehicle_heartbeat_monotonic = 0.0
        self.visual_follow_parameter_values: dict[str, float] = {}
        self.visual_follow_parameter_last_request_monotonic = 0.0
        self.visual_follow_was_observed = False
        self.qgc_socket: socket.socket | None = None
        self.fast_pose_cache = FastPoseCache()
        self.peer_state_sequence = 0
        self.last_peer_state_publish_monotonic = 0.0

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

    def request_fast_pose_stream(self) -> None:
        if self.connection is None:
            return
        interval_us = int(1_000_000.0 / SEND_RATE_HZ)
        for message_id in (
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        ):
            self.connection.mav.command_long_send(
                self.expected_system_id,
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(message_id),
                float(interval_us),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
        self.connection.mav.command_long_send(
            self.expected_system_id,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            float(mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
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
        *,
        companion_authorized: bool = False,
    ) -> bool:
        if self.connection is None:
            return False
        # Requesting OFFBOARD is itself a writer action and is interlocked.
        # POSCTL is deliberately NOT interlocked: it is the recovery path
        # every abort rule depends on, and must stay available no matter
        # which authority is configured.
        companion_request = (
            companion_authorized
            and self.companion_holds_offboard()
        )
        if (
            main_mode == PX4_MAIN_MODE_OFFBOARD
            and not companion_request
            and not legacy_writer_permitted()
        ):
            self.legacy_offboard_refused_count += 1
            return False
        if (
            main_mode == PX4_MAIN_MODE_AUTO
            and sub_mode == PX4_SUB_MODE_AUTO_FOLLOW_TARGET
        ):
            if not SAFE_NATIVE_FOLLOW_MODE_REQUEST_ENABLED:
                self.visual_follow_mode_ack = "blocked_by_feature_flag"
                self.visual_follow_mode_ack_result = None
                self.visual_follow_mode_ack_monotonic = time.monotonic()
                self.visual_follow_entry_blocked = True
                LOGGER.warning(
                    "%s blocked PX4 Follow entry: feature flag is off",
                    self.drone_id,
                )
                return False
            self.visual_follow_mode_ack = "pending"
            self.visual_follow_mode_ack_result = None
            self.visual_follow_mode_ack_monotonic = 0.0
        elif main_mode == PX4_MAIN_MODE_POSCTL:
            self.visual_follow_exit_ack = "pending"
            self.visual_follow_exit_ack_result = None
            self.visual_follow_exit_ack_monotonic = 0.0
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
        return True

    def companion_holds_offboard(self) -> bool:
        """Whether the companion active sender is the thing driving OFFBOARD.

        The legacy control loop calls request_position_mode() on every idle
        iteration, and that function treats any observed OFFBOARD as a mode
        nothing is driving. That was correct while the companion could not
        transmit; once it can, the two fight, and the legacy path wins
        because it runs every loop. Measured: the first armed flight held
        OFFBOARD for 0.5 s before being pulled back to POSCTL.

        Deliberately false when the sender has latched an abort, so an
        aborted companion loses OFFBOARD to the legacy recovery instead of
        keeping it.
        """
        sender = self.active_offboard_sender
        return sender is not None and sender.holds_offboard_authority

    def request_position_mode(self) -> None:
        # An OFFBOARD the companion is actively driving is expected, not an
        # anomaly to recover from.
        offboard_observed = (
            self.last_px4_main_mode == PX4_MAIN_MODE_OFFBOARD
            and not self.companion_holds_offboard()
        )
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

    def should_request_visual_follow_mode(
        self,
        now: float,
        follow_mode_observed: bool,
    ) -> bool:
        if not SAFE_NATIVE_FOLLOW_MODE_REQUEST_ENABLED:
            return False
        if follow_mode_observed:
            self.visual_follow_was_observed = True
            return False
        if self.visual_follow_was_observed:
            self.visual_follow_entry_blocked = True
            self.visual_follow_mode_ack = "reacquire_required"
            return False
        return bool(
            self.visual_follow_stream_frames
            >= VISUAL_FOLLOW_PRESTREAM_FRAMES
            and self.visual_follow_settle_frames
            >= VISUAL_FOLLOW_SETTLE_FRAMES
            and self.visual_follow_request_attempts
            < OFFBOARD_MODE_MAX_ATTEMPTS
            and (
                not self.visual_follow_mode_requested
                or now - self.last_mode_request_monotonic
                >= OFFBOARD_MODE_RETRY_S
            )
        )

    def send_visual_follow_target(
        self,
        latitude_deg: float,
        longitude_deg: float,
        altitude_msl_m: float,
        velocity_north_m_s: float,
        velocity_east_m_s: float,
        velocity_down_m_s: float,
        quality: float,
        position_covariance: tuple[float, float, float] = (
            100.0,
            100.0,
            100.0,
        ),
        est_capabilities: int = 3,
    ) -> bool:
        assert self.connection is not None
        numeric_values = (
            latitude_deg,
            longitude_deg,
            altitude_msl_m,
            velocity_north_m_s,
            velocity_east_m_s,
            velocity_down_m_s,
            quality,
        )
        if (
            not all(math.isfinite(float(value)) for value in numeric_values)
            or not -90.0 <= float(latitude_deg) <= 90.0
            or not -180.0 <= float(longitude_deg) <= 180.0
            or (
                float(latitude_deg) == 0.0
                and float(longitude_deg) == 0.0
            )
            or not 0.0 <= float(quality) <= 1.0
            or len(position_covariance) != 3
            or not all(
                math.isfinite(float(value)) and float(value) >= 0.0
                for value in position_covariance
            )
            or not 1 <= int(est_capabilities) <= 255
        ):
            return False
        publish_now = time.monotonic()
        previous = getattr(
            self,
            "visual_follow_last_publish_monotonic",
            0.0,
        )
        if previous > 0.0 and publish_now > previous:
            instantaneous_rate = 1.0 / (publish_now - previous)
            old_rate = float(
                getattr(self, "visual_follow_publish_rate_hz", 0.0)
            )
            self.visual_follow_publish_rate_hz = (
                instantaneous_rate
                if old_rate <= 0.0
                else 0.85 * old_rate + 0.15 * instantaneous_rate
            )
        self.visual_follow_last_publish_monotonic = publish_now
        self.visual_follow_publish_count = int(
            getattr(self, "visual_follow_publish_count", 0)
        ) + 1
        self.connection.mav.follow_target_send(
            int(time.monotonic() * 1000.0),
            int(est_capabilities),
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
            [float(value) for value in position_covariance],
            0,
        )
        return True

    def configure_visual_follow_parameters(
        self,
        follow_distance_m: float,
        follow_height_m: float,
        follow_angle_deg: float = 180.0,
    ) -> bool:
        """Read and validate PX4 Follow parameters without changing them."""

        assert self.connection is not None
        del follow_distance_m, follow_height_m, follow_angle_deg
        now = time.monotonic()
        values = getattr(self, "visual_follow_parameter_values", None)
        if not isinstance(values, dict):
            values = {}
            self.visual_follow_parameter_values = values
        missing = [
            name for name in VISUAL_FOLLOW_PARAMETER_NAMES
            if name not in values
        ]
        last_request = float(
            getattr(
                self,
                "visual_follow_parameter_last_request_monotonic",
                0.0,
            )
        )
        if missing and (
            last_request <= 0.0
            or now - last_request >= VISUAL_FOLLOW_PARAMETER_REQUEST_RETRY_S
        ):
            requester = getattr(
                self.connection.mav,
                "param_request_read_send",
                None,
            )
            if not callable(requester):
                self.visual_follow_parameter_ack = "rejected"
                self.visual_follow_parameter_ack_reason = (
                    "read_only_parameter_transport_unavailable"
                )
                self.visual_follow_parameter_ack_monotonic = now
                return False
            for name in missing:
                requester(
                    self.expected_system_id,
                    mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                    name.encode("ascii"),
                    -1,
                )
            self.visual_follow_parameter_last_request_monotonic = now
        if missing:
            self.visual_follow_parameter_ack = "pending"
            self.visual_follow_parameter_ack_reason = (
                "missing:" + ",".join(missing)
            )
            self.visual_follow_parameter_ack_monotonic = now
            return False

        if not all(
            math.isfinite(float(values[name]))
            for name in VISUAL_FOLLOW_PARAMETER_NAMES
        ):
            self.visual_follow_parameter_ack = "rejected"
            self.visual_follow_parameter_ack_reason = (
                "read_only_follow_parameter_non_finite"
            )
            self.visual_follow_parameter_ack_monotonic = now
            return False
        checks = (
            1.0 <= values["FLW_TGT_DST"],
            values["FLW_TGT_HT"] >= 8.0,
            -180.0 <= values["FLW_TGT_FA"] <= 180.0,
            int(round(values["FLW_TGT_ALT_M"])) == 0,
            0.0 <= values["FLW_TGT_MAX_VEL"] <= 20.0,
            0.0 <= values["FLW_TGT_RS"] <= 1.0,
        )
        if not all(checks):
            self.visual_follow_parameter_ack = "rejected"
            self.visual_follow_parameter_ack_reason = (
                "read_only_follow_parameter_validation_failed"
            )
            self.visual_follow_parameter_ack_monotonic = now
            return False
        self.visual_follow_parameter_ack = "accepted"
        self.visual_follow_parameter_ack_reason = ""
        self.visual_follow_parameter_ack_monotonic = now
        return True

    def handle_visual_follow_parameter_value(self, message: Any) -> bool:
        if (
            message.get_type() != "PARAM_VALUE"
            or message.get_srcSystem() != self.expected_system_id
        ):
            return False
        parameter_id = getattr(message, "param_id", "")
        if isinstance(parameter_id, bytes):
            parameter_id = parameter_id.decode("ascii", errors="ignore")
        parameter_id = str(parameter_id).rstrip("\x00")
        if parameter_id not in VISUAL_FOLLOW_PARAMETER_NAMES:
            return False
        try:
            value = float(getattr(message, "param_value"))
        except (TypeError, ValueError):
            return False
        if not math.isfinite(value):
            return False
        self.visual_follow_parameter_values[parameter_id] = value
        return True

    def visual_follow_status(self) -> dict[str, Any]:
        state_status = self.visual_follow_target_state.status()
        state_status.update(
            {
                "publish_rate_hz": round(
                    float(self.visual_follow_publish_rate_hz),
                    2,
                ),
                "publish_count": self.visual_follow_publish_count,
                "prestream_count": self.visual_follow_stream_frames,
                "prestream_required": VISUAL_FOLLOW_PRESTREAM_FRAMES,
                "parameter_ack": self.visual_follow_parameter_ack,
                "parameter_ack_reason": (
                    self.visual_follow_parameter_ack_reason
                ),
                "read_only_follow_parameters": dict(
                    self.visual_follow_parameter_values
                ),
                "mode_request_feature_enabled": (
                    SAFE_NATIVE_FOLLOW_MODE_REQUEST_ENABLED
                ),
                "neutral_heartbeat_feature_enabled": False,
                "mode_request_ack": self.visual_follow_mode_ack,
                "mode_request_ack_result": (
                    self.visual_follow_mode_ack_result
                ),
                "exit_request_ack": self.visual_follow_exit_ack,
                "exit_request_ack_result": (
                    self.visual_follow_exit_ack_result
                ),
                "exit_nav_state_confirmed": (
                    self.last_px4_main_mode == PX4_MAIN_MODE_POSCTL
                ),
                "mode_request_attempts": (
                    self.visual_follow_request_attempts
                ),
                "px4_main_mode": self.last_px4_main_mode,
                "px4_sub_mode": self.last_px4_sub_mode,
                "px4_nav_state": (
                    19
                    if (
                        self.last_px4_main_mode == PX4_MAIN_MODE_AUTO
                        and self.last_px4_sub_mode
                        == PX4_SUB_MODE_AUTO_FOLLOW_TARGET
                    )
                    else None
                ),
            }
        )
        return state_status

    def handle_visual_follow_command_ack(self, message: Any) -> bool:
        if (
            message.get_type() != "COMMAND_ACK"
            or message.get_srcSystem() != self.expected_system_id
            or int(getattr(message, "command", -1))
            != mavutil.mavlink.MAV_CMD_DO_SET_MODE
        ):
            return False
        result = int(getattr(message, "result", -1))
        accepted_results = {
            int(mavutil.mavlink.MAV_RESULT_ACCEPTED),
            int(mavutil.mavlink.MAV_RESULT_IN_PROGRESS),
        }
        if self.visual_follow_exit_ack == "pending":
            self.visual_follow_exit_ack_result = result
            self.visual_follow_exit_ack_monotonic = time.monotonic()
            self.visual_follow_exit_ack = (
                "accepted" if result in accepted_results else "rejected"
            )
            return True
        if self.visual_follow_mode_ack != "pending":
            return False
        self.visual_follow_mode_ack_result = result
        self.visual_follow_mode_ack_monotonic = time.monotonic()
        self.visual_follow_mode_ack = (
            "accepted" if result in accepted_results else "rejected"
        )
        if self.visual_follow_mode_ack == "rejected":
            self.visual_follow_entry_blocked = True
        return True

    def handle_mission_mode_command_ack(self, message: Any) -> bool:
        """Capture only the ACK belonging to a pending mission OFFBOARD request."""
        if (
            not self.mission_mode_ack_pending
            or message.get_type() != "COMMAND_ACK"
            or message.get_srcSystem() != self.expected_system_id
            or int(getattr(message, "command", -1))
            != mavutil.mavlink.MAV_CMD_DO_SET_MODE
        ):
            return False
        self.last_offboard_mode_ack_result = int(
            getattr(message, "result", -1)
        )
        self.mission_mode_ack_pending = False
        return True

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
        # Single-writer interlock. Placed inside the transmit method rather
        # than at its call sites so that every caller -- including any added
        # later -- is covered by construction. Resolved per call, never
        # cached: a stale authority is the failure this interlock prevents.
        if not legacy_writer_permitted():
            self.legacy_offboard_refused_count += 1
            return
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
        if not legacy_writer_permitted():
            self.legacy_offboard_refused_count += 1
            return
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

    def install_pending_mission(self) -> None:
        """Validate and install the newest operator mission, if one arrived.

        Authoritative validation, deliberately repeating what the dashboard
        already did: MQTT is not a trusted channel, and this side is the one
        holding the ENU origin the geofence check needs. Every failure leaves
        the previously configured nominal exactly as it was -- a rejected
        mission must never degrade into a partially applied one.
        """
        payload = mission_inbox.take(self.drone_id)
        if payload is None or self.companion_safety is None:
            return
        if not MISSION_RUNTIME_ENABLED:
            self.mission_status = {
                "runtime_enabled": False,
                "state": "refused",
                "reason": "mission runtime is disabled (SWARM_MISSION_RUNTIME_ENABLED)",
                "execution_state": "idle",
                "execution_reason": "",
                "start_ready": False,
                "start_block_reason": "mission runtime is disabled",
            }
            LOGGER.warning(
                "Refused mission for %s: mission runtime is disabled", self.drone_id
            )
            return
        if PEER_STATE_ORIGIN is None:
            self.mission_status = {
                "runtime_enabled": True,
                "state": "refused",
                "reason": "common ENU origin is not configured",
                "execution_state": "idle",
                "execution_reason": "",
                "start_ready": False,
                "start_block_reason": "mission is not installed",
            }
            return
        execution_state = str(self.mission_status.get("execution_state", "idle"))
        if execution_state in MISSION_ACTIVE_EXECUTION_STATES:
            LOGGER.warning(
                "Refused replacement mission for %s: mission is already %s",
                self.drone_id,
                execution_state,
            )
            return
        try:
            trajectory = validate_mission(payload, PEER_STATE_ORIGIN)
            self.companion_safety.set_trajectory(trajectory)
        except (MissionRejected, RuntimeError, ValueError) as error:
            self.mission_status = {
                "runtime_enabled": True,
                "state": "refused",
                "reason": str(error),
                "execution_state": "idle",
                "execution_reason": "",
                "start_ready": False,
                "start_block_reason": "mission is not installed",
            }
            LOGGER.warning("Refused mission for %s: %s", self.drone_id, error)
            return
        self.mission_status = {
            "runtime_enabled": True,
            "state": "installed",
            "reason": "",
            "waypoints": len(trajectory.waypoints_enu_m),
            "perimeter_m": round(trajectory.perimeter_m(), 2),
            "lap_duration_s": round(trajectory.lap_duration_s(), 1),
            "speed_m_s": trajectory.speed_m_s,
            "start_enu_m": list(trajectory.reference(0.0).position_enu_m),
            "execution_state": "idle",
            "execution_reason": "",
            "start_ready": False,
            "start_block_reason": "waiting for companion readiness",
        }
        LOGGER.info(
            "Installed mission for %s: %d waypoints, %.1f m lap",
            self.drone_id,
            len(trajectory.waypoints_enu_m),
            trajectory.perimeter_m(),
        )

    def prepare_mission_execution_state(self) -> None:
        """Synchronize mode/disarm observations before deriving aborts."""
        execution_state = str(
            self.mission_status.get("execution_state", "idle")
        )
        if self.last_px4_armed is not True:
            self.mission_start_pending = False
            self.mission_offboard_expected = False
            self.mission_mode_ack_pending = False
            self.last_offboard_mode_ack_result = None
            if execution_state in MISSION_ACTIVE_EXECUTION_STATES:
                self.mission_status["execution_state"] = "idle"
                self.mission_status["execution_reason"] = "vehicle is disarmed"
            return
        if (
            self.mission_start_pending
            and self.last_px4_main_mode == PX4_MAIN_MODE_OFFBOARD
        ):
            self.mission_start_pending = False
            self.mission_offboard_expected = True
            self.mission_mode_ack_pending = False
            self.last_offboard_mode_ack_result = None
            self.mission_status["execution_state"] = "running"
            self.mission_status["execution_reason"] = ""
            LOGGER.info("Mission running for %s in PX4 OFFBOARD", self.drone_id)
        elif (
            execution_state == "stopping"
            and self.last_px4_main_mode != PX4_MAIN_MODE_OFFBOARD
        ):
            self.mission_status["execution_state"] = "idle"
            self.mission_status["execution_reason"] = ""

    def mission_start_block_reason(
        self,
        payload: dict[str, Any],
        *,
        retry: bool = False,
    ) -> str:
        """One authoritative explanation for the dashboard and start gate."""
        if not MISSION_RUNTIME_ENABLED:
            return "mission runtime is disabled"
        if self.mission_status.get("state") != "installed":
            return "mission is not installed"
        execution_state = str(
            self.mission_status.get("execution_state", "idle")
        )
        if not retry and execution_state in MISSION_ACTIVE_EXECUTION_STATES:
            return f"mission is already {execution_state}"
        if self.last_px4_armed is not True:
            return "vehicle is not armed"
        holding_mode = (
            self.last_px4_main_mode == PX4_MAIN_MODE_POSCTL
            or (
                self.last_px4_main_mode == PX4_MAIN_MODE_AUTO
                and self.last_px4_sub_mode == PX4_SUB_MODE_AUTO_LOITER
            )
        )
        if not holding_mode:
            return "PX4 must be in Position or Hold before mission start"
        sender = self.active_offboard_sender
        if sender is None:
            return "active companion sender is unavailable"
        sender_status = sender.status()
        if not sender_status.get("transmit_sink_attached"):
            return "companion OFFBOARD authority is disabled"
        if not sender_status.get("explicit_opt_in"):
            return "vehicle is not cleared for active companion flight"
        if sender_status.get("latched_abort"):
            return f"abort latched: {sender_status['latched_abort']}"
        conditions = tuple(payload.get("active_offboard_conditions") or ())
        if conditions:
            return f"safety condition active: {conditions[0]}"
        frame = payload.get("active_offboard_frame") or {}
        if frame.get("transmitted") is not True:
            return f"setpoint stream is not transmitting: {frame.get('reason') or 'unknown'}"
        duration = (
            None
            if sender.stream_started_monotonic_s is None
            or sender.last_transmit_monotonic_s is None
            else (
                sender.last_transmit_monotonic_s
                - sender.stream_started_monotonic_s
            )
        )
        if duration is None or duration < sender.warmup.minimum_duration_s:
            return "setpoint warmup is not complete"
        if sender.transmit_count < sender.warmup.minimum_valid_samples:
            return "setpoint warmup has too few samples"
        if sender.max_transmit_gap_s > sender.warmup.maximum_gap_s:
            return "setpoint stream gap exceeds the warmup contract"
        return ""

    def request_mission_offboard(self, now_monotonic_s: float) -> bool:
        self.mission_mode_ack_pending = True
        self.last_offboard_mode_ack_result = None
        sent = self.request_px4_main_mode(
            PX4_MAIN_MODE_OFFBOARD,
            "MISSION OFFBOARD",
            companion_authorized=True,
        )
        if not sent:
            self.mission_mode_ack_pending = False
            return False
        self.mission_start_pending = True
        self.mission_start_attempts += 1
        self.mission_start_requested_monotonic = now_monotonic_s
        return True

    def handle_pending_mission_action(
        self,
        payload: dict[str, Any],
        now_monotonic_s: float,
    ) -> None:
        """Consume UI start/stop and advance the guarded OFFBOARD entry."""
        action = mission_inbox.take_action(self.drone_id)
        if action == "mission_stop":
            self.mission_start_pending = False
            self.mission_offboard_expected = False
            self.mission_mode_ack_pending = False
            self.last_offboard_mode_ack_result = None
            self.mission_start_attempts = 0
            if self.last_px4_armed is True and (
                self.last_px4_main_mode == PX4_MAIN_MODE_OFFBOARD
                or self.mission_status.get("execution_state") == "entering"
            ):
                sent = self.request_px4_main_mode(
                    PX4_MAIN_MODE_POSCTL,
                    "POSITION (MISSION STOP)",
                )
                self.mission_status["execution_state"] = (
                    "stopping" if sent else "blocked"
                )
                self.mission_status["execution_reason"] = (
                    "" if sent else "could not request Position mode"
                )
            else:
                self.mission_status["execution_state"] = "idle"
                self.mission_status["execution_reason"] = ""
        elif action == "mission_start":
            reason = self.mission_start_block_reason(payload)
            if reason:
                execution_state = str(
                    self.mission_status.get("execution_state", "idle")
                )
                if execution_state in MISSION_ACTIVE_EXECUTION_STATES:
                    LOGGER.warning(
                        "Ignored duplicate mission start for %s: %s",
                        self.drone_id,
                        reason,
                    )
                else:
                    self.mission_status["execution_state"] = "blocked"
                    self.mission_status["execution_reason"] = reason
                    LOGGER.warning(
                        "Blocked mission start for %s: %s", self.drone_id, reason
                    )
            else:
                self.mission_start_attempts = 0
                if self.request_mission_offboard(now_monotonic_s):
                    self.mission_status["execution_state"] = "entering"
                    self.mission_status["execution_reason"] = ""
                else:
                    self.mission_status["execution_state"] = "blocked"
                    self.mission_status["execution_reason"] = (
                        "companion OFFBOARD mode request was refused"
                    )

        frame = payload.get("active_offboard_frame") or {}
        if (
            self.mission_status.get("execution_state") in {"entering", "running"}
            and frame.get("abort_condition")
        ):
            condition = str(frame["abort_condition"])
            self.mission_start_pending = False
            self.mission_offboard_expected = False
            self.mission_mode_ack_pending = False
            self.last_offboard_mode_ack_result = None
            self.mission_status["execution_state"] = "aborted"
            self.mission_status["execution_reason"] = condition

        if self.mission_start_pending:
            elapsed = now_monotonic_s - self.mission_start_requested_monotonic
            if elapsed >= OFFBOARD_MODE_RETRY_S:
                if self.mission_start_attempts >= OFFBOARD_MODE_MAX_ATTEMPTS:
                    self.mission_start_pending = False
                    self.mission_mode_ack_pending = False
                    self.last_offboard_mode_ack_result = None
                    self.mission_status["execution_state"] = "blocked"
                    self.mission_status["execution_reason"] = (
                        "PX4 did not enter OFFBOARD after 3 attempts"
                    )
                else:
                    reason = self.mission_start_block_reason(payload, retry=True)
                    if reason:
                        self.mission_start_pending = False
                        self.mission_mode_ack_pending = False
                        self.last_offboard_mode_ack_result = None
                        self.mission_status["execution_state"] = "blocked"
                        self.mission_status["execution_reason"] = reason
                    else:
                        self.request_mission_offboard(now_monotonic_s)

        reason = self.mission_start_block_reason(payload)
        self.mission_status["start_ready"] = not reason
        self.mission_status["start_block_reason"] = reason

    def evaluate_companion_safety(self, now_monotonic_s: float) -> None:
        """Run the local nominal + CBF from peer-to-peer state only.

        Nothing here reaches PX4: the result is published for observability and
        appended to the optional local trace. The server is not consulted, so
        this keeps producing commands while the server is down -- which is the
        property the whole companion-side split exists to provide.
        """
        if self.companion_safety is None or self.peer_state_transport is None:
            return
        self.install_pending_mission()
        self.prepare_mission_execution_state()
        healthy = (
            now_monotonic_s - self.last_vehicle_heartbeat_monotonic
            <= VEHICLE_HEARTBEAT_TIMEOUT_S
        )
        swarm_state: dict[str, Any] = {
            self.drone_id: self.fast_pose_cache.own_swarm_state(
                now_monotonic_s, PEER_STATE_ORIGIN, healthy
            )
        }
        peers = self.peer_state_transport.registry.snapshot(now_monotonic_s)["peers"]
        for peer_id, peer in peers.items():
            # A peer echoing our own id would otherwise overwrite the state we
            # just measured locally with a round-tripped copy of it.
            if peer_id != self.drone_id:
                swarm_state[peer_id] = peer
        # The companion is the controlling authority only when PX4 is armed
        # AND in OFFBOARD. Anything less and its setpoints are not flying the
        # vehicle, so latching an altitude reference then would capture a
        # value the drone is about to leave -- on the ground, before an AUTO
        # takeoff, that reference would command a dive the instant OFFBOARD
        # engaged. `is True` on both: unknown is not authority.
        station_keeping = (
            self.last_px4_armed is True
            and self.last_px4_main_mode == PX4_MAIN_MODE_OFFBOARD
        )
        status = self.companion_safety.evaluate(
            swarm_state, now_monotonic_s, station_keeping=station_keeping
        )
        payload = status.as_dict()
        payload["evaluated_monotonic_s"] = round(now_monotonic_s, 3)
        payload["self_state_reason"] = swarm_state[self.drone_id].get("reason")
        payload["station_keeping"] = station_keeping
        payload["altitude_reference_m"] = (
            self.companion_safety.altitude_hold.reference_altitude_m
        )
        payload["self_message_age_ms"] = swarm_state[self.drone_id].get(
            "message_age_ms"
        )
        payload["position_covariance_m2_by_drone"] = {
            drone_id: state.get("position_covariance_m2")
            for drone_id, state in swarm_state.items()
        }
        payload["cbf_covariance_sigma"] = self.companion_safety.gate.config.covariance_sigma
        payload["cbf_require_position_covariance"] = (
            self.companion_safety.gate.config.require_position_covariance
        )
        # The shared-ENU position the nominal controllers and CBF actually
        # consumed on this frame, and -- when a trajectory is configured --
        # where that trajectory says the vehicle should begin. Published as a
        # pair so an external check can compare them in the SAME frame the
        # controller uses, rather than re-deriving a position from
        # LOCAL_POSITION_NED (a different estimator, per-vehicle origin) and
        # comparing across frames. Absent/None on an invalid own state, so a
        # consumer fails closed instead of reading a stale coordinate.
        payload["own_position_enu_m"] = swarm_state[self.drone_id].get(
            "position_enu_m"
        )
        # The REAL measured velocity, as opposed to nominal/output above which
        # are commanded values -- the ground-truth response signal a latency
        # system-ID (comparing it against a known scheduled input, e.g.
        # SquareWaveVelocityTrajectory) needs and neither of those fields can
        # substitute for.
        payload["own_velocity_enu_m_s"] = swarm_state[self.drone_id].get(
            "velocity_enu_m_s"
        )
        reference_start = self.companion_safety.trajectory_reference_start_enu_m
        payload["trajectory_reference_start_enu_m"] = (
            list(reference_start) if reference_start is not None else None
        )
        payload["position_covariance_age_ms"] = (
            self.fast_pose_cache.position_covariance_age_ms(now_monotonic_s)
        )
        payload["odometry_reset_counter"] = (
            self.fast_pose_cache.odometry_reset_counter
        )
        payload["odometry_sample_count"] = self.fast_pose_cache.odometry_sample_count
        payload["odometry_covariance_sample_count"] = (
            self.fast_pose_cache.odometry_covariance_sample_count
        )
        payload["peer_message_age_ms_by_drone"] = {
            drone_id: state.get("message_age_ms")
            for drone_id, state in swarm_state.items()
            if drone_id != self.drone_id
        }
        if self.offboard_setpoint_sender is not None:
            # The command's own timestamp is this evaluation's timestamp: the
            # status was produced synchronously above, so there is no
            # separate age to track here. Staleness of the *inputs* is
            # already handled upstream (swarm-state age -> CBF -> supervisor).
            preview = self.offboard_setpoint_sender.preview(
                status,
                now_monotonic_s=now_monotonic_s,
                command_monotonic_s=now_monotonic_s,
                px4_main_mode=self.last_px4_main_mode,
                px4_armed=self.last_px4_armed,
                px4_offboard_main_mode=PX4_MAIN_MODE_OFFBOARD,
            )
            payload["offboard_setpoint_preview"] = preview.as_dict()
            payload["offboard_sender_status"] = self.offboard_setpoint_sender.status()
            authority = resolve_authority()
            payload["offboard_authority"] = {
                **authority.as_dict(),
                "legacy_offboard_refused_count": self.legacy_offboard_refused_count,
            }
            self.step_active_offboard_sender(payload, preview, now_monotonic_s, healthy)
        self.handle_pending_mission_action(payload, now_monotonic_s)
        payload["mission"] = dict(self.mission_status)
        self.last_companion_evaluation_monotonic = now_monotonic_s
        self.companion_safety_status = payload
        self.append_companion_safety_trace(payload)

    def transmit_active_offboard_setpoint(self, *arguments: Any) -> None:
        """The sink. The only place a companion-authored setpoint reaches PX4.

        Deliberately thin: it adds no decision of its own beyond refusing to
        touch a connection that does not exist. Every gate -- authority,
        identity, opt-in, preview validity, abort state -- has already been
        evaluated by the sender, and duplicating any of them here would
        create a second place to keep in sync.
        """
        if self.connection is None:
            return
        self.connection.mav.set_position_target_local_ned_send(*arguments)
        self.active_offboard_transmit_count += 1

    def reset_latched_active_sender(self, conditions: tuple[str, ...]) -> None:
        """Replace a latched sender, but only while disarmed and clear.

        Three conditions, each load-bearing:

        * Latched -- nothing to do otherwise.
        * Disarmed -- an abort latch is scoped to one flight and must never
          clear itself mid-flight, which is the whole point of latching.
          `is not False` rather than truthiness: `None` means "armed state
          not yet known", which is not a safe basis for clearing an abort.
        * No active conditions -- otherwise a *persistent* fault on a
          disarmed vehicle rebuilds the sender at 20 Hz, re-detecting and
          re-latching every frame, which churns objects and destroys the
          frame history that explains the abort.
        """
        if self.active_offboard_sender is None:
            return
        if self.active_offboard_sender.latched_abort is None:
            return
        if self.last_px4_armed is not False or conditions:
            return
        self.active_offboard_sender = build_active_offboard_setpoint_sender(
            self.drone_id,
            self.expected_system_id,
            self.transmit_active_offboard_setpoint,
            self.request_position_mode,
        )
        self.active_sender_reset_count += 1

    def step_active_offboard_sender(
        self,
        payload: dict[str, Any],
        preview: Any,
        now_monotonic_s: float,
        heartbeat_healthy: bool,
    ) -> None:
        """Run the active sender on this frame. It has no sink and sends
        nothing; the point is to exercise abort detection and latching at the
        real cadence, on the real previews."""
        if self.active_offboard_sender is None:
            return
        conditions = conditions_from_status(
            payload,
            now_monotonic_s=now_monotonic_s,
            last_heartbeat_monotonic_s=(
                self.last_vehicle_heartbeat_monotonic if heartbeat_healthy else None
            ),
            heartbeat_timeout_s=VEHICLE_HEARTBEAT_TIMEOUT_S,
            px4_main_mode=self.last_px4_main_mode,
            px4_offboard_main_mode=PX4_MAIN_MODE_OFFBOARD,
            offboard_expected=self.mission_offboard_expected,
            offboard_mode_ack_result=self.last_offboard_mode_ack_result,
            accepted_ack_results=OFFBOARD_ACCEPTED_ACK_RESULTS,
            previous_evaluation_monotonic_s=self.last_companion_evaluation_monotonic,
            maximum_command_age_s=PEER_STATE_MAX_AGE_S,
        )
        # Evaluated after the conditions, so a latch only clears once its
        # cause is actually gone.
        self.reset_latched_active_sender(conditions)
        frame = self.active_offboard_sender.step(
            preview,
            now_monotonic_s=now_monotonic_s,
            reported_conditions=conditions,
        )
        payload["active_offboard_frame"] = frame.as_dict()
        payload["active_offboard_conditions"] = list(conditions)
        payload["active_offboard_sender"] = {
            **self.active_offboard_sender.status(),
            "latch_reset_count": self.active_sender_reset_count,
            # Counted at the sink rather than by the sender, so the two can
            # be compared: a mismatch means something transmitted without
            # going through the gates.
            "sink_transmit_count": self.active_offboard_transmit_count,
        }

    def append_companion_safety_trace(self, payload: dict[str, Any]) -> None:
        if not COMPANION_SAFETY_LOG_PATH:
            return
        try:
            with open(COMPANION_SAFETY_LOG_PATH, "a", encoding="utf-8") as handle:
                handle.write(
                    json.dumps(
                        {"wall_clock_s": time.time(), **payload},
                        ensure_ascii=False,
                        separators=(",", ":"),
                    )
                    + "\n"
                )
        except OSError as error:
            LOGGER.warning("Companion safety trace write failed: %s", error)

    def report_slow_iteration(
        self,
        started: float,
        ended: float,
        marks: list[tuple[str, float]],
    ) -> None:
        """One WARNING per slow iteration, naming where the time went.

        Spans loop-start to loop-start, not just the body: that is what the
        companion trace measures (`evaluated_monotonic_s` is stamped at the
        top of the loop) and what `WarmupContract.maximum_gap_s` is a
        contract on. A stall inside `stop_event.wait` -- the process being
        descheduled, or the machine itself stalling -- leaves the body fast
        and would be invisible to a body-only timer, which is precisely the
        case worth distinguishing here. Whatever the marks do not account for
        is reported as `sleep_and_scheduler`.

        Reports the whole breakdown rather than the worst section, because a
        stall spread evenly across every section is itself the signature of a
        process-wide pause, and a "worst section" line would hide it. GC
        pauses get their own field for the same reason: they are charged to
        whichever section was unlucky enough to be running.
        """
        previous = started
        sections = []
        for name, at in marks:
            sections.append(f"{name}={(at - previous) * 1000.0:.0f}ms")
            previous = at
        sections.append(f"sleep_and_scheduler={(ended - previous) * 1000.0:.0f}ms")
        collections = GC_PAUSE_PROBE.during(started)
        LOGGER.warning(
            "%s slow iteration: %.0fms (budget %.0fms) | %s | gc=%s",
            self.drone_id,
            (ended - started) * 1000.0,
            SLOW_ITERATION_S * 1000.0,
            " ".join(sections),
            ", ".join(
                f"gen{generation}:{seconds * 1000.0:.0f}ms"
                for _, seconds, generation in collections
            )
            or "none",
        )

    def run_connected(self) -> None:
        assert self.connection is not None

        last_heartbeat_sent = 0.0
        last_control_source: str | None = None
        self.last_vehicle_heartbeat_monotonic = time.monotonic()
        self.visual_follow_parameter_values.clear()
        self.visual_follow_parameter_last_request_monotonic = 0.0
        self.request_fast_pose_stream()
        self.open_qgc_proxy()

        previous_started: float | None = None
        marks: list[tuple[str, float]] = []

        while not self.stop_event.is_set():
            loop_started = time.monotonic()
            # Reported one iteration late so the sleep is inside the window.
            if (
                previous_started is not None
                and loop_started - previous_started > SLOW_ITERATION_S
            ):
                self.report_slow_iteration(previous_started, loop_started, marks)
            previous_started = loop_started
            marks = []

            # Đọc và bỏ các gói telemetry đang chờ để duy trì socket.
            for _ in range(30):
                message = self.connection.recv_match(
                    blocking=False
                )

                if message is None:
                    break
                self.forward_px4_message_to_qgc(message)
                if not companion_telemetry_ingest_blocked(self.drone_id):
                    self.fast_pose_cache.update(message, time.monotonic())
                if not self.handle_mission_mode_command_ack(message):
                    self.handle_visual_follow_command_ack(message)
                self.handle_visual_follow_parameter_value(message)
                if (
                    message.get_type() == "HEARTBEAT"
                    and message.get_srcSystem() == self.expected_system_id
                ):
                    heartbeat_now = time.monotonic()
                    self.last_vehicle_heartbeat_monotonic = heartbeat_now
                    custom_mode = int(getattr(message, "custom_mode", 0))
                    self.last_px4_main_mode = (custom_mode >> 16) & 0xFF
                    self.last_px4_sub_mode = (custom_mode >> 24) & 0xFF
                    # Armed state, read from a HEARTBEAT field already being
                    # received. The companion previously knew only the flight
                    # mode; armed reached the dashboard solely through the
                    # server's ROS telemetry, which a companion-local safety
                    # decision must not depend on.
                    self.last_px4_armed = bool(
                        int(getattr(message, "base_mode", 0))
                        & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                    if self.last_px4_main_mode == PX4_MAIN_MODE_POSCTL:
                        self.offboard_attitude_state.finish_release()

            marks.append(("mavlink_drain", time.monotonic()))

            self.evaluate_companion_safety(loop_started)
            marks.append(("companion_safety", time.monotonic()))

            fast_pose_payload = self.fast_pose_cache.payload(
                self.drone_id,
                time.monotonic(),
            )
            if fast_pose_payload is not None:
                fast_pose_payload["visual_follow_bridge"] = (
                    self.visual_follow_status()
                )
                if self.peer_state_transport is not None:
                    fast_pose_payload["peer_state_status"] = (
                        self.peer_state_transport.status(loop_started)
                    )
                if self.companion_safety_status is not None:
                    fast_pose_payload["companion_safety"] = (
                        self.companion_safety_status
                    )
                self.mqtt_client.publish(
                    FAST_POSE_TOPIC_TEMPLATE.format(drone_id=self.drone_id),
                    json.dumps(
                        fast_pose_payload,
                        ensure_ascii=False,
                        separators=(",", ":"),
                    ),
                    qos=0,
                    retain=False,
                )
            marks.append(("mqtt_publish", time.monotonic()))

            if (
                self.peer_state_transport is not None
                and loop_started - self.last_peer_state_publish_monotonic >= PEER_STATE_PERIOD_S
            ):
                peer_payload = self.fast_pose_cache.peer_payload(
                    self.drone_id,
                    loop_started,
                    PEER_STATE_ORIGIN,
                    self.peer_state_sequence,
                    healthy=(
                        loop_started - self.last_vehicle_heartbeat_monotonic
                        <= VEHICLE_HEARTBEAT_TIMEOUT_S
                    ),
                )
                if peer_payload is not None and self.peer_state_transport.publish(peer_payload):
                    self.peer_state_sequence += 1
                    self.last_peer_state_publish_monotonic = loop_started

            marks.append(("peer_state_publish", time.monotonic()))

            self.forward_qgc_commands()
            marks.append(("qgc_forward", time.monotonic()))

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
                visual_follow_angle,
                visual_position_covariance,
                visual_est_capabilities,
                visual_follow_active,
            ) = self.visual_follow_target_state.output()
            visual_stream_fresh = (
                self.visual_follow_target_state.status()["timeout_state"]
                == "FRESH"
            )
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
                self.visual_follow_settle_frames = 0
                self.visual_follow_request_attempts = 0
                self.visual_follow_entry_blocked = False
                self.visual_follow_params_configured = False
                self.visual_follow_parameter_ack = "not_requested"
                self.visual_follow_parameter_ack_reason = ""
                self.visual_follow_mode_ack = "not_requested"
                self.visual_follow_mode_ack_result = None
                self.visual_follow_exit_ack = "not_requested"
                self.visual_follow_exit_ack_result = None
                self.visual_follow_last_publish_monotonic = 0.0
                self.visual_follow_publish_rate_hz = 0.0
                self.visual_follow_publish_count = 0
                self.visual_follow_was_observed = False
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
                    self.visual_follow_params_configured = (
                        self.configure_visual_follow_parameters(
                            visual_follow_distance,
                            visual_follow_height,
                            visual_follow_angle,
                        )
                    )
                    if not self.visual_follow_params_configured:
                        if self.visual_follow_parameter_ack == "rejected":
                            self.visual_follow_entry_blocked = True
                            self.request_position_mode()
                            control_source = (
                                "VISUAL_FOLLOW_PARAMETER_REJECTED"
                            )
                        else:
                            control_source = (
                                "VISUAL_FOLLOW_PARAMETERS_PENDING"
                            )
                        continue
                    # Restart the loop and drain the newest target command
                    # after validation; never publish the snapshot captured
                    # before the read-only PARAM_VALUE responses arrived.
                    control_source = "VISUAL_FOLLOW_PARAMS_CONFIGURED"
                    continue
                self.send_visual_follow_target(
                    visual_lat,
                    visual_lon,
                    visual_alt,
                    visual_vn,
                    visual_ve,
                    visual_vd,
                    visual_quality,
                    visual_position_covariance,
                    visual_est_capabilities,
                )
                self.visual_follow_stream_frames += 1
                current_velocity = self.fast_pose_cache.velocity_ned_m_s
                if (
                    visual_stream_fresh
                    and
                    current_velocity is not None
                    and len(current_velocity) == 3
                    and math.hypot(
                        float(current_velocity[0]),
                        float(current_velocity[1]),
                    )
                    <= VISUAL_FOLLOW_SETTLE_MAX_XY_SPEED_M_S
                ):
                    self.visual_follow_settle_frames += 1
                else:
                    self.visual_follow_settle_frames = 0
                now = time.monotonic()
                follow_mode_observed = bool(
                    self.last_px4_main_mode == PX4_MAIN_MODE_AUTO
                    and self.last_px4_sub_mode
                    == PX4_SUB_MODE_AUTO_FOLLOW_TARGET
                )
                if (
                    visual_stream_fresh
                    and self.should_request_visual_follow_mode(
                        now,
                        follow_mode_observed,
                    )
                ):
                    mode_request_sent = self.request_px4_main_mode(
                        PX4_MAIN_MODE_AUTO,
                        "AUTO FOLLOW TARGET",
                        PX4_SUB_MODE_AUTO_FOLLOW_TARGET,
                    )
                    if mode_request_sent:
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
                # NOT safe to suppress while the companion drives OFFBOARD,
                # however much it looks like the legacy fight that
                # `companion_holds_offboard` exists for. PX4 consumes this
                # stream as the manual-control liveness signal: cutting it
                # raised `manual_control_signal_lost`, dropped
                # `preflight_checks_pass` to false, and neither vehicle could
                # arm (measured 2026-08-12). Any future attempt to stop
                # fighting the companion for the vertical axis has to go
                # through PX4's COM_RCL_EXCEPT first, not through this send.
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

            marks.append(("control_output", time.monotonic()))

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


class MissionInbox:
    """Hands the newest operator mission from the MQTT thread to its worker.

    Only the raw payload crosses the thread boundary. Validation and install
    happen on the worker, which is the side that owns the ENU origin and the
    `CompanionSafetyMonitor` -- so nothing here can mutate a monitor while its
    20 Hz loop is reading it.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._payload_by_drone: dict[str, dict[str, Any]] = {}
        self._action_by_drone: dict[str, str] = {}

    def submit(self, drone_id: str, payload: dict[str, Any]) -> None:
        with self._lock:
            # Newest wins: an operator who redraws before the worker picks the
            # mission up meant the second drawing, not both.
            self._payload_by_drone[drone_id] = payload

    def take(self, drone_id: str) -> dict[str, Any] | None:
        with self._lock:
            return self._payload_by_drone.pop(drone_id, None)

    def submit_action(self, drone_id: str, action: str) -> None:
        with self._lock:
            self._action_by_drone[drone_id] = action

    def take_action(self, drone_id: str) -> str | None:
        with self._lock:
            return self._action_by_drone.pop(drone_id, None)


mission_inbox = MissionInbox()


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
    if message_type == "mission_path":
        mission_drone_id = str(payload.get("drone_id", ""))
        if mission_drone_id in states:
            mission_inbox.submit(mission_drone_id, payload)
        return
    if message_type in {"mission_start", "mission_stop"}:
        mission_drone_id = str(payload.get("drone_id", ""))
        if mission_drone_id in states:
            mission_inbox.submit_action(mission_drone_id, message_type)
        return
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
    if (
        message_type in {
            "tracking_yaw",
            "tracking_follow",
            "offboard_follow",
            "offboard_attitude_follow",
        }
        and not LEGACY_AUTOMATION_ENABLED
    ):
        LOGGER.warning(
            "Rejected legacy automation command while containment is active: %s",
            message_type,
        )
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
        if bool(payload.get("enabled", False)):
            visual_follow_target_states[drone_id].disable()
    elif message_type == "offboard_follow":
        offboard_attitude_states[drone_id].disable()
        # A neutral/disable packet is part of the Offboard-to-native handover.
        # It must not cancel a fresh FOLLOW_TARGET command that arrived just
        # before it. Only a newly active Offboard source may seize authority.
        if bool(payload.get("enabled", False)):
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

    peer_state_transports: dict[str, DirectPeerStateTransport] = {}
    if PEER_STATE_ENABLED:
        peer_state_transports = build_peer_state_transports(VEHICLES)

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
            mqtt_client=mqtt_client,
            stop_event=stop_event,
            peer_state_transport=peer_state_transports.get(drone_id),
        )

        workers.append(worker)
        worker.start()

    # Registered here rather than at import so test processes that merely
    # import this module do not inherit a GC callback.
    gc.callbacks.append(GC_PAUSE_PROBE)

    LOGGER.info(
        "Manual bridge started at %.1f Hz",
        SEND_RATE_HZ,
    )

    LOGGER.info(
        "MQTT command topic: %s",
        MQTT_TOPIC,
    )

    liveness_watchdog = WorkerLivenessWatchdog(stall_after_s=WORKER_LIVENESS_STALL_S)

    try:
        while not stop_event.wait(1.0):
            now_monotonic_s = time.monotonic()
            samples = tuple(
                WorkerLivenessSample(
                    drone_id=worker.drone_id,
                    companion_safety_enabled=worker.companion_safety is not None,
                    last_evaluation_monotonic_s=worker.last_companion_evaluation_monotonic,
                )
                for worker in workers
            )
            _results, transitions = liveness_watchdog.poll(
                samples, now_monotonic_s=now_monotonic_s
            )
            for result in transitions:
                if result.stalled:
                    LOGGER.error(
                        "%s companion safety loop stalled: no evaluation for %.1fs",
                        result.drone_id,
                        result.stalled_for_s,
                    )
                else:
                    LOGGER.info(
                        "%s companion safety loop liveness: %s",
                        result.drone_id,
                        result.reason,
                    )
                mqtt_client.publish(
                    WORKER_LIVENESS_TOPIC_TEMPLATE.format(drone_id=result.drone_id),
                    json.dumps(
                        {
                            "stalled": result.stalled,
                            "reason": result.reason,
                            "stalled_for_s": result.stalled_for_s,
                            "checked_monotonic_s": now_monotonic_s,
                        },
                        ensure_ascii=False,
                        separators=(",", ":"),
                    ),
                    qos=0,
                    retain=True,
                )

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

        for transport in peer_state_transports.values():
            transport.stop()

        mqtt_client.loop_stop()
        mqtt_client.disconnect()

        LOGGER.info(
            "Manual bridge stopped"
        )


if __name__ == "__main__":
    main()
