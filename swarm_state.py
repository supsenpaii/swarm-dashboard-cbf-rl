"""Common-frame swarm state for monitoring and deterministic controllers.

PX4 local-NED origins are vehicle-specific.  This module deliberately builds
relative positions only from a configured WGS-84 reference and never subtracts
two PX4 local positions.  It is transport-agnostic: peers may use the same
serializable contract, while the dashboard uses it for observability only.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any


EARTH_RADIUS_M = 6_378_137.0


@dataclass(frozen=True)
class GeodeticOrigin:
    latitude_deg: float
    longitude_deg: float
    altitude_msl_m: float

    def __post_init__(self) -> None:
        values = (self.latitude_deg, self.longitude_deg, self.altitude_msl_m)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("ENU origin must be finite")
        if not -90.0 <= self.latitude_deg <= 90.0:
            raise ValueError("ENU origin latitude is out of range")
        if not -180.0 <= self.longitude_deg <= 180.0:
            raise ValueError("ENU origin longitude is out of range")


def ned_to_enu(vector_ned: tuple[float, float, float]) -> tuple[float, float, float]:
    """Convert a vector, never a position origin, from NED to ENU."""
    north, east, down = vector_ned
    return east, north, -down


def ned_variance_to_enu(variance_ned: tuple[float, float, float]) -> tuple[float, float, float]:
    """Diagonal (per-axis) variance from NED to ENU: swap N/E, never negate.

    A variance is a squared quantity. Routing one through `ned_to_enu` would
    negate the third axis and produce a negative variance, so this is a
    separate function rather than a reuse of that one.
    """
    north, east, down = variance_ned
    return east, north, down


def geodetic_to_enu(
    latitude_deg: float,
    longitude_deg: float,
    altitude_msl_m: float,
    origin: GeodeticOrigin,
) -> tuple[float, float, float]:
    """Local tangent-plane approximation suitable for swarm-scale distances."""
    values = (latitude_deg, longitude_deg, altitude_msl_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("global position must be finite")
    if not -90.0 <= latitude_deg <= 90.0 or not -180.0 <= longitude_deg <= 180.0:
        raise ValueError("global position is out of range")
    north = math.radians(latitude_deg - origin.latitude_deg) * EARTH_RADIUS_M
    east_scale = math.cos(math.radians(origin.latitude_deg))
    if abs(east_scale) < 1e-9:
        raise ValueError("ENU origin is too close to a pole")
    east = math.radians(longitude_deg - origin.longitude_deg) * EARTH_RADIUS_M * east_scale
    return east, north, altitude_msl_m - origin.altitude_msl_m


def target_wgs84_to_state(
    latitude_deg: float | None,
    longitude_deg: float | None,
    altitude_msl_m: float | None,
    velocity_enu_m_s: tuple[float, float, float] | None,
    origin: GeodeticOrigin | None,
    valid: bool,
    message_age_ms: float | None,
) -> dict[str, Any]:
    """Bridge an externally-tracked target's WGS84 position into this
    project's common-ENU state shape, so formation logic can consume it
    exactly like any other swarm-state entry (see peer_state.make_peer_state
    and SwarmStateStore.snapshot, which reach the same shape from other
    sources). Never treat the result as a drone/peer identity: a target has
    no drone_id and must not be passed to CBF as a peer.

    `velocity_enu_m_s` may be None (an untrustworthy or unavailable target
    velocity estimate); the caller degrades to zero feed-forward rather than
    losing position tracking, matching how this codebase's own visual-follow
    projection already treats `TargetEstimate.velocity_valid`.
    """
    if not valid or origin is None or latitude_deg is None or longitude_deg is None or altitude_msl_m is None:
        return {
            "valid": False,
            "reason": "target_invalid" if origin is not None else "enu_origin_not_configured",
            "position_enu_m": None,
            "velocity_enu_m_s": None,
            "message_age_ms": message_age_ms,
        }
    try:
        position_enu_m = geodetic_to_enu(latitude_deg, longitude_deg, altitude_msl_m, origin)
    except ValueError:
        return {
            "valid": False,
            "reason": "target_enu_transform_failed",
            "position_enu_m": None,
            "velocity_enu_m_s": None,
            "message_age_ms": message_age_ms,
        }
    return {
        "valid": True,
        "reason": "ok",
        "position_enu_m": list(position_enu_m),
        "velocity_enu_m_s": list(velocity_enu_m_s) if velocity_enu_m_s is not None else None,
        "message_age_ms": message_age_ms,
    }


@dataclass(frozen=True)
class SwarmState:
    drone_id: str
    position_enu_m: tuple[float, float, float] | None
    velocity_enu_m_s: tuple[float, float, float] | None
    position_covariance_m2: tuple[float, float, float] | None
    timestamp_ms: int | None
    message_age_ms: float | None
    healthy: bool
    valid: bool
    reason: str
    command: dict[str, Any] | None

    def as_dict(self) -> dict[str, Any]:
        return {
            "drone_id": self.drone_id,
            "frame": "ENU",
            "position_enu_m": list(self.position_enu_m) if self.position_enu_m else None,
            "velocity_enu_m_s": list(self.velocity_enu_m_s) if self.velocity_enu_m_s else None,
            "position_covariance_m2": list(self.position_covariance_m2)
            if self.position_covariance_m2
            else None,
            "timestamp_ms": self.timestamp_ms,
            "message_age_ms": round(self.message_age_ms, 2)
            if self.message_age_ms is not None
            else None,
            "healthy": self.healthy,
            "valid": self.valid,
            "reason": self.reason,
            "command": self.command,
        }


class SwarmStateStore:
    """Maintains a common-ENU snapshot and invalidates stale telemetry."""

    def __init__(self, origin: GeodeticOrigin | None, max_message_age_s: float = 0.5) -> None:
        if not math.isfinite(max_message_age_s) or max_message_age_s <= 0.0:
            raise ValueError("max_message_age_s must be positive and finite")
        self.origin = origin
        self.max_message_age_s = float(max_message_age_s)
        self._received_monotonic_s: dict[str, float] = {}
        self._states: dict[str, SwarmState] = {}
        self._commands: dict[str, dict[str, Any]] = {}

    def record_command(self, drone_id: str, command: dict[str, Any]) -> None:
        self._commands[drone_id] = dict(command)

    def ingest(self, drone_id: str, payload: dict[str, Any], received_monotonic_s: float | None = None) -> SwarmState:
        received = time.monotonic() if received_monotonic_s is None else float(received_monotonic_s)
        if not math.isfinite(received):
            raise ValueError("received_monotonic_s must be finite")
        self._received_monotonic_s[drone_id] = received
        command = self._commands.get(drone_id)
        healthy = bool(payload.get("online", False))
        timestamp_ms = _integer_or_none(payload.get("dashboard_received_ms"))
        if self.origin is None:
            state = self._invalid(drone_id, healthy, timestamp_ms, "common_enu_origin_unconfigured", command)
        else:
            state = self._from_payload(drone_id, payload, healthy, timestamp_ms, command)
        self._states[drone_id] = state
        return state

    def snapshot(self, now_monotonic_s: float | None = None) -> dict[str, dict[str, Any]]:
        now = time.monotonic() if now_monotonic_s is None else float(now_monotonic_s)
        if not math.isfinite(now):
            raise ValueError("now_monotonic_s must be finite")
        result: dict[str, dict[str, Any]] = {}
        for drone_id, state in self._states.items():
            received = self._received_monotonic_s[drone_id]
            age_ms = max(0.0, now - received) * 1000.0
            if age_ms > self.max_message_age_s * 1000.0:
                state = SwarmState(
                    **{**state.__dict__, "message_age_ms": age_ms, "valid": False, "reason": "telemetry_stale"}
                )
            else:
                state = SwarmState(**{**state.__dict__, "message_age_ms": age_ms})
            result[drone_id] = state.as_dict()
        return result

    def _from_payload(self, drone_id: str, payload: dict[str, Any], healthy: bool, timestamp_ms: int | None, command: dict[str, Any] | None) -> SwarmState:
        global_position = payload.get("global_position")
        local_position = payload.get("local_position")
        if not isinstance(global_position, dict) or not isinstance(local_position, dict):
            return self._invalid(drone_id, healthy, timestamp_ms, "position_unavailable", command)
        try:
            position = geodetic_to_enu(
                float(global_position["latitude_deg"]),
                float(global_position["longitude_deg"]),
                float(global_position["altitude_msl_m"]),
                self.origin,
            )
            velocity = ned_to_enu((
                float(local_position["vx_m_s"]),
                float(local_position["vy_m_s"]),
                float(local_position["vz_m_s"]),
            ))
            if not all(math.isfinite(value) for value in velocity):
                raise ValueError("velocity must be finite")
        except (KeyError, TypeError, ValueError):
            return self._invalid(drone_id, healthy, timestamp_ms, "position_invalid", command)
        covariance = _covariance_from(global_position, payload.get("gps"))
        reason = "ok" if healthy else "vehicle_offline"
        return SwarmState(
            drone_id, position, velocity, covariance, timestamp_ms, 0.0,
            healthy, healthy, reason, command,
        )

    @staticmethod
    def _invalid(drone_id: str, healthy: bool, timestamp_ms: int | None, reason: str, command: dict[str, Any] | None) -> SwarmState:
        return SwarmState(drone_id, None, None, None, timestamp_ms, 0.0, healthy, False, reason, command)


def _integer_or_none(value: Any) -> int | None:
    try:
        number = int(value)
    except (TypeError, ValueError):
        return None
    return number if number >= 0 else None


def _covariance_from(global_position: dict[str, Any], gps: Any) -> tuple[float, float, float] | None:
    """Accept common horizontal/vertical accuracy telemetry, if present.

    ``eph_m``/``epv_m`` live in the telemetry ``gps`` block, not ``global_position``.
    """
    gps = gps if isinstance(gps, dict) else {}
    horizontal = global_position.get("horizontal_accuracy_m", gps.get("eph_m"))
    vertical = global_position.get("vertical_accuracy_m", gps.get("epv_m"))
    try:
        h, v = float(horizontal), float(vertical)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) and value >= 0.0 for value in (h, v)):
        return None
    return h * h, h * h, v * v
