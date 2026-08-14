from __future__ import annotations

import math
import os
import threading
from dataclasses import dataclass
from typing import Any, Callable, Mapping, Sequence


SIMULATION_GROUND_TRUTH_SOURCE = "gazebo_camera_to_target_center"
RTK_GROUND_TRUTH_SOURCE = "rtk_fixed_camera_to_target_center"
ALLOWED_GROUND_TRUTH_SOURCES = frozenset(
    (SIMULATION_GROUND_TRUTH_SOURCE, RTK_GROUND_TRUTH_SOURCE)
)
RTK_FIXED_FIX_TYPE = 6

_WGS84_SEMI_MAJOR_M = 6_378_137.0
_WGS84_FLATTENING = 1.0 / 298.257223563
_WGS84_E2 = _WGS84_FLATTENING * (2.0 - _WGS84_FLATTENING)


def configured_ground_truth_source() -> str:
    source = os.environ.get(
        "SWARM_RANGE_DATASET_GROUND_TRUTH_SOURCE",
        SIMULATION_GROUND_TRUTH_SOURCE,
    ).strip()
    if source not in ALLOWED_GROUND_TRUTH_SOURCES:
        raise ValueError("range_dataset_ground_truth_source_invalid")
    return source


def _finite(value: Any, name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name}_invalid")
    return number


def _required_vector_environment(name: str) -> tuple[float, float, float]:
    raw = os.environ.get(name)
    if raw is None or not raw.strip():
        raise ValueError(f"{name.lower()}_required")
    parts = [part.strip() for part in raw.split(",")]
    if len(parts) != 3:
        raise ValueError(f"{name.lower()}_invalid")
    vector = tuple(_finite(part, name.lower()) for part in parts)
    if math.sqrt(sum(value * value for value in vector)) > 10.0:
        raise ValueError(f"{name.lower()}_outside_limit")
    return vector  # type: ignore[return-value]


def _required_nonnegative_environment(name: str) -> float:
    raw = os.environ.get(name)
    if raw is None or not raw.strip():
        raise ValueError(f"{name.lower()}_required")
    value = _finite(raw, name.lower())
    if value < 0.0:
        raise ValueError(f"{name.lower()}_invalid")
    return value


@dataclass(frozen=True)
class RtkGroundTruthConfig:
    camera_antenna_to_center_body_frd_m: tuple[float, float, float]
    target_antenna_to_center_body_frd_m: tuple[float, float, float]
    lever_arm_uncertainty_m: float
    maximum_time_offset_ms: float = 100.0
    maximum_uncertainty_m: float = 0.20

    def __post_init__(self) -> None:
        values = (
            *self.camera_antenna_to_center_body_frd_m,
            *self.target_antenna_to_center_body_frd_m,
            self.lever_arm_uncertainty_m,
            self.maximum_time_offset_ms,
            self.maximum_uncertainty_m,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("rtk_ground_truth_config_not_finite")
        if self.lever_arm_uncertainty_m < 0.0:
            raise ValueError("rtk_lever_arm_uncertainty_invalid")
        if not 0.0 < self.maximum_time_offset_ms <= 100.0:
            raise ValueError("rtk_maximum_time_offset_invalid")
        if not 0.0 < self.maximum_uncertainty_m <= 0.20:
            raise ValueError("rtk_maximum_uncertainty_invalid")
        for name, vector in (
            (
                "camera",
                self.camera_antenna_to_center_body_frd_m,
            ),
            (
                "target",
                self.target_antenna_to_center_body_frd_m,
            ),
        ):
            if len(vector) != 3:
                raise ValueError(f"rtk_{name}_lever_arm_invalid")
            if math.sqrt(sum(value * value for value in vector)) > 10.0:
                raise ValueError(f"rtk_{name}_lever_arm_outside_limit")

    @classmethod
    def from_environment(cls) -> "RtkGroundTruthConfig":
        maximum_time_offset_ms = _finite(
            os.environ.get("SWARM_RANGE_RTK_MAX_TIME_OFFSET_MS", "100"),
            "rtk_maximum_time_offset_ms",
        )
        maximum_uncertainty_m = _finite(
            os.environ.get("SWARM_RANGE_RTK_MAX_UNCERTAINTY_M", "0.20"),
            "rtk_maximum_uncertainty_m",
        )
        return cls(
            camera_antenna_to_center_body_frd_m=(
                _required_vector_environment(
                    "SWARM_RANGE_RTK_CAMERA_ANTENNA_TO_CENTER_BODY_FRD_M"
                )
            ),
            target_antenna_to_center_body_frd_m=(
                _required_vector_environment(
                    "SWARM_RANGE_RTK_TARGET_ANTENNA_TO_CENTER_BODY_FRD_M"
                )
            ),
            lever_arm_uncertainty_m=_required_nonnegative_environment(
                "SWARM_RANGE_RTK_LEVER_ARM_UNCERTAINTY_M"
            ),
            maximum_time_offset_ms=maximum_time_offset_ms,
            maximum_uncertainty_m=maximum_uncertainty_m,
        )


def _body_frd_to_ned(
    vector: Sequence[float],
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> tuple[float, float, float]:
    forward, right, down = (float(value) for value in vector)
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    return (
        cp * cy * forward
        + (sr * sp * cy - cr * sy) * right
        + (cr * sp * cy + sr * sy) * down,
        cp * sy * forward
        + (sr * sp * sy + cr * cy) * right
        + (cr * sp * sy - sr * cy) * down,
        -sp * forward + sr * cp * right + cr * cp * down,
    )


def _geodetic_to_ecef(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
) -> tuple[float, float, float]:
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    normal_radius = _WGS84_SEMI_MAJOR_M / math.sqrt(
        1.0 - _WGS84_E2 * sin_latitude * sin_latitude
    )
    return (
        (normal_radius + altitude_m) * cos_latitude * math.cos(longitude),
        (normal_radius + altitude_m) * cos_latitude * math.sin(longitude),
        (
            normal_radius * (1.0 - _WGS84_E2) + altitude_m
        )
        * sin_latitude,
    )


def _ned_to_ecef(
    vector_ned: Sequence[float],
    latitude_deg: float,
    longitude_deg: float,
) -> tuple[float, float, float]:
    north, east, down = (float(value) for value in vector_ned)
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude, cos_latitude = math.sin(latitude), math.cos(latitude)
    sin_longitude, cos_longitude = math.sin(longitude), math.cos(longitude)
    return (
        -sin_latitude * cos_longitude * north
        - sin_longitude * east
        - cos_latitude * cos_longitude * down,
        -sin_latitude * sin_longitude * north
        + cos_longitude * east
        - cos_latitude * sin_longitude * down,
        cos_latitude * north - sin_latitude * down,
    )


class RtkGroundTruthProvider:
    """Fail-closed RTK-fixed camera-center to target-center labels."""

    def __init__(
        self,
        telemetry_provider: Callable[[str], Mapping[str, Any] | None],
        config: RtkGroundTruthConfig | None,
        *,
        load_error: str = "",
    ) -> None:
        self.telemetry_provider = telemetry_provider
        self.config = config
        self.load_error = str(load_error)

    @classmethod
    def from_environment(
        cls,
        telemetry_provider: Callable[[str], Mapping[str, Any] | None],
    ) -> "RtkGroundTruthProvider":
        try:
            return cls(telemetry_provider, RtkGroundTruthConfig.from_environment())
        except (TypeError, ValueError) as error:
            return cls(telemetry_provider, None, load_error=str(error))

    @property
    def enabled(self) -> bool:
        return bool(self.config is not None and not self.load_error)

    def _vehicle(
        self,
        drone_id: str,
        measurement_timestamp_s: float,
    ) -> dict[str, Any]:
        telemetry = self.telemetry_provider(drone_id)
        if not isinstance(telemetry, Mapping) or not telemetry.get("online", False):
            raise ValueError(f"{drone_id}_telemetry_offline")
        received = _finite(
            telemetry.get("dashboard_received_monotonic_s"),
            f"{drone_id}_telemetry_timestamp",
        )
        offset_ms = abs(received - measurement_timestamp_s) * 1000.0
        assert self.config is not None
        if offset_ms > self.config.maximum_time_offset_ms:
            raise ValueError(f"{drone_id}_telemetry_not_synchronized")
        gps = telemetry.get("gps")
        position = telemetry.get("global_position")
        attitude = telemetry.get("attitude")
        if not all(isinstance(value, Mapping) for value in (gps, position, attitude)):
            raise ValueError(f"{drone_id}_rtk_fields_unavailable")
        assert isinstance(gps, Mapping)
        assert isinstance(position, Mapping)
        assert isinstance(attitude, Mapping)
        fix_type = int(_finite(gps.get("fix_type", -1), f"{drone_id}_fix_type"))
        if fix_type < RTK_FIXED_FIX_TYPE:
            raise ValueError(f"{drone_id}_rtk_not_fixed")
        latitude = _finite(position.get("latitude_deg"), f"{drone_id}_latitude")
        longitude = _finite(position.get("longitude_deg"), f"{drone_id}_longitude")
        altitude = _finite(position.get("altitude_msl_m"), f"{drone_id}_altitude")
        if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise ValueError(f"{drone_id}_global_position_invalid")
        if not -1_000.0 <= altitude <= 100_000.0:
            raise ValueError(f"{drone_id}_altitude_invalid")
        eph = _finite(gps.get("eph_m"), f"{drone_id}_eph")
        epv = _finite(gps.get("epv_m"), f"{drone_id}_epv")
        if eph < 0.0 or epv < 0.0:
            raise ValueError(f"{drone_id}_rtk_uncertainty_invalid")
        if eph > self.config.maximum_uncertainty_m or epv > (
            self.config.maximum_uncertainty_m
        ):
            raise ValueError("rtk_ground_truth_uncertainty_too_large")
        attitude_rad = tuple(
            math.radians(
                _finite(attitude.get(name), f"{drone_id}_{name}")
            )
            for name in ("roll_deg", "pitch_deg", "yaw_deg")
        )
        return {
            "position": (latitude, longitude, altitude),
            "attitude_rad": attitude_rad,
            "eph_m": eph,
            "epv_m": epv,
            "received_monotonic_s": received,
            "offset_ms": offset_ms,
            "fix_type": fix_type,
        }

    @staticmethod
    def _corrected_center_ecef(
        vehicle: Mapping[str, Any],
        lever_arm_body_frd_m: Sequence[float],
    ) -> tuple[float, float, float]:
        latitude, longitude, altitude = vehicle["position"]
        roll, pitch, yaw = vehicle["attitude_rad"]
        antenna_ecef = _geodetic_to_ecef(latitude, longitude, altitude)
        lever_ned = _body_frd_to_ned(
            lever_arm_body_frd_m,
            roll,
            pitch,
            yaw,
        )
        lever_ecef = _ned_to_ecef(lever_ned, latitude, longitude)
        return tuple(
            antenna_ecef[index] + lever_ecef[index] for index in range(3)
        )

    def evaluate(
        self,
        camera_drone_id: str,
        measurement_timestamp_s: float,
        target_drone_id: str,
    ) -> dict[str, Any]:
        if not self.enabled or self.config is None:
            return {
                "available": False,
                "error": self.load_error or "rtk_ground_truth_disabled",
                "ground_truth_source": RTK_GROUND_TRUTH_SOURCE,
            }
        try:
            timestamp = _finite(
                measurement_timestamp_s,
                "measurement_timestamp_s",
            )
            camera_id = str(camera_drone_id).strip()
            target_id = str(target_drone_id).strip()
            if not camera_id or not target_id or camera_id == target_id:
                raise ValueError("rtk_ground_truth_vehicle_ids_invalid")
            camera = self._vehicle(camera_id, timestamp)
            target = self._vehicle(target_id, timestamp)
            pair_offset_ms = abs(
                camera["received_monotonic_s"]
                - target["received_monotonic_s"]
            ) * 1000.0
            time_offset_ms = max(
                camera["offset_ms"],
                target["offset_ms"],
                pair_offset_ms,
            )
            if time_offset_ms > self.config.maximum_time_offset_ms:
                raise ValueError("rtk_pair_not_synchronized")
            uncertainty_m = math.sqrt(
                camera["eph_m"] ** 2
                + camera["epv_m"] ** 2
                + target["eph_m"] ** 2
                + target["epv_m"] ** 2
                + self.config.lever_arm_uncertainty_m**2
            )
            if uncertainty_m > self.config.maximum_uncertainty_m:
                raise ValueError("rtk_ground_truth_uncertainty_too_large")
            camera_center = self._corrected_center_ecef(
                camera,
                self.config.camera_antenna_to_center_body_frd_m,
            )
            target_center = self._corrected_center_ecef(
                target,
                self.config.target_antenna_to_center_body_frd_m,
            )
            distance_m = math.sqrt(
                sum(
                    (target_center[index] - camera_center[index]) ** 2
                    for index in range(3)
                )
            )
            if not math.isfinite(distance_m) or distance_m <= 0.0:
                raise ValueError("rtk_ground_truth_distance_invalid")
            return {
                "available": True,
                "camera_drone_id": camera_id,
                "target_drone_id": target_id,
                "camera_to_target_center_m": distance_m,
                "ground_truth_source": RTK_GROUND_TRUTH_SOURCE,
                "ground_truth_uncertainty_m": uncertainty_m,
                "ground_truth_time_offset_ms": time_offset_ms,
                "ground_truth_quality": "rtk_fixed",
                "ground_truth_lever_arm_corrected": True,
                "camera_fix_type": camera["fix_type"],
                "target_fix_type": target["fix_type"],
            }
        except (KeyError, OverflowError, TypeError, ValueError) as error:
            return {
                "available": False,
                "error": str(error),
                "ground_truth_source": RTK_GROUND_TRUTH_SOURCE,
            }


class RangeGroundTruthRouter:
    """Select exactly one label source and expose auditable status."""

    def __init__(
        self,
        source: str,
        simulation_provider: Callable[..., dict[str, Any]],
        rtk_provider: RtkGroundTruthProvider,
        *,
        load_error: str = "",
    ) -> None:
        self.source = str(source)
        self.simulation_provider = simulation_provider
        self.rtk_provider = rtk_provider
        self.load_error = str(load_error)
        self.request_count = 0
        self.available_count = 0
        self.rejected_count = 0
        self.last_reason = self.load_error or "ready"
        self._lock = threading.Lock()

    @classmethod
    def from_environment(
        cls,
        simulation_provider: Callable[..., dict[str, Any]],
        telemetry_provider: Callable[[str], Mapping[str, Any] | None],
    ) -> "RangeGroundTruthRouter":
        try:
            source = configured_ground_truth_source()
        except ValueError as error:
            return cls(
                "invalid",
                simulation_provider,
                RtkGroundTruthProvider(telemetry_provider, None),
                load_error=str(error),
            )
        rtk_provider = RtkGroundTruthProvider.from_environment(telemetry_provider)
        return cls(source, simulation_provider, rtk_provider)

    def __call__(
        self,
        camera_drone_id: str,
        measurement_timestamp_s: float,
        source_sim_timestamp_s: float | None,
        target_drone_id: str,
    ) -> dict[str, Any]:
        if self.load_error:
            result = {"available": False, "error": self.load_error}
        elif self.source == RTK_GROUND_TRUTH_SOURCE:
            result = self.rtk_provider.evaluate(
                camera_drone_id,
                measurement_timestamp_s,
                target_drone_id,
            )
        else:
            result = dict(
                self.simulation_provider(
                    camera_drone_id,
                    source_sim_timestamp_s,
                    target_drone_id,
                )
            )
            if result.get("available", False):
                result.update(
                    {
                        "ground_truth_source": SIMULATION_GROUND_TRUTH_SOURCE,
                        "ground_truth_uncertainty_m": 0.0,
                        "ground_truth_time_offset_ms": float(
                            result.get("pose_age_ms", 0.0)
                        ),
                        "ground_truth_quality": "simulation_exact",
                        "ground_truth_lever_arm_corrected": True,
                    }
                )
        available = bool(result.get("available", False))
        reason = "ok" if available else str(
            result.get("error", "ground_truth_unavailable")
        )
        with self._lock:
            self.request_count += 1
            if available:
                self.available_count += 1
            else:
                self.rejected_count += 1
            self.last_reason = reason
        return result

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "source": self.source,
                "load_error": self.load_error,
                "enabled": not bool(self.load_error)
                and (
                    self.source == SIMULATION_GROUND_TRUTH_SOURCE
                    or self.rtk_provider.enabled
                ),
                "request_count": self.request_count,
                "available_count": self.available_count,
                "rejected_count": self.rejected_count,
                "last_reason": self.last_reason,
                "rtk_provider_load_error": self.rtk_provider.load_error,
            }
