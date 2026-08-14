"""Validation and geometry for operator-drawn closed-loop flight missions.

A mission arrives from the dashboard as a list of map waypoints. This module
is the only place that turns one into a `ClosedPolylineTrajectory`, and it is
deliberately pure: no MQTT, no MAVLink, no flight authority. Both the web
front door (`main.py`) and the bridge that actually installs the trajectory
validate through here, so an operator gets an early, readable rejection and
the flight path still gets an authoritative one -- MQTT is not a trusted
channel and the bridge may not assume the dashboard already checked.

Every limit is inherited from an existing project constraint rather than
invented here: the CBF geofence box, the CBF velocity limit, and the CBF
minimum separation.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

from swarm_state import GeodeticOrigin, geodetic_to_enu
from trajectory_controller import ClosedPolylineTrajectory

Vector3 = tuple[float, float, float]

# A drawn loop with more vertices than this is a slip of the mouse, not a
# plan; 64 still allows a circle smooth enough that corner cutting is a
# rounding error rather than a shape change.
MAXIMUM_WAYPOINTS = 64
# Below this an edge is shorter than the corner-cutting scale of the vehicle
# and only adds a vertex the tracker cannot resolve.
MINIMUM_SEGMENT_M = 1.0
# The normal dashboard keeps headroom below the CBF ceiling. A validated SITL
# profile may explicitly raise SWARM_MISSION_MAXIMUM_VELOCITY_M_S; it is still
# clamped to the CBF ceiling here, at both dashboard and bridge trust boundaries.
SPEED_FRACTION_OF_CBF_LIMIT = 0.5


class MissionRejected(ValueError):
    """A mission that must not reach the flight path, with the operator reason."""


@dataclass(frozen=True)
class MissionLimits:
    geofence_min_enu_m: Vector3
    geofence_max_enu_m: Vector3
    maximum_speed_m_s: float
    minimum_separation_m: float

    @classmethod
    def from_environment(cls) -> "MissionLimits":
        def _vector(name: str, default: str) -> Vector3:
            values = tuple(
                float(part.strip()) for part in os.environ.get(name, default).split(",")
            )
            if len(values) != 3 or not all(math.isfinite(value) for value in values):
                raise ValueError(f"{name} requires three finite ENU components")
            return values  # type: ignore[return-value]

        def _float(name: str, default: float) -> float:
            try:
                return float(os.environ.get(name, str(default)))
            except (TypeError, ValueError):
                return default

        cbf_maximum = _float("SWARM_CBF_MAXIMUM_VELOCITY_M_S", 2.0)
        mission_maximum = _float(
            "SWARM_MISSION_MAXIMUM_VELOCITY_M_S",
            cbf_maximum * SPEED_FRACTION_OF_CBF_LIMIT,
        )
        return cls(
            geofence_min_enu_m=_vector("SWARM_CBF_GEOFENCE_MIN_ENU_M", "-100,-100,0"),
            geofence_max_enu_m=_vector("SWARM_CBF_GEOFENCE_MAX_ENU_M", "100,100,50"),
            maximum_speed_m_s=min(cbf_maximum, mission_maximum),
            minimum_separation_m=_float("SWARM_CBF_MINIMUM_SEPARATION_M", 4.0),
        )


def _finite(value: Any, label: str, *, minimum: float, maximum: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise MissionRejected(f"{label} is not a number") from None
    if not math.isfinite(number):
        raise MissionRejected(f"{label} is not finite")
    if not minimum <= number <= maximum:
        raise MissionRejected(f"{label} must be between {minimum} and {maximum}")
    return number


def waypoints_to_enu(
    waypoints: Sequence[Mapping[str, Any]],
    altitude_m: float,
    origin: GeodeticOrigin,
) -> tuple[Vector3, ...]:
    """Map waypoints (degrees) to the shared ENU frame at one altitude."""
    if not isinstance(waypoints, (list, tuple)):
        raise MissionRejected("waypoints must be a list")
    if not 3 <= len(waypoints) <= MAXIMUM_WAYPOINTS:
        raise MissionRejected(
            f"a closed mission needs 3 to {MAXIMUM_WAYPOINTS} waypoints, got {len(waypoints)}"
        )
    converted = []
    for index, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, Mapping):
            raise MissionRejected(f"waypoint {index} is not an object")
        latitude = _finite(
            waypoint.get("latitude_deg"), f"waypoint {index} latitude", minimum=-90.0, maximum=90.0
        )
        longitude = _finite(
            waypoint.get("longitude_deg"),
            f"waypoint {index} longitude",
            minimum=-180.0,
            maximum=180.0,
        )
        east, north, _ = geodetic_to_enu(
            latitude, longitude, origin.altitude_msl_m, origin
        )
        converted.append((east, north, altitude_m))
    return tuple(converted)


def validate_mission(
    payload: Mapping[str, Any],
    origin: GeodeticOrigin,
    limits: MissionLimits | None = None,
) -> ClosedPolylineTrajectory:
    """Turn a dashboard mission payload into a trajectory, or refuse it."""
    limits = limits or MissionLimits.from_environment()
    altitude_m = _finite(
        payload.get("altitude_m"),
        "altitude",
        minimum=limits.geofence_min_enu_m[2],
        maximum=limits.geofence_max_enu_m[2],
    )
    speed_m_s = _finite(
        payload.get("speed_m_s"),
        "speed",
        minimum=0.1,
        maximum=limits.maximum_speed_m_s,
    )
    waypoints = waypoints_to_enu(payload.get("waypoints"), altitude_m, origin)

    for index, point in enumerate(waypoints):
        for axis, name in enumerate("ENU"):
            if not limits.geofence_min_enu_m[axis] <= point[axis] <= limits.geofence_max_enu_m[axis]:
                raise MissionRejected(
                    f"waypoint {index} is outside the geofence on {name}"
                )

    count = len(waypoints)
    for index in range(count):
        following = waypoints[(index + 1) % count]
        if math.dist(waypoints[index], following) < MINIMUM_SEGMENT_M:
            raise MissionRejected(
                f"waypoints {index} and {(index + 1) % count} are closer than "
                f"{MINIMUM_SEGMENT_M} m apart"
            )

    try:
        return ClosedPolylineTrajectory(waypoints_enu_m=waypoints, speed_m_s=speed_m_s)
    except ValueError as error:
        raise MissionRejected(str(error)) from error


def closest_approach_m(
    first: ClosedPolylineTrajectory,
    second: ClosedPolylineTrajectory,
    *,
    sample_step_m: float = 0.5,
) -> float:
    """Conservative lower bound on the distance between two drawn paths.

    Purely geometric and therefore time-independent: if this is at least the
    CBF minimum separation, no pair of drones flying these paths can ever
    breach it, whatever their phase or speed. A smaller value does NOT prove a
    conflict -- the drones may never occupy the crossing at the same moment --
    it proves only that the geometry relies on CBF intervention.

    ponytail: O(n*m) over samples of both paths, with the sampling error
    subtracted so the answer errs low. A few thousand pairs for realistic
    missions; swap in exact segment-to-segment distance if a mission ever
    grows large enough for this to show up in a profile.
    """
    def _samples(trajectory: ClosedPolylineTrajectory) -> list[Vector3]:
        perimeter = trajectory.perimeter_m()
        count = max(2, int(math.ceil(perimeter / sample_step_m)))
        lap_s = trajectory.lap_duration_s()
        return [
            trajectory.reference(lap_s * index / count).position_enu_m
            for index in range(count)
        ]

    left = _samples(first)
    right = _samples(second)
    minimum = min(math.dist(a, b) for a in left for b in right)
    # Each sample stands for up to half a step of path on either side.
    return max(0.0, minimum - sample_step_m)


def review_missions(
    trajectories: Mapping[str, ClosedPolylineTrajectory],
    limits: MissionLimits | None = None,
) -> dict[str, Any]:
    """Report which drawn paths rely on CBF to stay separated."""
    limits = limits or MissionLimits.from_environment()
    conflicts = []
    drones = sorted(trajectories)
    for index, drone in enumerate(drones):
        for other in drones[index + 1 :]:
            distance = closest_approach_m(trajectories[drone], trajectories[other])
            if distance < limits.minimum_separation_m:
                conflicts.append(
                    {
                        "drones": [drone, other],
                        "closest_approach_m": round(distance, 2),
                        "minimum_separation_m": limits.minimum_separation_m,
                    }
                )
    return {
        "geometrically_separated": not conflicts,
        "conflicts": conflicts,
        "laps": {
            drone: {
                "perimeter_m": round(trajectory.perimeter_m(), 2),
                "lap_duration_s": round(trajectory.lap_duration_s(), 1),
            }
            for drone, trajectory in sorted(trajectories.items())
        },
    }
