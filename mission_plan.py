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
from trajectory_controller import ClosedPolylineTrajectory, LinearTrajectory

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
    # What two STATIONARY vehicles must keep between them. At a hold point the
    # closing speed is zero, so uncertainty, age_latency and stopping_distance
    # all vanish from the CBF's required separation and only these two terms
    # are left. Defaulted so existing constructions keep working.
    station_keeping_separation_m: float = 0.0

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
            station_keeping_separation_m=(
                _float("SWARM_CBF_MINIMUM_SEPARATION_M", 4.0)
                + _float("SWARM_CBF_TRACKING_RESERVE_M", 0.0)
            ),
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
    # Two points draw an open line, three or more a closed loop. Both are
    # missions an operator legitimately wants: a survey leg is not a lap.
    if not 2 <= len(waypoints) <= MAXIMUM_WAYPOINTS:
        raise MissionRejected(
            f"a mission needs 2 to {MAXIMUM_WAYPOINTS} waypoints, "
            f"got {len(waypoints)}"
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


def _orientation(a: Vector3, b: Vector3, c: Vector3) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a: Vector3, b: Vector3, point: Vector3) -> bool:
    return (
        min(a[0], b[0]) - 1e-9 <= point[0] <= max(a[0], b[0]) + 1e-9
        and min(a[1], b[1]) - 1e-9 <= point[1] <= max(a[1], b[1]) + 1e-9
    )


def _segments_cross(a: Vector3, b: Vector3, c: Vector3, d: Vector3) -> bool:
    """Do closed segments ab and cd meet? Planar: a mission flies one altitude."""
    d1, d2 = _orientation(c, d, a), _orientation(c, d, b)
    d3, d4 = _orientation(a, b, c), _orientation(a, b, d)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return True
    return (
        (abs(d1) <= 1e-9 and _on_segment(c, d, a))
        or (abs(d2) <= 1e-9 and _on_segment(c, d, b))
        or (abs(d3) <= 1e-9 and _on_segment(a, b, c))
        or (abs(d4) <= 1e-9 and _on_segment(a, b, d))
    )


def self_intersecting_pair(waypoints: Sequence[Vector3]) -> tuple[int, int] | None:
    """First pair of non-adjacent segments that cross, if any.

    A crossing is not a cosmetic problem. Progress along the path is found by
    nearest point, so at the crossing the two branches are equidistant and the
    follower can pick the wrong one -- flying the mission out of order, or
    skipping the leg between. Rejecting is the small, safe answer; supporting
    it means carrying arclength state across frames.
    """
    count = len(waypoints)
    for first in range(count):
        for second in range(first + 1, count):
            adjacent = (
                second == first + 1
                or (first == 0 and second == count - 1)
            )
            if adjacent:
                continue
            if _segments_cross(
                waypoints[first],
                waypoints[(first + 1) % count],
                waypoints[second],
                waypoints[(second + 1) % count],
            ):
                return first, second
    return None


def validate_mission(
    payload: Mapping[str, Any],
    origin: GeodeticOrigin,
    limits: MissionLimits | None = None,
) -> ClosedPolylineTrajectory | LinearTrajectory:
    """Turn a dashboard mission payload into a trajectory, or refuse it.

    Two waypoints give a LinearTrajectory -- an open leg, flown once and held
    at the far end. Three or more give a closed loop, flown until stopped.
    """
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
    open_line = count == 2
    # An open line has count - 1 segments; a loop wraps back to the start.
    for index in range(count - 1 if open_line else count):
        following = waypoints[(index + 1) % count]
        if math.dist(waypoints[index], following) < MINIMUM_SEGMENT_M:
            raise MissionRejected(
                f"waypoints {index} and {(index + 1) % count} are closer than "
                f"{MINIMUM_SEGMENT_M} m apart"
            )

    if not open_line:
        # A single segment cannot cross itself, and the ambiguity this
        # refuses -- two branches equidistant from the vehicle -- cannot
        # arise on a line.
        crossing = self_intersecting_pair(waypoints)
        if crossing is not None:
            raise MissionRejected(
                f"segments {crossing[0]} and {crossing[1]} cross: the follower "
                "cannot tell the branches apart at the crossing"
            )

    try:
        if open_line:
            return LinearTrajectory(
                start_enu_m=waypoints[0],
                end_enu_m=waypoints[1],
                speed_m_s=speed_m_s,
            )
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


def unreachable_destinations(
    trajectories: Mapping[str, Any],
    limits: MissionLimits | None = None,
) -> list[dict[str, Any]]:
    """Pairs of legs whose hold points are closer than the pair can ever hover.

    `review_missions` asks a different question and is deliberately only
    advisory: whether the PATHS pass close. Crossing legs are flyable, CBF
    resolves them, and four such scenarios are certified. This asks whether the
    ENDPOINTS can be occupied at the same time, which is not a conflict to be
    resolved -- it is arithmetic the barrier can never satisfy. Two vehicles
    sent to points closer than their station-keeping floor converge, meet the
    barrier, and hover at it short of BOTH points until someone cancels, with
    the trajectory reporting finished the whole time because
    `LinearTrajectory.is_finished` is a clock and not a position check.

    Only an open leg has a hold point. A closed polyline laps forever and never
    holds anywhere, so it has no destination to conflict over.

    Unlike `validate_mission` this cannot be re-run per drone on the bridge --
    it is a property of the PAIR, and each companion knows only its own
    mission. The server is the only place that sees both, so it is the only
    place this check can live. It is not a safety gate: CBF remains that, and
    CBF is what makes the outcome a stall rather than a collision. This only
    stops an unwinnable request from being accepted in the first place.
    """
    limits = limits or MissionLimits.from_environment()
    floor = limits.station_keeping_separation_m
    holds = {
        drone: trajectory.end_enu_m
        for drone, trajectory in trajectories.items()
        if getattr(trajectory, "end_enu_m", None) is not None
    }
    conflicts = []
    drones = sorted(holds)
    for index, drone in enumerate(drones):
        for other in drones[index + 1 :]:
            distance = math.dist(holds[drone], holds[other])
            if distance < floor:
                conflicts.append(
                    {
                        "drones": [drone, other],
                        "destination_separation_m": round(distance, 2),
                        "station_keeping_separation_m": round(floor, 2),
                    }
                )
    return conflicts


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
