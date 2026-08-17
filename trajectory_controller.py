"""Deterministic ENU trajectory-tracking nominal controller.

Sibling to `formation_controller.py`: same contract (produces only a bounded
nominal velocity, no PX4/MQTT/CBF dependency), but the target is a
time-indexed reference `p_ref(t)`/`v_ref(t)` instead of a leader's live
position. Output reuses `FormationCommand` -- CBF and everything downstream
of it only ever consumed `.velocity_enu_m_s`/`.active`/`.reason`, so a second,
near-identical dataclass would add a distinction the rest of the pipeline
does not care about.

The caller supplies `mission_elapsed_s`; this module stays free of any
wall-clock or monotonic-clock concept, same reasoning as
`AltitudeHoldController` staying free of any PX4 concept.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Protocol

from formation_controller import FormationCommand, FormationConfig

Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class TrajectoryReference:
    position_enu_m: Vector3
    velocity_enu_m_s: Vector3

    def __post_init__(self) -> None:
        if not _finite_vector(self.position_enu_m) or not _finite_vector(self.velocity_enu_m_s):
            raise ValueError("trajectory reference is invalid")


class Trajectory(Protocol):
    def reference(self, t_s: float) -> TrajectoryReference: ...

    def is_finished(self, t_s: float) -> bool: ...


@dataclass(frozen=True)
class LinearTrajectory:
    """Constant-velocity straight line from `start_enu_m` to `end_enu_m`.

    Holds at `end_enu_m` with zero feed-forward once `t_s` passes the
    computed duration, rather than overshooting past the endpoint.
    """

    start_enu_m: Vector3
    end_enu_m: Vector3
    speed_m_s: float

    def __post_init__(self) -> None:
        if not _finite_vector(self.start_enu_m) or not _finite_vector(self.end_enu_m):
            raise ValueError("linear trajectory endpoints are invalid")
        if not math.isfinite(self.speed_m_s) or self.speed_m_s <= 0.0:
            raise ValueError("linear trajectory speed must be positive")

    def _duration_s(self) -> float:
        return _norm(tuple(self.end_enu_m[i] - self.start_enu_m[i] for i in range(3))) / self.speed_m_s

    def nearest_time_s(self, position_enu_m: Vector3) -> float:
        """Trajectory time at the closest point on this finite segment."""
        delta = tuple(
            self.end_enu_m[axis] - self.start_enu_m[axis] for axis in range(3)
        )
        length_squared = sum(component * component for component in delta)
        if length_squared <= 1e-18:
            return 0.0
        offset = tuple(
            position_enu_m[axis] - self.start_enu_m[axis] for axis in range(3)
        )
        fraction = max(
            0.0,
            min(
                1.0,
                sum(offset[axis] * delta[axis] for axis in range(3))
                / length_squared,
            ),
        )
        return fraction * self._duration_s()

    @property
    def waypoints_enu_m(self) -> tuple[Vector3, Vector3]:
        """The two ends. A line has waypoints like any other drawn path, and
        naming them the same thing the closed polyline does is what lets the
        mission bridge and the dashboard report an open leg without either of
        them having to know which kind it is holding."""
        return (self.start_enu_m, self.end_enu_m)

    def perimeter_m(self) -> float:
        """Length of the leg. Named for the closed-polyline protocol the
        mission bridge reads, which has no notion of an open path -- for a
        line there is no perimeter, only the distance from end to end."""
        return _norm(
            tuple(self.end_enu_m[i] - self.start_enu_m[i] for i in range(3))
        )

    def lap_duration_s(self) -> float:
        """Time to fly it once. A line is not a lap and does not repeat."""
        return self._duration_s()

    def is_finished(self, t_s: float) -> bool:
        return t_s >= self._duration_s()

    def reference(self, t_s: float) -> TrajectoryReference:
        duration = self._duration_s()
        if duration <= 1e-9 or t_s >= duration:
            return TrajectoryReference(self.end_enu_m, (0.0, 0.0, 0.0))
        fraction = max(0.0, t_s) / duration
        position = tuple(
            self.start_enu_m[i] + fraction * (self.end_enu_m[i] - self.start_enu_m[i]) for i in range(3)
        )
        direction = tuple((self.end_enu_m[i] - self.start_enu_m[i]) / duration for i in range(3))
        return TrajectoryReference(position, direction)  # type: ignore[arg-type]


@dataclass(frozen=True)
class CircularTrajectory:
    """Constant-angular-rate orbit around `center_enu_m` at its own altitude.

    Never finishes (`is_finished` is always False) -- a loiter, not a
    point-to-point leg.
    """

    center_enu_m: Vector3
    radius_m: float
    angular_rate_rad_s: float
    start_angle_rad: float = 0.0

    def __post_init__(self) -> None:
        if not _finite_vector(self.center_enu_m):
            raise ValueError("circular trajectory center is invalid")
        if not math.isfinite(self.radius_m) or self.radius_m <= 0.0:
            raise ValueError("circular trajectory radius must be positive")
        if not math.isfinite(self.angular_rate_rad_s) or self.angular_rate_rad_s == 0.0:
            raise ValueError("circular trajectory angular rate must be nonzero")
        if not math.isfinite(self.start_angle_rad):
            raise ValueError("circular trajectory start angle is invalid")

    def is_finished(self, t_s: float) -> bool:
        return False

    def reference(self, t_s: float) -> TrajectoryReference:
        angle = self.start_angle_rad + self.angular_rate_rad_s * t_s
        cx, cy, cz = self.center_enu_m
        position = (
            cx + self.radius_m * math.cos(angle),
            cy + self.radius_m * math.sin(angle),
            cz,
        )
        tangential = self.radius_m * self.angular_rate_rad_s
        velocity = (-tangential * math.sin(angle), tangential * math.cos(angle), 0.0)
        return TrajectoryReference(position, velocity)


@dataclass(frozen=True)
class ClosedPolylineTrajectory:
    """Requested-speed lap around a closed polygon of ENU waypoints.

    One shape covers everything the mission planner can draw: a square is four
    waypoints, a circle is a many-sided polygon, a freehand loop is whatever
    the operator clicked. The polygon is implicitly closed -- do not repeat the
    first waypoint at the end.

    Never finishes (like `CircularTrajectory`): this is a lap, not a leg, so
    `TrajectoryTrackingController` never reports `trajectory_reached` and the
    success criterion becomes cross-track error rather than arrival at a point.

    The geometric path remains exactly what the operator drew. The tracking
    controller rounds each vertex inside a configurable cross-track tolerance
    and brakes early enough to make that turn with the configured acceleration.
    """

    waypoints_enu_m: tuple[Vector3, ...]
    speed_m_s: float

    def __post_init__(self) -> None:
        if len(self.waypoints_enu_m) < 3:
            raise ValueError("closed polyline needs at least three waypoints")
        if not all(_finite_vector(point) for point in self.waypoints_enu_m):
            raise ValueError("closed polyline waypoints are invalid")
        if not math.isfinite(self.speed_m_s) or self.speed_m_s <= 0.0:
            raise ValueError("closed polyline speed must be positive")
        if any(length <= 1e-6 for _, _, length in self._segments()):
            raise ValueError("closed polyline has a zero-length segment")

    def _segments(self) -> list[tuple[Vector3, Vector3, float]]:
        """(start, unit direction, length) per edge, wrapping to close the loop."""
        segments = []
        count = len(self.waypoints_enu_m)
        for index in range(count):
            start = self.waypoints_enu_m[index]
            end = self.waypoints_enu_m[(index + 1) % count]
            delta = tuple(end[axis] - start[axis] for axis in range(3))
            length = _norm(delta)  # type: ignore[arg-type]
            direction = (
                tuple(component / length for component in delta)
                if length > 1e-9
                else (0.0, 0.0, 0.0)
            )
            segments.append((start, direction, length))
        return segments  # type: ignore[return-value]

    def perimeter_m(self) -> float:
        return sum(length for _, _, length in self._segments())

    def lap_duration_s(self) -> float:
        return self.perimeter_m() / self.speed_m_s

    def nearest_time_s(self, position_enu_m: Vector3) -> float:
        """Lap time whose reference point is closest to `position_enu_m`.

        A loop has no privileged starting vertex, so a vehicle joining it
        should pick up wherever it already is rather than fly to waypoint 0
        first. Exact per-edge projection, not sampling: the answer decides
        where a real vehicle is sent.
        """
        best_distance = math.inf
        best_arclength = 0.0
        travelled = 0.0
        for start, direction, length in self._segments():
            offset = tuple(position_enu_m[axis] - start[axis] for axis in range(3))
            along = max(
                0.0,
                min(length, sum(offset[axis] * direction[axis] for axis in range(3))),
            )
            closest = tuple(start[axis] + direction[axis] * along for axis in range(3))
            distance = math.dist(position_enu_m, closest)
            if distance < best_distance:
                best_distance = distance
                best_arclength = travelled + along
            travelled += length
        return best_arclength / self.speed_m_s

    def is_finished(self, t_s: float) -> bool:
        return False

    def reference(self, t_s: float) -> TrajectoryReference:
        segments = self._segments()
        perimeter = sum(length for _, _, length in segments)
        # Wrap by arclength, not by time: the lap repeats forever and float
        # time keeps growing, so the modulo has to happen before the walk.
        distance = (self.speed_m_s * max(0.0, t_s)) % perimeter
        for start, direction, length in segments:
            if distance <= length:
                position = tuple(
                    start[axis] + direction[axis] * distance for axis in range(3)
                )
                velocity = tuple(component * self.speed_m_s for component in direction)
                return TrajectoryReference(position, velocity)  # type: ignore[arg-type]
            distance -= length
        # Only reachable through float rounding at the very end of a lap.
        start, direction, _ = segments[-1]
        return TrajectoryReference(
            self.waypoints_enu_m[0],
            tuple(component * self.speed_m_s for component in direction),  # type: ignore[arg-type]
        )


@dataclass(frozen=True)
class SquareWaveVelocityTrajectory:
    """Bounded, zero-net-drift velocity square wave, for latency system-ID.

    Built for `ACTIVE_CBF_CROSSING_TRAJECTORY`'s blocker: two idealized-
    simulation retries both failed in real flight because `command_latency_s`
    (the CBF margin's assumed command-to-motion delay) has never been
    measured, and neither of two independent estimation attempts on cruise/
    crossing telemetry after the fact produced a trustworthy number (see the
    2026-08-10 handoff). A dedicated step input is the standard fix: excite
    the real dynamics directly instead of inferring lag from telemetry that
    was never designed to expose it.

    `step_velocity_enu_m_s` alternates with its own negation every
    `half_period_s`, exactly like a square wave. Position is the closed-form
    open-loop integral of that SCHEDULED velocity (never read from real
    state), same pattern as `LinearTrajectory` -- so the scheduled edge times
    are known exactly regardless of how well the real vehicle tracks them,
    and any real lag shows up purely as accumulated `nominal_position_error_m`
    for free. Unlike `LinearTrajectory`, position never drifts past one
    period's amplitude from `start_enu_m` (a triangle wave, not a ramp): many
    edges can be flown back-to-back inside one small, bounded volume rather
    than one flight buying only one step. Never finishes (like
    `CircularTrajectory`): this is a repeated-cycle measurement, not a
    point-to-point leg.
    """

    start_enu_m: Vector3
    step_velocity_enu_m_s: Vector3
    half_period_s: float

    def __post_init__(self) -> None:
        if not _finite_vector(self.start_enu_m) or not _finite_vector(self.step_velocity_enu_m_s):
            raise ValueError("square wave trajectory vectors are invalid")
        if not math.isfinite(self.half_period_s) or self.half_period_s <= 0.0:
            raise ValueError("square wave trajectory half_period_s must be positive")

    def is_finished(self, t_s: float) -> bool:
        return False

    def reference(self, t_s: float) -> TrajectoryReference:
        t = max(0.0, t_s)
        period = 2.0 * self.half_period_s
        phase = t - period * math.floor(t / period)
        if phase < self.half_period_s:
            velocity = self.step_velocity_enu_m_s
            offset = tuple(v * phase for v in velocity)
        else:
            velocity = tuple(-v for v in self.step_velocity_enu_m_s)
            peak = tuple(v * self.half_period_s for v in self.step_velocity_enu_m_s)
            offset = tuple(peak[i] + velocity[i] * (phase - self.half_period_s) for i in range(3))
        position = tuple(self.start_enu_m[i] + offset[i] for i in range(3))
        return TrajectoryReference(position, velocity)  # type: ignore[arg-type]


def _turn_angle_rad(incoming: Vector3, outgoing: Vector3) -> float:
    """How far the path bends between two unit segment directions."""
    return math.acos(
        max(-1.0, min(1.0, sum(incoming[i] * outgoing[i] for i in range(3))))
    )


def fillet_radius_m(tracking_tolerance_m: float, turn_angle_rad: float) -> float:
    """Largest fillet whose arc still passes within `tracking_tolerance_m` of the corner.

    An arc of radius r tangent to both edges has its centre r/cos(half) from the
    vertex, so it misses the vertex by r*(1/cos(half) - 1).  Solving that for the
    tolerance is what keeps the turn inside its budget; leaving the cosine out
    overshoots by 1.41x at a 90-degree turn and 2x at 120.
    """
    cosine_half = math.cos(0.5 * turn_angle_rad)
    return tracking_tolerance_m * cosine_half / max(1e-9, 1.0 - cosine_half)


def braking_speed_limit_m_s(
    corner_speed_m_s: float,
    braking_distance_m: float,
    acceleration_m_s2: float,
    response_time_constant_s: float = 0.0,
) -> float:
    """Fastest approach speed that still arrives at a corner at its limit.

    Without lag this is the textbook `v^2 = vc^2 + 1.4*a*d`. With it, the
    vehicle spends the first `v * tau` of the braking run still travelling at
    the approach speed, because its velocity has not caught up to the command
    yet -- so the distance actually available for braking is `d - v * tau`.
    Substituting that back in leaves a quadratic in v with one positive root.

    This is the other half of the response-lag correction, and the larger half:
    capping the corner speed alone left the vehicle arriving at a 90 degree
    corner at 3.99 m/s against a 1.08 m/s command, because it had never been
    given room to shed the speed.
    """
    reachable = corner_speed_m_s**2 + 1.4 * acceleration_m_s2 * braking_distance_m
    if response_time_constant_s <= 0.0:
        return math.sqrt(reachable)
    lead = 1.4 * acceleration_m_s2 * response_time_constant_s
    return 0.5 * (-lead + math.sqrt(lead * lead + 4.0 * reachable))


def corner_profile(
    turn_angle_rad: float,
    edge_a_m: float,
    edge_b_m: float,
    ceiling_m_s: float,
    acceleration_m_s2: float,
    tracking_tolerance_m: float,
    response_time_constant_s: float = 0.0,
) -> tuple[float, float, float]:
    """Fillet radius, the speed it supports, and its tangent length.

    The fillet is pure geometry: it says where an arc of radius r sits relative
    to the vertex, and it is exact for a vehicle that adopts a commanded
    velocity instantly. A real one does not. Sparrow's velocity lags its
    command with a measured 0.860 s time constant, so it carries its approach
    heading roughly `v * tau` past the point the turn was commanded, and that
    displacement adds to the geometric miss the fillet already budgets for.
    Measured on a 240 m square with a 1 m tolerance and tau = 0.86 s: at
    0.5 m/s2 the corner speed is 0.93 m/s and cross-track peaks at 0.65 m, but
    at 4.0 m/s2 it is 2.64 m/s and cross-track reaches 3.49 m -- the geometry
    was inside budget the whole time and the aircraft was not.

    Holding `v * tau` inside the same tolerance keeps the lag term the same
    size as the geometric one, which measured 0.91 m at the resulting speed.
    `response_time_constant_s = 0` restores the pure-geometry behaviour, which
    is what every caller that has not measured its vehicle should keep.
    """
    if turn_angle_rad <= 1e-6:
        return 0.0, ceiling_m_s, 0.0
    tangent = math.tan(0.5 * turn_angle_rad)
    radius_for_error_m = fillet_radius_m(tracking_tolerance_m, turn_angle_rad)
    radius_for_edges_m = (
        0.45 * min(edge_a_m, edge_b_m) / tangent
        if tangent > 1e-9
        else radius_for_error_m
    )
    radius_m = max(0.0, min(radius_for_error_m, radius_for_edges_m))
    lateral_limit_m_s = 0.85 * math.sqrt(acceleration_m_s2 * radius_m)
    if response_time_constant_s > 0.0:
        # While the velocity catches up the vehicle keeps its approach heading
        # for `v * tau` of travel, which puts it `v * tau * sin(half)` off a
        # path that has already turned. Holding that inside the same tolerance
        # is the lag's share of the corner budget. The sine matters: an
        # angle-blind `tolerance / tau` would hold a 1 degree bend to the same
        # 1.16 m/s as a right-angle one, and the corner window on a gentle bend
        # covers nearly half the leg.
        sine_half = math.sin(0.5 * turn_angle_rad)
        if sine_half > 1e-9:
            lateral_limit_m_s = min(
                lateral_limit_m_s,
                tracking_tolerance_m / (response_time_constant_s * sine_half),
            )
    speed_m_s = min(ceiling_m_s, max(0.5, lateral_limit_m_s))
    return radius_m, speed_m_s, radius_m * tangent


def mission_speed_preview(
    trajectory: "ClosedPolylineTrajectory | LinearTrajectory",
    *,
    maximum_acceleration_m_s2: float,
    corner_tracking_tolerance_m: float,
    response_time_constant_s: float = 0.0,
) -> dict[str, float]:
    """What the drone will actually fly, before anyone is told it will cruise.

    The requested speed is a ceiling, not a promise: every corner imposes its
    own limit, and a leg can be too short to accelerate back up. Showing the
    operator the requested number when the geometry forbids it is the one
    dishonest thing the mission pipeline could do, so this returns the real
    profile for the dashboard to display next to the request.
    """
    if isinstance(trajectory, LinearTrajectory):
        # An open line has no corner to slow for and no next leg to brake
        # into: it accelerates once and holds at the end. The only thing that
        # can hold it under the request is the leg being too short to reach
        # it, which is the same v^2 = 2*a*L the corner case uses.
        length_m = trajectory.perimeter_m()
        achievable_m_s = min(
            trajectory.speed_m_s,
            math.sqrt(max(0.0, maximum_acceleration_m_s2 * length_m)),
        )
        return {
            "requested_speed_m_s": round(trajectory.speed_m_s, 3),
            "achievable_speed_m_s": round(achievable_m_s, 3),
            # No corner exists, so the slowest one is the leg itself.
            "slowest_corner_m_s": round(achievable_m_s, 3),
            "reaches_requested_speed": achievable_m_s
            >= trajectory.speed_m_s - 1e-6,
            "estimated_lap_s": round(length_m / max(0.1, achievable_m_s), 1),
        }
    segments = trajectory._segments()
    ceiling_m_s = trajectory.speed_m_s
    corner_speeds = []
    for index, (_, direction, length_m) in enumerate(segments):
        _, next_direction, next_length_m = segments[(index + 1) % len(segments)]
        _, speed_m_s, _ = corner_profile(
            _turn_angle_rad(direction, next_direction),
            length_m,
            next_length_m,
            ceiling_m_s,
            maximum_acceleration_m_s2,
            corner_tracking_tolerance_m,
            response_time_constant_s,
        )
        corner_speeds.append(speed_m_s)

    # Between two corners a leg can only reach what it can brake back down
    # from: v^2 = v_corner^2 + 2*a*(L/2) each way, hence the halved length.
    leg_peaks = []
    for index, (_, _, length_m) in enumerate(segments):
        entry_m_s = corner_speeds[index - 1]
        exit_m_s = corner_speeds[index]
        reachable_m_s = math.sqrt(
            min(entry_m_s, exit_m_s) ** 2 + maximum_acceleration_m_s2 * length_m
        )
        leg_peaks.append(min(ceiling_m_s, reachable_m_s))

    lap_s = sum(
        length_m / max(0.1, 0.5 * (leg_peaks[index] + corner_speeds[index]))
        for index, (_, _, length_m) in enumerate(segments)
    )
    return {
        "requested_speed_m_s": round(ceiling_m_s, 3),
        "achievable_speed_m_s": round(max(leg_peaks), 3),
        "slowest_corner_m_s": round(min(corner_speeds), 3),
        "reaches_requested_speed": max(leg_peaks) >= ceiling_m_s - 1e-6,
        "estimated_lap_s": round(lap_s, 1),
    }


class TrajectoryTrackingController:
    """P-controller tracking a time-indexed `Trajectory`, feed-forward `v_ref(t)`."""

    def __init__(
        self,
        drone_id: str,
        trajectory: Trajectory,
        config: FormationConfig | None = None,
        *,
        maximum_acceleration_m_s2: float | None = None,
        corner_tracking_tolerance_m: float | None = None,
        response_time_constant_s: float = 0.0,
    ) -> None:
        if not drone_id.strip():
            raise ValueError("drone_id is required")
        if maximum_acceleration_m_s2 is not None and (
            not math.isfinite(maximum_acceleration_m_s2)
            or maximum_acceleration_m_s2 <= 0.0
        ):
            raise ValueError("maximum_acceleration_m_s2 must be positive")
        if corner_tracking_tolerance_m is not None and (
            not math.isfinite(corner_tracking_tolerance_m)
            or corner_tracking_tolerance_m <= 0.0
        ):
            raise ValueError("corner_tracking_tolerance_m must be positive")
        if not math.isfinite(response_time_constant_s) or response_time_constant_s < 0.0:
            raise ValueError("response_time_constant_s must be zero or positive")
        self.drone_id = drone_id
        self.trajectory = trajectory
        self.config = config or FormationConfig()
        self.maximum_acceleration_m_s2 = maximum_acceleration_m_s2
        self.corner_tracking_tolerance_m = corner_tracking_tolerance_m
        # Zero means "assume the vehicle adopts commanded velocity instantly",
        # which is the behaviour every caller had before this existed.
        self.response_time_constant_s = response_time_constant_s
        self._previous_command_time_s: float | None = None
        self._previous_command_velocity_enu_m_s: Vector3 | None = None

    def reset_velocity_limiter(self) -> None:
        """Forget command history when trajectory authority is released."""
        self._previous_command_time_s = None
        self._previous_command_velocity_enu_m_s = None

    def _limit_acceleration(
        self,
        requested: Vector3,
        mission_elapsed_s: float,
        swarm_state: dict[str, Any],
    ) -> Vector3:
        maximum = self.maximum_acceleration_m_s2
        if maximum is None:
            return requested

        previous_time = self._previous_command_time_s
        previous = self._previous_command_velocity_enu_m_s
        if previous_time is None or previous is None or mission_elapsed_s <= previous_time:
            previous = (
                _usable_vector(swarm_state.get(self.drone_id), "velocity_enu_m_s")
                or (0.0, 0.0, 0.0)
            )
            delta_limit = 0.0
        else:
            delta_limit = maximum * (mission_elapsed_s - previous_time)

        delta = tuple(requested[axis] - previous[axis] for axis in range(3))
        limited_delta = _limit_norm(delta, delta_limit)
        limited = tuple(previous[axis] + limited_delta[axis] for axis in range(3))
        self._previous_command_time_s = mission_elapsed_s
        self._previous_command_velocity_enu_m_s = limited  # type: ignore[assignment]
        return limited  # type: ignore[return-value]

    def _corner_profile(
        self,
        turn_angle_rad: float,
        edge_a_m: float,
        edge_b_m: float,
        ceiling_m_s: float,
        acceleration_m_s2: float,
    ) -> tuple[float, float, float]:
        assert self.corner_tracking_tolerance_m is not None
        return corner_profile(
            turn_angle_rad,
            edge_a_m,
            edge_b_m,
            ceiling_m_s,
            acceleration_m_s2,
            self.corner_tracking_tolerance_m,
            self.response_time_constant_s,
        )

    def _command_closed_polyline(
        self,
        mission_elapsed_s: float,
        own_position: Vector3,
        swarm_state: dict[str, Any],
    ) -> FormationCommand:
        """Follow the drawn path from measured progress, not a runaway clock."""
        trajectory = self.trajectory
        assert isinstance(trajectory, ClosedPolylineTrajectory)

        nearest_time_s = trajectory.nearest_time_s(own_position)
        nearest = trajectory.reference(nearest_time_s)
        cross_track_error_m = math.dist(own_position, nearest.position_enu_m)
        ceiling_m_s = min(trajectory.speed_m_s, self.config.maximum_velocity_m_s)
        segments = trajectory._segments()
        perimeter_m = sum(length for _, _, length in segments)
        arclength_m = (nearest_time_s * trajectory.speed_m_s) % perimeter_m
        segment_index = 0
        along_segment_m = arclength_m
        for index, (_, _, length) in enumerate(segments):
            segment_index = index
            if along_segment_m <= length:
                break
            along_segment_m -= length

        _, direction, segment_length_m = segments[segment_index]
        _, next_direction, next_length_m = segments[(segment_index + 1) % len(segments)]
        _, previous_direction, previous_length_m = segments[segment_index - 1]
        distance_to_corner_m = max(0.0, segment_length_m - along_segment_m)
        speed_limit_m_s = ceiling_m_s
        corner_radius_m = 0.0
        acceleration = self.maximum_acceleration_m_s2
        if acceleration is not None and self.corner_tracking_tolerance_m is not None:
            # Backward pass: brake down to the corner ahead.
            corner_radius_m, corner_speed_m_s, tangent_distance_m = self._corner_profile(
                _turn_angle_rad(direction, next_direction),
                segment_length_m,
                next_length_m,
                ceiling_m_s,
                acceleration,
            )
            braking_distance_m = max(0.0, distance_to_corner_m - tangent_distance_m)
            speed_limit_m_s = min(
                speed_limit_m_s,
                braking_speed_limit_m_s(
                    corner_speed_m_s,
                    braking_distance_m,
                    acceleration,
                    self.response_time_constant_s,
                ),
            )
            # Forward pass: accelerate away from the corner behind. Crossing the
            # vertex reopens the ceiling to cruise while the velocity vector is
            # still rotating, and the acceleration budget then goes into speed
            # instead of into the turn -- which is what pushes the drone wide on
            # the way out. Measured on a 300 m square at 25 m/s: 3.03 m of
            # cross-track without this pass, 2.00 m with it.
            _, exit_speed_m_s, exit_tangent_m = self._corner_profile(
                _turn_angle_rad(previous_direction, direction),
                previous_length_m,
                segment_length_m,
                ceiling_m_s,
                acceleration,
            )
            exit_distance_m = max(0.0, along_segment_m - exit_tangent_m)
            speed_limit_m_s = min(
                speed_limit_m_s,
                math.sqrt(exit_speed_m_s**2 + 1.4 * acceleration * exit_distance_m),
            )

        # Look ahead only far enough to round this corner inside its allowed
        # radius. The old v^2/a lookahead used the 15 m/s straight-line speed
        # and started cutting a 90-degree corner tens of metres too early.
        physical_lookahead_m = (
            speed_limit_m_s**2 / acceleration
            if acceleration is not None
            else speed_limit_m_s
        )
        if corner_radius_m > 0.0:
            physical_lookahead_m = min(physical_lookahead_m, corner_radius_m)
        shortest_edge_m = min(length for _, _, length in segments)
        lookahead_m = min(
            max(0.5 * speed_limit_m_s, physical_lookahead_m),
            0.5 * shortest_edge_m,
        )
        target = trajectory.reference(
            nearest_time_s + lookahead_m / trajectory.speed_m_s
        )
        error = tuple(
            target.position_enu_m[axis] - own_position[axis] for axis in range(3)
        )
        distance_to_target_m = _norm(error)
        requested = (
            tuple(component * speed_limit_m_s / distance_to_target_m for component in error)
            if distance_to_target_m > 1e-9
            else _limit_norm(target.velocity_enu_m_s, speed_limit_m_s)
        )
        return FormationCommand(
            self.drone_id,
            target.position_enu_m,
            self._limit_acceleration(
                requested, mission_elapsed_s, swarm_state  # type: ignore[arg-type]
            ),
            True,
            "tracking_trajectory",
            cross_track_error_m,
        )

    def command(self, mission_elapsed_s: float, swarm_state: dict[str, Any]) -> FormationCommand:
        if not math.isfinite(mission_elapsed_s) or mission_elapsed_s < 0.0:
            return _hold(self.drone_id, "mission_elapsed_invalid")
        own_position = _usable_vector(swarm_state.get(self.drone_id), "position_enu_m")
        if own_position is None:
            return _hold(self.drone_id, "own_state_invalid")

        if isinstance(self.trajectory, ClosedPolylineTrajectory):
            return self._command_closed_polyline(
                mission_elapsed_s, own_position, swarm_state
            )

        # A straight leg is followed from measured progress, like the closed
        # polyline above and for the same reason. Parameterised by the mission
        # clock, the reference leaves the start point at full cruise the
        # instant the lap latches, while the vehicle is still accelerating at
        # maximum_acceleration_m_s2. `error` is then dominated by a purely
        # transient along-track gap of v*t - a*t^2/2, peaking at v^2/2a: 48.5 m
        # at 19.69 m/s and 4 m/s^2. Measured on the 20 m/s rung, 46.8 m, and
        # already 5.84 m by t=0.30 s against a predicted 5.73.
        #
        # That gap is not a tracking failure -- it is the vehicle obeying its
        # acceleration limit -- but the companion's runaway guard reads it as
        # one at 5 m, resets the lap, and the vehicle never escapes the entry
        # loop. It peaked at 2.391 m/s over a full 120 s hold. With the guard
        # raised past the transient so the loop could not close, the same
        # profile reached 20.77 m/s.
        #
        # Projected, `error` carries no along-track term at all -- only
        # cross-track -- so it stays small at any speed and acceleration, and
        # the 5 m guard means what it says again.
        #
        # Only LinearTrajectory: CircularTrajectory has no projection, and
        # SquareWaveVelocityTrajectory is a velocity profile for system
        # identification where position projection is meaningless.
        progress_s = (
            self.trajectory.nearest_time_s(own_position)
            if isinstance(self.trajectory, LinearTrajectory)
            else mission_elapsed_s
        )
        reference = self.trajectory.reference(progress_s)
        error = tuple(reference.position_enu_m[i] - own_position[i] for i in range(3))
        error_norm = _norm(error)

        if self.trajectory.is_finished(progress_s) and error_norm <= self.config.arrival_radius_m:
            return FormationCommand(
                self.drone_id, reference.position_enu_m, (0.0, 0.0, 0.0), True, "trajectory_reached", error_norm
            )

        requested = tuple(
            reference.velocity_enu_m_s[i] + self.config.position_gain_s_inv * error[i] for i in range(3)
        )
        ceiling_m_s = self.config.maximum_velocity_m_s
        if (
            isinstance(self.trajectory, LinearTrajectory)
            and self.maximum_acceleration_m_s2 is not None
        ):
            # Brake into the endpoint instead of arriving at cruise and
            # discovering the wall. A closed polyline already does this for
            # every corner; the end of an open leg is the same problem, a
            # place the vehicle has to reach at rest.
            #
            # The proportional term alone cannot do it. Past the end the
            # reference feeds forward zero and the P term brakes, but under an
            # acceleration limit that takes v^2/2a -- 50 m at 20 m/s and
            # 4 m/s^2 -- so the vehicle sails past. Measured in the offline
            # matrix once it started obeying that limit: the 20 m/s cases
            # overshot by 32 to 58 m and settled OUTSIDE the geofence, ten
            # cases that were safe the whole way and simply never arrived.
            remaining_m = _norm(
                tuple(
                    self.trajectory.end_enu_m[i] - own_position[i] for i in range(3)
                )
            )
            ceiling_m_s = min(
                ceiling_m_s,
                braking_speed_limit_m_s(
                    0.0,
                    remaining_m,
                    self.maximum_acceleration_m_s2,
                    self.response_time_constant_s,
                ),
            )
        requested = _limit_norm(requested, ceiling_m_s)
        return FormationCommand(
            self.drone_id,
            reference.position_enu_m,
            self._limit_acceleration(requested, mission_elapsed_s, swarm_state),
            True,
            "tracking_trajectory",
            error_norm,
        )


def _hold(drone_id: str, reason: str) -> FormationCommand:
    return FormationCommand(drone_id, None, (0.0, 0.0, 0.0), False, reason, None)


def _usable_vector(state: Any, field: str) -> Vector3 | None:
    if not isinstance(state, dict) or not state.get("valid", False):
        return None
    value = state.get(field)
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        vector = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        return None
    return vector if _finite_vector(vector) else None


def _finite_vector(vector: Vector3) -> bool:
    return all(math.isfinite(item) for item in vector)


def _norm(vector: Vector3) -> float:
    return math.sqrt(sum(item * item for item in vector))


def _limit_norm(vector: Vector3, maximum: float) -> Vector3:
    magnitude = _norm(vector)
    if magnitude <= maximum:
        return vector
    scale = maximum / magnitude
    return tuple(item * scale for item in vector)  # type: ignore[return-value]
