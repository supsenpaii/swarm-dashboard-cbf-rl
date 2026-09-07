"""Companion-local nominal controller and CBF safety filter.

The server (`main.py`) also runs a formation nominal and a CBF gate, but it
runs them for observability: it sits outside the collision-avoidance loop, so
losing it must not remove collision avoidance. This module is the companion
side of that split -- it is driven entirely by the drone's own PX4 state and
by neighbour states received over the direct peer-to-peer link, never by the
server.

Two deliberate differences from the server-side wiring in `main.py`:

* CBF runs on **every** frame, not only when the formation nominal is active.
  A drone with no formation slot (the leader) still has neighbours, and a zero
  nominal is not automatically safe: when a neighbour is closing inside the
  barrier, satisfying `h_dot + alpha*h >= 0` requires actively moving away, so
  the filter can and should turn a zero nominal into an evasive command.
* Configuration comes from the same environment variables the server reads, so
  the two layers cannot silently disagree about separation or geofence.

Pipeline, per architecture doc §24's minimal companion architecture:

    nominal command -> local CBF -> EmergencySupervisor -> command validator -> shadow output

`EmergencySupervisor` implements the §20 fallback ladder for whenever CBF
cannot produce a feasible command; see `emergency_supervisor.py` for the
ladder itself and its documented ambiguity resolutions. The command
validator here is the last line of defence: it rejects any non-finite or
over-limit output regardless of which upstream stage produced it.

Authority: shadow only. Nothing here is sent to PX4.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
import time
from typing import Any

from cbf_command_gate import (
    CbfCommand,
    CbfCommandGate,
    CbfConfig,
    cbf_covariance_sigma,
    cbf_position_covariance_required,
)
from cbf_rl_shadow import CbfRlShadow
from conflict_coordinator import (
    ConflictCoordinator,
    x500_20m_conflict_config,
)
from emergency_supervisor import EmergencyConfig, EmergencyDecision, EmergencySupervisor
from formation_controller import (
    AltitudeHoldConfig,
    AltitudeHoldController,
    DeterministicFormationController,
    FormationCommand,
    FormationConfig,
    FormationSlot,
)
from trajectory_controller import (
    CircularTrajectory,
    ClosedPolylineTrajectory,
    LinearTrajectory,
    SquareWaveVelocityTrajectory,
    Trajectory,
    TrajectoryTrackingController,
)


Vector3 = tuple[float, float, float]

ZERO_VELOCITY: Vector3 = (0.0, 0.0, 0.0)


def _coordinated_cbf_rl(
    drone_id: str, all_drone_ids: tuple[str, ...]
) -> CbfRlShadow:
    runtime = CbfRlShadow.from_environment()
    peer_id = next(other for other in all_drone_ids if other != drone_id)
    config = None
    if runtime.policy is not None:
        if runtime.policy.vehicle_profile == "x500":
            config = x500_20m_conflict_config(runtime.policy.maximum_velocity_m_s)
    runtime.coordinator = ConflictCoordinator.shared(drone_id, peer_id, config)
    return runtime


def _command_acceleration_limit(
    trajectory: Any, configured_m_s2: float
) -> float | None:
    """The command rate limit for this path, or None to leave it unlimited.

    Applies to every trajectory a vehicle is asked to FLY. It used to apply
    only to a closed polyline, which left LinearTrajectory -- every corridor
    rung, and now every two-point drawn mission -- commanding a step change
    to cruise and letting PX4 sort it out.

    That is not a cosmetic difference. It is why the corridor replay could
    not reproduce the runaway-clock failure that grounded the ladder: with no
    limit the replay's first-order plant followed the step at an effective
    ~14 m/s^2 and reached 13.7 m/s one second in, while the real vehicle was
    held to MPC_ACC_HOR_MAX = 4 m/s^2. The simulated transient gap was a
    quarter of the real one -- 11.4 m against 46.8 -- so it stayed close
    enough to the 5 m runaway threshold to recover every time, and the
    offline tool reported a healthy flight the aircraft could not fly.

    SquareWaveVelocityTrajectory is the exception, and the reason is in its
    name: it exists to measure the response to a step, and a limited step is
    not one.
    """
    if trajectory is None or isinstance(trajectory, SquareWaveVelocityTrajectory):
        return None
    return configured_m_s2


def _own_position(state: Any) -> Vector3 | None:
    """This drone's ENU position, or None when the state cannot be trusted.

    An invalid or malformed state must read as absent, never as the origin:
    a silent (0, 0, 0) here would be a position error the size of the whole
    map handed straight to a velocity controller.
    """
    if not isinstance(state, dict) or not state.get("valid", False):
        return None
    position = state.get("position_enu_m")
    if not isinstance(position, (list, tuple)) or len(position) != 3:
        return None
    try:
        vector = tuple(float(component) for component in position)
    except (TypeError, ValueError):
        return None
    return vector if all(math.isfinite(c) for c in vector) else None  # type: ignore[return-value]


def _float_env(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except (TypeError, ValueError):
        return default


def _vector_env(name: str, default: str) -> Vector3:
    values = tuple(
        float(value.strip()) for value in os.environ.get(name, default).split(",")
    )
    if len(values) != 3:
        raise ValueError(f"{name} requires three ENU components")
    return values  # type: ignore[return-value]


def _waypoints_env(name: str) -> tuple[Vector3, ...]:
    """`e,n,u:e,n,u:...` -- colons between waypoints, commas inside one.

    Colon rather than semicolon because run_all.sh sources these profiles as
    shell, where an unquoted semicolon ends the assignment.
    """
    raw = os.environ.get(name, "").strip()
    if not raw:
        raise ValueError(f"{name} is required for a polyline trajectory")
    waypoints = []
    for chunk in raw.split(":"):
        values = tuple(float(value.strip()) for value in chunk.split(","))
        if len(values) != 3:
            raise ValueError(f"{name} requires three ENU components per waypoint")
        waypoints.append(values)
    return tuple(waypoints)


def _slot_env(drone_id: str) -> Vector3:
    return _vector_env(
        f"SWARM_FORMATION_SLOT_{drone_id.replace('-', '_')}_ENU_M", "-10,0,0"
    )


def _trajectory_env(drone_id: str) -> Trajectory | None:
    """Opt-in per-drone trajectory. Unset (default) means "no trajectory" --
    the drone keeps its existing formation/altitude-hold nominal, exactly as
    before this existed. Set means the trajectory REPLACES that nominal
    entirely for this drone, formation slot or not."""
    prefix = f"SWARM_TRAJECTORY_{drone_id.replace('-', '_')}"
    kind = os.environ.get(f"{prefix}_KIND", "").strip().lower()
    if not kind:
        return None
    if kind == "linear":
        return LinearTrajectory(
            start_enu_m=_vector_env(f"{prefix}_START_ENU_M", "0,0,10"),
            end_enu_m=_vector_env(f"{prefix}_END_ENU_M", "10,0,10"),
            speed_m_s=_float_env(f"{prefix}_SPEED_M_S", 1.0),
        )
    if kind == "circular":
        return CircularTrajectory(
            center_enu_m=_vector_env(f"{prefix}_CENTER_ENU_M", "0,0,10"),
            radius_m=_float_env(f"{prefix}_RADIUS_M", 5.0),
            angular_rate_rad_s=_float_env(f"{prefix}_ANGULAR_RATE_RAD_S", 0.2),
            start_angle_rad=_float_env(f"{prefix}_START_ANGLE_RAD", 0.0),
        )
    if kind == "polyline":
        # The corner logic -- angle-scaled speed cap and lag-aware braking --
        # only ever runs on a polygon vertex, and no env-driven profile could
        # reach it. ClosedPolylineTrajectory validates the shape itself.
        return ClosedPolylineTrajectory(
            waypoints_enu_m=_waypoints_env(f"{prefix}_WAYPOINTS_ENU_M"),
            speed_m_s=_float_env(f"{prefix}_SPEED_M_S", 1.0),
        )
    if kind == "square_wave":
        return SquareWaveVelocityTrajectory(
            start_enu_m=_vector_env(f"{prefix}_START_ENU_M", "0,0,10"),
            step_velocity_enu_m_s=_vector_env(f"{prefix}_STEP_VELOCITY_ENU_M_S", "1.5,0,0"),
            half_period_s=_float_env(f"{prefix}_HALF_PERIOD_S", 2.5),
        )
    raise ValueError(f"unknown {prefix}_KIND: {kind!r}")


def validate_command(velocity: Vector3, maximum_velocity_m_s: float) -> tuple[bool, Vector3]:
    """Final command-validator stage before shadow output.

    Everything upstream already produces finite, bounded velocities by
    construction, so this should never actually trigger -- it exists as the
    named validator node the architecture diagram calls for, and as
    defence-in-depth against a future bug anywhere upstream. On failure it
    substitutes zero velocity rather than passing through a bad value.
    """
    if len(velocity) != 3 or not all(math.isfinite(v) for v in velocity):
        return False, ZERO_VELOCITY
    magnitude = math.sqrt(sum(v * v for v in velocity))
    if magnitude > maximum_velocity_m_s + 1e-6:
        return False, ZERO_VELOCITY
    return True, velocity


_EMPTY_MARGIN_EXTREMA: dict[str, Any] = {
    "minimum_margin_m": None,
    "frames": 0,
    "breach_frames": 0,
}


@dataclass(frozen=True)
class CompanionSafetyStatus:
    drone_id: str
    nominal_velocity_enu_m_s: Vector3
    nominal_active: bool
    nominal_reason: str
    nominal_position_error_m: float | None
    command: CbfCommand
    emergency: EmergencyDecision
    output_velocity_enu_m_s: Vector3
    output_valid: bool
    peer_ids_used: tuple[str, ...]
    peer_ids_missing: tuple[str, ...]
    cbf_rl_shadow: dict[str, Any] | None = None
    # Extrema of minimum_margin_m accumulated at the companion's own rate.
    # The per-frame value below is a sample; this is the whole population.
    cbf_margin_extrema: dict[str, Any] | None = None

    @property
    def intervened(self) -> bool:
        """True when CBF changed the nominal, either by editing or refusing it."""
        return not self.command.active or self.command.intervention_norm_m_s > 0.0

    def as_dict(self) -> dict[str, Any]:
        return {
            "drone_id": self.drone_id,
            "frame": "ENU",
            "nominal_velocity_enu_m_s": list(self.nominal_velocity_enu_m_s),
            "nominal_active": self.nominal_active,
            "nominal_reason": self.nominal_reason,
            "nominal_position_error_m": (
                round(self.nominal_position_error_m, 3)
                if self.nominal_position_error_m is not None
                else None
            ),
            "cbf": self.command.as_dict(),
            # A consumer polling this payload sees one frame in ten or worse;
            # the barrier is evaluated every frame. Anything that must not be
            # missed -- a breach, the true minimum -- has to be accumulated
            # here, where every frame is seen, and read from this field. See
            # `_accumulate_margin`.
            "cbf_margin_extrema": self.cbf_margin_extrema or _EMPTY_MARGIN_EXTREMA,
            "emergency": self.emergency.as_dict(),
            "output_velocity_enu_m_s": list(self.output_velocity_enu_m_s),
            "output_valid": self.output_valid,
            "peer_ids_used": list(self.peer_ids_used),
            "peer_ids_missing": list(self.peer_ids_missing),
            "intervened": self.intervened,
            "cbf_rl_shadow": self.cbf_rl_shadow or CbfRlShadow.off().status(),
            "source": "companion_local_peer_to_peer",
            "authority": "shadow_companion_only",
            "nominal_source": (
                "cbf_rl_active"
                if (self.cbf_rl_shadow or {}).get("applied") is True
                else "deterministic"
            ),
        }


class CompanionSafetyMonitor:
    """Per-drone local nominal + CBF, evaluated from peer-to-peer state only."""

    def __init__(
        self,
        drone_id: str,
        peer_ids: tuple[str, ...],
        leader_id: str,
        slots: tuple[FormationSlot, ...],
        formation_config: FormationConfig | None = None,
        cbf_config: CbfConfig | None = None,
        emergency_config: EmergencyConfig | None = None,
        trajectory: Trajectory | None = None,
        cbf_rl_shadow: CbfRlShadow | None = None,
    ) -> None:
        if not drone_id.strip():
            raise ValueError("drone_id is required")
        self.drone_id = drone_id
        self.peer_ids = tuple(peer_id for peer_id in peer_ids if peer_id != drone_id)
        self.leader_id = leader_id
        self.formation = DeterministicFormationController(
            leader_id, slots, formation_config
        )
        # Reuses the formation controller's own gain and arrival radius, so
        # vertical station-keeping introduces no new tuning constant. The
        # velocity ceiling is FlightEnvelope's vertical limit.
        self.altitude_hold = AltitudeHoldController(
            AltitudeHoldConfig(
                gain_s_inv=self.formation.config.position_gain_s_inv,
                maximum_velocity_m_s=self.formation.config.maximum_velocity_m_s / 2.0,
                arrival_radius_m=self.formation.config.arrival_radius_m,
            )
        )
        # Dashboard polygons turn instantaneously at their vertices. Limit
        # only those missions so environment/system-ID trajectories retain
        # their real steps.
        self.mission_maximum_acceleration_m_s2 = _float_env(
            "SWARM_MISSION_MAXIMUM_ACCELERATION_M_S2", 0.5
        )
        self.mission_corner_tracking_tolerance_m = _float_env(
            "SWARM_MISSION_CORNER_TRACKING_TOLERANCE_M", 1.0
        )
        # The corner budget is geometric; a vehicle with a real velocity lag
        # spends part of it just catching up. Zero keeps the pure-geometry
        # behaviour, so only a profile that has MEASURED its aircraft sets it.
        self.mission_response_time_constant_s = _float_env(
            "SWARM_MISSION_RESPONSE_TIME_CONSTANT_S", 0.0
        )
        # Opt-in, per-drone: when set, REPLACES the formation/altitude-hold
        # nominal for this drone entirely (see evaluate()). Reuses the
        # formation controller's own resolved gain/velocity/arrival config,
        # same reasoning as altitude_hold above -- no new tuning constant.
        self.trajectory_tracking = (
            TrajectoryTrackingController(
                drone_id,
                trajectory,
                self.formation.config,
                maximum_acceleration_m_s2=_command_acceleration_limit(
                    trajectory, self.mission_maximum_acceleration_m_s2
                ),
                corner_tracking_tolerance_m=self.mission_corner_tracking_tolerance_m,
                response_time_constant_s=self.mission_response_time_constant_s,
            )
            if trajectory is not None
            else None
        )
        self.trajectory_start_monotonic_s: float | None = None
        # Where on the path the lap begins, chosen once when the companion
        # takes authority. None means the entry point has not been picked yet.
        self.trajectory_entry_time_s: float | None = None
        # How close the vehicle must be to that entry point before the lap
        # clock starts. Inherited from the flight driver's own
        # INITIAL_ERROR_LIMIT_DEFAULT_M rather than picked fresh: the same
        # question ("is the vehicle actually where the path begins?") already
        # had an answer there. Latching the clock regardless is what put a
        # vehicle into the ground on 2026-08-12 -- the reference ran away at
        # mission speed from a point the drone was 19 m from, and the tracker
        # answered with a permanently saturated chase.
        self.trajectory_entry_radius_m = _float_env(
            "SWARM_TRAJECTORY_ENTRY_RADIUS_M", 1.5
        )
        # Well above both the entry radius and the tracker's own steady-state
        # lag (mission speed / position gain, 2.5 m at 1.5 m/s and gain 0.6),
        # so normal tracking never trips it and only a genuine runaway does.
        # The gap between the two radii IS the hysteresis: re-entry at 5 m,
        # release back to the lap at 1.5 m, so it cannot chatter.
        self.trajectory_reentry_error_m = _float_env(
            "SWARM_TRAJECTORY_REENTRY_ERROR_M", 5.0
        )
        self.gate = CbfCommandGate(drone_id, self.peer_ids, cbf_config)
        self.cbf_rl_shadow = cbf_rl_shadow or CbfRlShadow.off()
        resolved_cbf_config = self.gate.config
        self.emergency = EmergencySupervisor(
            drone_id=drone_id,
            peer_ids=self.peer_ids,
            geofence_min_enu_m=resolved_cbf_config.geofence_min_enu_m,
            geofence_max_enu_m=resolved_cbf_config.geofence_max_enu_m,
            maximum_velocity_m_s=resolved_cbf_config.maximum_velocity_m_s,
            altitude_gain_s_inv=self.formation.config.position_gain_s_inv,
            altitude_arrival_radius_m=self.formation.config.arrival_radius_m,
            config=emergency_config,
        )
        self._margin_minimum_m: float | None = None
        self._margin_frames = 0
        self._margin_breach_frames = 0

    def _accumulate_margin(
        self, margin_m: float | None, station_keeping: bool
    ) -> dict[str, Any]:
        """Running extrema of the CBF margin, at the rate it is evaluated.

        A poller reads this payload every ~0.5 s while the barrier runs at
        20 Hz, so it sees roughly one frame in ten. Measured 2026-08-18 on the
        signed corridor ladder: the 20 m/s rung's true minimum was -1.332 m
        and the poller reported +2.740 -- a breach, invisible in 6 of 10
        sampling phases, on three consecutive flights that all signed
        FLIGHT_PASS. Sampling faster only thins the odds; the extremum has to
        be accumulated where every frame is seen, which is here.

        Only while station-keeping: off-authority the vehicle is parked or
        hand-flown, and its margin is not this system's claim to make. The
        counters reset on the falling edge so each authority period reports
        its own numbers rather than a previous flight's floor.
        """
        if not station_keeping:
            self._margin_minimum_m = None
            self._margin_frames = 0
            self._margin_breach_frames = 0
        elif margin_m is not None:
            self._margin_frames += 1
            if self._margin_minimum_m is None or margin_m < self._margin_minimum_m:
                self._margin_minimum_m = margin_m
            if margin_m < 0.0:
                self._margin_breach_frames += 1
        return {
            "minimum_margin_m": (
                None
                if self._margin_minimum_m is None
                else round(self._margin_minimum_m, 3)
            ),
            "frames": self._margin_frames,
            "breach_frames": self._margin_breach_frames,
        }

    def _enter_trajectory(
        self, swarm_state: dict[str, Any], now: float
    ) -> FormationCommand:
        """Fly to the path before flying the path.

        Returns an approach command until the vehicle is within
        `trajectory_entry_radius_m` of its entry point, then latches the lap
        clock. The lap therefore always begins with the vehicle ON the path,
        which is the assumption `TrajectoryTrackingController` is built on and
        the only condition under which its error term stays small.

        Fails closed by holding: an entry point that cannot be reached leaves
        the vehicle hovering short of the path, never dashing at it.
        """
        assert self.trajectory_tracking is not None
        own_position = _own_position(swarm_state.get(self.drone_id))
        if own_position is None:
            return FormationCommand(
                self.drone_id, None, ZERO_VELOCITY, False, "trajectory_entry_state_invalid", None
            )
        trajectory = self.trajectory_tracking.trajectory
        if self.trajectory_entry_time_s is None:
            # A closed loop has no privileged first waypoint, so join it where
            # the vehicle already is. Anything else starts at its own t=0.
            nearest = getattr(trajectory, "nearest_time_s", None)
            self.trajectory_entry_time_s = (
                float(nearest(own_position)) if callable(nearest) else 0.0
            )
        entry = trajectory.reference(self.trajectory_entry_time_s)
        error = tuple(
            entry.position_enu_m[axis] - own_position[axis] for axis in range(3)
        )
        error_norm = math.sqrt(sum(component * component for component in error))
        if error_norm <= self.trajectory_entry_radius_m:
            self.trajectory_start_monotonic_s = now
            return self.trajectory_tracking.command(
                self.trajectory_entry_time_s, swarm_state
            )
        # Approach at no more than the mission's own speed: the operator asked
        # to fly the path at that speed and never asked for a faster dash to
        # reach it.
        config = self.trajectory_tracking.config
        ceiling = min(
            config.maximum_velocity_m_s,
            float(getattr(trajectory, "speed_m_s", config.maximum_velocity_m_s)),
        )
        requested = tuple(config.position_gain_s_inv * component for component in error)
        magnitude = math.sqrt(sum(component * component for component in requested))
        if magnitude > ceiling:
            requested = tuple(component * ceiling / magnitude for component in requested)
        return FormationCommand(
            self.drone_id,
            entry.position_enu_m,
            requested,  # type: ignore[arg-type]
            True,
            "trajectory_entering",
            error_norm,
        )

    def set_trajectory(self, trajectory: Trajectory | None) -> None:
        """Install an operator-sent trajectory for this drone, or clear it.

        Refused while a mission is already running. Swapping the reference
        under a flying vehicle steps the tracking error from centimetres to
        the width of the drawn map, and the tracker answers a step like that
        with a full-rate dash toward the new path -- CBF would still bound the
        separation, but nobody asked for the dash. Hand authority back (leave
        OFFBOARD, or land) and the mission clock unlatches, after which a new
        path may be installed. `trajectory_start_monotonic_s` is exactly that
        latch, so it is also the test.

        ponytail: no mid-flight re-path. Add one by ramping the reference from
        the vehicle's current position onto the new loop if an operator ever
        needs to redraw without landing.
        """
        if self.trajectory_start_monotonic_s is not None:
            raise RuntimeError("mission is already running")
        # Same trap as the env path: the entry controller and the tracker share
        # one ceiling, so a mission drawn faster than it degrades to an endless
        # crawl toward a reference that has already left.
        configured_speed_m_s = getattr(trajectory, "speed_m_s", None)
        if (
            configured_speed_m_s is not None
            and configured_speed_m_s > self.formation.config.maximum_velocity_m_s
        ):
            raise ValueError(
                f"mission speed {configured_speed_m_s:g} m/s exceeds this "
                f"vehicle's {self.formation.config.maximum_velocity_m_s:g} m/s "
                "tracking ceiling"
            )
        self.trajectory_entry_time_s = None
        self.trajectory_tracking = (
            TrajectoryTrackingController(
                self.drone_id,
                trajectory,
                self.formation.config,
                maximum_acceleration_m_s2=_command_acceleration_limit(
                    trajectory, self.mission_maximum_acceleration_m_s2
                ),
                corner_tracking_tolerance_m=self.mission_corner_tracking_tolerance_m,
                response_time_constant_s=self.mission_response_time_constant_s,
            )
            if trajectory is not None
            else None
        )

    @property
    def trajectory_reference_start_enu_m(self) -> Vector3 | None:
        """Where the configured trajectory says this drone begins, or None
        when no trajectory is configured.

        Deliberately independent of `station_keeping`: this is configuration,
        not state, so an external pre-flight check can compare it against the
        vehicle's real position BEFORE handing the companion authority --
        which is the only moment at which a frame/position mismatch can still
        be refused for free rather than flown into.
        """
        if self.trajectory_tracking is None:
            return None
        return self.trajectory_tracking.trajectory.reference(0.0).position_enu_m

    @classmethod
    def from_environment(
        cls, drone_id: str, all_drone_ids: tuple[str, ...]
    ) -> CompanionSafetyMonitor:
        leader_id = os.environ.get("SWARM_FORMATION_LEADER_ID", "UAV-01").strip()
        followers = tuple(
            sorted(other for other in all_drone_ids if other != leader_id)
        )
        formation_maximum_velocity_m_s = _float_env(
            "SWARM_FORMATION_MAXIMUM_VELOCITY_M_S", 2.0
        )
        trajectory = _trajectory_env(drone_id)
        # One ceiling serves both the entry controller and the tracker (they
        # share formation config), so a trajectory faster than it can never be
        # flown -- the reference runs away, the error crosses the re-entry
        # threshold, and the vehicle falls back to approaching at the ceiling
        # forever. That failure is a silent crawl, not a crash, so it reads on
        # a dashboard as "flying, slowly" for as long as anyone lets it. Refuse
        # the pair up front instead, naming both knobs.
        configured_speed_m_s = getattr(trajectory, "speed_m_s", None)
        if (
            configured_speed_m_s is not None
            and configured_speed_m_s > formation_maximum_velocity_m_s
        ):
            raise ValueError(
                f"{drone_id} trajectory speed {configured_speed_m_s:g} m/s exceeds "
                f"SWARM_FORMATION_MAXIMUM_VELOCITY_M_S "
                f"({formation_maximum_velocity_m_s:g} m/s); the entry controller "
                "and the tracker share that ceiling, so the path could never be "
                "entered"
            )
        return cls(
            drone_id=drone_id,
            peer_ids=tuple(sorted(other for other in all_drone_ids if other != drone_id)),
            leader_id=leader_id,
            slots=tuple(
                FormationSlot(follower, _slot_env(follower)) for follower in followers
            ),
            formation_config=FormationConfig(
                position_gain_s_inv=_float_env(
                    "SWARM_FORMATION_POSITION_GAIN_S_INV", 0.6
                ),
                maximum_velocity_m_s=formation_maximum_velocity_m_s,
                arrival_radius_m=_float_env("SWARM_FORMATION_ARRIVAL_RADIUS_M", 0.25),
            ),
            cbf_config=CbfConfig(
                minimum_separation_m=_float_env("SWARM_CBF_MINIMUM_SEPARATION_M", 4.0),
                barrier_gain_s_inv=_float_env("SWARM_CBF_BARRIER_GAIN_S_INV", 2.0),
                maximum_velocity_m_s=_float_env("SWARM_CBF_MAXIMUM_VELOCITY_M_S", 2.0),
                command_latency_s=_float_env("SWARM_CBF_COMMAND_LATENCY_S", 0.10),
                relative_braking_acceleration_m_s2=_float_env(
                    "SWARM_CBF_RELATIVE_BRAKING_ACCELERATION_M_S2", 0.0
                ),
                tracking_reserve_m=_float_env("SWARM_CBF_TRACKING_RESERVE_M", 0.0),
                design_margin_buffer_m=_float_env("SWARM_CBF_DESIGN_MARGIN_BUFFER_M", 0.0),
                covariance_sigma=cbf_covariance_sigma(),
                require_position_covariance=cbf_position_covariance_required(),
                geofence_min_enu_m=_vector_env(
                    "SWARM_CBF_GEOFENCE_MIN_ENU_M", "-100,-100,0"
                ),
                geofence_max_enu_m=_vector_env(
                    "SWARM_CBF_GEOFENCE_MAX_ENU_M", "100,100,50"
                ),
            ),
            emergency_config=EmergencyConfig(
                stage_dwell_s=_float_env("SWARM_EMERGENCY_STAGE_DWELL_S", 2.0),
                recovery_confirmation_s=_float_env(
                    "SWARM_EMERGENCY_RECOVERY_CONFIRMATION_S", 0.5
                ),
                altitude_base_m=_float_env("SWARM_EMERGENCY_ALTITUDE_BASE_M", 10.0),
                altitude_step_m=_float_env("SWARM_EMERGENCY_ALTITUDE_STEP_M", 3.0),
            ),
            trajectory=trajectory,
            cbf_rl_shadow=(
                CbfRlShadow.from_environment()
                if len(all_drone_ids) != 2
                else _coordinated_cbf_rl(drone_id, all_drone_ids)
            ),
        )

    def evaluate(
        self,
        swarm_state: dict[str, Any],
        now_monotonic_s: float | None = None,
        station_keeping: bool = False,
    ) -> CompanionSafetyStatus:
        """`station_keeping` means the companion is the controlling authority
        right now (armed and in OFFBOARD). It gates altitude hold, whose
        reference must be captured on that transition rather than whenever
        state happens to be valid -- see AltitudeHoldController.
        """
        now = time.monotonic() if now_monotonic_s is None else float(now_monotonic_s)
        if self.trajectory_tracking is not None:
            # Trajectory tracking replaces formation/altitude-hold entirely
            # for this drone -- both are for the fixed-slot/hold case this
            # supersedes once a trajectory is assigned. The mission clock is
            # latched on the same station-keeping transition altitude_hold
            # uses, for the same reason: capturing it earlier would count time
            # spent on the ground or mid-takeoff as already-elapsed mission
            # time.
            self.altitude_hold.reset()
            if not station_keeping:
                self.trajectory_start_monotonic_s = None
                self.trajectory_entry_time_s = None
                self.trajectory_tracking.reset_velocity_limiter()
                nominal = FormationCommand(
                    self.drone_id, None, ZERO_VELOCITY, False, "trajectory_inactive", None
                )
            elif self.trajectory_start_monotonic_s is None:
                nominal = self._enter_trajectory(swarm_state, now)
            else:
                mission_elapsed_s = now - self.trajectory_start_monotonic_s
                nominal = self.trajectory_tracking.command(
                    mission_elapsed_s + (self.trajectory_entry_time_s or 0.0),
                    swarm_state,
                )
                # The spatial polygon follower cannot run away in phase, but a
                # genuine displacement from the path still returns to the
                # bounded entry controller instead of chasing longitudinally.
                #
                # Not while a conflict is latched, though.  A one-sided yield in
                # a shared corridor REQUIRES the yielding vehicle to leave its
                # path by the full separation floor -- ~22.6 m at the 20 m
                # floor, against a 5 m re-entry threshold -- so re-entering
                # there hands the coordinator an entry command pointing back at
                # the path in place of the corridor direction, and the lane
                # change comes out perpendicular.  The displacement is the
                # maneuver, not a tracking failure; re-entry resumes on release.
                # The coordinator state read here is one frame old, which is
                # nothing against a latch that lasts the whole encounter.
                # Linear legs only.  A closed polygon has nowhere to yield TO
                # -- the path comes back -- and suppressing re-entry there cost
                # the 240 m counter-rotating square 2.399 m of dynamic margin,
                # down to 0.174 m.
                coordinator = self.cbf_rl_shadow.coordinator
                yielding_on_a_leg = (
                    coordinator is not None
                    and coordinator.active
                    and isinstance(self.trajectory_tracking.trajectory, LinearTrajectory)
                )
                if (
                    not yielding_on_a_leg
                    and nominal.position_error_m is not None
                    and nominal.position_error_m > self.trajectory_reentry_error_m
                ):
                    scheduled_time_s = mission_elapsed_s + (
                        self.trajectory_entry_time_s or 0.0
                    )
                    trajectory = self.trajectory_tracking.trajectory
                    own_position = _own_position(swarm_state.get(self.drone_id))
                    self.trajectory_start_monotonic_s = None
                    self.trajectory_entry_time_s = (
                        max(
                            scheduled_time_s,
                            trajectory.nearest_time_s(own_position),
                        )
                        if isinstance(trajectory, LinearTrajectory)
                        and own_position is not None
                        else None
                    )
                    self.trajectory_tracking.reset_velocity_limiter()
                    nominal = self._enter_trajectory(swarm_state, now)
        else:
            nominal = self.formation.command(self.drone_id, swarm_state)
            if not station_keeping or nominal.active:
                # Outside station-keeping the formation controller's own reason
                # is preserved rather than masked: "formation_slot_unassigned"
                # and "follower_state_invalid" are different diagnoses and
                # only one of them is normal.
                self.altitude_hold.reset()
            else:
                # A drone with no formation slot -- the leader -- has no
                # reference in any axis. Horizontally PX4 holds well on its
                # own; vertically it does not, because gravity is a standing
                # disturbance and velocity-mode OFFBOARD holds velocity rather
                # than position. Measured on the first armed flight: +1.36 m
                # in 15 s.
                altitude = self.altitude_hold.command(
                    self.drone_id, swarm_state.get(self.drone_id), station_keeping
                )
                # Only replace the nominal if altitude hold actually produced
                # one; its own failure reasons must not overwrite the
                # formation controller's more specific diagnosis either.
                if altitude.active:
                    nominal = altitude
        # A drone without a formation slot still needs the barrier enforced, so
        # an inactive nominal degrades to "hold" rather than skipping the filter.
        nominal_velocity = (
            tuple(nominal.velocity_enu_m_s) if nominal.active else ZERO_VELOCITY
        )
        used = tuple(
            peer_id
            for peer_id in self.peer_ids
            if isinstance(swarm_state.get(peer_id), dict)
            and swarm_state[peer_id].get("valid", False)
        )
        deterministic_command = self.gate.filter(nominal_velocity, swarm_state)
        shadow_target = nominal.target_enu_m
        if (
            self.trajectory_tracking is not None
            and isinstance(self.trajectory_tracking.trajectory, LinearTrajectory)
        ):
            # Training uses the final goal of a linear leg.  Feeding the
            # time-indexed reference here made that goal only a few metres
            # away, tapered the learned lateral action almost to zero, and let
            # the coordinator turn a head-on yield into a full reversal.
            shadow_target = self.trajectory_tracking.trajectory.end_enu_m
        if shadow_target is None and self.trajectory_tracking is not None:
            shadow_target = self.trajectory_reference_start_enu_m
        cbf_rl_shadow, _, cbf_rl_command = self.cbf_rl_shadow.evaluate_candidate(
            drone_id=self.drone_id,
            peer_ids=self.peer_ids,
            target_enu_m=shadow_target,
            swarm_state=swarm_state,
            gate=self.gate,
            deterministic_nominal_enu_m_s=nominal_velocity,  # type: ignore[arg-type]
            deterministic_command=deterministic_command,
            apply=station_keeping and nominal.active,
        )
        command = cbf_rl_command or deterministic_command

        own = swarm_state.get(self.drone_id)
        own_valid = isinstance(own, dict) and bool(own.get("valid", False))
        own_altitude_m: float | None = None
        if own_valid:
            position = own.get("position_enu_m")
            if isinstance(position, (list, tuple)) and len(position) == 3:
                try:
                    candidate = float(position[2])
                except (TypeError, ValueError):
                    candidate = float("nan")
                if math.isfinite(candidate):
                    own_altitude_m = candidate
                else:
                    own_valid = False
            else:
                own_valid = False

        decision = self.emergency.evaluate(command, now, own_valid, own_altitude_m)
        output_valid, output_velocity = validate_command(
            decision.velocity_enu_m_s, self.gate.config.maximum_velocity_m_s
        )

        return CompanionSafetyStatus(
            drone_id=self.drone_id,
            nominal_velocity_enu_m_s=nominal_velocity,  # type: ignore[arg-type]
            nominal_active=nominal.active,
            nominal_reason=nominal.reason,
            nominal_position_error_m=nominal.position_error_m,
            command=command,
            emergency=decision,
            output_velocity_enu_m_s=output_velocity,
            output_valid=output_valid,
            peer_ids_used=used,
            peer_ids_missing=tuple(
                peer_id for peer_id in self.peer_ids if peer_id not in used
            ),
            cbf_rl_shadow=cbf_rl_shadow,
            cbf_margin_extrema=self._accumulate_margin(
                command.minimum_margin_m, station_keeping
            ),
        )
