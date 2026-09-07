"""Deterministic two-UAV environment for offline CBF-shielded RL.

The policy proposes normalized velocity actions.  Only velocities returned by
the existing :class:`CbfCommandGate` advance the simulated vehicles; nominal
actions never have a direct dynamics path.  This module has no runtime wiring,
network access, simulator process, or flight authority.
"""

from __future__ import annotations

import dataclasses
import math
import random
from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

from cbf_command_gate import CbfCommandGate, CbfConfig
from conflict_coordinator import ConflictCoordinator, ConflictCoordinatorConfig
from formation_controller import FormationConfig
from trajectory_controller import LinearTrajectory, TrajectoryTrackingController


DRONE_IDS = ("UAV-01", "UAV-02")
OBSERVATION_FIELDS = (
    "goal_relative_e_m",
    "goal_relative_n_m",
    "goal_relative_u_m",
    "self_velocity_e_m_s",
    "self_velocity_n_m_s",
    "self_velocity_u_m_s",
    "peer_relative_e_m",
    "peer_relative_n_m",
    "peer_relative_u_m",
    "peer_relative_velocity_e_m_s",
    "peer_relative_velocity_n_m_s",
    "peer_relative_velocity_u_m_s",
    "self_sqrt_trace_cov_m",
    "peer_sqrt_trace_cov_m",
    "self_age_s",
    "peer_age_s",
    "self_state_valid",
    "peer_state_valid",
    "self_covariance_valid",
    "peer_covariance_valid",
)

Vector3 = tuple[float, float, float]


def _production_cbf_config() -> CbfConfig:
    return CbfConfig(
        minimum_separation_m=4.0,
        barrier_gain_s_inv=2.0,
        maximum_velocity_m_s=2.0,
        covariance_sigma=0.10,
        command_latency_s=0.65,
        require_position_covariance=True,
    )


def x500_20m_cbf_config(maximum_velocity_m_s: float = 10.0) -> CbfConfig:
    """x500 20 m contract using PX4's 3 m/s^2 XY acceleration."""
    return CbfConfig(
        minimum_separation_m=20.0,
        barrier_gain_s_inv=2.0,
        maximum_velocity_m_s=maximum_velocity_m_s,
        covariance_sigma=0.10,
        command_latency_s=0.65,
        relative_braking_acceleration_m_s2=6.0,
        tracking_reserve_m=2.0,
        design_margin_buffer_m=5.0,
        require_position_covariance=True,
        geofence_min_enu_m=(-500.0, -500.0, 0.0),
        geofence_max_enu_m=(500.0, 500.0, 200.0),
    )


def sparrow_10m_floor_cbf_config(maximum_velocity_m_s: float = 10.0) -> CbfConfig:
    """Sparrow contract with the 10 m emergency floor the operator asked for.

    "10 to 20 m" is one contract, not two. The floor is the distance the pair
    may never close inside; the 20 m end of that range appears on its own,
    because `required_margin` grows with closing speed and already exceeds
    20 m once the pair is closing at 6.81 m/s. So a static 10 m floor gives
    exactly the asked-for behaviour with no ramp and no extra gain in the
    feedback loop `design_margin_buffer_m` warns about: drones on parallel
    tracks may sit 12 m apart and hold their paths, while anything genuinely
    converging is held off at 20 m or more.

    At the speeds this project cares about the floor is a minor term anyway --
    at 25 m/s head-on it is 20 m of a 206 m requirement.
    """
    return dataclasses.replace(
        sparrow_20m_cbf_config(maximum_velocity_m_s), minimum_separation_m=10.0
    )


def sparrow_20m_cbf_config(maximum_velocity_m_s: float = 10.0) -> CbfConfig:
    """Sparrow 20 m contract using the airframe's 4 m/s^2 XY limit."""
    return CbfConfig(
        minimum_separation_m=20.0,
        barrier_gain_s_inv=2.0,
        maximum_velocity_m_s=maximum_velocity_m_s,
        covariance_sigma=0.10,
        # 0.86 s, measured, not the inherited 0.65. The 2026-08-15 Sparrow
        # flight fits the horizontal response at tau = 0.860 s, and a head-on
        # pair at 1 m/s breached the requirement by 0.13 m with the gate
        # commanding a reversal the vehicle had not yet made. The shortfall
        # scales with closing speed -- 0.42 m at 2 m/s closing, 8.4 m at 40 --
        # so the old value was least accurate exactly where it mattered most.
        command_latency_s=0.86,
        relative_braking_acceleration_m_s2=8.0,
        tracking_reserve_m=2.0,
        design_margin_buffer_m=0.0,
        require_position_covariance=True,
        geofence_min_enu_m=(-500.0, -500.0, 0.0),
        geofence_max_enu_m=(500.0, 500.0, 200.0),
    )


@dataclass(frozen=True)
class CbfRlEnvConfig:
    dt_s: float = 0.05
    maximum_steps: int = 800
    arrival_radius_m: float = 0.25
    state_max_age_ms: float = 100.0
    default_position_covariance_m2: Vector3 = (0.041, 0.041, 0.071)
    cbf: CbfConfig = field(default_factory=_production_cbf_config)
    # First-order vehicle response, opt-in. Zero keeps the frozen Phase 0
    # contract exactly: the vehicle reaches the safe velocity within one step.
    #
    # Real SITL does not. Fitting the 2026-08-12 active v2 trace gives a clean
    # first-order lag on the horizontal axes -- 0.538 s (UAV-01) and 0.555 s
    # (UAV-02), steady-state gain 1.033, residual 0.0014 m/s. Training against
    # zero is why a policy could pass the offline gate in exactly 700 steps
    # and still fail in flight.
    #
    # Distinct from `CbfConfig.command_latency_s` (0.65 s), which is the
    # margin the barrier assumes, not the dynamics the vehicle has.
    response_time_constant_s: float = 0.0
    # Sampled per episode when set, so a policy cannot tune itself to one
    # exact lag. Fixed lag is a simulator; a range is a vehicle.
    response_time_constant_range_s: tuple[float, float] | None = None
    response_seed: int = 7
    # Zero preserves legacy instantaneous velocity response. High-speed
    # profiles pair this with their CBF relative-braking contract.
    maximum_acceleration_m_s2: float = 0.0
    # Deterministic yielding between the pair, off by default.
    #
    # The runtime has run a ConflictCoordinator between the policy and the CBF
    # since the mission milestone (cbf_rl_shadow.py:181), but the offline gate
    # never did, so the matrix has been judging a stack strictly weaker than
    # the one that flies. That gap is invisible until a geometry needs someone
    # to yield: at 20 m/s the vertical head-on cases deadlocked with thousands
    # of hold frames while staying perfectly safe, because nothing in the
    # evaluated stack decides which vehicle goes first.
    #
    # Off by default so every already-certified rung keeps meaning what it
    # meant. Turning it on changes what PASS is, which is a decision about the
    # certification method, not a tuning knob.
    conflict_coordination: ConflictCoordinatorConfig | None = None
    # Speed of the deterministic mission velocity handed to the coordinator.
    # None means the CBF ceiling, which is right for training; an evaluation
    # that caps the policy below that ceiling must cap the mission with it, or
    # the mission outruns the case it is meant to fly.
    mission_speed_m_s: float | None = None
    # The tracker gain the coordinator's `mission` is produced with. 0.6 is
    # SWARM_FORMATION_POSITION_GAIN_S_INV's default, which is what every
    # Sparrow profile flies; the excursion the yield can hold is
    # yield_lateral_speed_m_s / this, so the matrix has to use the same one.
    position_gain_s_inv: float = 0.6

    def __post_init__(self) -> None:
        if (
            not math.isfinite(self.dt_s)
            or self.dt_s <= 0.0
            or self.maximum_steps <= 0
            or not math.isfinite(self.arrival_radius_m)
            or self.arrival_radius_m <= 0.0
            or not math.isfinite(self.state_max_age_ms)
            or self.state_max_age_ms < 0.0
            or not math.isfinite(self.response_time_constant_s)
            or self.response_time_constant_s < 0.0
            or not math.isfinite(self.maximum_acceleration_m_s2)
            or self.maximum_acceleration_m_s2 < 0.0
        ):
            raise ValueError("CBF-RL environment configuration is invalid")
        if self.mission_speed_m_s is not None and (
            not math.isfinite(self.mission_speed_m_s) or self.mission_speed_m_s <= 0.0
        ):
            raise ValueError("mission speed is invalid")
        span = self.response_time_constant_range_s
        if span is not None and (
            len(span) != 2
            or not all(math.isfinite(value) and value >= 0.0 for value in span)
            or span[0] > span[1]
        ):
            raise ValueError("response time constant range is invalid")


class CbfRlEnvironment:
    """Small dependency-free API shaped like a multi-agent Gym environment."""

    def __init__(
        self,
        goals_enu_m: Mapping[str, Sequence[float]],
        config: CbfRlEnvConfig | None = None,
    ) -> None:
        self.config = config or CbfRlEnvConfig()
        self.goals = self._vectors(goals_enu_m, "goals")
        self.gates = {
            drone: CbfCommandGate(
                drone,
                tuple(peer for peer in DRONE_IDS if peer != drone),
                self.config.cbf,
            )
            for drone in DRONE_IDS
        }
        self.coordinators = (
            # pair(), never shared(): shared() hangs the encounter state off a
            # module global, which the matrix's parallel workers would trample.
            ConflictCoordinator.pair(DRONE_IDS, self.config.conflict_coordination)
            if self.config.conflict_coordination is not None
            else None
        )
        self.positions: dict[str, Vector3] = {}
        self.trackers: dict[str, TrajectoryTrackingController] = {}
        self.velocities: dict[str, Vector3] = {}
        self.covariances: dict[str, Vector3 | None] = {}
        self.message_ages_ms: dict[str, float] = {}
        self.source_valid: dict[str, bool] = {}
        self.reached: set[str] = set()
        self.steps = 0
        self._random = random.Random(self.config.response_seed)
        self.response_time_constant_s = self.config.response_time_constant_s

    def reset(
        self,
        spawn_enu_m: Mapping[str, Sequence[float]],
        *,
        covariance_by_drone: Mapping[str, Sequence[float] | None] | None = None,
        message_age_ms_by_drone: Mapping[str, float] | None = None,
        valid_by_drone: Mapping[str, bool] | None = None,
        velocity_by_drone: Mapping[str, Sequence[float]] | None = None,
    ) -> dict[str, tuple[float, ...]]:
        self.positions = self._vectors(spawn_enu_m, "spawn")
        self.trackers = self._build_trackers()
        self.velocities = (
            self._vectors(velocity_by_drone, "velocity")
            if velocity_by_drone is not None
            else {drone: (0.0, 0.0, 0.0) for drone in DRONE_IDS}
        )
        self.covariances = {
            drone: self._covariance(
                None
                if covariance_by_drone is not None
                and covariance_by_drone.get(drone) is None
                else (
                    covariance_by_drone[drone]
                    if covariance_by_drone is not None and drone in covariance_by_drone
                    else self.config.default_position_covariance_m2
                )
            )
            for drone in DRONE_IDS
        }
        self.message_ages_ms = {
            drone: float((message_age_ms_by_drone or {}).get(drone, 0.0))
            for drone in DRONE_IDS
        }
        if not all(
            math.isfinite(age) and age >= 0.0 for age in self.message_ages_ms.values()
        ):
            raise ValueError("message ages must be finite and nonnegative")
        self.source_valid = {
            drone: bool((valid_by_drone or {}).get(drone, True))
            for drone in DRONE_IDS
        }
        self.reached = set()
        self.steps = 0
        if self.coordinators is not None:
            # Encounter state is per episode. Leaking a latched yield role from
            # the previous case would make the matrix order-dependent.
            for coordinator in self.coordinators.values():
                coordinator.reset()
        span = self.config.response_time_constant_range_s
        if span is not None:
            self.response_time_constant_s = self._random.uniform(*span)
        return self.observations()

    def observations(self) -> dict[str, tuple[float, ...]]:
        self._require_reset()
        result = {}
        for drone in DRONE_IDS:
            peer = self._peer(drone)
            own_position = self.positions[drone]
            peer_position = self.positions[peer]
            own_velocity = self.velocities[drone]
            peer_velocity = self.velocities[peer]
            own_covariance = self.covariances[drone]
            peer_covariance = self.covariances[peer]
            result[drone] = (
                *(self.goals[drone][axis] - own_position[axis] for axis in range(3)),
                *own_velocity,
                *(peer_position[axis] - own_position[axis] for axis in range(3)),
                *(peer_velocity[axis] - own_velocity[axis] for axis in range(3)),
                self._sqrt_trace(own_covariance),
                self._sqrt_trace(peer_covariance),
                self.message_ages_ms[drone] / 1000.0,
                self.message_ages_ms[peer] / 1000.0,
                float(self._state_valid(drone)),
                float(self._state_valid(peer)),
                float(own_covariance is not None),
                float(peer_covariance is not None),
            )
        return result

    def step(
        self, actions: Mapping[str, Sequence[float]]
    ) -> tuple[
        dict[str, tuple[float, ...]],
        dict[str, float],
        bool,
        bool,
        dict[str, dict[str, Any]],
    ]:
        self._require_reset()
        if set(actions) != set(DRONE_IDS):
            raise ValueError("actions must contain exactly UAV-01 and UAV-02")
        nominal = {drone: self._nominal_action(actions[drone]) for drone in DRONE_IDS}
        state = self._swarm_state()
        before = {drone: self._goal_distance(drone) for drone in DRONE_IDS}
        coordination: dict[str, dict[str, Any]] = {}
        if self.coordinators is not None:
            # Same seat as the runtime: after the policy, before the barrier,
            # and handed the same two arguments the runtime hands it -- the
            # policy as candidate, the trajectory tracker's command as mission.
            filtered = {
                drone: self.coordinators[drone].filter(
                    nominal[drone], self._mission_velocity(drone), state
                )
                for drone in DRONE_IDS
            }
            nominal = {drone: value[0] for drone, value in filtered.items()}
            # Kept, not discarded. The status used to be dropped on the floor,
            # so nothing downstream could answer the one question that decides
            # whether a matrix result says anything about the coordinator:
            # did it ever engage in this case at all?
            coordination = {drone: value[1] for drone, value in filtered.items()}
        commands = {
            drone: self.gates[drone].filter(nominal[drone], state)
            for drone in DRONE_IDS
        }
        applied = {
            drone: commands[drone].velocity_enu_m_s
            if commands[drone].active
            else (0.0, 0.0, 0.0)
            for drone in DRONE_IDS
        }
        # The vehicle chases the safe velocity rather than adopting it, and
        # it is the ACHIEVED velocity that moves it, feeds the observation,
        # and reaches the barrier -- so the whole loop sees the same lag a
        # real vehicle imposes.
        achieved = {
            drone: self._respond(self.velocities[drone], applied[drone])
            for drone in DRONE_IDS
        }
        self.positions = {
            drone: tuple(
                self.positions[drone][axis]
                + achieved[drone][axis] * self.config.dt_s
                for axis in range(3)
            )
            for drone in DRONE_IDS
        }
        self.velocities = achieved
        self.steps += 1

        rewards: dict[str, float] = {}
        info: dict[str, dict[str, Any]] = {}
        at_goal: dict[str, bool] = {}
        for drone in DRONE_IDS:
            distance = self._goal_distance(drone)
            at_goal[drone] = distance <= self.config.arrival_radius_m
            newly_reached = at_goal[drone] and drone not in self.reached
            if newly_reached:
                self.reached.add(drone)
            command = commands[drone]
            negative_margin = max(0.0, -(command.minimum_margin_m or 0.0))
            rewards[drone] = (
                5.0 * (before[drone] - distance)
                - 0.01
                - 0.10 * command.intervention_norm_m_s
                - (2.0 if not command.active else 0.0)
                - 5.0 * negative_margin
                + (10.0 if newly_reached else 0.0)
            )
            info[drone] = {
                "nominal_velocity_enu_m_s": nominal[drone],
                "safe_velocity_enu_m_s": applied[drone],
                "goal_distance_m": distance,
                "cbf": command.as_dict(),
                "conflict_coordination": coordination.get(drone, {}),
            }
        terminated = all(at_goal.values())
        truncated = self.steps >= self.config.maximum_steps and not terminated
        return self.observations(), rewards, terminated, truncated, info

    def _respond(self, current: Vector3, requested: Vector3) -> Vector3:
        """One step of first-order velocity response toward `requested`."""
        tau = self.response_time_constant_s
        if tau <= 0.0:
            target = requested
        else:
            retained = math.exp(-self.config.dt_s / tau)
            target = tuple(
                retained * current[axis] + (1.0 - retained) * requested[axis]
                for axis in range(3)
            )
        maximum_acceleration = self.config.maximum_acceleration_m_s2
        delta = tuple(target[axis] - current[axis] for axis in range(3))
        delta_norm = math.sqrt(sum(value * value for value in delta))
        maximum_delta = maximum_acceleration * self.config.dt_s
        if maximum_acceleration > 0.0 and delta_norm > maximum_delta:
            scale = maximum_delta / delta_norm
            return tuple(
                current[axis] + delta[axis] * scale for axis in range(3)
            )  # type: ignore[return-value]
        return target  # type: ignore[return-value]

    def _swarm_state(self) -> dict[str, dict[str, Any]]:
        return {
            drone: {
                "valid": self._state_valid(drone),
                "position_enu_m": self.positions[drone],
                "velocity_enu_m_s": self.velocities[drone],
                "position_covariance_m2": self.covariances[drone],
                "message_age_ms": self.message_ages_ms[drone],
            }
            for drone in DRONE_IDS
        }

    def _state_valid(self, drone: str) -> bool:
        return bool(
            self.source_valid[drone]
            and self.message_ages_ms[drone] <= self.config.state_max_age_ms
        )

    def _nominal_action(self, value: Sequence[float]) -> Vector3:
        vector = self._vector(value, "action")
        return tuple(
            max(-1.0, min(1.0, component))
            * self.config.cbf.maximum_velocity_m_s
            for component in vector
        )  # type: ignore[return-value]

    def _build_trackers(self) -> dict[str, TrajectoryTrackingController]:
        """The real tracker, on the real leg, as the coordinator's `mission`.

        This used to be a hand-written goal-direction vector, and that was the
        matrix's blind spot. The runtime hands the coordinator a TRACKER
        command, which carries a position-feedback term; a bare direction does
        not. The yield lane change is built from `mission`, so on 2026-08-17
        the feedback re-entered it weighted by forward_speed and cancelled the
        lateral push -- the excursion stalled at 5 m in flight against ~52 m
        required -- and every one of these 1260 cases certified PASS through
        it, because offline there was no feedback to cancel anything.

        A matrix case is a straight leg from spawn to goal, so the same
        LinearTrajectory and TrajectoryTrackingController the companion
        installs reproduces it exactly, feedback included.
        """
        speed = self.config.mission_speed_m_s or self.config.cbf.maximum_velocity_m_s
        trackers: dict[str, TrajectoryTrackingController] = {}
        for drone in DRONE_IDS:
            start, goal = self.positions[drone], self.goals[drone]
            if math.dist(start, goal) <= self.config.arrival_radius_m:
                continue  # nowhere to go; _mission_velocity holds instead
            trackers[drone] = TrajectoryTrackingController(
                drone,
                LinearTrajectory(
                    start_enu_m=start, end_enu_m=goal, speed_m_s=speed
                ),
                FormationConfig(
                    position_gain_s_inv=self.config.position_gain_s_inv,
                    maximum_velocity_m_s=speed,
                    arrival_radius_m=self.config.arrival_radius_m,
                ),
                maximum_acceleration_m_s2=(
                    self.config.maximum_acceleration_m_s2 or None
                ),
                response_time_constant_s=self.response_time_constant_s,
            )
        return trackers

    def _mission_velocity(self, drone: str) -> Vector3:
        """The deterministic goal-seeking command, the coordinator's `mission`.

        The runtime feeds the coordinator two different things: the policy as
        candidate, and the trajectory tracker's command as mission. This
        environment used to pass the policy for both, which quietly certified
        a wiring no vehicle flies -- and it mattered, because
        `clear_uses_mission_velocity` and the yield lane-change rebuild the
        output from `mission` at every role.
        """
        tracker = self.trackers.get(drone)
        if tracker is None or self._goal_distance(drone) <= self.config.arrival_radius_m:
            return (0.0, 0.0, 0.0)
        command = tracker.command(
            self.steps * self.config.dt_s,
            {
                other: {
                    "position_enu_m": self.positions[other],
                    "velocity_enu_m_s": self.velocities[other],
                    "valid": True,
                }
                for other in DRONE_IDS
            },
        )
        return tuple(command.velocity_enu_m_s) if command.active else (0.0, 0.0, 0.0)  # type: ignore[return-value]

    def _goal_distance(self, drone: str) -> float:
        return math.sqrt(
            sum(
                (self.goals[drone][axis] - self.positions[drone][axis]) ** 2
                for axis in range(3)
            )
        )

    @staticmethod
    def _sqrt_trace(covariance: Vector3 | None) -> float:
        return math.sqrt(sum(covariance)) if covariance is not None else 0.0

    @staticmethod
    def _peer(drone: str) -> str:
        return DRONE_IDS[1] if drone == DRONE_IDS[0] else DRONE_IDS[0]

    @classmethod
    def _vectors(
        cls, values: Mapping[str, Sequence[float]], label: str
    ) -> dict[str, Vector3]:
        if set(values) != set(DRONE_IDS):
            raise ValueError(f"{label} must contain exactly UAV-01 and UAV-02")
        return {drone: cls._vector(values[drone], label) for drone in DRONE_IDS}

    @staticmethod
    def _vector(value: Sequence[float], label: str) -> Vector3:
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ValueError(f"{label} must be a three-vector")
        vector = tuple(float(component) for component in value)
        if not all(math.isfinite(component) for component in vector):
            raise ValueError(f"{label} must be finite")
        return vector  # type: ignore[return-value]

    @classmethod
    def _covariance(cls, value: Sequence[float] | None) -> Vector3 | None:
        if value is None:
            return None
        covariance = cls._vector(value, "covariance")
        if any(component < 0.0 for component in covariance):
            raise ValueError("covariance must be nonnegative")
        return covariance

    def _require_reset(self) -> None:
        if not self.positions:
            raise RuntimeError("reset must be called before using the environment")
