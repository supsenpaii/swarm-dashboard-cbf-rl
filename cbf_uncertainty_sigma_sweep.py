#!/usr/bin/env python3
"""CBF_UNCERTAINTY_OFFLINE_SWEEP: characterize `CbfConfig.covariance_sigma`.

Offline only -- nothing here talks to PX4, MQTT or a peer socket. Every
scenario is a kinematic replay of the REAL flight-path objects
(`CompanionSafetyMonitor` -> nominal controller + `CbfCommandGate` +
`EmergencySupervisor`), never reimplemented math, exactly as
test_crossing_geometry_buffer.py and test_formation_spawn_geometry.py already
do for geometry decisions.

WHY THIS SWEEP EXISTS
---------------------
`covariance_sigma` has always multiplied a zero covariance, so it has never
been exercised. CBF_UNCERTAINTY_PLUMBING_VALIDATED (67cec6f) made the real
ODOMETRY covariance reachable, which makes the parameter live for the first
time. At its untouched default of 2.0 the audit predicted +1.11 m added to
`required_margin` -- more than the +0.68 m minimum margin of the flight that
already passed. This sweep measures what actually happens instead of
extrapolating that one subtraction, because the CBF REACTS to a larger
required margin: it pushes the vehicles further apart, so physical distance
grows and the reported margin does not fall by the full uncertainty term.

WHAT IS HELD FIXED
------------------
`minimum_separation_m`, `command_latency_s` and `design_margin_buffer_m` are
read from the production `.env` / from the driver module that owns each
scenario, and are IDENTICAL across every sigma in a scenario. Geometry is
taken from the modules that already fly it (`two_uav_trajectory_flight`,
`two_uav_crossing_trajectory_flight`, `run_all.sh` spawns, `.env` slot) and is
never adjusted to make a sigma look better. Only `covariance_sigma` moves.

THE COVARIANCE VALUE
--------------------
The per-axis variance measured on the real MAVLink link during the
2026-08-11 audit and pinned by test_cbf_uncertainty_plumbing.py:
NED (0.0410, 0.0411, 0.0708) m^2, held constant for both vehicles. It is a
ground reading; in-flight amplitude is not yet known, which is stated in the
verdict rather than assumed away.

Run: python3 cbf_uncertainty_sigma_sweep.py [--json PATH]
"""

from __future__ import annotations

import argparse
import json
import math
import re
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from cbf_command_gate import CbfConfig
from companion_safety import CompanionSafetyMonitor
from conflict_coordinator import reset_shared_conflict_state
from formation_controller import FormationConfig, FormationSlot
from swarm_state import ned_variance_to_enu
from trajectory_controller import LinearTrajectory, Trajectory
from two_uav_crossing_trajectory_flight import (
    DEFAULT_TRAJECTORY_ENV as CROSSING_ENV,
    REQUIRED_CBF_ENV as CROSSING_CBF_ENV,
)
from two_uav_trajectory_flight import DEFAULT_TRAJECTORY_ENV as PARALLEL_ENV

UAV_01 = "UAV-01"
UAV_02 = "UAV-02"
DT_S = 0.02
SIGMAS = (0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0)

# Measured on the real link, 2026-08-11 audit; pinned in the plumbing tests.
VARIANCE_NED_M2 = (0.0410, 0.0411, 0.0708)
VARIANCE_ENU_M2 = ned_variance_to_enu(VARIANCE_NED_M2)

# This project's measured PX4 horizontal-velocity tracking noise floor. A
# correction below this could not be attributed to CBF on a real flight, so
# "intervention" is counted above it -- same threshold the crossing-geometry
# test uses for the same reason.
NOISE_FLOOR_M_S = 0.1
# Considered "not moving". Well below the noise floor: this is about a
# commanded velocity of essentially zero, not about what a real vehicle could
# resolve. Deadlock is a vehicle whose NOMINAL wants to move while its output
# is pinned at zero -- station-keeping hold, where the nominal is itself zero,
# is the intended terminal state and must not be counted as a deadlock.
STOPPED_M_S = 0.02

REPO = Path(__file__).resolve().parent


def _env_file() -> dict[str, str]:
    path = REPO / ".env"
    if not path.exists():
        return {}
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        match = re.match(r"^([A-Z0-9_]+)=(.*)$", line.strip())
        if match:
            values[match.group(1)] = match.group(2).strip()
    return values


ENV = _env_file()


def _env_float(name: str, default: float) -> float:
    try:
        return float(ENV.get(name, default))
    except ValueError:
        return default


def _env_vector(name: str, default: str) -> tuple[float, float, float]:
    parts = tuple(float(part) for part in ENV.get(name, default).split(","))
    if len(parts) != 3:
        raise ValueError(f"{name} needs three components")
    return parts  # type: ignore[return-value]


def _vector(value: str) -> tuple[float, float, float]:
    parts = tuple(float(part.strip()) for part in value.split(","))
    if len(parts) != 3:
        raise ValueError(f"expected three components, got {value!r}")
    return parts  # type: ignore[return-value]


def _leg(env: dict[str, str], drone: str) -> LinearTrajectory:
    tag = drone.replace("-", "_")
    return LinearTrajectory(
        _vector(env[f"SWARM_TRAJECTORY_{tag}_START_ENU_M"]),
        _vector(env[f"SWARM_TRAJECTORY_{tag}_END_ENU_M"]),
        float(env[f"SWARM_TRAJECTORY_{tag}_SPEED_M_S"]),
    )


FORMATION_CONFIG = FormationConfig(
    position_gain_s_inv=_env_float("SWARM_FORMATION_POSITION_GAIN_S_INV", 0.6),
    maximum_velocity_m_s=_env_float("SWARM_FORMATION_MAXIMUM_VELOCITY_M_S", 2.0),
    arrival_radius_m=_env_float("SWARM_FORMATION_ARRIVAL_RADIUS_M", 0.25),
)
SLOT_UAV_02 = _env_vector("SWARM_FORMATION_SLOT_UAV_02_ENU_M", "-10,0,0")
# run_all.sh spawn poses, at the shared-ENU hover altitude the trajectories use.
SPAWN = {UAV_01: (0.0, 0.0, 9.0), UAV_02: (-5.0, 2.0, 9.0)}


@dataclass(frozen=True)
class Scenario:
    name: str
    note: str
    horizon_s: float
    spawn: dict[str, tuple[float, float, float]]
    trajectories: dict[str, Trajectory] | None = None
    slots: tuple[FormationSlot, ...] = ()
    command_latency_s: float = 0.65
    design_margin_buffer_m: float = 0.0
    peer_age_ms: float = 25.0
    # (start_s, end_s, peer_age_ms) -- models a degraded window inside a run.
    degraded_window: tuple[float, float, float] | None = None
    # The vehicle this scenario is meant to be about. Zero means the historical
    # default: no lag and no acceleration limit, i.e. velocity changes
    # instantaneously. Fine for a question purely about uncertainty, wrong for
    # any question about braking -- see the limiter in the integration loop.
    response_time_constant_s: float = 0.0
    maximum_acceleration_m_s2: float = 0.0
    completion_reasons: tuple[str, ...] = ("trajectory_reached", "slot_reached")


def scenarios() -> tuple[Scenario, ...]:
    crossing = {d: _leg(CROSSING_ENV, d) for d in (UAV_01, UAV_02)}
    return (
        Scenario(
            name="formation_baseline",
            note=(
                "run_all.sh spawns -> .env formation slot, the geometry "
                "test_formation_spawn_geometry.py already pins."
            ),
            horizon_s=60.0,
            spawn=SPAWN,
            slots=(FormationSlot(UAV_02, SLOT_UAV_02),),
        ),
        Scenario(
            name="parallel_trajectory_baseline",
            note=(
                "TWO_UAV_TRAJECTORY_TRACKING legs from two_uav_trajectory_flight."
                "DEFAULT_TRAJECTORY_ENV: parallel, equal speed, non-conflict."
            ),
            horizon_s=30.0,
            spawn={d: crossing_start for d, crossing_start in (
                (UAV_01, _vector(PARALLEL_ENV["SWARM_TRAJECTORY_UAV_01_START_ENU_M"])),
                (UAV_02, _vector(PARALLEL_ENV["SWARM_TRAJECTORY_UAV_02_START_ENU_M"])),
            )},
            trajectories={d: _leg(PARALLEL_ENV, d) for d in (UAV_01, UAV_02)},
        ),
        Scenario(
            name="crossing_validated",
            note=(
                "ACTIVE_CBF_CROSSING_TRAJECTORY's FLIGHT_PASS configuration: "
                "two_uav_crossing_trajectory_flight.DEFAULT_TRAJECTORY_ENV with "
                "its REQUIRED_CBF_ENV buffer. The genuine-conflict case."
            ),
            horizon_s=120.0,
            spawn={d: t.start_enu_m for d, t in crossing.items()},
            trajectories=crossing,
            design_margin_buffer_m=float(
                CROSSING_CBF_ENV["SWARM_CBF_DESIGN_MARGIN_BUFFER_M"]
            ),
            # Measured horizontal response for the high-speed profile.
            # and the airframe's own MPC_ACC_HOR_MAX. Without them this
            # scenario's vehicle stops dead in one 20 ms step, which flatters
            # every result that depends on how fast it can slow down.
            response_time_constant_s=0.860,
            maximum_acceleration_m_s2=4.0,
        ),
        Scenario(
            name="stale_peer_crossing_450ms",
            note=(
                "Same crossing geometry and CBF config, peer state aged to "
                "450 ms -- just inside SWARM_PEER_STATE_MAX_AGE_MS=500, where "
                "age_latency x relative_speed is largest. Only the age moves."
            ),
            horizon_s=120.0,
            spawn={d: t.start_enu_m for d, t in crossing.items()},
            trajectories=crossing,
            design_margin_buffer_m=float(
                CROSSING_CBF_ENV["SWARM_CBF_DESIGN_MARGIN_BUFFER_M"]
            ),
            peer_age_ms=450.0,
            # Same airframe as crossing_validated; only the age moves.
            response_time_constant_s=0.860,
            maximum_acceleration_m_s2=4.0,
        ),
        Scenario(
            name="stale_peer_crossing_150ms",
            note=(
                "Same again at 150 ms -- a delay the link can plausibly show "
                "(the PASS flight measured a 52 ms max loop gap) rather than "
                "the legal worst case, so this row still discriminates sigma."
            ),
            horizon_s=120.0,
            spawn={d: t.start_enu_m for d, t in crossing.items()},
            trajectories=crossing,
            design_margin_buffer_m=float(
                CROSSING_CBF_ENV["SWARM_CBF_DESIGN_MARGIN_BUFFER_M"]
            ),
            peer_age_ms=150.0,
            # Same airframe as crossing_validated; only the age moves.
            response_time_constant_s=0.860,
            maximum_acceleration_m_s2=4.0,
        ),
        Scenario(
            name="server_loss_station_keeping",
            note=(
                "Offline analogue of ACTIVE_FLIGHT_FAULT_INJECTION: the same "
                "formation spawn->slot->hold the PASS flight flew, with a 30 s "
                "window at t=30 s where peer age rises to the 52 ms max loop "
                "gap that flight measured. Server loss does not change CBF "
                "inputs offline -- the companion already runs on P2P state "
                "only -- so the degraded evaluation cadence is what is modeled."
            ),
            horizon_s=90.0,
            spawn=SPAWN,
            slots=(FormationSlot(UAV_02, SLOT_UAV_02),),
            degraded_window=(30.0, 60.0, 52.0),
        ),
    )


@dataclass
class Run:
    scenario: str
    sigma: float
    uncertainty_m: float
    required_separation_max_m: float = 0.0
    min_distance_m: float = float("inf")
    min_margin_reported_m: float = float("inf")
    min_physical_slack_m: float = float("inf")
    intervention_frames: int = 0
    vehicle_frames: int = 0
    max_correction_m_s: float = 0.0
    infeasible_frames: int = 0
    first_infeasible_s: float | None = None
    longest_deadlock_s: float = 0.0
    completed: dict[str, float | None] = field(default_factory=dict)
    max_stage: dict[str, str] = field(default_factory=dict)
    escalations: dict[str, int] = field(default_factory=dict)
    hold_frames: int = 0

    @property
    def intervention_rate(self) -> float:
        return self.intervention_frames / max(1, self.vehicle_frames)

    def as_dict(self) -> dict[str, Any]:
        return {
            "scenario": self.scenario,
            "covariance_sigma": self.sigma,
            "uncertainty_m": round(self.uncertainty_m, 4),
            "required_separation_max_m": round(self.required_separation_max_m, 3),
            "min_distance_m": round(self.min_distance_m, 3),
            "min_margin_reported_m": round(self.min_margin_reported_m, 3),
            "min_physical_slack_m": round(self.min_physical_slack_m, 3),
            "intervention_rate": round(self.intervention_rate, 4),
            "max_correction_m_s": round(self.max_correction_m_s, 4),
            "infeasible_frames": self.infeasible_frames,
            "first_infeasible_s": self.first_infeasible_s,
            "hold_frames": self.hold_frames,
            "longest_deadlock_s": round(self.longest_deadlock_s, 2),
            "completed": self.completed,
            "max_emergency_stage": self.max_stage,
            "escalations": self.escalations,
        }


def _cbf_config(scenario: Scenario, sigma: float, *, with_covariance: bool) -> CbfConfig:
    # This module replays the archived 4 m / 2 m/s flight-validation milestone.
    # Keep that versioned contract independent of today's 20 m production
    # envelope so historical evidence remains exactly reproducible.
    return CbfConfig(
        minimum_separation_m=4.0,
        barrier_gain_s_inv=_env_float("SWARM_CBF_BARRIER_GAIN_S_INV", 2.0),
        maximum_velocity_m_s=2.0,
        command_latency_s=scenario.command_latency_s,
        design_margin_buffer_m=scenario.design_margin_buffer_m,
        covariance_sigma=sigma,
        require_position_covariance=with_covariance,
        geofence_min_enu_m=_env_vector("SWARM_CBF_GEOFENCE_MIN_ENU_M", "-100,-100,0"),
        geofence_max_enu_m=_env_vector("SWARM_CBF_GEOFENCE_MAX_ENU_M", "100,100,50"),
    )


def simulate(
    scenario: Scenario,
    sigma: float,
    *,
    with_covariance: bool = True,
    covariance_frames: tuple[
        dict[str, tuple[float, float, float]], ...
    ] | None = None,
    covariance_sample_period_s: float = DT_S,
    command_delay_s: float = 0.0,
    velocity_time_constant_s: float | None = None,
    maximum_acceleration_m_s2: float | None = None,
) -> Run:
    """One kinematic replay. `with_covariance=False` is the feature-off flight
    baseline: no covariance published anywhere, gate not requiring one."""
    if velocity_time_constant_s is None:
        velocity_time_constant_s = scenario.response_time_constant_s
    if maximum_acceleration_m_s2 is None:
        maximum_acceleration_m_s2 = scenario.maximum_acceleration_m_s2
    if not math.isfinite(command_delay_s) or command_delay_s < 0.0:
        raise ValueError("command delay must be finite and nonnegative")
    if not math.isfinite(velocity_time_constant_s) or velocity_time_constant_s < 0.0:
        raise ValueError("velocity time constant must be finite and nonnegative")
    if not math.isfinite(maximum_acceleration_m_s2) or maximum_acceleration_m_s2 < 0.0:
        raise ValueError("maximum acceleration must be finite and nonnegative")

    drones = tuple(scenario.spawn)
    # The coordinator keys its encounter ledger by drone pair in a module
    # global, so run N+1 would otherwise inherit N's priority alternation and
    # its release latch and answer a different question than the one asked --
    # the same reason the corridor replay resets it. Without this, a sweep
    # gives one answer under pytest and another standalone, which was how the
    # 2026-08-17 acceleration-limit change surfaced as a mysterious failure in
    # a test that passed on its own.
    reset_shared_conflict_state(drones)
    config = _cbf_config(scenario, sigma, with_covariance=with_covariance)
    monitors = {
        drone: CompanionSafetyMonitor(
            drone_id=drone,
            peer_ids=tuple(other for other in drones if other != drone),
            leader_id=UAV_01,
            slots=scenario.slots,
            formation_config=FORMATION_CONFIG,
            cbf_config=config,
            trajectory=(
                scenario.trajectories.get(drone) if scenario.trajectories else None
            ),
        )
        for drone in drones
    }
    if covariance_frames is not None:
        if not with_covariance or not covariance_frames:
            raise ValueError("covariance trace requires covariance publication")
        if not math.isfinite(covariance_sample_period_s) or covariance_sample_period_s <= 0.0:
            raise ValueError("covariance sample period must be positive")
        if any(any(drone not in frame for drone in drones) for frame in covariance_frames):
            raise ValueError("covariance trace is missing a drone")
        uncertainty = sigma * max(
            math.sqrt(sum(sum(frame[drone]) for drone in drones))
            for frame in covariance_frames
        )
    else:
        uncertainty = (
            sigma * math.sqrt(2.0 * sum(VARIANCE_ENU_M2))
            if with_covariance
            else 0.0
        )
    covariance = list(VARIANCE_ENU_M2) if with_covariance else None

    position = {drone: list(scenario.spawn[drone]) for drone in drones}
    velocity = {drone: [0.0, 0.0, 0.0] for drone in drones}
    delay_steps = math.ceil(command_delay_s / DT_S)
    pending_commands = {
        drone: deque([(0.0, 0.0, 0.0)] * delay_steps) for drone in drones
    }
    run = Run(scenario.name, sigma, uncertainty)
    run.completed = {drone: None for drone in drones}
    run.max_stage = {drone: "normal" for drone in drones}
    run.escalations = {drone: 0 for drone in drones}
    previous_stage = {drone: "normal" for drone in drones}
    blocked_since: dict[str, float | None] = {drone: None for drone in drones}

    for step in range(int(scenario.horizon_s / DT_S)):
        now = step * DT_S
        covariance_frame = (
            covariance_frames[
                min(int(now / covariance_sample_period_s), len(covariance_frames) - 1)
            ]
            if covariance_frames is not None
            else None
        )
        peer_age_ms = scenario.peer_age_ms
        if scenario.degraded_window is not None:
            start, end, degraded = scenario.degraded_window
            if start <= now < end:
                peer_age_ms = degraded
        state = {
            drone: {
                "valid": True,
                "position_enu_m": list(position[drone]),
                "velocity_enu_m_s": list(velocity[drone]),
                "position_covariance_m2": (
                    list(covariance_frame[drone])
                    if covariance_frame is not None
                    else covariance
                ),
                "message_age_ms": peer_age_ms,
            }
            for drone in drones
        }
        commanded = {}
        for drone in drones:
            status = monitors[drone].evaluate(state, now, station_keeping=True)
            command = status.command
            run.vehicle_frames += 1
            if command.minimum_margin_m is not None:
                run.min_margin_reported_m = min(
                    run.min_margin_reported_m, command.minimum_margin_m
                )
                distance = _distance(position, drones)
                run.required_separation_max_m = max(
                    run.required_separation_max_m, distance - command.minimum_margin_m
                )
            if command.reason == "cbf_constraints_infeasible":
                run.infeasible_frames += 1
                if run.first_infeasible_s is None:
                    run.first_infeasible_s = round(now, 2)
            if not command.active:
                run.hold_frames += 1
            if command.intervention_norm_m_s > NOISE_FLOOR_M_S:
                run.intervention_frames += 1
            run.max_correction_m_s = max(
                run.max_correction_m_s, command.intervention_norm_m_s
            )
            stage = status.emergency.stage.value
            if stage != previous_stage[drone]:
                if _stage_index(stage) > _stage_index(previous_stage[drone]):
                    run.escalations[drone] += 1
                previous_stage[drone] = stage
            if _stage_index(stage) > _stage_index(run.max_stage[drone]):
                run.max_stage[drone] = stage
            if (
                run.completed[drone] is None
                and status.nominal_reason in scenario.completion_reasons
            ):
                run.completed[drone] = round(now, 2)
            commanded[drone] = (
                status.output_velocity_enu_m_s if status.output_valid else (0.0, 0.0, 0.0)
            )
            wants_to_move = _norm(status.nominal_velocity_enu_m_s) > STOPPED_M_S
            if wants_to_move and _norm(commanded[drone]) < STOPPED_M_S:
                blocked_since[drone] = now if blocked_since[drone] is None else blocked_since[drone]
                run.longest_deadlock_s = max(
                    run.longest_deadlock_s, now - blocked_since[drone] + DT_S
                )
            else:
                blocked_since[drone] = None

        distance = _distance(position, drones)
        run.min_distance_m = min(run.min_distance_m, distance)

        for drone in drones:
            target_velocity = commanded[drone]
            if delay_steps:
                pending_commands[drone].append(target_velocity)
                target_velocity = pending_commands[drone].popleft()
            if velocity_time_constant_s > 0.0:
                alpha = 1.0 - math.exp(-DT_S / velocity_time_constant_s)
                applied_velocity = tuple(
                    velocity[drone][axis]
                    + alpha * (target_velocity[axis] - velocity[drone][axis])
                    for axis in range(3)
                )
            else:
                applied_velocity = target_velocity
            # Zero means unlimited, which is what this sweep has always used:
            # with the lag also defaulting to zero, its vehicle changes
            # velocity instantaneously. That is a defensible simplification
            # for a question about uncertainty and a disqualifying one for any
            # question about braking dynamics -- an aircraft that can already
            # stop instantly gains nothing from stopping sooner.
            if maximum_acceleration_m_s2 > 0.0:
                change = math.sqrt(
                    sum(
                        (applied_velocity[axis] - velocity[drone][axis]) ** 2
                        for axis in range(3)
                    )
                )
                allowed = maximum_acceleration_m_s2 * DT_S
                if change > allowed:
                    scale = allowed / change
                    applied_velocity = tuple(
                        velocity[drone][axis]
                        + (applied_velocity[axis] - velocity[drone][axis]) * scale
                        for axis in range(3)
                    )
            for axis in range(3):
                position[drone][axis] += applied_velocity[axis] * DT_S
            velocity[drone] = list(applied_velocity)

    run.min_physical_slack_m = run.min_distance_m - config.minimum_separation_m
    return run


def _norm(vector: Any) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in vector))


def _distance(position: dict[str, list[float]], drones: tuple[str, ...]) -> float:
    a, b = position[drones[0]], position[drones[1]]
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


_STAGES = (
    "normal",
    "stop_horizontal",
    "reduce_velocity",
    "altitude_separation",
    "hold",
    "recommend_position_hold",
    "recommend_rtl_or_land",
)


def _stage_index(stage: str) -> int:
    return _STAGES.index(stage) if stage in _STAGES else -1


# Resolution of the fine scan that locates the feasibility edge. A linear
# scan, not a bisection: nothing guarantees the feasibility predicate is
# monotone in sigma (required_margin feeds relative_speed on the next frame,
# the same feedback loop design_margin_buffer_m documents), so the scan also
# reports whether any feasible point exists ABOVE the first failure.
SCAN_STEP = 0.05
SCAN_MAX = 2.0
DEADLOCK_S = 3.0


def _expected_completers(scenario: Scenario) -> tuple[str, ...]:
    """Who is supposed to finish. The formation leader holds station and has
    no slot, so it has no completion event -- absence of one is not a stall."""
    if scenario.trajectories:
        return tuple(scenario.trajectories)
    return tuple(slot.drone_id for slot in scenario.slots)


def _feasible(row: dict[str, Any], completers: tuple[str, ...]) -> tuple[bool, bool]:
    """(hard, strict).

    hard   -- the run stays flyable: no infeasible frame, no emergency
              escalation, no stall, everyone who should finish finishes, and
              the vehicles never come closer than minimum_separation_m.
    strict -- hard, AND the CBF's own reported margin never goes negative,
              i.e. the filter met its own latency+uncertainty requirement
              rather than merely avoiding the hard floor.
    """
    hard = (
        row["infeasible_frames"] == 0
        and all(stage == "normal" for stage in row["max_emergency_stage"].values())
        and all(row["completed"].get(drone) is not None for drone in completers)
        and row["min_physical_slack_m"] > 0.0
        and row["longest_deadlock_s"] < DEADLOCK_S
    )
    return hard, hard and row["min_margin_reported_m"] >= 0.0


def characterize(scenario: Scenario) -> dict[str, Any]:
    completers = _expected_completers(scenario)
    steps = int(round(SCAN_MAX / SCAN_STEP)) + 1
    hard_edge: float | None = None
    strict_edge: float | None = None
    first_hard_failure: float | None = None
    first_strict_failure: float | None = None
    feasible_above_failure: list[float] = []
    for index in range(steps):
        sigma = round(index * SCAN_STEP, 2)
        hard, strict = _feasible(simulate(scenario, sigma).as_dict(), completers)
        if first_hard_failure is None:
            if hard:
                hard_edge = sigma
            else:
                first_hard_failure = sigma
        elif hard:
            feasible_above_failure.append(sigma)
        if first_strict_failure is None:
            if strict:
                strict_edge = sigma
            else:
                first_strict_failure = sigma
    return {
        "scenario": scenario.name,
        "expected_completers": list(completers),
        "hard_feasible_up_to_sigma": hard_edge,
        "strict_feasible_up_to_sigma": strict_edge,
        "first_hard_failure_sigma": first_hard_failure,
        "feasible_islands_above_failure": feasible_above_failure,
        "scan_step": SCAN_STEP,
    }


def run_sweep() -> dict[str, Any]:
    results: list[dict[str, Any]] = []
    baselines: dict[str, dict[str, Any]] = {}
    sigma_zero_reproduces_baseline = True
    for scenario in scenarios():
        baseline = simulate(scenario, 0.0, with_covariance=False).as_dict()
        baseline["covariance_sigma"] = "feature_off"
        baselines[scenario.name] = baseline
        for sigma in SIGMAS:
            row = simulate(scenario, sigma).as_dict()
            if sigma == 0.0:
                comparable = {k: v for k, v in row.items() if k != "covariance_sigma"}
                if comparable != {
                    k: v for k, v in baseline.items() if k != "covariance_sigma"
                }:
                    sigma_zero_reproduces_baseline = False
            results.append(row)
    edges = [characterize(scenario) for scenario in scenarios()]
    return {
        "milestone": "CBF_UNCERTAINTY_OFFLINE_SWEEP",
        "variance_ned_m2": list(VARIANCE_NED_M2),
        "variance_enu_m2": list(VARIANCE_ENU_M2),
        "held_fixed": {
            "minimum_separation_m": _env_float("SWARM_CBF_MINIMUM_SEPARATION_M", 4.0),
            "command_latency_s": 0.65,
            "design_margin_buffer_m_per_scenario": {
                scenario.name: scenario.design_margin_buffer_m
                for scenario in scenarios()
            },
            "dt_s": DT_S,
        },
        "sigma_zero_reproduces_feature_off_baseline": sigma_zero_reproduces_baseline,
        "feature_off_baselines": baselines,
        "runs": results,
        "feasible_region": edges,
        "notes": {scenario.name: scenario.note for scenario in scenarios()},
    }


def _print(report: dict[str, Any]) -> None:
    header = (
        f"{'scenario':<28}{'sigma':>6}{'unc_m':>8}{'req_max':>9}{'min_d':>8}"
        f"{'margin':>9}{'slack':>8}{'interv':>8}{'maxcorr':>9}{'infeas':>7}"
        f"{'dead_s':>7}  completion / stage"
    )
    print(header)
    print("-" * len(header))
    current = ""
    for row in report["runs"]:
        if row["scenario"] != current:
            current = row["scenario"]
            base = report["feature_off_baselines"][current]
            print(
                f"{current:<28}{'OFF':>6}{base['uncertainty_m']:>8.3f}"
                f"{base['required_separation_max_m']:>9.3f}{base['min_distance_m']:>8.3f}"
                f"{base['min_margin_reported_m']:>9.3f}{base['min_physical_slack_m']:>8.3f}"
                f"{base['intervention_rate']:>8.3f}{base['max_correction_m_s']:>9.3f}"
                f"{base['infeasible_frames']:>7}{base['longest_deadlock_s']:>7.1f}"
                f"  {base['completed']} {base['max_emergency_stage']}"
            )
        print(
            f"{'':<28}{row['covariance_sigma']:>6.2f}{row['uncertainty_m']:>8.3f}"
            f"{row['required_separation_max_m']:>9.3f}{row['min_distance_m']:>8.3f}"
            f"{row['min_margin_reported_m']:>9.3f}{row['min_physical_slack_m']:>8.3f}"
            f"{row['intervention_rate']:>8.3f}{row['max_correction_m_s']:>9.3f}"
            f"{row['infeasible_frames']:>7}{row['longest_deadlock_s']:>7.1f}"
            f"  {row['completed']} {row['max_emergency_stage']}"
        )
    print()
    print(
        "sigma=0 reproduces the feature-off baseline: "
        f"{report['sigma_zero_reproduces_feature_off_baseline']}"
    )
    print()
    edge_header = (
        f"{'scenario':<30}{'hard<=':>8}{'strict<=':>10}{'1st fail':>10}"
        f"{'islands above':>15}"
    )
    print(edge_header)
    print("-" * len(edge_header))
    for edge in report["feasible_region"]:
        islands = edge["feasible_islands_above_failure"]
        print(
            f"{edge['scenario']:<30}{str(edge['hard_feasible_up_to_sigma']):>8}"
            f"{str(edge['strict_feasible_up_to_sigma']):>10}"
            f"{str(edge['first_hard_failure_sigma']):>10}"
            f"{(str(len(islands)) + ' pts') if islands else '-':>15}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--json",
        default="artifacts/cbf_uncertainty_sweep/sigma_sweep.json",
        help="where to write the full report",
    )
    arguments = parser.parse_args()
    report = run_sweep()
    _print(report)
    destination = Path(arguments.json)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"wrote {destination}")


if __name__ == "__main__":
    main()
