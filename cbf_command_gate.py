"""Deterministic control-barrier safety gate for ENU nominal velocities.

This is a shadow command gate.  It never publishes to PX4.  The solver finds
the nearest bounded velocity satisfying pairwise zeroing-CBF constraints and
geofence/altitude bounds; it returns Hold when state is stale or infeasible.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from typing import Any


Vector3 = tuple[float, float, float]


def cbf_uncertainty_source_enabled() -> bool:
    """SWARM_CBF_UNCERTAINTY_SOURCE: `off` (default) or `odometry`.

    Read here so the producer (companion bridge) and the consumer (this gate)
    cannot disagree about whether covariance is live. Anything other than
    `odometry` reads as off, so a typo cannot silently arm a safety change.
    """
    return os.environ.get("SWARM_CBF_UNCERTAINTY_SOURCE", "off").strip().lower() == "odometry"


def cbf_covariance_sigma() -> float:
    """Runtime sigma knob; invalid text preserves the historical default."""
    try:
        return float(os.environ.get("SWARM_CBF_COVARIANCE_SIGMA", "2.0"))
    except (TypeError, ValueError):
        return 2.0


def cbf_position_covariance_required() -> bool:
    """Whether missing covariance must hold; independent of publication."""
    return os.environ.get(
        "SWARM_CBF_REQUIRE_POSITION_COVARIANCE", "false"
    ).strip().lower() in {"1", "true", "yes", "on"}


@dataclass(frozen=True)
class CbfConfig:
    minimum_separation_m: float = 4.0
    barrier_gain_s_inv: float = 2.0
    maximum_velocity_m_s: float = 2.0
    geofence_min_enu_m: Vector3 = (-100.0, -100.0, 0.0)
    geofence_max_enu_m: Vector3 = (100.0, 100.0, 50.0)
    lookahead_s: float = 1.0
    covariance_sigma: float = 2.0
    command_latency_s: float = 0.10
    # Optional second-order reserve.  This is the guaranteed *relative*
    # deceleration of the pair, so two vehicles each limited to 4 m/s^2 use
    # 8 m/s^2.  Zero preserves the legacy first-order CBF contract.
    relative_braking_acceleration_m_s2: float = 0.0
    tracking_reserve_m: float = 0.0
    # Solve-time-only buffer added to required_margin, NOT to the reported
    # minimum_margin_m (see filter() below). Exists because a minimally-
    # invasive CBF, when a constraint is genuinely active, converges its
    # solution exactly onto the constraint boundary by construction -- not
    # approximately, but to within simulation discretization noise. Measured
    # directly: ACTIVE_CBF_CROSSING_TRAJECTORY's 2026-08-10 investigation
    # swept crossing angle x speed x spawn-separation scale with
    # command_latency_s at its real measured value (0.65s, see that
    # constant's own comment) and found NO geometry, anywhere in that space,
    # with both real intervention (>0.1 m/s, above PX4's measured tracking
    # noise floor) and a comfortable reported margin -- every point with
    # margin >= 0.3m had zero intervention, and margin collapsed to
    # ~0.000-0.03m at every point where intervention turned on. The two real
    # aborted flights' small "positive" simulated margins (+0.045m, +0.124m)
    # were most likely discretization artifacts of that same zero-convergence
    # property, not genuine buffer -- consistent with why real PX4 tracking
    # imperfection, however small, pushed them negative in practice. Default
    # 0.0 preserves that exact prior (razor's-edge) behavior for every
    # already-validated milestone; a positive value is an opt-in design
    # decision for scenarios (like a genuine crossing) that need CBF to hold
    # real distance in reserve, not just react at the last instant.
    #
    # NOT a universal "safer is bigger" knob -- verified unsafe at higher
    # closing speeds. required_margin_solve feeds age_latency (via
    # relative_speed) on the NEXT frame too, so a larger buffer commands a
    # stronger correction, which raises relative_speed, which raises
    # required_margin, in a loop. Swept 2026-08-10: at low closing speed
    # (~0.3-0.7 m/s per vehicle) a buffer of 0.4-0.6m is stable and gives a
    # genuine ~0.3-0.7m reported margin with real correction; at higher
    # closing speed (>=1.0 m/s) the SAME buffer values make margin and
    # infeasible-frame counts WORSE than buffer=0.0, not better (e.g.
    # buffer=1.0 at 1.0 m/s: 253 infeasible frames vs. 0 at buffer=0.0).
    # Choose this per-scenario against the real closing speed involved, by
    # simulation -- never as a blanket increase.
    design_margin_buffer_m: float = 0.0
    # Fail closed when a state carries no position covariance, instead of
    # reading the absence as (0,0,0) -- "perfectly certain" -- which shrinks
    # required_margin and is the less conservative direction. Off by default,
    # This is deliberately independent of covariance publication, allowing
    # publish + sigma=0 observation without changing the CBF margin math.
    require_position_covariance: bool = False

    def __post_init__(self) -> None:
        scalars = (
            self.minimum_separation_m, self.barrier_gain_s_inv,
            self.maximum_velocity_m_s, self.lookahead_s,
            self.covariance_sigma, self.command_latency_s,
            self.relative_braking_acceleration_m_s2,
            self.tracking_reserve_m,
            self.design_margin_buffer_m,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in scalars):
            raise ValueError("CBF configuration is invalid")
        if self.minimum_separation_m <= 0.0 or self.maximum_velocity_m_s <= 0.0 or self.lookahead_s <= 0.0:
            raise ValueError("CBF configuration must have positive limits")
        if not _finite(self.geofence_min_enu_m + self.geofence_max_enu_m):
            raise ValueError("CBF geofence must be finite")
        if any(self.geofence_min_enu_m[i] >= self.geofence_max_enu_m[i] for i in range(3)):
            raise ValueError("CBF geofence bounds are invalid")


@dataclass(frozen=True)
class CbfCommand:
    drone_id: str
    velocity_enu_m_s: Vector3
    active: bool
    reason: str
    minimum_margin_m: float | None
    intervention_norm_m_s: float
    critical_peer_id: str | None = None
    critical_distance_m: float | None = None
    critical_required_separation_m: float | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "drone_id": self.drone_id,
            "frame": "ENU",
            "velocity_enu_m_s": list(self.velocity_enu_m_s),
            "active": self.active,
            "reason": self.reason,
            "minimum_margin_m": round(self.minimum_margin_m, 3)
            if self.minimum_margin_m is not None else None,
            "critical_peer_id": self.critical_peer_id,
            "critical_distance_m": round(self.critical_distance_m, 3)
            if self.critical_distance_m is not None else None,
            "critical_required_separation_m": round(
                self.critical_required_separation_m, 3
            )
            if self.critical_required_separation_m is not None else None,
            "intervention_norm_m_s": round(self.intervention_norm_m_s, 4),
            "authority": "shadow_safety_gate_only",
        }


class CbfCommandGate:
    def __init__(self, drone_id: str, required_peer_ids: tuple[str, ...], config: CbfConfig | None = None) -> None:
        if not drone_id.strip() or drone_id in required_peer_ids:
            raise ValueError("CBF identity is invalid")
        self.drone_id = drone_id
        self.required_peer_ids = tuple(required_peer_ids)
        self.config = config or CbfConfig()

    def filter(self, nominal_velocity_enu_m_s: Any, swarm_state: dict[str, Any]) -> CbfCommand:
        nominal = _vector(nominal_velocity_enu_m_s)
        own = _state(swarm_state.get(self.drone_id), self.config.require_position_covariance)
        if nominal is None or own is None:
            return self._hold("nominal_or_self_state_invalid")
        peer_states = {}
        for peer_id in self.required_peer_ids:
            peer = _state(swarm_state.get(peer_id), self.config.require_position_covariance)
            if peer is None:
                return self._hold("peer_state_invalid")
            peer_states[peer_id] = peer
        constraints: list[tuple[Vector3, float]] = []
        lower = [
            (self.config.geofence_min_enu_m[i] - own[0][i]) / self.config.lookahead_s
            for i in range(3)
        ]
        upper = [
            (self.config.geofence_max_enu_m[i] - own[0][i]) / self.config.lookahead_s
            for i in range(3)
        ]
        if any(lower[i] > upper[i] for i in range(3)):
            return self._hold("outside_geofence")
        margin_values = []
        critical_peer_id: str | None = None
        critical_distance_m: float | None = None
        critical_required_separation_m: float | None = None
        critical_margin_m = math.inf
        for peer_id, (
            peer_position,
            peer_velocity,
            peer_covariance,
            peer_age_ms,
        ) in peer_states.items():
            relative = tuple(own[0][i] - peer_position[i] for i in range(3))
            distance = _norm(relative)
            if distance <= 1.0e-6:
                return self._hold(
                    "peer_position_overlap",
                    critical_peer_id=peer_id,
                    critical_distance_m=distance,
                )
            relative_velocity = tuple(
                own[1][i] - peer_velocity[i] for i in range(3)
            )
            relative_speed = _norm(relative_velocity)
            radial_rate = _dot(relative, relative_velocity) / distance
            closing_speed = max(0.0, -radial_rate)
            uncertainty = self.config.covariance_sigma * math.sqrt(
                max(0.0, sum(own[2]) + sum(peer_covariance))
            )
            if self.config.relative_braking_acceleration_m_s2 > 0.0:
                reserve_speed = closing_speed
                stopping_distance = closing_speed * closing_speed / (
                    2.0 * self.config.relative_braking_acceleration_m_s2
                )
            else:
                # Frozen legacy behavior for old 4 m policies.
                reserve_speed = relative_speed
                stopping_distance = 0.0
            age_latency = (
                max(own[3], peer_age_ms) / 1000.0
                + self.config.command_latency_s
            ) * reserve_speed
            required_margin = (
                self.config.minimum_separation_m
                + uncertainty
                + age_latency
                + stopping_distance
                + self.config.tracking_reserve_m
            )
            # Reported margin stays anchored to the UNBUFFERED required_margin
            # -- the true physical/latency floor -- so minimum_margin_m keeps
            # meaning "distance in reserve above the real collision-avoidance
            # limit" regardless of design_margin_buffer_m. Only the solve
            # target below is buffered.
            margin = distance - required_margin
            margin_values.append(margin)
            if margin < critical_margin_m:
                critical_margin_m = margin
                critical_peer_id = peer_id
                critical_distance_m = distance
                critical_required_separation_m = required_margin
            required_margin_solve = required_margin + self.config.design_margin_buffer_m
            h = distance * distance - required_margin_solve * required_margin_solve
            # d/dt h + alpha*h >= 0, where relative = own - peer.
            required_dot = _dot(relative, peer_velocity) - 0.5 * self.config.barrier_gain_s_inv * h
            constraints.append((relative, required_dot))
        candidate = list(_limit_norm(nominal, self.config.maximum_velocity_m_s))
        for _ in range(12):
            for axis in range(3):
                candidate[axis] = min(upper[axis], max(lower[axis], candidate[axis]))
            candidate[:] = _limit_norm(tuple(candidate), self.config.maximum_velocity_m_s)
            for normal, required_dot in constraints:
                violation = required_dot - _dot(normal, tuple(candidate))
                if violation > 0.0:
                    scale = violation / max(_dot(normal, normal), 1e-9)
                    for axis in range(3):
                        candidate[axis] += scale * normal[axis]
        velocity = tuple(candidate)
        if (
            any(velocity[i] < lower[i] - 1e-5 or velocity[i] > upper[i] + 1e-5 for i in range(3))
            or _norm(velocity) > self.config.maximum_velocity_m_s + 1e-5
            or any(_dot(normal, velocity) < required_dot - 1e-4 for normal, required_dot in constraints)
        ):
            return self._hold(
                "cbf_constraints_infeasible",
                min(margin_values, default=None),
                critical_peer_id=critical_peer_id,
                critical_distance_m=critical_distance_m,
                critical_required_separation_m=critical_required_separation_m,
            )
        return CbfCommand(
            self.drone_id, velocity, True, "cbf_filtered",
            min(margin_values, default=None), _norm(tuple(velocity[i] - nominal[i] for i in range(3))),
            critical_peer_id,
            critical_distance_m,
            critical_required_separation_m,
        )

    def _hold(
        self,
        reason: str,
        margin: float | None = None,
        *,
        critical_peer_id: str | None = None,
        critical_distance_m: float | None = None,
        critical_required_separation_m: float | None = None,
    ) -> CbfCommand:
        return CbfCommand(
            self.drone_id,
            (0.0, 0.0, 0.0),
            False,
            reason,
            margin,
            0.0,
            critical_peer_id,
            critical_distance_m,
            critical_required_separation_m,
        )


def _state(value: Any, require_covariance: bool = False) -> tuple[Vector3, Vector3, Vector3, float] | None:
    if not isinstance(value, dict) or not value.get("valid", False):
        return None
    position = _vector(value.get("position_enu_m"))
    velocity = _vector(value.get("velocity_enu_m_s"))
    raw_covariance = value.get("position_covariance_m2")
    if not raw_covariance:
        # Absent covariance: zero (current, less conservative behavior) unless
        # the caller demands one, in which case the state is dropped and the
        # gate holds. A malformed one below fails closed either way.
        if require_covariance:
            return None
        raw_covariance = (0.0, 0.0, 0.0)
    covariance = _vector(raw_covariance, nonnegative=True)
    try:
        age_ms = float(value.get("message_age_ms", 0.0))
    except (TypeError, ValueError):
        return None
    if position is None or velocity is None or covariance is None or not math.isfinite(age_ms) or age_ms < 0.0:
        return None
    return position, velocity, covariance, age_ms


def _vector(value: Any, *, nonnegative: bool = False) -> Vector3 | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        vector = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        return None
    if not _finite(vector) or (nonnegative and any(item < 0.0 for item in vector)):
        return None
    return vector  # type: ignore[return-value]


def _finite(values: tuple[float, ...]) -> bool:
    return all(math.isfinite(value) for value in values)


def _dot(left: Vector3, right: Vector3) -> float:
    return sum(left[i] * right[i] for i in range(3))


def _norm(vector: Vector3) -> float:
    return math.sqrt(_dot(vector, vector))


def _limit_norm(vector: Vector3, maximum: float) -> Vector3:
    length = _norm(vector)
    if length <= maximum:
        return vector
    return tuple(value * maximum / length for value in vector)  # type: ignore[return-value]
