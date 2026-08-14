"""Deterministic two-UAV right-of-way layer ahead of the final CBF shield."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
from typing import Any, Mapping, Sequence


Vector3 = tuple[float, float, float]


@dataclass
class _EncounterState:
    encounter: int = 0
    active: bool = False
    priority_drone_id: str | None = None
    release_count: int = 0
    released_until_clear: bool = False
    yield_side: float = 1.0


_SHARED_STATES: dict[tuple[str, str], _EncounterState] = {}
_SHARED_STATES_LOCK = threading.RLock()


def reset_shared_conflict_state(drone_ids: Sequence[str]) -> None:
    key = tuple(sorted(str(drone_id) for drone_id in drone_ids))
    with _SHARED_STATES_LOCK:
        _SHARED_STATES.pop(key, None)


@dataclass(frozen=True)
class ConflictCoordinatorConfig:
    trigger_distance_m: float = 12.0
    release_distance_m: float = 6.2
    encounter_reset_distance_m: float = 8.0
    predicted_miss_distance_m: float = 6.0
    prediction_horizon_s: float = 8.0
    reserve_separation_m: float = 5.2
    yield_gain_s_inv: float = 0.6
    release_frames: int = 10
    release_at_reserve_when_nonclosing: bool = False
    clear_uses_mission_velocity: bool = False
    yield_lateral_speed_m_s: float = 0.0

    def __post_init__(self) -> None:
        values = (
            self.trigger_distance_m,
            self.release_distance_m,
            self.encounter_reset_distance_m,
            self.predicted_miss_distance_m,
            self.prediction_horizon_s,
            self.reserve_separation_m,
            self.yield_gain_s_inv,
        )
        if (
            not all(math.isfinite(value) and value > 0.0 for value in values)
            or self.release_distance_m >= self.encounter_reset_distance_m
            or self.encounter_reset_distance_m >= self.trigger_distance_m
            or self.reserve_separation_m >= self.release_distance_m
            or self.release_frames <= 0
            or not math.isfinite(self.yield_lateral_speed_m_s)
            or self.yield_lateral_speed_m_s < 0.0
        ):
            raise ValueError("conflict coordinator configuration is invalid")


def x500_20m_conflict_config(
    maximum_velocity_m_s: float = 10.0,
) -> ConflictCoordinatorConfig:
    """Early one-sided yielding for the x500 20 m high-speed envelope."""
    if not math.isfinite(maximum_velocity_m_s) or maximum_velocity_m_s <= 0.0:
        raise ValueError("maximum velocity must be positive and finite")
    relative_speed_m_s = 2.0 * maximum_velocity_m_s
    dynamic_boundary_m = (
        20.0
        + 2.0
        + relative_speed_m_s * (0.65 + 0.15)
        + relative_speed_m_s * relative_speed_m_s / (2.0 * 6.0)
        + 5.0
    )
    return ConflictCoordinatorConfig(
        trigger_distance_m=max(120.0, dynamic_boundary_m + 25.0),
        release_distance_m=80.0,
        encounter_reset_distance_m=100.0,
        predicted_miss_distance_m=20.0,
        prediction_horizon_s=8.0,
        reserve_separation_m=20.0,
        yield_gain_s_inv=0.6,
        release_frames=10,
        release_at_reserve_when_nonclosing=True,
        clear_uses_mission_velocity=True,
        # Keep the authenticated 10 m/s lane change unchanged. Above it, add
        # enough lateral authority to offset the shorter head-on encounter.
        yield_lateral_speed_m_s=(
            3.0
            if maximum_velocity_m_s <= 10.0
            else 0.5 * maximum_velocity_m_s + 2.0
        ),
    )


class ConflictCoordinator:
    """Latch one encounter and alternate right-of-way between encounters.

    Every companion can derive the same priority using only peer-to-peer state:
    the first encounter gives the lexicographically first vehicle priority, the
    next encounter the other. The yielding vehicle alone receives an early,
    one-sided radial constraint; the production CBF remains the final shield.
    """

    def __init__(
        self,
        drone_id: str,
        peer_id: str,
        config: ConflictCoordinatorConfig | None = None,
        *,
        state: _EncounterState | None = None,
        release_observations_per_frame: int = 1,
    ) -> None:
        if not drone_id.strip() or not peer_id.strip() or drone_id == peer_id:
            raise ValueError("conflict coordinator identity is invalid")
        self.drone_id = drone_id
        self.peer_id = peer_id
        self.config = config or ConflictCoordinatorConfig()
        self._state = state or _EncounterState()
        self._lock = threading.Lock()
        self._release_observations_per_frame = release_observations_per_frame

    @classmethod
    def shared(
        cls,
        drone_id: str,
        peer_id: str,
        config: ConflictCoordinatorConfig | None = None,
    ) -> ConflictCoordinator:
        key = tuple(sorted((drone_id, peer_id)))
        with _SHARED_STATES_LOCK:
            state = _SHARED_STATES.setdefault(key, _EncounterState())
        coordinator = cls(
            drone_id,
            peer_id,
            config,
            state=state,
            release_observations_per_frame=2,
        )
        coordinator._lock = _SHARED_STATES_LOCK
        return coordinator

    @classmethod
    def pair(
        cls,
        drone_ids: Sequence[str],
        config: ConflictCoordinatorConfig | None = None,
    ) -> dict[str, ConflictCoordinator]:
        ordered = tuple(sorted(str(drone_id) for drone_id in drone_ids))
        if len(ordered) != 2 or len(set(ordered)) != 2:
            raise ValueError("conflict coordinator pair requires two identities")
        state = _EncounterState()
        lock = threading.RLock()
        result = {}
        for drone_id in ordered:
            peer_id = next(other for other in ordered if other != drone_id)
            coordinator = cls(
                drone_id,
                peer_id,
                config,
                state=state,
                release_observations_per_frame=2,
            )
            coordinator._lock = lock
            result[drone_id] = coordinator
        return result

    @property
    def encounter(self) -> int:
        return self._state.encounter

    @property
    def active(self) -> bool:
        return self._state.active

    @property
    def priority_drone_id(self) -> str | None:
        return self._state.priority_drone_id

    @property
    def release_count(self) -> int:
        return self._state.release_count

    def reset(self) -> None:
        with self._lock:
            self._state.active = False
            self._state.priority_drone_id = None
            self._state.release_count = 0
            self._state.released_until_clear = False
            self._state.yield_side = 1.0

    def filter(
        self,
        candidate_velocity_enu_m_s: Sequence[float],
        mission_velocity_enu_m_s: Sequence[float],
        swarm_state: Mapping[str, Any],
    ) -> tuple[Vector3, dict[str, Any]]:
        candidate = _vector(candidate_velocity_enu_m_s, "candidate")
        mission = _vector(mission_velocity_enu_m_s, "mission")
        own = _state(swarm_state.get(self.drone_id), "own")
        peer = _state(swarm_state.get(self.peer_id), "peer")
        relative = tuple(peer[0][axis] - own[0][axis] for axis in range(3))
        distance = math.sqrt(sum(value * value for value in relative))
        if distance <= 1.0e-6:
            raise ValueError("peer_position_overlap")
        normal = tuple(value / distance for value in relative)
        relative_velocity = tuple(peer[1][axis] - own[1][axis] for axis in range(3))
        closing_speed = -sum(
            normal[axis] * relative_velocity[axis] for axis in range(3)
        )
        predicted_relative_velocity = tuple(
            peer[1][axis] - mission[axis] for axis in range(3)
        )
        speed_squared = sum(value * value for value in predicted_relative_velocity)
        time_to_closest = (
            -sum(
                relative[axis] * predicted_relative_velocity[axis]
                for axis in range(3)
            )
            / speed_squared
            if speed_squared > 1.0e-9
            else math.inf
        )
        miss_distance = (
            math.sqrt(
                sum(
                    (
                        relative[axis]
                        + predicted_relative_velocity[axis] * time_to_closest
                    )
                    ** 2
                    for axis in range(3)
                )
            )
            if time_to_closest >= 0.0
            else distance
        )
        threat = bool(
            distance < self.config.trigger_distance_m
            and 0.0 < time_to_closest < self.config.prediction_horizon_s
            and miss_distance < self.config.predicted_miss_distance_m
        )
        with self._lock:
            if (
                self._state.released_until_clear
                and distance >= self.config.encounter_reset_distance_m
                and closing_speed < 0.0
            ):
                self._state.released_until_clear = False
                self._state.priority_drone_id = None

            if (
                not self._state.active
                and not self._state.released_until_clear
                and self._state.priority_drone_id is not None
                and distance >= self.config.encounter_reset_distance_m
                and closing_speed < 0.0
            ):
                self._state.priority_drone_id = None

            if (
                not self._state.active
                and not self._state.released_until_clear
                and threat
            ):
                if self._state.priority_drone_id is None:
                    self._state.encounter += 1
                    ordered = sorted((self.drone_id, self.peer_id))
                    self._state.priority_drone_id = ordered[
                        (self._state.encounter - 1) % len(ordered)
                    ]
                    self._state.yield_side = 1.0
                self._state.active = True
                self._state.release_count = 0

            released = False
            if self._state.active:
                release_distance = (
                    self.config.reserve_separation_m
                    if self.config.release_at_reserve_when_nonclosing
                    else self.config.release_distance_m
                )
                nonclosing = (
                    closing_speed <= 0.0
                    if self.config.release_at_reserve_when_nonclosing
                    else closing_speed < 0.0
                )
                if distance >= release_distance and nonclosing:
                    self._state.release_count += 1
                else:
                    self._state.release_count = 0
                release_threshold = (
                    self.config.release_frames
                    * self._release_observations_per_frame
                )
                if self._state.release_count >= release_threshold:
                    self._state.active = False
                    self._state.release_count = 0
                    if self.config.release_at_reserve_when_nonclosing:
                        self._state.released_until_clear = True
                    elif distance >= self.config.encounter_reset_distance_m:
                        self._state.priority_drone_id = None
                    released = True

            # Mission tracking owns the clear path.  The RL candidate is an
            # avoidance residual and only gains authority on the yielding
            # vehicle during a confirmed encounter.
            output = (
                mission if self.config.clear_uses_mission_velocity else candidate
            )
            role = "clear"
            intervention = 0.0
            if self._state.active and self._state.priority_drone_id == self.drone_id:
                output = mission
                role = "priority"
            elif self._state.active:
                output = candidate
                role = "yield"
                if self.config.yield_lateral_speed_m_s > 0.0:
                    # The x500 policy was trained against the end of a linear
                    # leg.  Keep its passing side, but bound the maneuver to a
                    # horizontal lane change: reversing along the leg caused a
                    # second head-on encounter, while using altitude for a
                    # horizontal conflict destroyed trajectory tracking.
                    horizontal_normal_norm = math.hypot(normal[0], normal[1])
                    mission_horizontal_speed = math.hypot(mission[0], mission[1])
                    if (
                        horizontal_normal_norm > 1.0e-6
                        and mission_horizontal_speed > 1.0e-6
                    ):
                        tangent = (
                            -normal[1] / horizontal_normal_norm,
                            normal[0] / horizontal_normal_norm,
                        )
                        side = self._state.yield_side
                        lateral_speed = min(
                            self.config.yield_lateral_speed_m_s,
                            mission_horizontal_speed,
                        )
                        forward_speed = math.sqrt(
                            max(
                                0.0,
                                mission_horizontal_speed * mission_horizontal_speed
                                - lateral_speed * lateral_speed,
                            )
                        )
                        mission_direction = (
                            mission[0] / mission_horizontal_speed,
                            mission[1] / mission_horizontal_speed,
                        )
                        output = (
                            mission_direction[0] * forward_speed
                            + side * tangent[0] * lateral_speed,
                            mission_direction[1] * forward_speed
                            + side * tangent[1] * lateral_speed,
                            mission[2],
                        )
                maximum_toward_peer = sum(
                    peer[1][axis] * normal[axis] for axis in range(3)
                ) + self.config.yield_gain_s_inv * (
                    distance - self.config.reserve_separation_m
                )
                toward_peer = sum(output[axis] * normal[axis] for axis in range(3))
                if toward_peer > maximum_toward_peer:
                    correction = toward_peer - maximum_toward_peer
                    output = tuple(
                        output[axis] - correction * normal[axis]
                        for axis in range(3)
                    )
                    intervention = correction
            active = self._state.active
            priority_drone_id = self._state.priority_drone_id
            encounter = self._state.encounter

        return output, {
            "active": active,
            "role": role,
            "priority_drone_id": priority_drone_id,
            "encounter": encounter,
            "distance_m": round(distance, 3),
            "closing_speed_m_s": round(closing_speed, 3),
            "time_to_closest_s": (
                round(time_to_closest, 3) if math.isfinite(time_to_closest) else None
            ),
            "predicted_miss_distance_m": round(miss_distance, 3),
            "intervention_norm_m_s": round(intervention, 4),
            "released": released,
        }


def _state(value: Any, label: str) -> tuple[Vector3, Vector3]:
    if not isinstance(value, Mapping) or not value.get("valid", False):
        raise ValueError(f"{label}_state_invalid")
    return (
        _vector(value.get("position_enu_m"), f"{label}_position"),
        _vector(value.get("velocity_enu_m_s"), f"{label}_velocity"),
    )


def _vector(value: Any, label: str) -> Vector3:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{label}_invalid")
    try:
        result = tuple(float(component) for component in value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label}_invalid") from error
    if not all(math.isfinite(component) for component in result):
        raise ValueError(f"{label}_invalid")
    return result  # type: ignore[return-value]
