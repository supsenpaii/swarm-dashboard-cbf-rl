"""Deterministic two-UAV right-of-way layer ahead of the final CBF shield."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
import threading
from typing import Any, Mapping, Sequence


Vector3 = tuple[float, float, float]


def _float_env(name: str, default: float) -> float:
    """An override knob for measuring this value, not for tuning it in flight.

    Every certified rung is pinned to the default; a profile that sets this is
    running an experiment, and its results are not a certification.
    """
    raw = os.environ.get(name)
    if raw is None:
        return default
    value = float(raw)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{name} must be finite and nonnegative")
    return value


@dataclass
class _EncounterState:
    encounter: int = 0
    active: bool = False
    priority_drone_id: str | None = None
    release_count: int = 0
    released_until_clear: bool = False
    yield_side: float = 1.0
    # The along-path direction the lane change is built on, frozen for the
    # duration of one yield.  See the latch site in `filter` for why.
    yield_heading: tuple[float, float] | None = None
    # Whether this yield is a vertical one, decided on its first frame and
    # held.  Also see `filter`: a vertical yield creates the very horizontal
    # mission component that would otherwise switch it to the horizontal
    # branch, one frame after it starts working.
    yield_vertical: bool | None = None
    # Consecutive non-threatening frames since a release.  See `filter`.
    clear_frames: int = 0


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
    # Require the predicted encounter to have cleared before releasing, not
    # merely the instantaneous radial rate to have reached zero. Off by default
    # because the x500 rungs are signed off against the older semantics; see
    # the release block in `filter` for what it fixes and why Sparrow needs it.
    release_needs_threat_cleared: bool = False
    clear_uses_mission_velocity: bool = False
    yield_lateral_speed_m_s: float = 0.0
    # A fixed prediction horizon is a distance that shrinks with closing
    # speed: 8 s of lead time is 160 m at 20 m/s closing but only 16 m at
    # 2 m/s, which is already inside the margin the CBF will demand. Below
    # this distance an encounter is latched on geometry alone, so the
    # coordinator can never arrive after the shield it is meant to precede.
    minimum_engagement_distance_m: float = 0.0

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
            or not math.isfinite(self.minimum_engagement_distance_m)
            or self.minimum_engagement_distance_m < 0.0
            or self.minimum_engagement_distance_m > self.trigger_distance_m
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
        minimum_engagement_distance_m=dynamic_boundary_m,
    )


def sparrow_20m_conflict_config(
    maximum_velocity_m_s: float = 10.0,
) -> ConflictCoordinatorConfig:
    """Early one-sided yielding for the Sparrow 20 m envelope."""
    if not math.isfinite(maximum_velocity_m_s) or maximum_velocity_m_s <= 0.0:
        raise ValueError("maximum velocity must be positive and finite")
    relative_speed_m_s = 2.0 * maximum_velocity_m_s
    dynamic_boundary_m = (
        20.0
        + 2.0
        # 0.86 s is Sparrow's measured command response, not the 0.65 s the
        # x500 rungs were signed off against; the CBF gate uses the same
        # number, and this boundary is now the coordinator's own engagement
        # floor, so the two must agree or the shield acts first.
        + relative_speed_m_s * (0.86 + 0.10)
        + relative_speed_m_s * relative_speed_m_s / (2.0 * 8.0)
    )
    # The coordinator has to finish a lane change before the barrier runs out
    # of room, so what it needs is LEAD TIME, and a fixed 25 m of extra
    # distance is not that: `dynamic_boundary_m` grows with the square of
    # speed, so the same 25 m buys less and less time as the rung rises.
    # Measured 2026-08-18 against the 20 Hz flight logs:
    #
    #   rung   trigger   boundary   slack    lead     TRUE margin
    #   10     120.0 m     66.2 m   53.8 m   2.69 s     +4.965
    #   15     132.1 m    107.0 m   25.0 m   0.83 s     +0.502
    #   20     185.4 m    160.4 m   25.0 m   0.62 s     -0.596
    #
    # Trigger now 136 / 212 / 300 m at 3.5 s of lead.
    #
    # The margin tracks the lead time, not the distance. Rung 10 is the only
    # one with real lead, and it has that by accident -- the 120 m floor
    # happened to be generous there. Nothing chose 2.69 s.
    #
    # So choose it. 2.7 s was the first choice, because it is what rung 10
    # already flew on; swept 2026-08-18 it turned out to be the low end of a
    # monotone gain with no liveness cost at any rung (corridor replay):
    #
    #   lead    10 m/s          15 m/s          20 m/s
    #   2.7 s   5.238 / 2.62%   8.938 / 2.75%  12.483 / 2.80%
    #   3.0 s   5.622 / 2.52%   9.435 / 2.70%  12.935 / 2.77%
    #   3.5 s   6.192 / 2.39%  10.135 / 2.61%  13.796 / 2.68%
    #   3.9 s   8.798 / 2.55%  10.823 / 2.52%  14.020 / 2.64%
    #
    # 3.5 s, not 3.9: at 3.9 the trigger sits 7.91 s out at 20 m/s against an
    # 8 s horizon, and a design pressed against its own limit has nowhere to
    # go when the next thing moves. 3.5 keeps half a second of it.
    #
    # Raising `yield_lateral_speed_m_s` was swept alongside and changes NOTHING
    # -- 9.5, 12 and 15 m/s give the same margin to three decimals. The yield
    # stopped saturating on lateral authority when it got the time to use it,
    # so that knob is spent and this one is not.
    #
    # x500 keeps the fixed 25 m: its rungs are signed off against that number.
    engagement_lead_s = _float_env("SWARM_CONFLICT_ENGAGEMENT_LEAD_S", 3.5)
    trigger_distance_m = max(
        120.0, dynamic_boundary_m + relative_speed_m_s * engagement_lead_s
    )
    prediction_horizon_s = 8.0
    # The threat test needs BOTH `distance < trigger_distance_m` AND
    # `time_to_closest < prediction_horizon_s`. Push the trigger past what the
    # horizon can see and the second condition quietly becomes the real
    # trigger: the coordinator engages later than configured and reports
    # nothing. At 2.7 s of lead the trigger sits 6.0 to 6.7 s out, so the
    # horizon still covers it -- but only just, and this refusal is here so
    # that raising the lead fails loudly instead of silently doing nothing.
    if trigger_distance_m > relative_speed_m_s * prediction_horizon_s:
        raise ValueError(
            "engagement lead exceeds the prediction horizon: trigger "
            f"{trigger_distance_m:.1f} m is "
            f"{trigger_distance_m / relative_speed_m_s:.2f} s out against a "
            f"{prediction_horizon_s:.1f} s horizon"
        )
    return ConflictCoordinatorConfig(
        trigger_distance_m=trigger_distance_m,
        release_distance_m=80.0,
        encounter_reset_distance_m=100.0,
        predicted_miss_distance_m=20.0,
        prediction_horizon_s=prediction_horizon_s,
        reserve_separation_m=20.0,
        yield_gain_s_inv=0.6,
        release_frames=10,
        release_at_reserve_when_nonclosing=True,
        release_needs_threat_cleared=True,
        clear_uses_mission_velocity=True,
        # Continuous, unlike the x500 line this was copied from, which holds
        # 3.0 at and below 10 m/s to preserve an authenticated x500 rung.
        # Sparrow inherited that step without inheriting the reason, and it
        # landed exactly on the 10 m/s rung: 3.0 m/s of lateral authority
        # against 9.5 at the next rung up. The 2026-08-17 ladder shows what
        # that cost -- minimum CBF margin 0.013 m at 10 m/s, 2.28 at 15 and
        # 3.12 at 20, so the SLOWEST rung was the fragile one. Thirteen
        # millimetres is the barrier doing the coordinator's job at its own
        # boundary, which is precisely the situation the coordinator exists
        # to prevent. Extending the same line down gives 7.0 m/s here.
        yield_lateral_speed_m_s=0.5 * maximum_velocity_m_s + 2.0,
        minimum_engagement_distance_m=dynamic_boundary_m,
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
            self._state.yield_heading = None
            self._state.yield_vertical = None
            self._state.clear_frames = 0

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
            and time_to_closest > 0.0
            and (
                time_to_closest < self.config.prediction_horizon_s
                or distance < self.config.minimum_engagement_distance_m
            )
            and miss_distance < self.config.predicted_miss_distance_m
        )
        with self._lock:
            if self._state.released_until_clear:
                self._state.clear_frames = (
                    0 if threat else self._state.clear_frames + 1
                )
            if self._state.released_until_clear and (
                (
                    distance >= self.config.encounter_reset_distance_m
                    and closing_speed < 0.0
                )
                # Or simply: the pair stopped being a threat. This suppression
                # exists to stop a released encounter re-latching immediately,
                # on geometry that has not changed yet. Requiring
                # encounter_reset_distance_m WHILE separating is a condition
                # two vehicles orbiting a shared waypoint never meet, so
                # coordination stayed off for the rest of the flight after the
                # first pass. Measured on diagonal_cross: latched at 90.1 m,
                # yielded 9.7 s, released at 25.6 m, then held role "clear"
                # through a convergence with a 3.4 m predicted miss distance
                # against a 20 m threshold, to a CBF margin of -0.840 m.
                or self._state.clear_frames >= self.config.release_frames
            ):
                self._state.released_until_clear = False
                self._state.priority_drone_id = None
                self._state.clear_frames = 0

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
                # Releasing needs the encounter to be OVER, not merely
                # momentarily non-closing. `closing_speed` is the instantaneous
                # radial rate, and a lane change drives it through zero while
                # the pair is still converging -- which is every frame of a
                # geometry where both vehicles are routed to the same point.
                # `threat` is the predicted version of the same question and is
                # already computed above, so requiring it to have cleared costs
                # nothing and closes the hole: release used to fire mid-approach
                # and then `released_until_clear` could never lift, because
                # lifting it needs frames that are NOT a threat and the pair
                # went on converging. The coordinator switched itself off for
                # the rest of the encounter.
                #
                # Measured on diagonal_cross, where both missions share a
                # waypoint: latched at 91.2 m, yielded 9.5 s, released at
                # 25.3 m while still closing, held role "clear" through the
                # convergence, and the barrier was left alone to take it to
                # -0.091 m. With this, the yield holds until the priority
                # vehicle is predicted to miss, which is the sequencing a
                # shared waypoint requires and cannot get any other way.
                threat_cleared = (
                    not threat
                    if self.config.release_needs_threat_cleared
                    else True
                )
                if distance >= release_distance and nonclosing and threat_cleared:
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
                    self._state.yield_heading = None
                    self._state.yield_vertical = None
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
                    side = self._state.yield_side
                    # Latched, not re-decided per frame: a vertical yield
                    # steps sideways, and one frame later that displacement
                    # gives `mission` a horizontal component pointing back at
                    # the axis. Re-deciding would hand the horizontal branch
                    # that pull-back as its along-path heading and fly the
                    # vehicle straight back into the standoff.
                    if self._state.yield_vertical is None:
                        self._state.yield_vertical = (
                            mission_horizontal_speed <= 1.0e-6
                        )
                    if (
                        not self._state.yield_vertical
                        and mission_horizontal_speed > 1.0e-6
                    ):
                        if horizontal_normal_norm > 1.0e-6:
                            tangent = (
                                -normal[1] / horizontal_normal_norm,
                                normal[0] / horizontal_normal_norm,
                            )
                        else:
                            # A vertically stacked pair has no horizontal
                            # normal, and then every horizontal direction is
                            # perpendicular to it. Step across the mission.
                            tangent = (
                                -mission[1] / mission_horizontal_speed,
                                mission[0] / mission_horizontal_speed,
                            )
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
                        # Frozen: `mission` is the tracker's corrected
                        # command, so once the lane change moves the vehicle
                        # off its line it carries a pull-back term -- and it
                        # re-enters here weighted by forward_speed, nearly the
                        # whole cruise.  Live, that cancels the lateral push
                        # and the excursion stalls at yield_lateral_speed_m_s
                        # / position_gain_s_inv: 5 m in flight against ~52 m
                        # of barrier.  ponytail: stale if one encounter spans
                        # a corner; re-latch per leg if a rung yields at a
                        # vertex.
                        if self._state.yield_heading is None:
                            self._state.yield_heading = mission_direction
                        mission_direction = self._state.yield_heading
                        output = (
                            mission_direction[0] * forward_speed
                            + side * tangent[0] * lateral_speed,
                            mission_direction[1] * forward_speed
                            + side * tangent[1] * lateral_speed,
                            mission[2],
                        )
                    elif self._state.yield_vertical and abs(mission[2]) > 1.0e-6:
                        # A purely vertical mission: the block above needs a
                        # horizontal mission to preserve and has none, so it
                        # used to leave the yield with no maneuver at all --
                        # only the maximum_toward_peer clamp slowing the
                        # approach. Nothing then broke the symmetry of a
                        # vertical head-on, and the pair held separation
                        # forever without ever passing: the two liveness
                        # failures in the 20 m/s matrix, still safe at 22.3 m
                        # and 0.156 m of margin after 17,218 steps.
                        #
                        # Stepping sideways is free here for the same reason
                        # the horizontal lane change spends forward speed:
                        # the mission has no horizontal component to give up.
                        # Never spend the WHOLE speed budget sideways. At and
                        # below yield_lateral_speed_m_s the old `min` returned
                        # the mission speed itself, leaving `remaining` at zero:
                        # the yielding vehicle stepped across forever and never
                        # advanced along its mission again. That was invisible
                        # while release fired on the instantaneous radial rate,
                        # because the yield ended before the stall could be
                        # seen; gating release on the predicted encounter
                        # exposed it as seven vertical head-on cases at 1-4 m/s
                        # running 15,520 steps without arriving -- safe the
                        # whole time at 22.06 m, and never finishing.
                        #
                        # Capping the lane change at 70% of the budget keeps
                        # ~71% of it as forward speed by Pythagoras, so the step
                        # across still separates the pair and the vehicle still
                        # closes on its goal. Above the cap the min is unchanged
                        # and every faster rung sees the maneuver it was
                        # certified with.
                        vertical_speed = abs(mission[2])
                        lateral_speed = min(
                            self.config.yield_lateral_speed_m_s,
                            0.7 * vertical_speed,
                        )
                        remaining = math.sqrt(
                            max(0.0, vertical_speed * vertical_speed
                                - lateral_speed * lateral_speed)
                        )
                        step = (
                            (-normal[1] / horizontal_normal_norm,
                             normal[0] / horizontal_normal_norm)
                            if horizontal_normal_norm > 1.0e-6
                            else (1.0, 0.0)
                        )
                        output = (
                            side * step[0] * lateral_speed,
                            side * step[1] * lateral_speed,
                            math.copysign(remaining, mission[2]),
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
