"""Readiness contract for TWO_UAV_ACTIVE_SITL_FLIGHT: UAV-01 and UAV-02 both
armed and in OFFBOARD in SITL, at the same time.

Succeeds `one_uav_active_readiness.py` rather than editing it. That module's
title, its own tests, and its own dry-run artifact (`one_uav_active_dry_run.py`)
are the ONE_UAV_ACTIVE_SITL_FLIGHT milestone's reviewed record -- "UAV-02 must
be impossible to actuate in this gate, not merely disabled by default" is a
deliberate, load-bearing statement there, and stays true of that gate
specifically. Everything vehicle-agnostic in it -- `ReadinessState`,
`FlightEnvelope`, `WarmupContract`, `ABORT_MATRIX`, `transition_guard`, PX4's
own OFFBOARD preconditions -- is imported here unchanged: none of it is
specific to how many vehicles are flying, so duplicating it would only create
a second copy to keep in sync. Only the authorization gate differs, and this
module supplies the one this milestone uses.

Scope, stated plainly: this widens WHO may be authorized (UAV-01 and UAV-02
both), not HOW MANY may fly unsupervised at once, and not any control logic.
CBF collision avoidance, the formation controller, and EmergencySupervisor
are unchanged and already vehicle-count-agnostic -- they were built for a
swarm from the start. What was single-vehicle was only ever the flight-gate
identity check, because that is what a first-flight safety margin should be.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from typing import Any

from offboard_authority import ACTIVE_FLIGHT_AUTHORIZED_VEHICLES
from one_uav_active_readiness import (
    ABORT_MATRIX,
    PX4_FACTS,
    STATE_SEQUENCE,
    AbortAction,
    AbortRule,
    FlightEnvelope,
    ReadinessState,
    VelocityReadiness,
    WarmupContract,
    abort_rule_for,
    evaluate_velocity_readiness,
    transition_guard,
)

__all__ = [
    "ABORT_MATRIX",
    "PX4_FACTS",
    "STATE_SEQUENCE",
    "AbortAction",
    "AbortRule",
    "FlightEnvelope",
    "ReadinessState",
    "VelocityReadiness",
    "WarmupContract",
    "abort_rule_for",
    "evaluate_velocity_readiness",
    "transition_guard",
    "is_authorized_for_two_uav_active_flight",
]


def is_authorized_for_two_uav_active_flight(
    drone_id: str, px4_system_id: int, explicit_opt_in: bool
) -> tuple[bool, str]:
    """Authorization gate for TWO_UAV_ACTIVE_SITL_FLIGHT.

    Same structure and same three required conditions as
    `one_uav_active_readiness.is_authorized_for_first_active_flight`, widened
    from equality against one hardcoded pair to membership in
    `offboard_authority.ACTIVE_FLIGHT_AUTHORIZED_VEHICLES` -- imported, not
    copied, so this module and offboard_authority.py cannot drift apart on
    which vehicles are eligible (test_two_uav_active_readiness pins this).
    """
    if (drone_id, int(px4_system_id)) not in ACTIVE_FLIGHT_AUTHORIZED_VEHICLES:
        return False, f"vehicle_not_authorized:{drone_id}:{px4_system_id}"
    if int(px4_system_id) == 0:
        # Unreachable given the check above -- no authorized pair uses system
        # id 0 -- kept as an explicit guard because target_system 0 is a
        # MAVLink broadcast accepted by every PX4 instance.
        return False, "broadcast_system_id_forbidden"
    if not explicit_opt_in:
        return False, "explicit_opt_in_absent"
    return True, ""


def extrema_minimum_margin_m(samples: Iterable[Mapping[str, Any]]) -> float | None:
    """The true minimum CBF margin over a hold, not the sampled one.

    Each sample carries `cbf_extrema_minimum_margin_m`, the companion's own
    running minimum accumulated at the rate the barrier runs. The smallest of
    those is therefore the smallest the barrier ever saw, including the nine
    frames in ten a 1.9 Hz poller never asked about. Reporting
    `min(cbf_minimum_margin_m)` instead reports the smallest *sample*, which
    on 2026-08-18 was +2.740 m against a true -1.332 m.
    """
    values = [
        sample["cbf_extrema_minimum_margin_m"]
        for sample in samples
        if sample.get("cbf_extrema_minimum_margin_m") is not None
    ]
    return round(min(values), 3) if values else None
