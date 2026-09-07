#!/usr/bin/env python3
"""DRY RUN of the first one-UAV active flight sequence. Sends nothing.

Walks the ONE_UAV_ACTIVE_SITL state machine and every abort rule, emitting
WOULD_* records instead of commands. There is no MAVLink, MQTT, or socket
import anywhere in this module or in `one_uav_active_readiness`, so the
command counters it reports are zero by construction rather than by
discipline -- the same structural argument used for the shadow sender.

Reads live companion telemetry (GET /api/drones only) when available, so the
precheck evidence is real rather than fabricated.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

from active_offboard_setpoint_sender import (
    CALLER_REPORTED_CONDITIONS,
    ActiveOffboardSetpointSender,
    TransmitDecision,
)
from cbf_command_gate import CbfCommand
from companion_safety import CompanionSafetyStatus
from emergency_supervisor import EmergencyDecision, EmergencyStage
from offboard_authority import (
    ACTIVE_FLIGHT_AUTHORIZED_VEHICLES,
    companion_active_transmit_authorized,
    resolve_authority,
)
from offboard_setpoint_sender import ShadowOffboardSetpointSender
from one_uav_active_readiness import (
    ABORT_MATRIX,
    FIRST_ACTIVE_FLIGHT_DRONE_ID,
    FIRST_ACTIVE_FLIGHT_SYSTEM_ID,
    PX4_FACTS,
    STATE_SEQUENCE,
    AbortAction,
    ActiveOffboardSetpointSenderPlan,
    FlightEnvelope,
    ReadinessState,
    WarmupContract,
    VelocityReadiness,
    abort_rule_for,
    evaluate_velocity_readiness,
    is_authorized_for_first_active_flight,
    transition_guard,
)
from two_uav_active_readiness import is_authorized_for_two_uav_active_flight

API_URL = "http://127.0.0.1:8000/api/drones"

# Actions this dry run would take, per state. Nothing here is executed.
WOULD_ACTIONS: dict[ReadinessState, str] = {
    ReadinessState.PRECHECK: "WOULD_VERIFY_PRECHECKS",
    ReadinessState.SETPOINT_WARMUP: "WOULD_START_SETPOINT_STREAM",
    ReadinessState.ARM: "WOULD_ARM_UAV01",
    ReadinessState.OFFBOARD: "WOULD_REQUEST_OFFBOARD",
    ReadinessState.CONTROLLED_ASCENT: "WOULD_ASCEND",
    ReadinessState.HOVER: "WOULD_HOVER",
    ReadinessState.ZERO_VELOCITY: "WOULD_ZERO_VELOCITY",
    ReadinessState.EXIT_OFFBOARD: "WOULD_EXIT_OFFBOARD",
    ReadinessState.LAND_DISARM: "WOULD_LAND_DISARM",
}


class CommandCounters:
    """Counts real commands. Must stay all-zero: nothing in this module can
    increment them, because nothing here can transmit."""

    def __init__(self) -> None:
        self.arm = 0
        self.offboard = 0
        self.setpoint_transmit = 0
        self.takeoff = 0
        self.land = 0

    def as_dict(self) -> dict[str, int]:
        return {
            "ARM": self.arm,
            "OFFBOARD": self.offboard,
            "SETPOINT_TRANSMIT": self.setpoint_transmit,
            "TAKEOFF": self.takeoff,
            "LAND": self.land,
        }

    def all_zero(self) -> bool:
        return not any(self.as_dict().values())


def live_api() -> dict[str, Any] | None:
    result = subprocess.run(
        ["curl", "-fsS", "-m", "5", API_URL], capture_output=True, text=True, check=False
    )
    if result.returncode != 0:
        return None
    try:
        return json.loads(result.stdout)
    except json.JSONDecodeError:
        return None


def precheck_evidence(payload: dict[str, Any] | None) -> dict[str, Any]:
    """Build PRECHECK evidence from live telemetry where available."""
    if payload is None:
        return {"source": "unavailable"}
    drone = payload["drones"][FIRST_ACTIVE_FLIGHT_DRONE_ID]
    stream = payload["tracking_pose_streams"][FIRST_ACTIVE_FLIGHT_DRONE_ID]
    safety = stream.get("companion_safety") or {}
    flags = drone.get("failsafe_flags") or {}
    # Age the failsafe sample against the same monotonic clock the telemetry
    # node stamped it with. Both run on this host, so the comparison is valid.
    velocity = evaluate_velocity_readiness(
        flags, time.monotonic(), maximum_age_s=3.0
    )
    return {
        "source": "live",
        "preflight_checks_pass": bool(drone["status"].get("preflight_checks_pass")),
        "local_position_invalid_false": flags.get("local_position_invalid") is False,
        "companion_safety_running": bool(safety),
        "peer_state_fresh": bool(safety.get("peer_ids_used")),
        "armed": drone["status"].get("armed"),
        "nav_state": drone["status"].get("nav_state"),
        "offboard_control_signal_lost": flags.get("offboard_control_signal_lost"),
        "local_velocity_invalid": flags.get("local_velocity_invalid", "ABSENT"),
        "velocity_readiness": velocity,
        "velocity_readiness_detail": velocity.as_dict(),
        "offboard_authority": (safety.get("offboard_authority") or resolve_authority().as_dict()),
    }


def nominal_evidence(state: ReadinessState, envelope: FlightEnvelope, warmup: WarmupContract) -> dict[str, Any]:
    """Evidence a healthy run WOULD present at each state. Used to prove the
    guards accept a good path, never to claim the path was flown."""
    return {
        ReadinessState.SETPOINT_WARMUP: {
            "warmup_duration_s": warmup.minimum_duration_s,
            "valid_sample_count": warmup.minimum_valid_samples,
            "max_gap_s": 0.051,
            "all_previews_valid": True,
        },
        ReadinessState.ARM: {"armed": True, "stream_still_healthy": True},
        ReadinessState.OFFBOARD: {"nav_state": PX4_FACTS["nav_state_offboard"]["value"]},
        ReadinessState.CONTROLLED_ASCENT: {
            "altitude_m": envelope.hover_altitude_m,
            "vertical_velocity_m_s": envelope.maximum_vertical_velocity_m_s,
        },
        ReadinessState.HOVER: {"hover_duration_s": 5.0},
        ReadinessState.ZERO_VELOCITY: {"speed_m_s": 0.0},
        ReadinessState.EXIT_OFFBOARD: {"nav_state": PX4_FACTS["nav_state_posctl"]["value"]},
        ReadinessState.LAND_DISARM: {"armed": False},
    }.get(state, {})


def run_sequence(envelope: FlightEnvelope, warmup: WarmupContract, counters: CommandCounters) -> dict[str, Any]:
    payload = live_api()
    live = precheck_evidence(payload)
    records: list[dict[str, Any]] = []
    blocked_at: str | None = None

    authorized, auth_reason = is_authorized_for_first_active_flight(
        FIRST_ACTIVE_FLIGHT_DRONE_ID, FIRST_ACTIVE_FLIGHT_SYSTEM_ID, explicit_opt_in=True
    )
    records.append(
        {
            "step": "AUTHORIZATION",
            "would": "WOULD_AUTHORIZE_UAV01",
            "authorized": authorized,
            "reason": auth_reason,
        }
    )

    for state in STATE_SEQUENCE:
        evidence = dict(live) if state is ReadinessState.PRECHECK else {}
        evidence.update(nominal_evidence(state, envelope, warmup))
        may_advance, reason = transition_guard(state, evidence, envelope, warmup)
        records.append(
            {
                "step": state.value,
                "would": WOULD_ACTIONS[state],
                "guard_passed": may_advance,
                "reason": reason,
                "evidence_keys": sorted(evidence),
            }
        )
        if not may_advance and blocked_at is None:
            blocked_at = f"{state.value}:{reason}"

    return {
        "records": records,
        "blocked_at": blocked_at,
        "live_precheck": {
            k: (v.as_dict() if isinstance(v, VelocityReadiness) else v)
            for k, v in live.items()
        },
        "command_counters": counters.as_dict(),
    }


def _preview(
    velocity,
    *,
    stage=EmergencyStage.NORMAL,
    mode=None,
    now=100.0,
    drone_id=FIRST_ACTIVE_FLIGHT_DRONE_ID,
    px4_system_id=FIRST_ACTIVE_FLIGHT_SYSTEM_ID,
):
    """A real OffboardSetpointPreview, produced by the real shadow sender.

    Built through the actual pipeline objects rather than fabricated, so the
    active sender is exercised against the contract it will really receive.
    `drone_id`/`px4_system_id` default to the ONE_UAV vehicle but must be
    overridden to match whichever sender consumes this preview -- a mismatch
    is refused as `preview_drone_mismatch` regardless of authorization, which
    would silently test the wrong gate.
    """
    shadow = ShadowOffboardSetpointSender(
        drone_id=drone_id,
        px4_system_id=px4_system_id,
        maximum_velocity_m_s=2.0,
        maximum_command_age_s=0.5,
    )
    status = CompanionSafetyStatus(
        drone_id=drone_id,
        nominal_velocity_enu_m_s=velocity,
        nominal_active=True,
        nominal_reason="dry_run",
        nominal_position_error_m=None,
        command=CbfCommand(
            drone_id, velocity, True, "cbf_filtered", 1.0, 0.0
        ),
        emergency=EmergencyDecision(
            stage=stage,
            velocity_enu_m_s=velocity,
            active=stage is not EmergencyStage.NORMAL,
            reason="dry_run",
            recommended_px4_mode=mode,
            time_in_stage_s=0.0,
        ),
        output_velocity_enu_m_s=velocity,
        output_valid=True,
        peer_ids_used=("UAV-02",),
        peer_ids_missing=(),
    )
    # armed/OFFBOARD are asserted as *inputs* here so the preview is valid and
    # the active sender's own gates are what decide the outcome. Nothing is
    # armed: this is a hypothetical vehicle state, and the sender below has no
    # transmit sink, so no command can result either way.
    return shadow.preview(
        status,
        now_monotonic_s=now,
        command_monotonic_s=now,
        px4_main_mode=6,
        px4_armed=True,
        px4_offboard_main_mode=6,
    )


def repository_environment() -> dict[str, str]:
    """The deployed configuration, read from .env rather than this process's
    environment -- the dry run is not launched by the stack's own runner, so
    os.environ would report "not configured" and understate what is deployed.
    """
    environment: dict[str, str] = {}
    path = Path(".env")
    if not path.exists():
        return environment
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        environment[key.strip()] = value.strip()
    return environment


def run_active_sender_dry_run(counters: CommandCounters) -> dict[str, Any]:
    """Exercise the real ActiveOffboardSetpointSender with NO transmit sink.

    A sender constructed without a sink cannot put anything on the wire, so
    this walks the full decision logic -- including the paths that would
    transmit -- while transmission remains impossible by construction rather
    than by configuration.
    """
    cases: list[dict[str, Any]] = []
    deployed = repository_environment()
    # Deliberately permissive authority, used only where the *identity* gate
    # is the thing under test. Refusing UAV-02 because the ambient authority
    # happened to be off would prove nothing about isolation.
    permissive = {"SWARM_OFFBOARD_AUTHORITY": "companion_safety"}

    # 1. Resting configuration: the deployed .env, no opt-in.
    resting = ActiveOffboardSetpointSender(
        FIRST_ACTIVE_FLIGHT_DRONE_ID,
        FIRST_ACTIVE_FLIGHT_SYSTEM_ID,
        environment=deployed,
    )
    now = 100.0
    for _ in range(40):  # 2 s at the companion's 20 Hz cadence
        frame = resting.step(_preview((0.6, 1.2, 0.3), now=now), now_monotonic_s=now)
        now += 0.05
    cases.append(
        {
            "case": "resting_repository_configuration",
            "authority": deployed.get("SWARM_OFFBOARD_AUTHORITY", "ABSENT"),
            "frames": len(resting.frames),
            "decision": frame.decision.value,
            "reason": frame.reason,
            "transmit_count": resting.transmit_count,
        }
    )

    # 2. Authority permitted but no opt-in: the remaining gate must hold.
    no_opt_in = ActiveOffboardSetpointSender(
        FIRST_ACTIVE_FLIGHT_DRONE_ID,
        FIRST_ACTIVE_FLIGHT_SYSTEM_ID,
        explicit_opt_in=False,
        environment=dict(permissive),
    )
    frame = no_opt_in.step(_preview((0.6, 1.2, 0.3)), now_monotonic_s=100.0)
    cases.append(
        {
            "case": "companion_authority_without_opt_in",
            "decision": frame.decision.value,
            "reason": frame.reason,
            "transmit_count": no_opt_in.transmit_count,
        }
    )

    # 3. Every gate satisfied. The decision reaches TRANSMIT and still nothing
    # goes out, because no sink exists -- the structural argument, shown.
    cleared = ActiveOffboardSetpointSender(
        FIRST_ACTIVE_FLIGHT_DRONE_ID,
        FIRST_ACTIVE_FLIGHT_SYSTEM_ID,
        explicit_opt_in=True,
        environment=dict(permissive),
    )
    frame = cleared.step(_preview((0.6, 1.2, 0.3)), now_monotonic_s=100.0)
    cases.append(
        {
            "case": "all_gates_satisfied_but_no_sink",
            "decision": frame.decision.value,
            "reason": frame.reason,
            "transmitted": frame.transmitted,
            "would_send_velocity_ned_m_s": list(frame.velocity_ned_m_s),
            "transmit_count": cleared.transmit_count,
        }
    )

    # 5. Every caller-reported abort condition selects its matrix action.
    # Run with every gate satisfied so the abort is demonstrably what stops
    # the setpoint, not an authorization that would have stopped it anyway.
    for condition in CALLER_REPORTED_CONDITIONS:
        send = ActiveOffboardSetpointSender(
            FIRST_ACTIVE_FLIGHT_DRONE_ID,
            FIRST_ACTIVE_FLIGHT_SYSTEM_ID,
            explicit_opt_in=True,
            environment=dict(permissive),
        )
        frame = send.step(
            _preview((0.6, 1.2, 0.3)),
            now_monotonic_s=100.0,
            reported_conditions=[condition],
        )
        rule = abort_rule_for(condition)
        cases.append(
            {
                "case": f"abort:{condition}",
                "decision": frame.decision.value,
                "abort_action": frame.abort_action,
                "expected_action": rule.immediate_action.value if rule else None,
                "matches_matrix": bool(
                    rule and frame.abort_action == rule.immediate_action.value
                ),
                "latched": frame.latched,
                "expected_latched": bool(rule and not rule.recovery_allowed),
                "transmit_count": send.transmit_count,
            }
        )

    # 6. Emergency stages derived from the preview alone.
    for stage, mode, label in (
        (EmergencyStage.HOLD, None, "emergency_supervisor_hold"),
        (
            EmergencyStage.RECOMMEND_POSITION_HOLD,
            "POSITION_HOLD",
            "emergency_recommends_position_hold",
        ),
        (
            EmergencyStage.RECOMMEND_RTL_OR_LAND,
            "RTL_OR_LAND",
            "emergency_recommends_rtl_or_land",
        ),
    ):
        send = ActiveOffboardSetpointSender(
            FIRST_ACTIVE_FLIGHT_DRONE_ID,
            FIRST_ACTIVE_FLIGHT_SYSTEM_ID,
            explicit_opt_in=True,
            environment=dict(permissive),
        )
        frame = send.step(
            _preview((0.0, 0.0, 0.0), stage=stage, mode=mode), now_monotonic_s=100.0
        )
        cases.append(
            {
                "case": f"self_derived:{label}",
                "detected": frame.abort_condition,
                "decision": frame.decision.value,
                "matches_matrix": frame.abort_condition == label,
                "transmit_count": send.transmit_count,
            }
        )

    # 4. UAV-02 must be authorized at its own system id (2) -- added for
    # TWO_UAV_ACTIVE_SITL_FLIGHT, see two_uav_active_readiness -- and refused
    # at every other system id it could present, under a permissive
    # authority and with the opt-in granted. ActiveOffboardSetpointSender
    # now consults two_uav_active_readiness.is_authorized_for_two_uav_active_flight
    # rather than one_uav_active_readiness's narrower gate (see
    # active_offboard_setpoint_sender._authorization), so system id 2 clears
    # both gates end to end.
    for system_id in (0, 1, 2, 99):
        send = ActiveOffboardSetpointSender(
            "UAV-02",
            system_id,
            explicit_opt_in=True,
            environment=dict(permissive),
        )
        # Preview built to match this exact (drone_id, system_id), so a
        # mismatch there cannot masquerade as the authorization refusal this
        # case exists to check. system id 0 is the one exception:
        # ShadowOffboardSetpointSender itself refuses to construct with
        # system id 0 (out of MAVLink's valid 1-255 range), so no matching
        # preview can exist -- irrelevant here, since
        # companion_active_transmit_authorized already refuses broadcast
        # system id 0 before _authorization() ever reaches the preview check.
        frame = send.step(
            _preview((0.6, 1.2, 0.3), drone_id="UAV-02", px4_system_id=system_id)
            if system_id != 0
            else _preview((0.6, 1.2, 0.3)),
            now_monotonic_s=100.0,
        )
        cases.append(
            {
                "case": f"uav02_system_id_{system_id}",
                "decision": frame.decision.value,
                "reason": frame.reason,
                "refused": frame.decision is TransmitDecision.WITHHOLD,
                "expected_refused": system_id != 2,
                "transmit_count": send.transmit_count,
            }
        )

    # Which vehicles the real bridge builder would grant a transmit path to,
    # under the deployed .env rather than a synthetic environment.
    vehicle_pairs = (
        (FIRST_ACTIVE_FLIGHT_DRONE_ID, FIRST_ACTIVE_FLIGHT_SYSTEM_ID),
        ("UAV-02", 2),
    )
    # Mirrors build_active_offboard_setpoint_sender's own two-gate logic
    # rather than importing it: that function also spins up MQTT/threading
    # module state at import, which this read-only dry run should not carry.
    # See that function's docstring for why both gates -- not just the
    # authority one -- must clear before a sink is granted.
    sink_grants: dict[str, bool] = {}
    for drone_id, system_id in vehicle_pairs:
        permitted, _ = companion_active_transmit_authorized(
            drone_id, system_id, deployed
        )
        if permitted:
            cleared, _ = is_authorized_for_two_uav_active_flight(
                drone_id, system_id, permitted
            )
            permitted = cleared
        sink_grants[drone_id] = permitted

    # Independent expectation, derived directly from
    # ACTIVE_FLIGHT_AUTHORIZED_VEHICLES rather than by re-running the two
    # gates above -- so comparing this to sink_grants actually cross-checks
    # them instead of comparing them to themselves.
    companion_authority_deployed = resolve_authority(deployed).companion_writer_permitted
    expected_sink_grants = {
        drone_id: (
            companion_authority_deployed
            and (drone_id, system_id) in ACTIVE_FLIGHT_AUTHORIZED_VEHICLES
        )
        for drone_id, system_id in vehicle_pairs
    }

    counters.setpoint_transmit += sum(int(c.get("transmit_count", 0)) for c in cases)
    return {
        "cases": cases,
        "sink_grants": sink_grants,
        "expected_sink_grants": expected_sink_grants,
        "all_abort_actions_match_matrix": all(
            c["matches_matrix"] for c in cases if "matches_matrix" in c
        ),
        "all_latching_matches_matrix": all(
            c["latched"] == c["expected_latched"]
            for c in cases
            if "expected_latched" in c
        ),
        "uav02_authorized_only_at_its_own_system_id": all(
            c["refused"] == c["expected_refused"] for c in cases if "refused" in c
        ),
        "total_transmissions": sum(int(c.get("transmit_count", 0)) for c in cases),
    }


def run_fault_dry_runs(envelope: FlightEnvelope, warmup: WarmupContract) -> list[dict[str, Any]]:
    """Each fault must select its expected abort path."""
    cases: list[tuple[str, str, dict[str, Any], ReadinessState]] = [
        (
            "stream_lost_during_warmup",
            "setpoint_stream_gap",
            {
                "warmup_duration_s": warmup.minimum_duration_s,
                "valid_sample_count": warmup.minimum_valid_samples,
                "max_gap_s": 1.5,
                "all_previews_valid": True,
            },
            ReadinessState.SETPOINT_WARMUP,
        ),
        (
            "stale_own_state_before_arm",
            "self_telemetry_stale",
            {
                "preflight_checks_pass": True,
                "local_position_invalid_false": True,
                "companion_safety_running": True,
                "peer_state_fresh": False,
                "armed": False,
            },
            ReadinessState.PRECHECK,
        ),
        (
            "stale_own_state_during_hover",
            "self_telemetry_stale",
            {"hover_duration_s": 0.0},
            ReadinessState.HOVER,
        ),
        (
            "emergency_supervisor_hold",
            "emergency_supervisor_hold",
            {"speed_m_s": 99.0},
            ReadinessState.ZERO_VELOCITY,
        ),
        (
            "terminal_emergency_recommendation",
            "emergency_recommends_rtl_or_land",
            {},
            ReadinessState.HOVER,
        ),
        (
            "offboard_request_rejected",
            "px4_rejects_offboard",
            {"nav_state": PX4_FACTS["nav_state_posctl"]["value"]},
            ReadinessState.OFFBOARD,
        ),
        (
            "offboard_lost_after_entry",
            "px4_exits_offboard_unexpectedly",
            {"nav_state": PX4_FACTS["nav_state_posctl"]["value"]},
            ReadinessState.OFFBOARD,
        ),
        (
            "active_sender_disabled",
            "companion_safety_loop_stops",
            {
                "preflight_checks_pass": True,
                "local_position_invalid_false": True,
                "companion_safety_running": False,
                "peer_state_fresh": True,
                "armed": False,
            },
            ReadinessState.PRECHECK,
        ),
    ]
    results = []
    for name, condition, evidence, state in cases:
        rule = abort_rule_for(condition)
        may_advance, reason = transition_guard(state, evidence, envelope, warmup)
        results.append(
            {
                "case": name,
                "at_state": state.value,
                "guard_blocked": not may_advance,
                "guard_reason": reason,
                "abort_condition": condition,
                "abort_rule_found": rule is not None,
                "immediate_action": rule.immediate_action.value if rule else None,
                "px4_policy": rule.px4_policy if rule else None,
                "manual_intervention_required": rule.manual_intervention_required if rule else None,
                "recovery_allowed": rule.recovery_allowed if rule else None,
            }
        )
    return results


def run_isolation_proof() -> dict[str, Any]:
    """UAV-02 must be impossible to authorize in this gate."""
    checks = []
    for drone_id, system_id, opt_in, expect in (
        ("UAV-01", 1, True, True),
        ("UAV-01", 1, False, False),
        ("UAV-02", 2, True, False),
        ("UAV-02", 1, True, False),
        ("UAV-01", 2, True, False),
        ("UAV-01", 0, True, False),
        ("UAV-03", 3, True, False),
    ):
        allowed, reason = is_authorized_for_first_active_flight(drone_id, system_id, opt_in)
        checks.append(
            {
                "drone_id": drone_id,
                "system_id": system_id,
                "opt_in": opt_in,
                "authorized": allowed,
                "expected": expect,
                "reason": reason,
                "pass": allowed == expect,
            }
        )
    return {"checks": checks, "all_pass": all(c["pass"] for c in checks)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="artifacts/one_uav_readiness/dry_run.json")
    arguments = parser.parse_args()

    envelope = FlightEnvelope.from_environment()
    warmup = WarmupContract()
    counters = CommandCounters()

    sequence = run_sequence(envelope, warmup, counters)
    faults = run_fault_dry_runs(envelope, warmup)
    isolation = run_isolation_proof()
    active_sender = run_active_sender_dry_run(counters)

    for record in sequence["records"]:
        print(f"{record['would']:<28} guard_passed={record.get('guard_passed')} {record.get('reason','')}")

    result = {
        "px4_facts": PX4_FACTS,
        "envelope": asdict(envelope),
        "warmup": asdict(warmup),
        "active_sender_plan": asdict(ActiveOffboardSetpointSenderPlan()),
        "active_sender": active_sender,
        "sequence": sequence,
        "fault_dry_runs": faults,
        "isolation": isolation,
        "abort_matrix": [
            {**asdict(rule), "immediate_action": rule.immediate_action.value}
            for rule in ABORT_MATRIX
        ],
        "command_counters": counters.as_dict(),
        "command_counters_all_zero": counters.all_zero(),
    }
    checks = {
        "authorization_uav01_only": isolation["all_pass"],
        "sequence_guards_all_pass": sequence["blocked_at"] is None,
        "every_fault_has_abort_rule": all(f["abort_rule_found"] for f in faults),
        "every_fault_blocked_or_has_action": all(
            f["guard_blocked"] or f["immediate_action"] is not None for f in faults
        ),
        "command_counters_all_zero": counters.all_zero(),
        # The invariant is no longer "wired nowhere" -- the flight gate wires
        # it -- but "wired only where authorized". Checked by comparing the
        # real builder's decision (sink_grants) against an independently
        # derived expectation (expected_sink_grants, built straight from
        # ACTIVE_FLIGHT_AUTHORIZED_VEHICLES) rather than trusting the builder
        # about itself. Both are False at the resting "disabled" authority
        # (see .env) and that is correct, not a failure.
        "active_sender_wired_only_where_authorized": active_sender["sink_grants"]
        == active_sender["expected_sink_grants"],
        "active_sender_transmitted_nothing": active_sender["total_transmissions"] == 0,
        "active_sender_abort_actions_match_matrix": active_sender[
            "all_abort_actions_match_matrix"
        ],
        "active_sender_latching_matches_matrix": active_sender[
            "all_latching_matches_matrix"
        ],
        "active_sender_authorizes_uav02_only_at_its_system_id": active_sender[
            "uav02_authorized_only_at_its_own_system_id"
        ],
    }
    result["checks"] = checks
    result["conclusion"] = (
        "DRY_RUN_PASS" if all(checks.values()) else "DRY_RUN_FAIL"
    )

    output = Path(arguments.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print()
    print(json.dumps({"checks": checks, "conclusion": result["conclusion"]}, indent=2))
    print(f"command counters: {counters.as_dict()}")
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
