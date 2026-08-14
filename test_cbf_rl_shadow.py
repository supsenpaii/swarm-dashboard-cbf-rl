import hashlib
from pathlib import Path

from cbf_command_gate import CbfCommandGate, CbfConfig
from cbf_rl_policy import ProximityCbfRlPolicy
from cbf_rl_shadow import CbfRlShadow
from conflict_coordinator import ConflictCoordinator, x500_20m_conflict_config
from companion_safety import CompanionSafetyMonitor
from formation_controller import FormationSlot
from offboard_setpoint_sender import ShadowOffboardSetpointSender
from trajectory_controller import LinearTrajectory


MODEL = Path(__file__).with_name("cbf_rl_policy_v1.json")
MODEL_SHA256 = hashlib.sha256(MODEL.read_bytes()).hexdigest()
COVARIANCE = (0.041, 0.041, 0.071)


def _states() -> dict[str, dict[str, object]]:
    return {
        "UAV-01": {
            "valid": True,
            "position_enu_m": (0.0, 0.0, 9.0),
            "velocity_enu_m_s": (0.0, 0.0, 0.0),
            "position_covariance_m2": COVARIANCE,
            "message_age_ms": 10.0,
        },
        "UAV-02": {
            "valid": True,
            "position_enu_m": (0.0, 10.0, 9.0),
            "velocity_enu_m_s": (0.0, 0.0, 0.0),
            "position_covariance_m2": COVARIANCE,
            "message_age_ms": 12.0,
        },
    }


def _shadow(monkeypatch) -> CbfRlShadow:
    monkeypatch.setenv("SWARM_CBF_RL_MODE", "shadow")
    monkeypatch.setenv("SWARM_CBF_RL_MODEL", str(MODEL))
    monkeypatch.setenv("SWARM_CBF_RL_MODEL_SHA256", MODEL_SHA256)
    return CbfRlShadow.from_environment()


def _active(monkeypatch) -> CbfRlShadow:
    monkeypatch.setenv("SWARM_CBF_RL_MODE", "active")
    monkeypatch.setenv("SWARM_CBF_RL_ACTIVE_ACK", CbfRlShadow.ACTIVE_ACK)
    monkeypatch.setenv("SWARM_PEER_STATE_MAX_AGE_MS", "100")
    monkeypatch.setenv("SWARM_CBF_RL_MODEL", str(MODEL))
    monkeypatch.setenv("SWARM_CBF_RL_MODEL_SHA256", MODEL_SHA256)
    return CbfRlShadow.from_environment()


def _active_cbf() -> CbfConfig:
    return CbfConfig(
        covariance_sigma=0.1,
        command_latency_s=0.65,
        require_position_covariance=True,
    )


def test_shadow_is_default_off_and_does_not_touch_model(monkeypatch) -> None:
    monkeypatch.delenv("SWARM_CBF_RL_MODE", raising=False)
    monkeypatch.setenv("SWARM_CBF_RL_MODEL", "/does/not/exist")

    status = CbfRlShadow.from_environment().status(reason="disabled")

    assert status["mode"] == "off"
    assert not status["model_loaded"]
    assert not status["transmit_authority"]
    assert not status["applied"]


def test_shadow_requires_complete_artifact_digest(monkeypatch) -> None:
    monkeypatch.setenv("SWARM_CBF_RL_MODE", "shadow")
    monkeypatch.setenv("SWARM_CBF_RL_MODEL", str(MODEL))
    monkeypatch.delenv("SWARM_CBF_RL_MODEL_SHA256", raising=False)

    shadow = CbfRlShadow.from_environment()

    assert shadow.policy is None
    assert shadow.load_error == "model_sha256_required"


def test_active_mode_requires_explicit_ack(monkeypatch) -> None:
    monkeypatch.setenv("SWARM_CBF_RL_MODE", "active")
    monkeypatch.delenv("SWARM_CBF_RL_ACTIVE_ACK", raising=False)

    runtime = CbfRlShadow.from_environment()

    assert runtime.mode == "active"
    assert runtime.policy is None
    assert runtime.load_error == "active_ack_required"
    assert runtime.status()["fallback_active"]


def test_authenticated_shadow_proposal_is_cbf_filtered(monkeypatch) -> None:
    shadow = _shadow(monkeypatch)
    states = _states()
    gate = CbfCommandGate(
        "UAV-02",
        ("UAV-01",),
        CbfConfig(covariance_sigma=0.1, require_position_covariance=True),
    )
    deterministic_nominal = (0.0, -1.0, 0.0)
    deterministic_command = gate.filter(deterministic_nominal, states)

    status = shadow.evaluate(
        drone_id="UAV-02",
        peer_ids=("UAV-01",),
        target_enu_m=(0.0, 8.0, 9.0),
        swarm_state=states,
        gate=gate,
        deterministic_nominal_enu_m_s=deterministic_nominal,
        deterministic_command=deterministic_command,
    )

    assert status["valid"]
    assert status["model_loaded"]
    assert status["cbf"]["active"]
    assert not status["transmit_authority"]
    assert not status["applied"]


def test_companion_output_is_identical_with_shadow_enabled(monkeypatch) -> None:
    states = _states()
    config = CbfConfig(covariance_sigma=0.1, require_position_covariance=True)
    arguments = {
        "drone_id": "UAV-02",
        "peer_ids": ("UAV-01",),
        "leader_id": "UAV-01",
        "slots": (FormationSlot("UAV-02", (0.0, 8.0, 0.0)),),
        "cbf_config": config,
    }
    baseline = CompanionSafetyMonitor(**arguments).evaluate(
        states, now_monotonic_s=1.0
    )
    shadowed = CompanionSafetyMonitor(
        **arguments, cbf_rl_shadow=_shadow(monkeypatch)
    ).evaluate(states, now_monotonic_s=1.0)

    baseline_payload = baseline.as_dict()
    shadow_payload = shadowed.as_dict()
    baseline_payload.pop("cbf_rl_shadow")
    shadow_status = shadow_payload.pop("cbf_rl_shadow")

    assert shadow_payload == baseline_payload
    assert shadow_status["valid"]
    assert shadow_status["request_count"] == 1
    assert not shadow_status["applied"]


def test_disarmed_trajectory_start_is_available_to_shadow(monkeypatch) -> None:
    monitor = CompanionSafetyMonitor(
        drone_id="UAV-02",
        peer_ids=("UAV-01",),
        leader_id="UAV-01",
        slots=(FormationSlot("UAV-02", (0.0, 8.0, 0.0)),),
        cbf_config=CbfConfig(covariance_sigma=0.1, require_position_covariance=True),
        trajectory=LinearTrajectory(
            start_enu_m=(0.0, 10.0, 9.0),
            end_enu_m=(10.0, 10.0, 9.0),
            speed_m_s=1.0,
        ),
        cbf_rl_shadow=_shadow(monkeypatch),
    )

    status = monitor.evaluate(
        _states(), now_monotonic_s=1.0, station_keeping=False
    ).as_dict()

    assert status["nominal_reason"] == "trajectory_inactive"
    assert status["nominal_velocity_enu_m_s"] == [0.0, 0.0, 0.0]
    assert status["cbf_rl_shadow"]["valid"]
    assert not status["cbf_rl_shadow"]["applied"]


def test_active_policy_is_selected_only_after_the_same_cbf(monkeypatch) -> None:
    runtime = _active(monkeypatch)
    states = _states()
    gate = CbfCommandGate("UAV-02", ("UAV-01",), _active_cbf())
    deterministic_nominal = (0.0, -1.0, 0.0)
    deterministic_command = gate.filter(deterministic_nominal, states)

    status, rl_nominal, rl_command = runtime.evaluate_candidate(
        drone_id="UAV-02",
        peer_ids=("UAV-01",),
        target_enu_m=(0.0, 8.0, 9.0),
        swarm_state=states,
        gate=gate,
        deterministic_nominal_enu_m_s=deterministic_nominal,
        deterministic_command=deterministic_command,
        apply=True,
    )

    assert status["valid"]
    assert status["applied"]
    assert status["transmit_authority"]
    assert status["control_source"] == "cbf_rl_active"
    assert rl_nominal is not None
    assert rl_command is not None and rl_command.active
    assert list(rl_command.velocity_enu_m_s) == status["shielded_velocity_enu_m_s"]


def test_active_policy_cannot_exceed_the_mission_nominal_speed(monkeypatch) -> None:
    runtime = _active(monkeypatch)
    states = _states()
    gate = CbfCommandGate("UAV-02", ("UAV-01",), _active_cbf())
    deterministic_nominal = (0.0, -0.4, 0.0)

    status, rl_nominal, _ = runtime.evaluate_candidate(
        drone_id="UAV-02",
        peer_ids=("UAV-01",),
        target_enu_m=(10.0, 0.0, 9.0),
        swarm_state=states,
        gate=gate,
        deterministic_nominal_enu_m_s=deterministic_nominal,
        deterministic_command=gate.filter(deterministic_nominal, states),
        apply=True,
    )

    assert status["valid"]
    assert rl_nominal is not None
    assert sum(value * value for value in rl_nominal) ** 0.5 <= 0.4


def test_active_policy_respects_the_zero_speed_acceleration_ramp_frame(monkeypatch) -> None:
    runtime = _active(monkeypatch)
    states = _states()
    gate = CbfCommandGate("UAV-02", ("UAV-01",), _active_cbf())
    zero = (0.0, 0.0, 0.0)

    _, rl_nominal, rl_command = runtime.evaluate_candidate(
        drone_id="UAV-02",
        peer_ids=("UAV-01",),
        target_enu_m=(10.0, 0.0, 9.0),
        swarm_state=states,
        gate=gate,
        deterministic_nominal_enu_m_s=zero,
        deterministic_command=gate.filter(zero, states),
        apply=True,
    )

    assert rl_nominal == zero
    assert rl_command is not None
    assert rl_command.velocity_enu_m_s == zero


def test_active_contract_mismatch_falls_back_to_deterministic(monkeypatch) -> None:
    states = _states()
    monitor = CompanionSafetyMonitor(
        drone_id="UAV-02",
        peer_ids=("UAV-01",),
        leader_id="UAV-01",
        slots=(FormationSlot("UAV-02", (0.0, 8.0, 0.0)),),
        cbf_config=CbfConfig(covariance_sigma=0.1, require_position_covariance=True),
        cbf_rl_shadow=_active(monkeypatch),
    )
    baseline = CompanionSafetyMonitor(
        drone_id="UAV-02",
        peer_ids=("UAV-01",),
        leader_id="UAV-01",
        slots=(FormationSlot("UAV-02", (0.0, 8.0, 0.0)),),
        cbf_config=CbfConfig(covariance_sigma=0.1, require_position_covariance=True),
    ).evaluate(states, now_monotonic_s=1.0, station_keeping=True)

    active = monitor.evaluate(states, now_monotonic_s=1.0, station_keeping=True)

    assert active.output_velocity_enu_m_s == baseline.output_velocity_enu_m_s
    assert active.cbf_rl_shadow["reason"] == "active_cbf_contract_mismatch"
    assert active.cbf_rl_shadow["fallback_active"]
    assert not active.cbf_rl_shadow["applied"]


def test_companion_active_output_is_the_rl_proposal_after_cbf(monkeypatch) -> None:
    monitor = CompanionSafetyMonitor(
        drone_id="UAV-02",
        peer_ids=("UAV-01",),
        leader_id="UAV-01",
        slots=(FormationSlot("UAV-02", (0.0, 8.0, 0.0)),),
        cbf_config=_active_cbf(),
        cbf_rl_shadow=_active(monkeypatch),
    )

    status = monitor.evaluate(
        _states(), now_monotonic_s=1.0, station_keeping=True
    ).as_dict()

    rl = status["cbf_rl_shadow"]
    assert rl["applied"]
    assert rl["control_source"] == "cbf_rl_active"
    assert status["authority"] == "shadow_companion_only"
    assert status["nominal_source"] == "cbf_rl_active"
    assert status["output_velocity_enu_m_s"] == rl["shielded_velocity_enu_m_s"]
    assert status["cbf"] == rl["cbf"]

    preview = ShadowOffboardSetpointSender(
        drone_id="UAV-02",
        px4_system_id=2,
        maximum_velocity_m_s=2.0,
        maximum_command_age_s=0.5,
    ).preview(
        monitor.evaluate(_states(), now_monotonic_s=1.0, station_keeping=True),
        now_monotonic_s=1.0,
        command_monotonic_s=1.0,
        px4_main_mode=6,
        px4_armed=True,
        px4_offboard_main_mode=6,
    )
    assert preview.valid
    assert not preview.inhibited


def test_active_runtime_reports_conflict_coordination(monkeypatch) -> None:
    runtime = _active(monkeypatch)
    runtime.coordinator = ConflictCoordinator("UAV-02", "UAV-01")
    states = _states()
    states["UAV-01"]["position_enu_m"] = (-5.0, 0.0, 9.0)
    states["UAV-01"]["velocity_enu_m_s"] = (1.0, 0.0, 0.0)
    states["UAV-02"]["position_enu_m"] = (5.0, 0.0, 9.0)
    states["UAV-02"]["velocity_enu_m_s"] = (-1.0, 0.0, 0.0)
    gate = CbfCommandGate("UAV-02", ("UAV-01",), _active_cbf())
    mission = (-1.0, 0.0, 0.0)

    status, _, _ = runtime.evaluate_candidate(
        drone_id="UAV-02",
        peer_ids=("UAV-01",),
        target_enu_m=(-10.0, 0.0, 9.0),
        swarm_state=states,
        gate=gate,
        deterministic_nominal_enu_m_s=mission,
        deterministic_command=gate.filter(mission, states),
        apply=True,
    )

    assert status["conflict_coordination"]["active"]
    assert status["conflict_coordination"]["role"] == "yield"


def test_active_runtime_cannot_replace_mission_velocity_while_clear(monkeypatch) -> None:
    runtime = _active(monkeypatch)
    runtime.coordinator = ConflictCoordinator(
        "UAV-02", "UAV-01", x500_20m_conflict_config()
    )
    states = _states()
    states["UAV-01"]["position_enu_m"] = (0.0, -100.0, 9.0)
    gate = CbfCommandGate("UAV-02", ("UAV-01",), _active_cbf())
    mission = (0.0, -1.0, 0.0)
    deterministic = gate.filter(mission, states)

    status, coordinated_nominal, coordinated_command = runtime.evaluate_candidate(
        drone_id="UAV-02",
        peer_ids=("UAV-01",),
        target_enu_m=(100.0, 100.0, 9.0),
        swarm_state=states,
        gate=gate,
        deterministic_nominal_enu_m_s=mission,
        deterministic_command=deterministic,
        apply=True,
    )

    assert status["conflict_coordination"]["role"] == "clear"
    assert status["nominal_delta_norm_m_s"] == 0.0
    assert coordinated_nominal == mission
    assert coordinated_command == deterministic
