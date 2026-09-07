"""Default-off authenticated runtime for the frozen CBF-RL policy."""

from __future__ import annotations

import hashlib
import math
import os
from pathlib import Path
from typing import Any, Mapping, Sequence

from cbf_command_gate import CbfCommand, CbfCommandGate
from cbf_rl_policy import MINIMUM_SEPARATION_M, ProximityCbfRlPolicy
from conflict_coordinator import ConflictCoordinator


Vector3 = tuple[float, float, float]


class CbfRlShadow:
    """Authenticated RL runtime with shadow and fail-safe active modes."""

    ACTIVE_ACK = "cbf_rl_active_v1"

    def __init__(
        self,
        *,
        mode: str = "off",
        policy: ProximityCbfRlPolicy | None = None,
        model_path: str = "",
        model_sha256: str = "",
        load_error: str = "",
        coordinator: ConflictCoordinator | None = None,
    ) -> None:
        self.mode = mode
        self.policy = policy
        self.model_path = model_path
        self.model_sha256 = model_sha256
        self.load_error = load_error
        self.coordinator = coordinator
        self.request_count = 0
        self.valid_count = 0
        self.rejected_count = 0
        self.last: dict[str, Any] = {}

    @classmethod
    def off(cls) -> "CbfRlShadow":
        return cls()

    @classmethod
    def from_environment(cls) -> "CbfRlShadow":
        mode = os.environ.get("SWARM_CBF_RL_MODE", "off").strip().lower()
        if mode == "off":
            return cls.off()
        if mode not in {"shadow", "active"}:
            return cls(mode="off", load_error=f"invalid_mode:{mode}")
        if mode == "active":
            if os.environ.get("SWARM_CBF_RL_ACTIVE_ACK", "").strip() != cls.ACTIVE_ACK:
                return cls(mode=mode, load_error="active_ack_required")

        default_path = Path(__file__).with_name("cbf_rl_policy_v1.json")
        model_path = Path(
            os.environ.get("SWARM_CBF_RL_MODEL", str(default_path)).strip()
        )
        expected_sha256 = os.environ.get("SWARM_CBF_RL_MODEL_SHA256", "").strip().lower()
        try:
            payload = model_path.read_bytes()
            actual_sha256 = hashlib.sha256(payload).hexdigest()
            if len(expected_sha256) != 64:
                raise ValueError("model_sha256_required")
            if actual_sha256 != expected_sha256:
                raise ValueError("model_sha256_mismatch")
            policy = ProximityCbfRlPolicy.load(model_path)
        except Exception as error:
            return cls(
                mode=mode,
                model_path=str(model_path),
                model_sha256=expected_sha256,
                load_error=str(error),
            )
        if mode == "active":
            try:
                peer_age_ms = float(
                    os.environ.get("SWARM_PEER_STATE_MAX_AGE_MS", "100")
                )
            except ValueError:
                peer_age_ms = float("nan")
            expected_peer_age_ms = (
                150.0 if policy.vehicle_profile == "x500" else 100.0
            )
            if not math.isclose(peer_age_ms, expected_peer_age_ms):
                return cls(
                    mode=mode,
                    model_path=str(model_path),
                    model_sha256=actual_sha256,
                    load_error="active_peer_age_contract_mismatch",
                )
        return cls(
            mode=mode,
            policy=policy,
            model_path=str(model_path),
            model_sha256=actual_sha256,
        )

    def evaluate(
        self,
        *,
        drone_id: str,
        peer_ids: Sequence[str],
        target_enu_m: Sequence[float] | None,
        swarm_state: Mapping[str, Any],
        gate: CbfCommandGate,
        deterministic_nominal_enu_m_s: Vector3,
        deterministic_command: CbfCommand,
    ) -> dict[str, Any]:
        status, _, _ = self.evaluate_candidate(
            drone_id=drone_id,
            peer_ids=peer_ids,
            target_enu_m=target_enu_m,
            swarm_state=swarm_state,
            gate=gate,
            deterministic_nominal_enu_m_s=deterministic_nominal_enu_m_s,
            deterministic_command=deterministic_command,
            apply=False,
        )
        return status

    def evaluate_candidate(
        self,
        *,
        drone_id: str,
        peer_ids: Sequence[str],
        target_enu_m: Sequence[float] | None,
        swarm_state: Mapping[str, Any],
        gate: CbfCommandGate,
        deterministic_nominal_enu_m_s: Vector3,
        deterministic_command: CbfCommand,
        apply: bool,
    ) -> tuple[dict[str, Any], Vector3 | None, CbfCommand | None]:
        if self.mode not in {"shadow", "active"}:
            return self.status(reason=self.load_error or "disabled"), None, None
        self.request_count += 1
        if self.policy is None:
            self.rejected_count += 1
            return self.status(reason=self.load_error or "model_unavailable"), None, None
        try:
            if len(peer_ids) != 1:
                raise ValueError("exactly_one_peer_required")
            if not math.isclose(
                gate.config.minimum_separation_m,
                self.policy.minimum_separation_m,
            ):
                raise ValueError("minimum_separation_contract_mismatch")
            if not math.isclose(
                gate.config.maximum_velocity_m_s,
                self.policy.maximum_velocity_m_s,
            ):
                raise ValueError("maximum_velocity_contract_mismatch")
            if self.mode == "active":
                _validate_active_contract(gate, self.policy)
            observation = _observation(
                drone_id,
                peer_ids[0],
                target_enu_m,
                swarm_state,
            )
            action = self.policy.act(observation)
            nominal = tuple(
                value * gate.config.maximum_velocity_m_s for value in action
            )
            deterministic_speed = _distance(
                deterministic_nominal_enu_m_s,
                (0.0, 0.0, 0.0),
            )
            nominal_speed = _distance(nominal, (0.0, 0.0, 0.0))
            if nominal_speed > deterministic_speed:
                nominal = tuple(
                    value * deterministic_speed / nominal_speed for value in nominal
                )
            coordination = {"active": False, "role": "disabled"}
            if self.coordinator is not None and apply:
                nominal, coordination = self.coordinator.filter(
                    nominal,
                    deterministic_nominal_enu_m_s,
                    swarm_state,
                )
            elif self.coordinator is not None:
                self.coordinator.reset()
            shadow_command = gate.filter(nominal, dict(swarm_state))
            applied = self.mode == "active" and apply
            self.valid_count += 1
            self.last = {
                "valid": True,
                "reason": "active_applied" if applied else "shadow_evaluated",
                "applied": applied,
                "transmit_authority": applied,
                "fallback_active": self.mode == "active" and not applied,
                "control_source": "cbf_rl_active" if applied else "deterministic",
                "normalized_action": list(action),
                "nominal_velocity_enu_m_s": list(nominal),
                "shielded_velocity_enu_m_s": list(shadow_command.velocity_enu_m_s),
                "conflict_coordination": coordination,
                "cbf": shadow_command.as_dict(),
                "nominal_delta_norm_m_s": _distance(
                    nominal, deterministic_nominal_enu_m_s
                ),
                "shielded_delta_norm_m_s": _distance(
                    shadow_command.velocity_enu_m_s,
                    deterministic_command.velocity_enu_m_s,
                ),
            }
            return (
                self.status(),
                nominal if applied else None,
                shadow_command if applied else None,
            )
        except Exception as error:
            self.rejected_count += 1
            self.last = {
                "valid": False,
                "reason": str(error),
                "fallback_active": self.mode == "active",
                "control_source": "deterministic_fallback",
            }
            return self.status(), None, None

    def status(self, *, reason: str | None = None) -> dict[str, Any]:
        current = dict(self.last)
        if reason is not None:
            current = {"valid": False, "reason": reason}
        return {
            "mode": self.mode,
            "model_loaded": self.policy is not None,
            "model_path": self.model_path,
            "model_sha256": self.model_sha256,
            "load_error": self.load_error,
            "request_count": self.request_count,
            "valid_count": self.valid_count,
            "rejected_count": self.rejected_count,
            "applied": False,
            "transmit_authority": False,
            "fallback_active": self.mode == "active",
            "control_source": "deterministic",
            **current,
        }


def _validate_active_contract(
    gate: CbfCommandGate, policy: ProximityCbfRlPolicy
) -> None:
    config = gate.config
    expected = [
        (config.minimum_separation_m, policy.minimum_separation_m),
        (config.barrier_gain_s_inv, 2.0),
        (config.maximum_velocity_m_s, policy.maximum_velocity_m_s),
        (config.command_latency_s, 0.65),
        (
            config.design_margin_buffer_m,
            5.0 if policy.vehicle_profile == "x500" else 0.0,
        ),
        (config.covariance_sigma, 0.10),
    ]
    if policy.avoids_vertically:
        relative_braking_acceleration = 6.0
        expected.extend(
            (
                (
                    config.relative_braking_acceleration_m_s2,
                    relative_braking_acceleration,
                ),
                (config.tracking_reserve_m, 2.0),
            )
        )
    if not config.require_position_covariance or any(
        not math.isclose(actual, required, rel_tol=0.0, abs_tol=1.0e-9)
        for actual, required in expected
    ):
        raise ValueError("active_cbf_contract_mismatch")


def _observation(
    drone_id: str,
    peer_id: str,
    target_enu_m: Sequence[float] | None,
    swarm_state: Mapping[str, Any],
) -> tuple[float, ...]:
    target = _vector(target_enu_m, "target")
    own = _state(swarm_state.get(drone_id), "own")
    peer = _state(swarm_state.get(peer_id), "peer")
    own_position, own_velocity, own_covariance, own_age, own_valid = own
    peer_position, peer_velocity, peer_covariance, peer_age, peer_valid = peer
    return (
        *(target[index] - own_position[index] for index in range(3)),
        *own_velocity,
        *(peer_position[index] - own_position[index] for index in range(3)),
        *(peer_velocity[index] - own_velocity[index] for index in range(3)),
        _sqrt_trace(own_covariance),
        _sqrt_trace(peer_covariance),
        own_age / 1000.0,
        peer_age / 1000.0,
        float(own_valid),
        float(peer_valid),
        float(own_covariance is not None),
        float(peer_covariance is not None),
    )


def _state(
    value: Any, label: str
) -> tuple[Vector3, Vector3, Vector3 | None, float, bool]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label}_state_missing")
    position = _vector(value.get("position_enu_m"), f"{label}_position")
    velocity = _vector(value.get("velocity_enu_m_s"), f"{label}_velocity")
    covariance = _covariance(value.get("position_covariance_m2"))
    try:
        age = float(value.get("message_age_ms", 0.0))
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label}_age_invalid") from error
    if not math.isfinite(age) or age < 0.0:
        raise ValueError(f"{label}_age_invalid")
    return position, velocity, covariance, age, bool(value.get("valid", False))


def _vector(value: Any, label: str) -> Vector3:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{label}_invalid")
    try:
        vector = tuple(float(item) for item in value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label}_invalid") from error
    if not all(math.isfinite(item) for item in vector):
        raise ValueError(f"{label}_invalid")
    return vector  # type: ignore[return-value]


def _covariance(value: Any) -> Vector3 | None:
    try:
        covariance = _vector(value, "covariance")
    except ValueError:
        return None
    return covariance if all(item >= 0.0 for item in covariance) else None


def _sqrt_trace(covariance: Vector3 | None) -> float:
    return math.sqrt(sum(covariance)) if covariance is not None else 0.0


def _distance(left: Sequence[float], right: Sequence[float]) -> float:
    return math.sqrt(sum((float(left[index]) - float(right[index])) ** 2 for index in range(3)))
