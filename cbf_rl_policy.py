"""Serializable dependency-free nominal policy for the offline CBF-RL path."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

from cbf_rl_env import OBSERVATION_FIELDS


MODEL_FORMAT = "cbf_rl_proximity_policy_v1"
MODEL_FORMAT_V2 = "cbf_rl_proximity_policy_v2"
MODEL_FORMAT_MISSION_V1 = "cbf_rl_mission_policy_v1"
MODEL_FORMAT_X500_20M_V1 = "cbf_rl_x500_20m_policy_v1"
OBSERVATION_SIZE = len(OBSERVATION_FIELDS)
MINIMUM_SEPARATION_M = 4.0


@dataclass(frozen=True)
class ProximityCbfRlPolicy:
    """Goal tracking with a learned, proximity-gated right-hand passing bias."""

    goal_gain: float
    avoidance_gain: float
    avoidance_radius_m: float
    avoidance_goal_taper_m: float | None = None
    risk_slowdown_gain: float | None = None
    closing_speed_scale_m_s: float | None = None
    minimum_separation_m: float = MINIMUM_SEPARATION_M
    maximum_velocity_m_s: float = 2.0
    vehicle_profile: str = "legacy"

    @property
    def avoids_vertically(self) -> bool:
        """Does this contract treat altitude as a real avoidance dimension?

        Four places used to ask `minimum_separation_m >= 20.0` for this, which
        conflated two independent things: how close the vehicles may get, and
        whether the policy may use the vertical axis to keep them apart. The
        legacy 4 m contract is horizontal-only; every 20 m contract so far has
        been three-dimensional, so the threshold happened to work -- right up
        until someone wants a 10 m floor that still avoids in 3D, at which
        point the coupling silently changes the behaviour instead of the limit.
        """
        return self.vehicle_profile == "x500"

    def __post_init__(self) -> None:
        values = (self.goal_gain, self.avoidance_gain, self.avoidance_radius_m)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("policy parameters must be finite")
        if (
            self.goal_gain <= 0.0
            or self.avoidance_gain < 0.0
            or not math.isfinite(self.minimum_separation_m)
            or self.minimum_separation_m <= 0.0
            or not math.isfinite(self.maximum_velocity_m_s)
            or self.maximum_velocity_m_s <= 0.0
            or self.vehicle_profile not in {"legacy", "x500"}
            or (
                self.vehicle_profile == "legacy"
                and (
                    self.minimum_separation_m != MINIMUM_SEPARATION_M
                    or self.maximum_velocity_m_s != 2.0
                )
            )
            or (
                # 10 m is the emergency floor the operator asked for, not a
                # loosening of the dynamic requirement: required_margin still
                # grows with closing speed and already exceeds 20 m by the time
                # the pair is closing at 6.81 m/s. Below 10 m nothing in the
                # latency and braking reserves is survivable, so it stays hard.
                self.vehicle_profile == "x500"
                and self.minimum_separation_m < 10.0
            )
            or self.avoidance_radius_m <= self.minimum_separation_m
            or (
                self.avoidance_goal_taper_m is not None
                and (
                    not math.isfinite(self.avoidance_goal_taper_m)
                    or self.avoidance_goal_taper_m <= 0.0
                )
                or (self.risk_slowdown_gain is None)
                != (self.closing_speed_scale_m_s is None)
                or (
                    self.risk_slowdown_gain is not None
                    and (
                        not math.isfinite(self.risk_slowdown_gain)
                        or self.risk_slowdown_gain < 0.0
                        or not math.isfinite(self.closing_speed_scale_m_s or 0.0)
                        or (self.closing_speed_scale_m_s or 0.0) <= 0.0
                    )
                )
            )
        ):
            raise ValueError("policy parameters are outside their valid range")

    def act(self, observation: Sequence[float]) -> tuple[float, float, float]:
        if len(observation) != OBSERVATION_SIZE:
            raise ValueError("policy observation shape is invalid")
        values = tuple(float(value) for value in observation)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("policy observation must be finite")

        goal_e, goal_n, goal_u = values[:3]
        peer_e, peer_n, peer_u = values[6:9]
        peer_horizontal_distance = math.hypot(peer_e, peer_n)
        peer_distance = (
            math.sqrt(peer_e * peer_e + peer_n * peer_n + peer_u * peer_u)
            if self.avoids_vertically
            else peer_horizontal_distance
        )
        proximity = max(
            0.0,
            min(
                1.0,
                (self.avoidance_radius_m - peer_distance)
                / (self.avoidance_radius_m - self.minimum_separation_m),
            ),
        )
        risk = proximity
        goal_scale = 1.0
        if self.risk_slowdown_gain is not None:
            own_e, own_n, own_u = values[3:6]
            relative_velocity_e, relative_velocity_n, relative_velocity_u = values[9:12]
            peer_velocity_e = own_e + relative_velocity_e
            peer_velocity_n = own_n + relative_velocity_n
            peer_velocity_u = own_u + relative_velocity_u
            predicted_relative_e = peer_velocity_e - self.maximum_velocity_m_s * math.tanh(
                self.goal_gain * goal_e / 20.0
            )
            predicted_relative_n = peer_velocity_n - self.maximum_velocity_m_s * math.tanh(
                self.goal_gain * goal_n / 20.0
            )
            predicted_relative_u = peer_velocity_u - self.maximum_velocity_m_s * math.tanh(
                self.goal_gain * goal_u / 10.0
            )
            predicted_closing_m_s = max(
                0.0,
                -(
                    peer_e * predicted_relative_e
                    + peer_n * predicted_relative_n
                    + (
                        peer_u * predicted_relative_u
                        if self.avoids_vertically
                        else 0.0
                    )
                )
                / max(peer_distance, 1.0e-9),
            )
            risk *= min(
                1.0,
                predicted_closing_m_s / (self.closing_speed_scale_m_s or 1.0),
            )
            goal_scale = max(0.0, 1.0 - self.risk_slowdown_gain * risk)
        turn = self.avoidance_gain * risk
        if self.avoidance_goal_taper_m is not None:
            turn *= min(
                1.0,
                (
                    math.sqrt(goal_e * goal_e + goal_n * goal_n + goal_u * goal_u)
                    if self.avoids_vertically
                    else math.hypot(goal_e, goal_n)
                )
                / self.avoidance_goal_taper_m,
            )
        if self.avoids_vertically and peer_horizontal_distance <= 1.0e-6:
            pass_e = math.copysign(1.0, peer_u)
            pass_n = 0.0
        else:
            inverse_horizontal_distance = 1.0 / max(
                peer_horizontal_distance, 1.0e-9
            )
            pass_e = -peer_n * inverse_horizontal_distance
            pass_n = peer_e * inverse_horizontal_distance
        return (
            math.tanh(goal_scale * self.goal_gain * goal_e / 20.0 + turn * pass_e),
            math.tanh(goal_scale * self.goal_gain * goal_n / 20.0 + turn * pass_n),
            math.tanh(goal_scale * self.goal_gain * goal_u / 10.0),
        )

    def as_dict(self, *, training: dict[str, Any] | None = None) -> dict[str, Any]:
        parameters = {
            "goal_gain": self.goal_gain,
            "avoidance_gain": self.avoidance_gain,
            "avoidance_radius_m": self.avoidance_radius_m,
        }
        model_format = MODEL_FORMAT
        if self.avoidance_goal_taper_m is not None:
            parameters["avoidance_goal_taper_m"] = self.avoidance_goal_taper_m
            model_format = MODEL_FORMAT_V2
        if self.risk_slowdown_gain is not None:
            parameters["risk_slowdown_gain"] = self.risk_slowdown_gain
            parameters["closing_speed_scale_m_s"] = self.closing_speed_scale_m_s
            model_format = MODEL_FORMAT_MISSION_V1
        if self.vehicle_profile == "x500":
            model_format = MODEL_FORMAT_X500_20M_V1
        encoded = json.dumps(parameters, sort_keys=True, separators=(",", ":")).encode()
        return {
            "format": model_format,
            "observation_fields": list(OBSERVATION_FIELDS),
            "action": "normalized_enu_velocity",
            "minimum_separation_m": self.minimum_separation_m,
            "maximum_velocity_m_s": self.maximum_velocity_m_s,
            "parameters": parameters,
            "parameters_sha256": hashlib.sha256(encoded).hexdigest(),
            "training": training or {},
        }

    @classmethod
    def from_dict(cls, value: Any) -> "ProximityCbfRlPolicy":
        if not isinstance(value, dict) or value.get("format") not in {
            MODEL_FORMAT,
            MODEL_FORMAT_V2,
            MODEL_FORMAT_MISSION_V1,
            MODEL_FORMAT_X500_20M_V1,
        }:
            raise ValueError("policy format is invalid")
        if tuple(value.get("observation_fields", ())) != OBSERVATION_FIELDS:
            raise ValueError("policy observation contract mismatch")
        model_format = value.get("format")
        high_speed_formats = {
            MODEL_FORMAT_X500_20M_V1,
        }
        if model_format in high_speed_formats:
            try:
                minimum_separation_m = float(value["minimum_separation_m"])
                maximum_velocity_m_s = float(value["maximum_velocity_m_s"])
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError("policy flight envelope is invalid") from error
        else:
            if value.get("minimum_separation_m") != MINIMUM_SEPARATION_M:
                raise ValueError("policy separation contract mismatch")
            minimum_separation_m = MINIMUM_SEPARATION_M
            maximum_velocity_m_s = 2.0
        parameters = value.get("parameters")
        if not isinstance(parameters, dict):
            raise ValueError("policy parameters are missing")
        encoded = json.dumps(parameters, sort_keys=True, separators=(",", ":")).encode()
        if value.get("parameters_sha256") != hashlib.sha256(encoded).hexdigest():
            raise ValueError("policy parameter checksum mismatch")
        try:
            return cls(
                float(parameters["goal_gain"]),
                float(parameters["avoidance_gain"]),
                float(parameters["avoidance_radius_m"]),
                (
                    float(parameters["avoidance_goal_taper_m"])
                    if value.get("format") in {MODEL_FORMAT_V2, MODEL_FORMAT_MISSION_V1, *high_speed_formats}
                    else None
                ),
                (
                    float(parameters["risk_slowdown_gain"])
                    if value.get("format") in {MODEL_FORMAT_MISSION_V1, *high_speed_formats}
                    else None
                ),
                (
                    float(parameters["closing_speed_scale_m_s"])
                    if value.get("format") in {MODEL_FORMAT_MISSION_V1, *high_speed_formats}
                    else None
                ),
                minimum_separation_m,
                maximum_velocity_m_s,
                {
                    MODEL_FORMAT_X500_20M_V1: "x500",
                }.get(model_format, "legacy"),
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("policy parameters are invalid") from error

    @classmethod
    def load(cls, path: str | Path) -> "ProximityCbfRlPolicy":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    def save(self, path: str | Path, *, training: dict[str, Any] | None = None) -> None:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_suffix(destination.suffix + ".tmp")
        temporary.write_text(
            json.dumps(self.as_dict(training=training), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary.replace(destination)
