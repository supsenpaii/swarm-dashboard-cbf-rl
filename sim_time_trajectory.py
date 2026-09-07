"""Pure deterministic contract for the Gazebo sim-time target driver."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math


TRAJECTORY_DRIVER_VERSION = "swarm_sim_time_trajectory_v1"
_CONTRACT = {
    "clock": "gazebo_sim_time",
    "function": "linear_range_and_yaw_clamped_then_hold",
    "target_entity": "sparrow_gimbal_1",
    "target_reference": "model_origin",
    "version": TRAJECTORY_DRIVER_VERSION,
}
TRAJECTORY_CONTRACT_SHA256 = hashlib.sha256(
    json.dumps(_CONTRACT, sort_keys=True, separators=(",", ":")).encode()
).hexdigest()


@dataclass(frozen=True)
class SimTimeTrajectoryContract:
    start_range_m: float
    end_range_m: float
    lateral_m: float
    target_z_m: float
    yaw_start_deg: float
    yaw_end_deg: float
    movement_duration_s: float
    hold_duration_s: float

    def __post_init__(self) -> None:
        values = tuple(float(value) for value in self.__dict__.values())
        if not all(math.isfinite(value) for value in values):
            raise ValueError("trajectory_parameter_nonfinite")
        if not 3.0 <= self.start_range_m <= 12.0 or not 3.0 <= self.end_range_m <= 12.0:
            raise ValueError("target_range_outside_core_domain")
        if abs(self.lateral_m) > 2.0:
            raise ValueError("target_lateral_outside_limit")
        if min(self.start_range_m, self.end_range_m) <= abs(self.lateral_m):
            raise ValueError("target_lateral_exceeds_range")
        if not 0.5 <= self.target_z_m <= 1.5 or max(abs(self.yaw_start_deg), abs(self.yaw_end_deg)) > 30.0:
            raise ValueError("target_pose_outside_precommitted_bounds")
        if self.movement_duration_s <= 0.0 or self.hold_duration_s < 0.0:
            raise ValueError("trajectory_duration_invalid")

    @property
    def total_duration_s(self) -> float:
        return self.movement_duration_s + self.hold_duration_s

    def pose_at(self, elapsed_sim_s: float) -> dict[str, float | str]:
        """Return target pose determined exclusively by elapsed simulation time."""
        elapsed = max(0.0, float(elapsed_sim_s))
        progress = min(1.0, elapsed / self.movement_duration_s)
        range_m = self.start_range_m + progress * (self.end_range_m - self.start_range_m)
        yaw_deg = self.yaw_start_deg + progress * (self.yaw_end_deg - self.yaw_start_deg)
        x_m = math.sqrt(range_m * range_m - self.lateral_m * self.lateral_m)
        moving = elapsed < self.movement_duration_s
        radial_velocity = (
            (self.end_range_m - self.start_range_m) / self.movement_duration_s
            if moving else 0.0
        )
        x_velocity = range_m * radial_velocity / x_m if moving else 0.0
        x_acceleration = (
            -(self.lateral_m**2) * radial_velocity**2 / (x_m**3)
            if moving else 0.0
        )
        return {
            "clock_domain": "gazebo_sim_time",
            "elapsed_sim_s": elapsed,
            "progress": progress,
            "range_m": range_m,
            "x_m": x_m,
            "y_m": self.lateral_m,
            "z_m": self.target_z_m,
            "yaw_deg": yaw_deg,
            "x_velocity_m_s": x_velocity,
            "x_acceleration_m_s2": x_acceleration,
            "radial_velocity_m_s": radial_velocity,
        }

    def command_payload(self, session_token: str) -> str:
        safe_token = str(session_token).strip()
        if not safe_token or "|" in safe_token or '"' in safe_token:
            raise ValueError("trajectory_session_token_invalid")
        fields = (
            "start", safe_token, self.start_range_m, self.end_range_m,
            self.lateral_m, self.target_z_m, self.yaw_start_deg,
            self.yaw_end_deg, self.movement_duration_s, self.hold_duration_s,
            TRAJECTORY_DRIVER_VERSION, TRAJECTORY_CONTRACT_SHA256,
        )
        return "|".join(str(value) for value in fields)


def contract_from_args(args: object) -> SimTimeTrajectoryContract:
    return SimTimeTrajectoryContract(
        start_range_m=float(getattr(args, "start_range_m")),
        end_range_m=float(getattr(args, "end_range_m")),
        lateral_m=float(getattr(args, "lateral_m")),
        target_z_m=float(getattr(args, "target_z_m")),
        yaw_start_deg=float(getattr(args, "yaw_start_deg")),
        yaw_end_deg=float(getattr(args, "yaw_end_deg")),
        movement_duration_s=float(getattr(args, "movement_duration_s")),
        hold_duration_s=float(getattr(args, "hold_duration_s")),
    )
