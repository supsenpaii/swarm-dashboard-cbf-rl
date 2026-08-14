from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class FusionEstimate:
    valid: bool
    reason: str
    timestamp_s: float
    position_ned_m: tuple[float, float, float] | None
    velocity_ned_m_s: tuple[float, float, float] | None
    covariance: tuple[tuple[float, ...], ...]
    position_std_m: float | None
    velocity_std_m_s: float | None
    velocity_valid: bool
    update_count: int
    estimator_mode: str = "STATIONARY"
    stationary_probability: float = 1.0
    moving_probability: float = 0.0
    measurement_age_s: float | None = None


class TargetFusionEKF:
    """Adaptive stationary/constant-velocity EKF in the local NED frame.

    Position measurements are already ego-motion compensated because they are
    formed from a timestamp-aligned camera pose and bearing. Bearing-only
    corrections deliberately do not refresh the metric-measurement freshness
    clock: a live bbox must not hide a lost range source indefinitely.
    """

    def __init__(
        self,
        *,
        acceleration_noise_m_s2: float = 2.0,
        maximum_prediction_dt_s: float = 0.5,
        velocity_ready_updates: int = 4,
        stationary_speed_m_s: float = 0.25,
        moving_speed_m_s: float = 0.55,
        zero_velocity_std_m_s: float = 0.12,
    ) -> None:
        self.acceleration_noise = max(0.05, acceleration_noise_m_s2)
        self.maximum_prediction_dt_s = max(0.05, maximum_prediction_dt_s)
        self.velocity_ready_updates = max(2, velocity_ready_updates)
        self.stationary_speed_m_s = max(0.02, float(stationary_speed_m_s))
        self.moving_speed_m_s = max(
            self.stationary_speed_m_s + 0.05,
            float(moving_speed_m_s),
        )
        self.zero_velocity_std_m_s = max(
            0.02,
            float(zero_velocity_std_m_s),
        )
        self.reset()

    def reset(self) -> None:
        self.x: np.ndarray | None = None
        self.p: np.ndarray | None = None
        self.timestamp_s: float | None = None
        self.update_count = 0
        self.last_measurement_timestamp_s: float | None = None
        self.last_bearing_timestamp_s: float | None = None
        self._last_observed_position: np.ndarray | None = None
        self._last_observed_position_timestamp_s: float | None = None
        self.stationary_probability = 1.0
        self.estimator_mode = "STATIONARY"

    def predict(self, timestamp_s: float) -> None:
        timestamp = float(timestamp_s)
        if self.x is None or self.p is None or self.timestamp_s is None:
            self.timestamp_s = timestamp
            return
        dt = timestamp - self.timestamp_s
        if dt <= 0.0:
            return
        dt = min(dt, self.maximum_prediction_dt_s)
        transition = np.eye(6)
        transition[0:3, 3:6] = np.eye(3) * dt
        g = np.vstack((np.eye(3) * (0.5 * dt * dt), np.eye(3) * dt))
        process = g @ (np.eye(3) * self.acceleration_noise**2) @ g.T
        self.x = transition @ self.x
        self.p = transition @ self.p @ transition.T + process
        self.timestamp_s = timestamp

    def update_position(
        self,
        position_ned_m: Sequence[float],
        covariance_m2: np.ndarray | Sequence[Sequence[float]],
        timestamp_s: float,
    ) -> None:
        position = self._vector(position_ned_m, "position")
        covariance = np.asarray(covariance_m2, dtype=np.float64)
        if covariance.shape != (3, 3) or not np.all(np.isfinite(covariance)):
            raise ValueError("position covariance must be finite 3x3")
        covariance = covariance + np.eye(3) * 1e-6
        self.predict(timestamp_s)
        measurement_speed = 0.0
        if (
            self._last_observed_position is not None
            and self._last_observed_position_timestamp_s is not None
        ):
            observation_dt = (
                float(timestamp_s)
                - self._last_observed_position_timestamp_s
            )
            if observation_dt > 1e-3:
                displacement = position - self._last_observed_position
                measurement_speed = float(
                    np.linalg.norm(displacement) / observation_dt
                )
        self._last_observed_position = position.copy()
        self._last_observed_position_timestamp_s = float(timestamp_s)
        self._update_model_probability(measurement_speed)
        if self.x is None or self.p is None:
            self.x = np.zeros(6, dtype=np.float64)
            self.x[:3] = position
            self.p = np.eye(6, dtype=np.float64)
            self.p[:3, :3] = covariance
            self.p[3:, 3:] *= 9.0
            self.timestamp_s = float(timestamp_s)
        else:
            observation = np.zeros((3, 6), dtype=np.float64)
            observation[:, :3] = np.eye(3)
            self._linear_update(
                observation,
                position - observation @ self.x,
                covariance,
            )
        if (
            self.x is not None
            and self.p is not None
            and self.update_count >= 2
            and self.stationary_probability >= 0.78
        ):
            observation = np.zeros((3, 6), dtype=np.float64)
            observation[:, 3:6] = np.eye(3)
            # Apply a probability-weighted ZUPT. It anchors a genuinely
            # stationary global target while allowing a soft transition to CV.
            zupt_std = self.zero_velocity_std_m_s / max(
                0.25,
                self.stationary_probability,
            )
            self._linear_update(
                observation,
                -observation @ self.x,
                np.eye(3) * zupt_std**2,
            )
        self.update_count += 1
        self.last_measurement_timestamp_s = float(timestamp_s)

    def update_bearing(
        self,
        camera_position_ned_m: Sequence[float],
        bearing_ned_unit: Sequence[float],
        angular_std_rad: float,
        timestamp_s: float,
    ) -> None:
        self.predict(timestamp_s)
        if self.x is None or self.p is None:
            return
        camera = self._vector(camera_position_ned_m, "camera position")
        bearing = self._unit(bearing_ned_unit, "bearing")
        relative = self.x[:3] - camera
        distance = max(1.0, float(np.linalg.norm(relative)))
        perpendicular = np.eye(3) - np.outer(bearing, bearing)
        observation = np.zeros((3, 6), dtype=np.float64)
        observation[:, :3] = perpendicular
        residual = -perpendicular @ relative
        lateral_std = max(0.05, distance * float(angular_std_rad))
        covariance = np.eye(3) * lateral_std**2
        self._linear_update(observation, residual, covariance)
        self.last_bearing_timestamp_s = float(timestamp_s)

    def update_range(
        self,
        camera_position_ned_m: Sequence[float],
        range_m: float,
        range_std_m: float,
        timestamp_s: float,
    ) -> None:
        self.predict(timestamp_s)
        if self.x is None or self.p is None:
            return
        camera = self._vector(camera_position_ned_m, "camera position")
        relative = self.x[:3] - camera
        predicted = float(np.linalg.norm(relative))
        if predicted <= 1e-6:
            return
        observation = np.zeros((1, 6), dtype=np.float64)
        observation[0, :3] = relative / predicted
        residual = np.asarray([float(range_m) - predicted])
        covariance = np.asarray([[max(0.05, float(range_std_m)) ** 2]])
        self._linear_update(observation, residual, covariance)
        self.update_count += 1
        self.last_measurement_timestamp_s = float(timestamp_s)

    def estimate(
        self,
        now_s: float,
        *,
        stale_timeout_s: float = 1.5,
    ) -> FusionEstimate:
        self.predict(now_s)
        if self.x is None or self.p is None:
            return FusionEstimate(
                False, "uninitialized", float(now_s), None, None, tuple(),
                None, None, False, self.update_count,
            )
        age = (
            math.inf
            if self.last_measurement_timestamp_s is None
            else max(0.0, float(now_s) - self.last_measurement_timestamp_s)
        )
        position_std = float(np.sqrt(max(0.0, np.max(np.diag(self.p)[:3]))))
        velocity_std = float(np.sqrt(max(0.0, np.max(np.diag(self.p)[3:]))))
        valid = bool(age <= stale_timeout_s and np.all(np.isfinite(self.x)))
        velocity_valid = bool(
            valid
            and self.update_count >= self.velocity_ready_updates
            and velocity_std <= 3.0
            and (
                self.estimator_mode == "STATIONARY"
                or (1.0 - self.stationary_probability) >= 0.60
            )
        )
        velocity = self.x[3:].copy()
        if self.estimator_mode == "STATIONARY":
            velocity[:] = 0.0
        return FusionEstimate(
            valid,
            "ok" if valid else "stale",
            float(now_s),
            tuple(float(value) for value in self.x[:3]),
            tuple(float(value) for value in velocity),
            tuple(tuple(float(value) for value in row) for row in self.p),
            position_std,
            velocity_std,
            velocity_valid,
            self.update_count,
            self.estimator_mode,
            self.stationary_probability,
            1.0 - self.stationary_probability,
            None if not math.isfinite(age) else age,
        )

    def status(self, now_s: float | None = None) -> dict[str, object]:
        timestamp = (
            self.timestamp_s
            if now_s is None and self.timestamp_s is not None
            else 0.0
            if now_s is None
            else float(now_s)
        )
        estimate = self.estimate(timestamp)
        return {
            "mode": estimate.estimator_mode,
            "stationary_probability": estimate.stationary_probability,
            "moving_probability": estimate.moving_probability,
            "velocity_valid": estimate.velocity_valid,
            "position_std_m": estimate.position_std_m,
            "velocity_std_m_s": estimate.velocity_std_m_s,
            "measurement_age_s": estimate.measurement_age_s,
            "update_count": estimate.update_count,
        }

    def _update_model_probability(self, measured_speed_m_s: float) -> None:
        speed = max(0.0, float(measured_speed_m_s))
        if speed <= self.stationary_speed_m_s:
            evidence = 1.0
        elif speed >= self.moving_speed_m_s:
            evidence = 0.0
        else:
            evidence = (
                self.moving_speed_m_s - speed
            ) / (self.moving_speed_m_s - self.stationary_speed_m_s)
        alpha = 0.40
        self.stationary_probability = (
            (1.0 - alpha) * self.stationary_probability
            + alpha * evidence
        )
        if (
            self.estimator_mode == "STATIONARY"
            and self.stationary_probability < 0.38
        ):
            self.estimator_mode = "CONSTANT_VELOCITY"
        elif (
            self.estimator_mode == "CONSTANT_VELOCITY"
            and self.stationary_probability > 0.72
        ):
            self.estimator_mode = "STATIONARY"

    def _linear_update(
        self,
        observation: np.ndarray,
        residual: np.ndarray,
        covariance: np.ndarray,
    ) -> None:
        assert self.x is not None and self.p is not None
        innovation_covariance = (
            observation @ self.p @ observation.T + covariance
        )
        gain = self.p @ observation.T @ np.linalg.pinv(
            innovation_covariance
        )
        self.x = self.x + gain @ residual
        identity = np.eye(6)
        joseph = identity - gain @ observation
        self.p = (
            joseph @ self.p @ joseph.T
            + gain @ covariance @ gain.T
        )
        self.p = 0.5 * (self.p + self.p.T)

    @staticmethod
    def _vector(value: Sequence[float], name: str) -> np.ndarray:
        vector = np.asarray(value, dtype=np.float64)
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            raise ValueError(f"{name} must contain three finite values")
        return vector

    @classmethod
    def _unit(cls, value: Sequence[float], name: str) -> np.ndarray:
        vector = cls._vector(value, name)
        norm = float(np.linalg.norm(vector))
        if norm <= 1e-9:
            raise ValueError(f"{name} has zero length")
        return vector / norm
