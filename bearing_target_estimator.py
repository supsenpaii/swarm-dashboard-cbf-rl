from __future__ import annotations

import math
import os
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np


def _env_float(name: str, default: float, minimum: float, maximum: float) -> float:
    try:
        value = float(os.environ.get(name, str(default)))
    except ValueError:
        value = default
    return max(minimum, min(maximum, value))


def _env_int(name: str, default: int, minimum: int, maximum: int) -> int:
    try:
        value = int(os.environ.get(name, str(default)))
    except ValueError:
        value = default
    return max(minimum, min(maximum, value))


def _vector3(value: Sequence[float], name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain three finite values")
    return array


def _unit_vector(value: Sequence[float], name: str) -> np.ndarray:
    array = _vector3(value, name)
    norm = float(np.linalg.norm(array))
    if norm <= 1e-9:
        raise ValueError(f"{name} has zero length")
    return array / norm


@dataclass(frozen=True)
class BearingObservation:
    timestamp_s: float
    camera_position_ned_m: tuple[float, float, float]
    bearing_ned_unit: tuple[float, float, float]
    bbox_center_px: tuple[float, float]
    bbox_size_px: tuple[float, float]
    tracking_score: float
    focal_length_px: float
    pose_age_s: float = 0.0
    gimbal_age_s: float = 0.0
    frame_index: int = 0
    redetecting: bool = False
    ambiguous: bool = False
    reject_reason: str = ""

    def validated(self) -> "BearingObservation":
        timestamp = float(self.timestamp_s)
        score = float(self.tracking_score)
        focal = float(self.focal_length_px)
        pose_age = float(self.pose_age_s)
        gimbal_age = float(self.gimbal_age_s)
        bbox_values = tuple(float(value) for value in (*self.bbox_center_px, *self.bbox_size_px))
        if not math.isfinite(timestamp):
            raise ValueError("observation timestamp is invalid")
        if not 0.0 <= score <= 1.0:
            raise ValueError("tracking score must be in [0, 1]")
        if not math.isfinite(focal) or focal <= 0.0:
            raise ValueError("focal length must be positive")
        if not all(math.isfinite(value) for value in bbox_values):
            raise ValueError("bbox values must be finite")
        if bbox_values[2] <= 0.0 or bbox_values[3] <= 0.0:
            raise ValueError("bbox size must be positive")
        if not all(math.isfinite(value) and value >= 0.0 for value in (pose_age, gimbal_age)):
            raise ValueError("pose and gimbal ages must be finite and non-negative")
        position = _vector3(self.camera_position_ned_m, "camera position")
        bearing = _unit_vector(self.bearing_ned_unit, "bearing")
        return BearingObservation(
            timestamp_s=timestamp,
            camera_position_ned_m=tuple(float(value) for value in position),
            bearing_ned_unit=tuple(float(value) for value in bearing),
            bbox_center_px=(bbox_values[0], bbox_values[1]),
            bbox_size_px=(bbox_values[2], bbox_values[3]),
            tracking_score=score,
            focal_length_px=focal,
            pose_age_s=pose_age,
            gimbal_age_s=gimbal_age,
            frame_index=int(self.frame_index),
            redetecting=bool(self.redetecting),
            ambiguous=bool(self.ambiguous),
            reject_reason=str(self.reject_reason),
        )


@dataclass(frozen=True)
class TargetEstimate:
    timestamp_s: float
    state: str
    valid: bool
    reason: str
    position_ned_m: tuple[float, float, float] | None = None
    velocity_ned_m_s: tuple[float, float, float] | None = None
    covariance: tuple[tuple[float, ...], ...] = field(default_factory=tuple)
    range_m: float | None = None
    range_std_m: float | None = None
    reprojection_error_px: float | None = None
    baseline_m: float = 0.0
    intersection_angle_deg: float = 0.0
    condition_number: float | None = None
    observation_count: int = 0
    observability_score: float = 0.0
    estimate_age_ms: float | None = None
    bootstrap_progress: float = 0.0
    velocity_valid: bool = False
    estimator_mode: str = "UNKNOWN"
    stationary_probability: float | None = None
    moving_probability: float | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "timestamp_s": self.timestamp_s,
            "state": self.state,
            "valid": self.valid,
            "reason": self.reason,
            "position_ned_m": list(self.position_ned_m) if self.position_ned_m else None,
            "velocity_ned_m_s": (
                list(self.velocity_ned_m_s) if self.velocity_ned_m_s else None
            ),
            "covariance": [list(row) for row in self.covariance],
            "range_m": self.range_m,
            "range_std_m": self.range_std_m,
            "reprojection_error_px": self.reprojection_error_px,
            "baseline_m": self.baseline_m,
            "intersection_angle_deg": self.intersection_angle_deg,
            "condition_number": self.condition_number,
            "observation_count": self.observation_count,
            "observability_score": self.observability_score,
            "estimate_age_ms": self.estimate_age_ms,
            "bootstrap_progress": self.bootstrap_progress,
            "velocity_valid": self.velocity_valid,
            "estimator_mode": self.estimator_mode,
            "stationary_probability": self.stationary_probability,
            "moving_probability": self.moving_probability,
        }


@dataclass(frozen=True)
class BearingTargetEstimatorConfig:
    minimum_observations: int = 6
    # At 30 FPS, 90 samples preserve three seconds of ego-motion. A 30-frame
    # deque only retained about 0.15 m of a physically ramped 0.5 m/s
    # bootstrap and could never satisfy the 0.50 m metric-baseline gate.
    maximum_observations: int = 90
    observation_window_s: float = 5.0
    minimum_tracking_score: float = 0.70
    maximum_pose_age_s: float = 0.50
    maximum_gimbal_age_s: float = 0.50
    minimum_baseline_m: float = 0.50
    minimum_intersection_angle_deg: float = 3.0
    maximum_reprojection_error_px: float = 3.0
    minimum_range_m: float = 1.0
    maximum_range_m: float = 80.0
    maximum_range_std_m: float = 1.0
    maximum_range_relative_std: float = 0.15
    stale_timeout_s: float = 0.50
    maximum_condition_number: float = 1.0e6
    bearing_noise_px: float = 1.5
    acceleration_noise_m_s2: float = 1.5
    innovation_gate_sigma: float = 4.0
    bootstrap_lateral_distance_m: float = 0.75
    bootstrap_lateral_speed_m_s: float = 0.50
    bootstrap_timeout_s: float = 4.0

    @classmethod
    def from_environment(cls) -> "BearingTargetEstimatorConfig":
        return cls(
            minimum_observations=_env_int(
                "SWARM_BEARING_MIN_OBSERVATIONS", 6, 3, 30
            ),
            maximum_observations=_env_int(
                "SWARM_BEARING_MAX_OBSERVATIONS", 90, 6, 120
            ),
            observation_window_s=_env_float(
                "SWARM_BEARING_WINDOW_S", 5.0, 0.25, 10.0
            ),
            minimum_tracking_score=_env_float(
                "SWARM_BEARING_MIN_TRACKING_SCORE", 0.70, 0.0, 1.0
            ),
            maximum_pose_age_s=_env_float(
                "SWARM_BEARING_MAX_POSE_AGE_S", 0.50, 0.02, 2.0
            ),
            maximum_gimbal_age_s=_env_float(
                "SWARM_BEARING_MAX_GIMBAL_AGE_S", 0.50, 0.02, 2.0
            ),
            minimum_baseline_m=_env_float(
                "SWARM_BEARING_MIN_BASELINE_M", 0.50, 0.05, 10.0
            ),
            minimum_intersection_angle_deg=_env_float(
                "SWARM_BEARING_MIN_INTERSECTION_DEG", 3.0, 0.25, 30.0
            ),
            maximum_reprojection_error_px=_env_float(
                "SWARM_BEARING_MAX_REPROJECTION_PX", 3.0, 0.25, 50.0
            ),
            minimum_range_m=_env_float(
                "SWARM_BEARING_MIN_RANGE_M", 1.0, 0.1, 20.0
            ),
            maximum_range_m=_env_float(
                "SWARM_BEARING_MAX_RANGE_M", 80.0, 2.0, 500.0
            ),
            maximum_range_std_m=_env_float(
                "SWARM_BEARING_MAX_RANGE_STD_M", 1.0, 0.05, 20.0
            ),
            maximum_range_relative_std=_env_float(
                "SWARM_BEARING_MAX_RANGE_REL_STD", 0.15, 0.01, 1.0
            ),
            stale_timeout_s=_env_float(
                "SWARM_BEARING_STALE_TIMEOUT_S", 0.50, 0.05, 5.0
            ),
            maximum_condition_number=_env_float(
                "SWARM_BEARING_MAX_CONDITION", 1.0e6, 10.0, 1.0e12
            ),
            bearing_noise_px=_env_float(
                "SWARM_BEARING_NOISE_PX", 1.5, 0.1, 20.0
            ),
            acceleration_noise_m_s2=_env_float(
                "SWARM_BEARING_ACCEL_NOISE_M_S2", 1.5, 0.01, 20.0
            ),
            innovation_gate_sigma=_env_float(
                "SWARM_BEARING_INNOVATION_GATE_SIGMA", 4.0, 1.0, 10.0
            ),
            bootstrap_lateral_distance_m=_env_float(
                "SWARM_BEARING_BOOTSTRAP_LATERAL_DISTANCE_M", 0.75, 0.20, 3.0
            ),
            bootstrap_lateral_speed_m_s=_env_float(
                "SWARM_BEARING_BOOTSTRAP_LATERAL_SPEED_M_S", 0.50, 0.10, 2.0
            ),
            bootstrap_timeout_s=_env_float(
                "SWARM_BEARING_BOOTSTRAP_TIMEOUT_S", 4.0, 2.0, 8.0
            ),
        )


@dataclass(frozen=True)
class _GeometryResult:
    valid: bool
    reason: str
    position: np.ndarray | None
    covariance: np.ndarray | None
    baseline_m: float
    intersection_angle_deg: float
    reprojection_error_px: float | None
    condition_number: float
    observation_count: int


class BearingTargetEstimator:
    """Bearing-only target position/velocity estimator.

    A robust multi-ray least-squares solve bootstraps metric position. Once
    initialized, a constant-velocity EKF consumes individual unit bearings.
    Metric scale comes exclusively from camera translation in NED.
    """

    def __init__(
        self,
        config: BearingTargetEstimatorConfig | None = None,
    ) -> None:
        self.config = config or BearingTargetEstimatorConfig.from_environment()
        self.observations: deque[BearingObservation] = deque(
            maxlen=self.config.maximum_observations
        )
        self.state_vector: np.ndarray | None = None
        self.state_covariance: np.ndarray | None = None
        self.filter_timestamp_s: float | None = None
        self.last_good_timestamp_s: float | None = None
        self.accepted_filter_updates = 0
        self.last_prediction_dt_s: float | None = None
        self.last_invalid_prediction_dt_s: float | None = None
        self.last_invalid_prediction_geometry_reason = ""
        self.last_estimate = TargetEstimate(
            timestamp_s=0.0,
            state="IDLE",
            valid=False,
            reason="idle",
        )

    def reset(self) -> None:
        self.observations.clear()
        self.state_vector = None
        self.state_covariance = None
        self.filter_timestamp_s = None
        self.last_good_timestamp_s = None
        self.accepted_filter_updates = 0
        self.last_estimate = TargetEstimate(
            timestamp_s=0.0,
            state="IDLE",
            valid=False,
            reason="idle",
        )

    def invalidate(self, timestamp_s: float, reason: str) -> TargetEstimate:
        now = float(timestamp_s)
        age_ms = (
            None
            if self.last_good_timestamp_s is None
            else max(0.0, now - self.last_good_timestamp_s) * 1000.0
        )
        state = "LOST" if reason in {"tracking_lost", "tracking_invalid"} else "DEGRADED"
        self.last_estimate = TargetEstimate(
            timestamp_s=now,
            state=state,
            valid=False,
            reason=str(reason),
            position_ned_m=self._position_tuple(),
            velocity_ned_m_s=self._velocity_tuple(),
            covariance=self._covariance_tuple(),
            observation_count=len(self.observations),
            estimate_age_ms=age_ms,
            velocity_valid=self.accepted_filter_updates >= 3,
        )
        return self.last_estimate

    def update(
        self,
        observation: BearingObservation,
        *,
        now_s: float | None = None,
    ) -> TargetEstimate:
        try:
            observation = observation.validated()
        except ValueError as error:
            return self.invalidate(
                float(now_s if now_s is not None else observation.timestamp_s),
                f"invalid_observation:{error}",
            )
        now = float(observation.timestamp_s if now_s is None else now_s)
        invalid_reason = self._observation_reject_reason(observation)
        if invalid_reason:
            return self.invalidate(now, invalid_reason)

        if self.observations and observation.timestamp_s <= self.observations[-1].timestamp_s:
            return self.invalidate(now, "out_of_order_observation")
        self.observations.append(observation)
        self._prune_observations(observation.timestamp_s)
        geometry = self._triangulate(list(self.observations))

        if self.state_vector is None:
            if not geometry.valid or geometry.position is None:
                return self._bootstrap_output(now, geometry)
            self._initialize_filter(observation.timestamp_s, geometry)
        else:
            predicted = self._predict_filter(observation.timestamp_s)
            if not predicted:
                if geometry.valid and geometry.position is not None:
                    # A delayed source frame may exceed the EKF propagation
                    # horizon while the retained multi-ray geometry is still
                    # fully observable. Re-bootstrap from that independently
                    # gated metric solve instead of destroying scale and then
                    # demanding new aircraft motion.
                    self._initialize_filter(observation.timestamp_s, geometry)
                else:
                    self.last_invalid_prediction_geometry_reason = (
                        geometry.reason
                    )
                    self._clear_filter()
                    return self.invalidate(now, "invalid_dt")
            if not self._update_filter_with_bearing(observation):
                return self._output_from_filter(
                    now,
                    observation,
                    geometry,
                    valid=False,
                    reason="innovation_rejected",
                    state="DEGRADED",
                )

        return self._output_from_filter(
            now,
            observation,
            geometry,
            valid=True,
            reason="ready",
            state="VALID",
        )

    def status(self, now_s: float | None = None) -> dict[str, Any]:
        now = (
            float(now_s)
            if now_s is not None
            else (
                self.last_estimate.timestamp_s
                if self.last_estimate.timestamp_s > 0.0
                else 0.0
            )
        )
        estimate = self.last_estimate
        if (
            estimate.valid
            and self.last_good_timestamp_s is not None
            and now - self.last_good_timestamp_s > self.config.stale_timeout_s
        ):
            estimate = self.invalidate(now, "stale_estimate")
        result = estimate.as_dict()
        result["filter_timestamp_s"] = self.filter_timestamp_s
        result["last_prediction_dt_s"] = self.last_prediction_dt_s
        result["last_invalid_prediction_dt_s"] = (
            self.last_invalid_prediction_dt_s
        )
        result["last_invalid_prediction_geometry_reason"] = (
            self.last_invalid_prediction_geometry_reason
        )
        result["bootstrap_guidance"] = self.bootstrap_guidance()
        return result

    def bootstrap_guidance(self) -> dict[str, Any]:
        """Describe a lateral bootstrap without authorizing flight motion."""

        estimate = self.last_estimate
        required = bool(
            not estimate.valid
            and estimate.state in {"TRACKING_2D", "BOOTSTRAPPING", "DEGRADED"}
        )
        return {
            "required": required,
            "command_authorized": False,
            "direction": "safety_layer_selects_body_left_or_right",
            "desired_lateral_distance_m": (
                self.config.bootstrap_lateral_distance_m
            ),
            "remaining_lateral_distance_m": max(
                0.0,
                self.config.bootstrap_lateral_distance_m - estimate.baseline_m,
            ),
            "maximum_lateral_speed_m_s": (
                self.config.bootstrap_lateral_speed_m_s
            ),
            "timeout_s": self.config.bootstrap_timeout_s,
            "abort_on_tracking_loss": True,
            "abort_on_stale_pose": True,
            "abort_on_failsafe": True,
        }

    def _observation_reject_reason(self, observation: BearingObservation) -> str:
        cfg = self.config
        if observation.redetecting:
            return "redetecting"
        if observation.ambiguous:
            return "ambiguous_tracking"
        if observation.reject_reason:
            return f"tracker_rejected:{observation.reject_reason}"
        if observation.tracking_score < cfg.minimum_tracking_score:
            return "low_tracking_score"
        if observation.pose_age_s > cfg.maximum_pose_age_s:
            return "stale_pose"
        if observation.gimbal_age_s > cfg.maximum_gimbal_age_s:
            return "stale_gimbal"
        return ""

    def _prune_observations(self, newest_timestamp_s: float) -> None:
        cutoff = newest_timestamp_s - self.config.observation_window_s
        while self.observations and self.observations[0].timestamp_s < cutoff:
            self.observations.popleft()

    @staticmethod
    def _baseline(observations: list[BearingObservation]) -> float:
        if len(observations) < 2:
            return 0.0
        positions = np.asarray(
            [observation.camera_position_ned_m for observation in observations],
            dtype=np.float64,
        )
        differences = positions[:, None, :] - positions[None, :, :]
        return float(np.max(np.linalg.norm(differences, axis=2)))

    @staticmethod
    def _intersection_angle(observations: list[BearingObservation]) -> float:
        if len(observations) < 2:
            return 0.0
        bearings = np.asarray(
            [observation.bearing_ned_unit for observation in observations],
            dtype=np.float64,
        )
        dots = np.clip(bearings @ bearings.T, -1.0, 1.0)
        return float(np.degrees(np.max(np.arccos(dots))))

    @staticmethod
    def _weighted_solve(
        observations: list[BearingObservation],
    ) -> tuple[np.ndarray, np.ndarray, float]:
        normal = np.zeros((3, 3), dtype=np.float64)
        rhs = np.zeros(3, dtype=np.float64)
        identity = np.eye(3, dtype=np.float64)
        for observation in observations:
            direction = np.asarray(observation.bearing_ned_unit, dtype=np.float64)
            camera = np.asarray(observation.camera_position_ned_m, dtype=np.float64)
            projector = identity - np.outer(direction, direction)
            weight = max(0.05, float(observation.tracking_score)) ** 2
            normal += weight * projector
            rhs += weight * projector @ camera
        eigenvalues = np.linalg.eigvalsh(normal)
        smallest = max(1e-15, float(eigenvalues[0]))
        condition = float(eigenvalues[-1]) / smallest
        position = np.linalg.solve(normal, rhs)
        return position, normal, condition

    @staticmethod
    def _perpendicular_residuals(
        position: np.ndarray,
        observations: list[BearingObservation],
    ) -> np.ndarray:
        identity = np.eye(3, dtype=np.float64)
        residuals = []
        for observation in observations:
            direction = np.asarray(observation.bearing_ned_unit, dtype=np.float64)
            camera = np.asarray(observation.camera_position_ned_m, dtype=np.float64)
            residuals.append(
                float(np.linalg.norm((identity - np.outer(direction, direction)) @ (position - camera)))
            )
        return np.asarray(residuals, dtype=np.float64)

    @staticmethod
    def _reprojection_errors_px(
        position: np.ndarray,
        observations: list[BearingObservation],
    ) -> np.ndarray:
        errors: list[float] = []
        for observation in observations:
            camera = np.asarray(observation.camera_position_ned_m, dtype=np.float64)
            predicted = position - camera
            predicted_norm = float(np.linalg.norm(predicted))
            if predicted_norm <= 1e-9:
                return np.full(len(observations), math.inf, dtype=np.float64)
            predicted /= predicted_norm
            measured = np.asarray(observation.bearing_ned_unit, dtype=np.float64)
            angle = math.acos(float(np.clip(np.dot(predicted, measured), -1.0, 1.0)))
            errors.append(angle * observation.focal_length_px)
        return np.asarray(errors, dtype=np.float64)

    @classmethod
    def _reprojection_error_px(
        cls,
        position: np.ndarray,
        observations: list[BearingObservation],
    ) -> float:
        errors = cls._reprojection_errors_px(position, observations)
        return float(math.sqrt(float(np.mean(errors ** 2))))

    def _triangulate(
        self,
        observations: list[BearingObservation],
    ) -> _GeometryResult:
        cfg = self.config
        baseline = self._baseline(observations)
        angle = self._intersection_angle(observations)
        count = len(observations)
        if count < cfg.minimum_observations:
            return _GeometryResult(
                False, "insufficient_observations", None, None,
                baseline, angle, None, math.inf, count,
            )
        if baseline < cfg.minimum_baseline_m:
            return _GeometryResult(
                False, "insufficient_baseline", None, None,
                baseline, angle, None, math.inf, count,
            )
        if angle < cfg.minimum_intersection_angle_deg:
            return _GeometryResult(
                False, "weak_intersection_angle", None, None,
                baseline, angle, None, math.inf, count,
            )
        # Establish a consensus before the full least-squares solve. A single
        # bad bbox centre can otherwise drag the initial solution far enough
        # that residual MAD classifies every good ray as an outlier.
        consensus = observations
        best_inliers: list[BearingObservation] = []
        best_median = math.inf
        for first_index in range(len(observations) - 1):
            first = observations[first_index]
            first_direction = np.asarray(first.bearing_ned_unit)
            for second in observations[first_index + 1:]:
                second_direction = np.asarray(second.bearing_ned_unit)
                pair_angle = math.degrees(
                    math.acos(
                        float(
                            np.clip(
                                np.dot(first_direction, second_direction),
                                -1.0,
                                1.0,
                            )
                        )
                    )
                )
                if pair_angle < max(0.5, cfg.minimum_intersection_angle_deg * 0.25):
                    continue
                try:
                    candidate, _, _ = self._weighted_solve([first, second])
                except np.linalg.LinAlgError:
                    continue
                candidate_range = float(
                    np.linalg.norm(
                        candidate
                        - np.asarray(observations[-1].camera_position_ned_m)
                    )
                )
                median_focal = float(
                    np.median(
                        [observation.focal_length_px for observation in observations]
                    )
                )
                threshold = max(
                    0.12,
                    min(
                        1.0,
                        candidate_range
                        * 3.5
                        * cfg.bearing_noise_px
                        / max(1.0, median_focal),
                    ),
                )
                candidate_residuals = self._perpendicular_residuals(
                    candidate, observations
                )
                inliers = [
                    observation
                    for observation, residual in zip(
                        observations, candidate_residuals
                    )
                    if residual <= threshold
                ]
                inlier_median = (
                    float(
                        np.median(
                            [
                                residual
                                for residual in candidate_residuals
                                if residual <= threshold
                            ]
                        )
                    )
                    if inliers
                    else math.inf
                )
                if (
                    len(inliers) > len(best_inliers)
                    or (
                        len(inliers) == len(best_inliers)
                        and inlier_median < best_median
                    )
                ):
                    best_inliers = inliers
                    best_median = inlier_median
        if len(best_inliers) >= cfg.minimum_observations:
            consensus = best_inliers
        try:
            position, normal, condition = self._weighted_solve(consensus)
        except np.linalg.LinAlgError:
            return _GeometryResult(
                False, "degenerate_geometry", None, None,
                baseline, angle, None, math.inf, count,
            )
        if not np.all(np.isfinite(position)) or condition > cfg.maximum_condition_number:
            return _GeometryResult(
                False, "degenerate_geometry", None, None,
                baseline, angle, None, condition, count,
            )

        residuals = self._perpendicular_residuals(position, consensus)
        median = float(np.median(residuals))
        mad = float(np.median(np.abs(residuals - median)))
        threshold = max(0.10, median + 3.5 * max(0.01, 1.4826 * mad))
        inliers = [
            observation
            for observation, residual in zip(consensus, residuals)
            if residual <= threshold
        ]
        if len(inliers) < cfg.minimum_observations:
            return _GeometryResult(
                False, "too_many_outliers", None, None,
                baseline, angle, None, condition, len(inliers),
            )
        if len(inliers) != len(consensus):
            try:
                position, normal, condition = self._weighted_solve(inliers)
            except np.linalg.LinAlgError:
                return _GeometryResult(
                    False, "degenerate_geometry", None, None,
                    baseline, angle, None, math.inf, len(inliers),
                )
            baseline = self._baseline(inliers)
            angle = self._intersection_angle(inliers)
            residuals = self._perpendicular_residuals(position, inliers)

        # Perpendicular residuals are expressed in metres and become permissive
        # at long range. Refine the consensus in the measurement domain too:
        # remove only the worst pixel ray, re-solve, and retain at least 65% of
        # the metric-consensus set. The configured reprojection gate itself is
        # never relaxed.
        minimum_pixel_inliers = max(
            cfg.minimum_observations,
            int(math.ceil(0.65 * len(inliers))),
        )
        reprojection = self._reprojection_error_px(position, inliers)
        while (
            reprojection > cfg.maximum_reprojection_error_px
            and len(inliers) > minimum_pixel_inliers
        ):
            pixel_errors = self._reprojection_errors_px(position, inliers)
            worst_index = int(np.argmax(pixel_errors))
            candidate_inliers = [
                observation
                for index, observation in enumerate(inliers)
                if index != worst_index
            ]
            try:
                candidate_position, candidate_normal, candidate_condition = (
                    self._weighted_solve(candidate_inliers)
                )
            except np.linalg.LinAlgError:
                break
            if (
                not np.all(np.isfinite(candidate_position))
                or candidate_condition > cfg.maximum_condition_number
            ):
                break
            inliers = candidate_inliers
            position = candidate_position
            normal = candidate_normal
            condition = candidate_condition
            reprojection = self._reprojection_error_px(position, inliers)

        baseline = self._baseline(inliers)
        angle = self._intersection_angle(inliers)
        residuals = self._perpendicular_residuals(position, inliers)
        if baseline < cfg.minimum_baseline_m:
            return _GeometryResult(
                False, "insufficient_baseline", None, None,
                baseline, angle, reprojection, condition, len(inliers),
            )
        if angle < cfg.minimum_intersection_angle_deg:
            return _GeometryResult(
                False, "weak_intersection_angle", None, None,
                baseline, angle, reprojection, condition, len(inliers),
            )

        depths = [
            float(
                np.dot(
                    position - np.asarray(observation.camera_position_ned_m),
                    np.asarray(observation.bearing_ned_unit),
                )
            )
            for observation in inliers
        ]
        if any(depth <= 0.0 for depth in depths):
            return _GeometryResult(
                False, "target_behind_camera", None, None,
                baseline, angle, None, condition, len(inliers),
            )
        latest_camera = np.asarray(inliers[-1].camera_position_ned_m)
        current_range = float(np.linalg.norm(position - latest_camera))
        if not cfg.minimum_range_m <= current_range <= cfg.maximum_range_m:
            return _GeometryResult(
                False, "range_out_of_bounds", None, None,
                baseline, angle, None, condition, len(inliers),
            )
        if reprojection > cfg.maximum_reprojection_error_px:
            return _GeometryResult(
                False, "reprojection_error", position, None,
                baseline, angle, reprojection, condition, len(inliers),
            )

        degrees_of_freedom = max(1, 2 * len(inliers) - 3)
        residual_variance = max(
            0.01 ** 2,
            float(np.sum(residuals ** 2)) / degrees_of_freedom,
        )
        try:
            covariance = residual_variance * np.linalg.inv(normal)
        except np.linalg.LinAlgError:
            covariance = residual_variance * np.linalg.pinv(normal)
        covariance += np.eye(3) * 0.02 ** 2
        return _GeometryResult(
            True, "ready", position, covariance,
            baseline, angle, reprojection, condition, len(inliers),
        )

    def _initialize_filter(
        self,
        timestamp_s: float,
        geometry: _GeometryResult,
    ) -> None:
        assert geometry.position is not None
        position_covariance = (
            geometry.covariance
            if geometry.covariance is not None
            else np.eye(3, dtype=np.float64)
        )
        self.state_vector = np.zeros(6, dtype=np.float64)
        self.state_vector[:3] = geometry.position
        self.state_covariance = np.zeros((6, 6), dtype=np.float64)
        self.state_covariance[:3, :3] = position_covariance
        self.state_covariance[3:, 3:] = np.eye(3) * 4.0
        self.filter_timestamp_s = float(timestamp_s)
        self.accepted_filter_updates = 0

    def _clear_filter(self) -> None:
        self.state_vector = None
        self.state_covariance = None
        self.filter_timestamp_s = None
        self.accepted_filter_updates = 0

    def _predict_filter(self, timestamp_s: float) -> bool:
        assert self.state_vector is not None
        assert self.state_covariance is not None
        assert self.filter_timestamp_s is not None
        dt = float(timestamp_s) - self.filter_timestamp_s
        self.last_prediction_dt_s = dt
        if dt <= 1e-4 or dt > 1.0:
            self.last_invalid_prediction_dt_s = dt
            return False
        transition = np.eye(6, dtype=np.float64)
        transition[:3, 3:] = np.eye(3) * dt
        noise = self.config.acceleration_noise_m_s2 ** 2
        process = np.zeros((6, 6), dtype=np.float64)
        process[:3, :3] = np.eye(3) * (0.25 * dt ** 4 * noise)
        process[:3, 3:] = np.eye(3) * (0.5 * dt ** 3 * noise)
        process[3:, :3] = process[:3, 3:]
        process[3:, 3:] = np.eye(3) * (dt ** 2 * noise)
        self.state_vector = transition @ self.state_vector
        self.state_covariance = (
            transition @ self.state_covariance @ transition.T + process
        )
        self.filter_timestamp_s = float(timestamp_s)
        return True

    def _update_filter_with_bearing(
        self,
        observation: BearingObservation,
    ) -> bool:
        assert self.state_vector is not None
        assert self.state_covariance is not None
        camera = np.asarray(observation.camera_position_ned_m, dtype=np.float64)
        delta = self.state_vector[:3] - camera
        distance = float(np.linalg.norm(delta))
        if distance <= 1e-6:
            return False
        predicted = delta / distance
        measured = np.asarray(observation.bearing_ned_unit, dtype=np.float64)
        residual = measured - predicted
        position_jacobian = (
            np.eye(3, dtype=np.float64) - np.outer(predicted, predicted)
        ) / distance
        measurement_jacobian = np.zeros((3, 6), dtype=np.float64)
        measurement_jacobian[:, :3] = position_jacobian
        angular_sigma = max(
            1e-5,
            self.config.bearing_noise_px / observation.focal_length_px,
        )
        measurement_noise = np.eye(3, dtype=np.float64) * angular_sigma ** 2
        innovation_covariance = (
            measurement_jacobian
            @ self.state_covariance
            @ measurement_jacobian.T
            + measurement_noise
        )
        try:
            inverse_innovation = np.linalg.inv(innovation_covariance)
        except np.linalg.LinAlgError:
            inverse_innovation = np.linalg.pinv(innovation_covariance)
        mahalanobis = float(residual.T @ inverse_innovation @ residual)
        if (
            not math.isfinite(mahalanobis)
            or mahalanobis > self.config.innovation_gate_sigma ** 2
        ):
            return False
        gain = (
            self.state_covariance
            @ measurement_jacobian.T
            @ inverse_innovation
        )
        self.state_vector = self.state_vector + gain @ residual
        identity = np.eye(6, dtype=np.float64)
        correction = identity - gain @ measurement_jacobian
        self.state_covariance = (
            correction @ self.state_covariance @ correction.T
            + gain @ measurement_noise @ gain.T
        )
        self.state_covariance = 0.5 * (
            self.state_covariance + self.state_covariance.T
        )
        self.accepted_filter_updates += 1
        return True

    def _bootstrap_output(
        self,
        now: float,
        geometry: _GeometryResult,
    ) -> TargetEstimate:
        progress_observations = min(
            1.0,
            geometry.observation_count / max(1, self.config.minimum_observations),
        )
        progress_baseline = min(
            1.0,
            geometry.baseline_m / max(1e-6, self.config.minimum_baseline_m),
        )
        progress_angle = min(
            1.0,
            geometry.intersection_angle_deg
            / max(1e-6, self.config.minimum_intersection_angle_deg),
        )
        progress = min(progress_observations, progress_baseline, progress_angle)
        state = "TRACKING_2D" if geometry.observation_count < 2 else "BOOTSTRAPPING"
        self.last_estimate = TargetEstimate(
            timestamp_s=now,
            state=state,
            valid=False,
            reason=geometry.reason,
            baseline_m=geometry.baseline_m,
            intersection_angle_deg=geometry.intersection_angle_deg,
            condition_number=geometry.condition_number,
            reprojection_error_px=geometry.reprojection_error_px,
            observation_count=geometry.observation_count,
            observability_score=self._observability_score(geometry),
            bootstrap_progress=progress,
        )
        return self.last_estimate

    def _output_from_filter(
        self,
        now: float,
        observation: BearingObservation,
        geometry: _GeometryResult,
        *,
        valid: bool,
        reason: str,
        state: str,
    ) -> TargetEstimate:
        assert self.state_vector is not None
        assert self.state_covariance is not None
        position = self.state_vector[:3]
        velocity = self.state_vector[3:]
        camera = np.asarray(observation.camera_position_ned_m, dtype=np.float64)
        line = position - camera
        range_m = float(np.linalg.norm(line))
        if range_m <= 1e-9:
            return self.invalidate(now, "invalid_range")
        radial = line / range_m
        range_variance = float(radial.T @ self.state_covariance[:3, :3] @ radial)
        range_std = math.sqrt(max(0.0, range_variance))
        current_reprojection = self._reprojection_error_px(
            position,
            [observation],
        )
        cfg = self.config
        quality_reason = reason
        quality_valid = bool(valid)
        # Baseline and ray-intersection angle are metric-scale bootstrap gates.
        # Once initialized, the EKF consumes individual bearings and must not
        # become invalid merely because the follower pauses for Follow Target
        # prestream.  At runtime, innovation, current-bearing reprojection,
        # range bounds and covariance remain the continuous safety gates.
        if current_reprojection > cfg.maximum_reprojection_error_px:
            quality_valid, quality_reason = False, "reprojection_error"
        elif not cfg.minimum_range_m <= range_m <= cfg.maximum_range_m:
            quality_valid, quality_reason = False, "range_out_of_bounds"
        elif (
            range_std > cfg.maximum_range_std_m
            or range_std / range_m > cfg.maximum_range_relative_std
        ):
            quality_valid, quality_reason = False, "range_uncertainty"

        if quality_valid:
            self.last_good_timestamp_s = observation.timestamp_s
            state = "VALID"
            quality_reason = "ready"
        else:
            state = "DEGRADED"
        age_ms = (
            None
            if self.last_good_timestamp_s is None
            else max(0.0, now - self.last_good_timestamp_s) * 1000.0
        )
        self.last_estimate = TargetEstimate(
            timestamp_s=observation.timestamp_s,
            state=state,
            valid=quality_valid,
            reason=quality_reason,
            position_ned_m=tuple(float(value) for value in position),
            velocity_ned_m_s=tuple(float(value) for value in velocity),
            covariance=self._covariance_tuple(),
            range_m=range_m,
            range_std_m=range_std,
            reprojection_error_px=current_reprojection,
            baseline_m=geometry.baseline_m,
            intersection_angle_deg=geometry.intersection_angle_deg,
            condition_number=geometry.condition_number,
            observation_count=len(self.observations),
            observability_score=self._observability_score(
                geometry,
                range_std_m=range_std,
                range_m=range_m,
                reprojection_error_px=current_reprojection,
            ),
            estimate_age_ms=age_ms,
            bootstrap_progress=1.0,
            velocity_valid=self.accepted_filter_updates >= 3,
        )
        return self.last_estimate

    def _observability_score(
        self,
        geometry: _GeometryResult,
        *,
        range_std_m: float | None = None,
        range_m: float | None = None,
        reprojection_error_px: float | None = None,
    ) -> float:
        cfg = self.config
        components = [
            min(1.0, geometry.observation_count / max(1, cfg.minimum_observations)),
            min(1.0, geometry.baseline_m / max(1e-6, cfg.minimum_baseline_m)),
            min(
                1.0,
                geometry.intersection_angle_deg
                / max(1e-6, cfg.minimum_intersection_angle_deg),
            ),
        ]
        reprojection = (
            reprojection_error_px
            if reprojection_error_px is not None
            else geometry.reprojection_error_px
        )
        if reprojection is not None:
            components.append(
                max(0.0, 1.0 - reprojection / cfg.maximum_reprojection_error_px)
            )
        if range_std_m is not None and range_m is not None and range_m > 0.0:
            relative = range_std_m / range_m
            components.append(
                max(0.0, 1.0 - relative / cfg.maximum_range_relative_std)
            )
        return float(max(0.0, min(1.0, sum(components) / len(components))))

    def _position_tuple(self) -> tuple[float, float, float] | None:
        if self.state_vector is None:
            return None
        return tuple(float(value) for value in self.state_vector[:3])

    def _velocity_tuple(self) -> tuple[float, float, float] | None:
        if self.state_vector is None:
            return None
        return tuple(float(value) for value in self.state_vector[3:])

    def _covariance_tuple(self) -> tuple[tuple[float, ...], ...]:
        if self.state_covariance is None:
            return ()
        return tuple(
            tuple(float(value) for value in row)
            for row in self.state_covariance
        )
