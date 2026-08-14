from __future__ import annotations

import math
from dataclasses import dataclass, replace
from collections import deque

import numpy as np


@dataclass(frozen=True)
class MetricCalibration:
    valid: bool
    reason: str
    scale: float | None = None
    offset: float | None = None
    inlier_count: int = 0
    anchor_count: int = 0
    residual_m_inv: float | None = None
    raw_scale: float | None = None
    raw_offset: float | None = None
    condition_number: float | None = None
    parameter_uncertainty: float | None = None
    stable_samples: int = 0
    stable: bool = False
    measurement_accepted: bool = False
    parameter_covariance_ab: tuple[tuple[float, float], ...] = ()
    residual_std_m_inv: float | None = None
    anchor_inverse_depth_quantiles: tuple[float, ...] = ()
    metric_depth_quantiles_m: tuple[float, ...] = ()
    change_point_reseeded: bool = False


class MetricDepthCalibrator:
    """Robustly fit inverse metric depth as `a * relative + b`."""

    def __init__(
        self,
        *,
        minimum_anchors: int = 12,
        ransac_iterations: int = 80,
        residual_threshold_m_inv: float = 0.025,
        minimum_inlier_fraction: float = 0.45,
        ema_alpha: float = 0.25,
        random_seed: int = 7,
        temporal_window: int = 5,
        stable_samples_required: int = 3,
        maximum_scale_step_fraction: float = 0.35,
        maximum_offset_step_m_inv: float = 0.08,
        maximum_condition_number: float = 1.0e5,
        temporal_uncertainty_floor_fraction: float = 0.35,
        change_point_samples_required: int = 5,
        change_point_scale_dispersion_fraction: float = 0.08,
        change_point_offset_dispersion_m_inv: float = 0.02,
    ) -> None:
        self.minimum_anchors = max(4, int(minimum_anchors))
        self.ransac_iterations = max(10, int(ransac_iterations))
        self.residual_threshold = max(1e-5, float(residual_threshold_m_inv))
        self.minimum_inlier_fraction = max(
            0.1,
            min(1.0, float(minimum_inlier_fraction)),
        )
        self.ema_alpha = max(0.01, min(1.0, float(ema_alpha)))
        self._rng = np.random.default_rng(random_seed)
        self.temporal_window = max(3, int(temporal_window))
        self.stable_samples_required = max(2, int(stable_samples_required))
        self.maximum_scale_step_fraction = max(
            0.05,
            float(maximum_scale_step_fraction),
        )
        self.maximum_offset_step_m_inv = max(
            0.005,
            float(maximum_offset_step_m_inv),
        )
        self.maximum_condition_number = max(
            10.0,
            float(maximum_condition_number),
        )
        self.temporal_uncertainty_floor_fraction = max(
            0.1,
            min(1.0, float(temporal_uncertainty_floor_fraction)),
        )
        self.change_point_samples_required = max(
            3,
            int(change_point_samples_required),
        )
        self.change_point_scale_dispersion_fraction = max(
            0.005,
            float(change_point_scale_dispersion_fraction),
        )
        self.change_point_offset_dispersion_m_inv = max(
            0.001,
            float(change_point_offset_dispersion_m_inv),
        )
        self.scale: float | None = None
        self.offset: float | None = None
        self._scale_history: deque[float] = deque(maxlen=self.temporal_window)
        self._offset_history: deque[float] = deque(maxlen=self.temporal_window)
        self._residual_history: deque[float] = deque(
            maxlen=self.temporal_window
        )
        self.stable_samples = 0
        self.last = MetricCalibration(False, "not_calibrated")
        self.last_valid = MetricCalibration(False, "not_calibrated")
        self._change_point_candidates: deque[tuple[float, float]] = deque(
            maxlen=max(3, self.change_point_samples_required),
        )
        self._change_point_state = "idle"
        self._change_point_scale_dispersion_fraction: float | None = None
        self._change_point_offset_dispersion_m_inv: float | None = None
        self._change_point_reseed_count = 0

    def reset(self) -> None:
        self.scale = None
        self.offset = None
        self._scale_history.clear()
        self._offset_history.clear()
        self._residual_history.clear()
        self.stable_samples = 0
        self.last = MetricCalibration(False, "not_calibrated")
        self.last_valid = MetricCalibration(False, "not_calibrated")
        self._change_point_candidates.clear()
        self._change_point_state = "idle"
        self._change_point_scale_dispersion_fraction = None
        self._change_point_offset_dispersion_m_inv = None
        self._change_point_reseed_count = 0

    def fit(
        self,
        relative_inverse_depth: np.ndarray,
        metric_optical_depth_m: np.ndarray,
    ) -> MetricCalibration:
        q = np.asarray(relative_inverse_depth, dtype=np.float64).reshape(-1)
        depth = np.asarray(metric_optical_depth_m, dtype=np.float64).reshape(-1)
        valid = (
            np.isfinite(q)
            & np.isfinite(depth)
            & (q > 0.0)
            & (depth > 0.1)
        )
        q = q[valid]
        y = 1.0 / depth[valid]
        anchor_count = int(q.size)
        if anchor_count < self.minimum_anchors:
            return self._invalid("insufficient_anchors", anchor_count)
        if float(np.ptp(q)) <= 1e-6:
            return self._invalid("anchor_depth_span_too_small", anchor_count)

        best_mask: np.ndarray | None = None
        best_score = (-1, float("inf"))
        for _ in range(self.ransac_iterations):
            sample = self._rng.choice(anchor_count, size=2, replace=False)
            dq = q[sample[1]] - q[sample[0]]
            if abs(dq) <= 1e-9:
                continue
            scale = (y[sample[1]] - y[sample[0]]) / dq
            offset = y[sample[0]] - scale * q[sample[0]]
            if not np.isfinite(scale) or scale <= 0.0:
                continue
            residual = np.abs(scale * q + offset - y)
            mask = residual <= self.residual_threshold
            count = int(np.count_nonzero(mask))
            median = (
                float(np.median(residual[mask]))
                if count
                else float("inf")
            )
            score = (count, -median)
            if score > best_score:
                best_score = score
                best_mask = mask
        minimum_inliers = max(
            self.minimum_anchors,
            int(np.ceil(anchor_count * self.minimum_inlier_fraction)),
        )
        if best_mask is None or np.count_nonzero(best_mask) < minimum_inliers:
            return self._invalid("ransac_consensus_failed", anchor_count)
        matrix = np.column_stack((q[best_mask], np.ones(np.count_nonzero(best_mask))))
        condition_number = float(np.linalg.cond(matrix))
        if (
            not np.isfinite(condition_number)
            or condition_number > self.maximum_condition_number
        ):
            return self._invalid(
                "calibration_ill_conditioned",
                anchor_count,
                condition_number=condition_number,
            )
        coefficients, *_ = np.linalg.lstsq(matrix, y[best_mask], rcond=None)
        raw_scale, raw_offset = (
            float(coefficients[0]),
            float(coefficients[1]),
        )
        if (
            not np.isfinite(raw_scale)
            or raw_scale <= 0.0
            or not np.isfinite(raw_offset)
        ):
            return self._invalid("invalid_calibration", anchor_count)
        signed_residual = (
            raw_scale * q[best_mask] + raw_offset - y[best_mask]
        )
        residual = np.abs(signed_residual)
        inlier_q = q[best_mask]
        inlier_depth = depth[valid][best_mask]
        anchor_inverse_depth_quantiles = tuple(
            float(value)
            for value in np.quantile(
                inlier_q,
                (0.0, 0.05, 0.25, 0.5, 0.75, 0.95, 1.0),
            )
        )
        metric_depth_quantiles_m = tuple(
            float(value)
            for value in np.quantile(
                inlier_depth,
                (0.0, 0.05, 0.25, 0.5, 0.75, 0.95, 1.0),
            )
        )
        residual_scale = max(
            1e-6,
            1.4826
            * float(
                np.median(
                    np.abs(signed_residual - np.median(signed_residual))
                )
            ),
        )
        normal_inverse = np.linalg.pinv(matrix.T @ matrix)
        fit_covariance = normal_inverse * residual_scale**2
        scale_history = np.asarray(
            (*self._scale_history, raw_scale),
            dtype=np.float64,
        )[-self.temporal_window :]
        offset_history = np.asarray(
            (*self._offset_history, raw_offset),
            dtype=np.float64,
        )[-self.temporal_window :]
        history = np.column_stack(
            (scale_history, offset_history),
        )
        temporal_covariance = np.zeros((2, 2), dtype=np.float64)
        if history.shape[0] >= 2:
            temporal_covariance = np.asarray(
                np.cov(history, rowvar=False, ddof=1),
                dtype=np.float64,
            )
        # The published calibration is a temporal consensus, not the latest
        # raw fit. Convert raw fit-to-fit scatter into uncertainty of that
        # consensus while retaining the current-frame fit covariance and a
        # small drift floor. This reduces random jitter without pretending
        # that scale/offset are exact.
        temporal_effective_samples = max(1.0, float(history.shape[0]))
        drift_floor = np.diag(
            (
                (0.02 * abs(raw_scale)) ** 2,
                0.001**2,
            )
        )
        parameter_covariance = (
            fit_covariance
            + temporal_covariance / temporal_effective_samples
            + drift_floor
        )
        parameter_covariance = 0.5 * (
            parameter_covariance + parameter_covariance.T
        )
        eigenvalues, eigenvectors = np.linalg.eigh(parameter_covariance)
        parameter_covariance = (
            eigenvectors
            @ np.diag(np.maximum(eigenvalues, 0.0))
            @ eigenvectors.T
        )
        parameter_uncertainty = float(
            np.sqrt(
                max(0.0, float(np.max(np.diag(parameter_covariance))))
            )
        )
        if self.scale is not None and self.offset is not None:
            scale_step_fraction = abs(raw_scale - self.scale) / max(
                abs(self.scale),
                1e-6,
            )
            offset_step = abs(raw_offset - self.offset)
            if (
                scale_step_fraction > self.maximum_scale_step_fraction
                or offset_step > self.maximum_offset_step_m_inv
            ):
                change_point_ready = self._add_change_point_candidate(
                    raw_scale,
                    raw_offset,
                )
                if change_point_ready:
                    scale, offset = self._reseed_from_change_point()
                    self._scale_history.append(scale)
                    self._offset_history.append(offset)
                    self._residual_history.append(residual_scale)
                    self.stable_samples = 1
                    self.last_valid = MetricCalibration(
                        False,
                        "change_point_reseeded",
                    )
                    self.last = MetricCalibration(
                        valid=True,
                        reason="calibration_change_point_reseeded",
                        scale=scale,
                        offset=offset,
                        inlier_count=int(np.count_nonzero(best_mask)),
                        anchor_count=anchor_count,
                        residual_m_inv=float(np.median(residual)),
                        raw_scale=raw_scale,
                        raw_offset=raw_offset,
                        condition_number=condition_number,
                        parameter_uncertainty=parameter_uncertainty,
                        stable_samples=self.stable_samples,
                        stable=False,
                        measurement_accepted=True,
                        parameter_covariance_ab=tuple(
                            tuple(float(value) for value in row)
                            for row in parameter_covariance
                        ),
                        residual_std_m_inv=residual_scale,
                        anchor_inverse_depth_quantiles=(
                            anchor_inverse_depth_quantiles
                        ),
                        metric_depth_quantiles_m=metric_depth_quantiles_m,
                        change_point_reseeded=True,
                    )
                    return self.last
                self.stable_samples = 0
                self.last = MetricCalibration(
                    valid=False,
                    reason="calibration_temporal_jump",
                    scale=self.scale,
                    offset=self.offset,
                    inlier_count=int(np.count_nonzero(best_mask)),
                    anchor_count=anchor_count,
                    residual_m_inv=float(np.median(residual)),
                    raw_scale=raw_scale,
                    raw_offset=raw_offset,
                    condition_number=condition_number,
                    parameter_uncertainty=parameter_uncertainty,
                    stable_samples=0,
                    stable=False,
                    measurement_accepted=False,
                    parameter_covariance_ab=tuple(
                        tuple(float(value) for value in row)
                        for row in parameter_covariance
                    ),
                    residual_std_m_inv=residual_scale,
                    anchor_inverse_depth_quantiles=(
                        anchor_inverse_depth_quantiles
                    ),
                    metric_depth_quantiles_m=metric_depth_quantiles_m,
                )
                return self.last
        self._change_point_candidates.clear()
        self._change_point_scale_dispersion_fraction = None
        self._change_point_offset_dispersion_m_inv = None
        self._scale_history.append(raw_scale)
        self._offset_history.append(raw_offset)
        self._residual_history.append(residual_scale)
        scale = float(np.median(np.asarray(self._scale_history)))
        offset = float(np.median(np.asarray(self._offset_history)))
        filtered_residual_scale = float(
            np.median(np.asarray(self._residual_history))
        )
        if self.scale is not None and self.offset is not None:
            alpha = self.ema_alpha
            scale = (1.0 - alpha) * self.scale + alpha * scale
            offset = (1.0 - alpha) * self.offset + alpha * offset
        self.scale = scale
        self.offset = offset
        self.stable_samples += 1
        stable = self.stable_samples >= self.stable_samples_required
        self._change_point_state = "stable" if stable else "stabilizing"
        self.last = MetricCalibration(
            valid=True,
            reason="ok" if stable else "calibration_stabilizing",
            scale=scale,
            offset=offset,
            inlier_count=int(np.count_nonzero(best_mask)),
            anchor_count=anchor_count,
            residual_m_inv=float(np.median(residual)),
            raw_scale=raw_scale,
            raw_offset=raw_offset,
            condition_number=condition_number,
            parameter_uncertainty=parameter_uncertainty,
            stable_samples=self.stable_samples,
            stable=stable,
            measurement_accepted=True,
            parameter_covariance_ab=tuple(
                tuple(float(value) for value in row)
                for row in parameter_covariance
            ),
            residual_std_m_inv=filtered_residual_scale,
            anchor_inverse_depth_quantiles=anchor_inverse_depth_quantiles,
            metric_depth_quantiles_m=metric_depth_quantiles_m,
        )
        if self.last.stable:
            self.last_valid = self.last
        return self.last

    def target_inverse_depth_support(
        self,
        relative_inverse_depth: float,
        *,
        margin_iqr: float = 0.50,
    ) -> dict[str, float | bool | str | None]:
        """Check whether a target sample is safely supported by fit anchors.

        Affine inverse-depth fits are unreliable when evaluated far outside the
        anchor domain. A bounded IQR margin permits small edge excursions while
        rejecting the severe extrapolation that otherwise creates huge ranges.
        """

        value = float(relative_inverse_depth)
        quantiles = self.last.anchor_inverse_depth_quantiles
        if not math.isfinite(value):
            return {
                "valid": False,
                "supported": False,
                "reason": "target_inverse_depth_invalid",
                "value": value,
            }
        if len(quantiles) != 7:
            return {
                "valid": False,
                "supported": False,
                "reason": "anchor_inverse_depth_support_unavailable",
                "value": value,
            }
        minimum, q05, q25, median, q75, q95, maximum = quantiles
        iqr = max(1e-9, q75 - q25)
        margin = max(0.0, float(margin_iqr)) * iqr
        accepted_minimum = max(1e-12, q05 - margin)
        accepted_maximum = q95 + margin
        extrapolation = (
            (q05 - value) / iqr
            if value < q05
            else ((value - q95) / iqr if value > q95 else 0.0)
        )
        supported = accepted_minimum <= value <= accepted_maximum
        return {
            "valid": True,
            "supported": supported,
            "reason": (
                "ok"
                if supported
                else "target_inverse_depth_outside_anchor_support"
            ),
            "value": value,
            "minimum": minimum,
            "q05": q05,
            "q25": q25,
            "median": median,
            "q75": q75,
            "q95": q95,
            "maximum": maximum,
            "iqr": iqr,
            "accepted_minimum": accepted_minimum,
            "accepted_maximum": accepted_maximum,
            "extrapolation_iqr": max(0.0, extrapolation),
        }

    def recovery_status(self) -> dict[str, float | int | str | None]:
        return {
            "state": self._change_point_state,
            "quarantine_sample_count": len(self._change_point_candidates),
            "required_samples": self.change_point_samples_required,
            "scale_dispersion_fraction": (
                self._change_point_scale_dispersion_fraction
            ),
            "offset_dispersion_m_inv": (
                self._change_point_offset_dispersion_m_inv
            ),
            "reseed_count": self._change_point_reseed_count,
        }

    def _add_change_point_candidate(
        self,
        scale: float,
        offset: float,
    ) -> bool:
        if self._change_point_candidates:
            values = np.asarray(self._change_point_candidates, dtype=np.float64)
            center_scale = float(np.median(values[:, 0]))
            center_offset = float(np.median(values[:, 1]))
            admission_scale = (
                abs(scale - center_scale) / max(abs(center_scale), 1e-9)
            )
            admission_offset = abs(offset - center_offset)
            if (
                admission_scale
                > 2.0 * self.change_point_scale_dispersion_fraction
                or admission_offset
                > 2.0 * self.change_point_offset_dispersion_m_inv
            ):
                self._change_point_candidates.clear()
        self._change_point_candidates.append((float(scale), float(offset)))
        values = np.asarray(self._change_point_candidates, dtype=np.float64)
        center_scale = float(np.median(values[:, 0]))
        center_offset = float(np.median(values[:, 1]))
        scale_mad = 1.4826 * float(
            np.median(np.abs(values[:, 0] - center_scale))
        )
        offset_mad = 1.4826 * float(
            np.median(np.abs(values[:, 1] - center_offset))
        )
        self._change_point_scale_dispersion_fraction = (
            scale_mad / max(abs(center_scale), 1e-9)
        )
        self._change_point_offset_dispersion_m_inv = offset_mad
        self._change_point_state = "quarantining"
        return bool(
            len(self._change_point_candidates)
            >= self.change_point_samples_required
            and self._change_point_scale_dispersion_fraction
            <= self.change_point_scale_dispersion_fraction
            and self._change_point_offset_dispersion_m_inv
            <= self.change_point_offset_dispersion_m_inv
        )

    def _reseed_from_change_point(self) -> tuple[float, float]:
        values = np.asarray(self._change_point_candidates, dtype=np.float64)
        scale = float(np.median(values[:, 0]))
        offset = float(np.median(values[:, 1]))
        self._scale_history.clear()
        self._offset_history.clear()
        self._residual_history.clear()
        self._change_point_candidates.clear()
        self._change_point_state = "reseeded"
        self._change_point_reseed_count += 1
        self.scale = scale
        self.offset = offset
        return scale, offset

    def restore_last_valid(self) -> bool:
        """Restore the most recent stable fit without inventing a new sample."""

        if (
            not self.last_valid.valid
            or not self.last_valid.stable
            or self.last_valid.scale is None
            or self.last_valid.offset is None
        ):
            return False
        self.scale = float(self.last_valid.scale)
        self.offset = float(self.last_valid.offset)
        self.stable_samples = self.last_valid.stable_samples
        self.last = self.last_valid
        return True

    def use_cached(
        self,
        age_s: float,
        *,
        scale_drift_fraction_per_s: float,
        offset_drift_m_inv_per_s: float,
    ) -> MetricCalibration | None:
        """Use a bounded stable calibration with age-dependent covariance."""

        if not self.restore_last_valid():
            return None
        base = self.last_valid
        age = max(0.0, float(age_s))
        covariance = np.asarray(
            base.parameter_covariance_ab,
            dtype=np.float64,
        )
        if covariance.shape != (2, 2) or not np.all(np.isfinite(covariance)):
            return None
        scale_std = (
            abs(float(base.scale))
            * max(0.0, float(scale_drift_fraction_per_s))
            * age
        )
        offset_std = (
            max(0.0, float(offset_drift_m_inv_per_s))
            * age
        )
        covariance = covariance + np.diag(
            (scale_std**2, offset_std**2)
        )
        parameter_uncertainty = math.sqrt(
            max(0.0, float(np.max(np.diag(covariance))))
        )
        self.last = replace(
            base,
            reason="cached_calibration",
            parameter_uncertainty=parameter_uncertainty,
            measurement_accepted=False,
            parameter_covariance_ab=tuple(
                tuple(float(value) for value in row)
                for row in covariance
            ),
        )
        return self.last

    def inverse_metric_uncertainty(
        self,
        relative_inverse_depth: float,
        relative_inverse_depth_std: float,
        temporal_sample_count: int = 1,
    ) -> dict[str, float]:
        """Propagate inverse-depth and affine-fit covariance at one target.

        The affine parameters have different leverage at a target sample:
        ``dy/da=q`` and ``dy/db=1`` for ``y=a*q+b``. Keeping their full
        covariance preserves the strong scale/offset correlation from the fit
        instead of multiplying one scalar uncertainty by ``q + 1``.
        """

        if self.scale is None or self.offset is None:
            raise RuntimeError("metric depth calibration is unavailable")
        q = float(relative_inverse_depth)
        q_std = max(0.0, float(relative_inverse_depth_std))
        if not math.isfinite(q) or not math.isfinite(q_std):
            raise ValueError("inverse-depth sample and uncertainty must be finite")
        covariance = np.asarray(
            self.last.parameter_covariance_ab,
            dtype=np.float64,
        )
        if covariance.shape != (2, 2) or not np.all(np.isfinite(covariance)):
            raise RuntimeError("calibration parameter covariance is unavailable")
        jacobian = np.asarray((q, 1.0), dtype=np.float64)
        parameter_variance = max(
            0.0,
            float(jacobian @ covariance @ jacobian),
        )
        inverse_depth_component = abs(float(self.scale)) * q_std
        parameter_component = math.sqrt(parameter_variance)
        raw_residual_component = max(
            0.0,
            float(self.last.residual_std_m_inv or 0.0),
        )
        temporal_count = max(1, int(temporal_sample_count))
        temporal_reduction = max(
            self.temporal_uncertainty_floor_fraction,
            1.0 / math.sqrt(temporal_count),
        )
        residual_component = raw_residual_component * temporal_reduction
        total = math.sqrt(
            inverse_depth_component**2
            + parameter_component**2
            + residual_component**2
        )
        return {
            "inverse_depth_std_m_inv": inverse_depth_component,
            "parameter_std_m_inv": parameter_component,
            "residual_std_m_inv": residual_component,
            "raw_residual_std_m_inv": raw_residual_component,
            "temporal_uncertainty_reduction": temporal_reduction,
            "temporal_sample_count": float(temporal_count),
            "total_std_m_inv": total,
        }

    def metric_depth(self, relative_inverse_depth: np.ndarray | float) -> np.ndarray:
        if self.scale is None or self.offset is None:
            raise RuntimeError("metric depth calibration is unavailable")
        q = np.asarray(relative_inverse_depth, dtype=np.float64)
        inverse_metric = self.scale * q + self.offset
        return np.where(inverse_metric > 1e-6, 1.0 / inverse_metric, np.nan)

    def _invalid(
        self,
        reason: str,
        anchors: int,
        *,
        condition_number: float | None = None,
    ) -> MetricCalibration:
        self.stable_samples = 0
        self._change_point_candidates.clear()
        self._change_point_state = "invalid"
        self._change_point_scale_dispersion_fraction = None
        self._change_point_offset_dispersion_m_inv = None
        self.last = MetricCalibration(
            valid=False,
            reason=reason,
            scale=self.scale,
            offset=self.offset,
            inlier_count=0,
            anchor_count=anchors,
            condition_number=condition_number,
        )
        return self.last
