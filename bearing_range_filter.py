from __future__ import annotations

import math
import os
from collections import deque
from dataclasses import dataclass
from typing import Sequence

import numpy as np


def _env_float(
    name: str,
    default: float,
    minimum: float,
    maximum: float,
) -> float:
    try:
        value = float(os.environ.get(name, str(default)))
    except ValueError:
        value = default
    return max(minimum, min(maximum, value))


def _env_int(
    name: str,
    default: int,
    minimum: int,
    maximum: int,
) -> int:
    try:
        value = int(os.environ.get(name, str(default)))
    except ValueError:
        value = default
    return max(minimum, min(maximum, value))


@dataclass(frozen=True)
class BearingFilterResult:
    valid: bool
    reason: str
    raw_bearing_ned_unit: tuple[float, float, float] | None
    bearing_ned_unit: tuple[float, float, float] | None
    innovation_deg: float | None
    outlier: bool
    sample_count: int


@dataclass(frozen=True)
class RangeFilterResult:
    valid: bool
    reason: str
    raw_range_m: float | None
    range_m: float | None
    range_std_m: float | None
    range_rate_m_s: float | None
    robust_center_m: float | None
    robust_sigma_m: float | None
    outlier: bool
    sample_count: int
    measurement_accepted: bool = False


@dataclass(frozen=True)
class InverseDepthFilterResult:
    valid: bool
    reason: str
    raw_inverse_depth: float | None
    filtered_inverse_depth: float | None
    inverse_depth_std: float | None
    robust_center: float | None
    robust_sigma: float | None
    sample_count: int
    measurement_accepted: bool = False
    outlier: bool = False


class RobustBearingRangeFilter:
    """Robust pre-filter for camera bearing and monocular metric range.

    Bearing is median-filtered on the unit sphere, slew-limited and then
    low-pass filtered. Range is Hampel-filtered, smoothed in log space and
    constrained by target range-rate and acceleration limits.
    """

    def __init__(
        self,
        *,
        bearing_window: int | None = None,
        bearing_tau_s: float | None = None,
        maximum_bearing_rate_deg_s: float | None = None,
        bearing_outlier_deg: float | None = None,
        range_window: int | None = None,
        range_minimum_samples: int | None = None,
        range_tau_s: float | None = None,
        range_hampel_sigma: float | None = None,
        range_absolute_gate_m: float | None = None,
        range_relative_gate: float | None = None,
        maximum_range_rate_m_s: float | None = None,
        maximum_range_acceleration_m_s2: float | None = None,
        minimum_range_m: float | None = None,
        maximum_range_m: float | None = None,
        maximum_measurement_std_m: float | None = None,
        maximum_measurement_relative_std: float | None = None,
        range_reacquire_samples: int | None = None,
        range_reacquire_max_sigma_m: float | None = None,
        range_reacquire_min_duration_s: float | None = None,
        inverse_depth_window: int | None = None,
        inverse_depth_minimum_samples: int | None = None,
        inverse_depth_tau_s: float | None = None,
        inverse_depth_hampel_sigma: float | None = None,
        inverse_depth_relative_gate: float | None = None,
        inverse_depth_uncertainty_floor_fraction: float | None = None,
    ) -> None:
        self.bearing_window = self._odd(
            bearing_window
            if bearing_window is not None
            else _env_int("SWARM_BR_FILTER_BEARING_WINDOW", 7, 3, 31)
        )
        self.bearing_tau_s = (
            float(bearing_tau_s)
            if bearing_tau_s is not None
            else _env_float(
                "SWARM_BR_FILTER_BEARING_TAU_S", 0.35, 0.02, 3.0
            )
        )
        self.maximum_bearing_rate_rad_s = math.radians(
            float(maximum_bearing_rate_deg_s)
            if maximum_bearing_rate_deg_s is not None
            else _env_float(
                "SWARM_BR_FILTER_MAX_BEARING_RATE_DEG_S",
                60.0,
                5.0,
                360.0,
            )
        )
        self.bearing_outlier_rad = math.radians(
            float(bearing_outlier_deg)
            if bearing_outlier_deg is not None
            else _env_float(
                "SWARM_BR_FILTER_BEARING_OUTLIER_DEG", 8.0, 1.0, 90.0
            )
        )
        self.range_window = self._odd(
            range_window
            if range_window is not None
            else _env_int("SWARM_BR_FILTER_RANGE_WINDOW", 9, 3, 31)
        )
        self.range_minimum_samples = min(
            self.range_window,
            (
                int(range_minimum_samples)
                if range_minimum_samples is not None
                else _env_int(
                    "SWARM_BR_FILTER_RANGE_MIN_SAMPLES", 5, 3, 15
                )
            ),
        )
        self.range_tau_s = (
            float(range_tau_s)
            if range_tau_s is not None
            else _env_float("SWARM_BR_FILTER_RANGE_TAU_S", 1.2, 0.05, 8.0)
        )
        self.range_hampel_sigma = (
            float(range_hampel_sigma)
            if range_hampel_sigma is not None
            else _env_float(
                "SWARM_BR_FILTER_RANGE_HAMPEL_SIGMA", 4.0, 2.0, 10.0
            )
        )
        self.range_absolute_gate_m = (
            float(range_absolute_gate_m)
            if range_absolute_gate_m is not None
            else _env_float(
                "SWARM_BR_FILTER_RANGE_ABS_GATE_M", 1.0, 0.1, 20.0
            )
        )
        self.range_relative_gate = (
            float(range_relative_gate)
            if range_relative_gate is not None
            else _env_float(
                "SWARM_BR_FILTER_RANGE_REL_GATE", 0.08, 0.01, 1.0
            )
        )
        self.maximum_range_rate_m_s = (
            float(maximum_range_rate_m_s)
            if maximum_range_rate_m_s is not None
            else _env_float(
                "SWARM_BR_FILTER_MAX_RANGE_RATE_M_S", 4.0, 0.2, 30.0
            )
        )
        self.maximum_range_acceleration_m_s2 = (
            float(maximum_range_acceleration_m_s2)
            if maximum_range_acceleration_m_s2 is not None
            else _env_float(
                "SWARM_BR_FILTER_MAX_RANGE_ACCEL_M_S2", 6.0, 0.2, 50.0
            )
        )
        self.minimum_range_m = (
            float(minimum_range_m)
            if minimum_range_m is not None
            else _env_float("SWARM_BR_FILTER_MIN_RANGE_M", 1.0, 0.1, 20.0)
        )
        self.maximum_range_m = max(
            self.minimum_range_m + 0.1,
            (
                float(maximum_range_m)
                if maximum_range_m is not None
                else _env_float(
                    "SWARM_BR_FILTER_MAX_RANGE_M", 80.0, 2.0, 1000.0
                )
            ),
        )
        self.maximum_measurement_std_m = (
            float(maximum_measurement_std_m)
            if maximum_measurement_std_m is not None
            else _env_float(
                "SWARM_BR_FILTER_MAX_MEASUREMENT_STD_M",
                3.0,
                0.1,
                100.0,
            )
        )
        self.maximum_measurement_relative_std = (
            float(maximum_measurement_relative_std)
            if maximum_measurement_relative_std is not None
            else _env_float(
                "SWARM_BR_FILTER_MAX_MEASUREMENT_REL_STD",
                0.20,
                0.01,
                1.0,
            )
        )
        self.range_reacquire_samples = (
            max(3, int(range_reacquire_samples))
            if range_reacquire_samples is not None
            else _env_int(
                "SWARM_BR_FILTER_RANGE_REACQUIRE_SAMPLES",
                7,
                3,
                15,
            )
        )
        self.range_reacquire_max_sigma_m = (
            max(0.1, float(range_reacquire_max_sigma_m))
            if range_reacquire_max_sigma_m is not None
            else _env_float(
                "SWARM_BR_FILTER_RANGE_REACQUIRE_MAX_SIGMA_M",
                1.5,
                0.1,
                10.0,
            )
        )
        self.range_reacquire_min_duration_s = (
            max(0.1, float(range_reacquire_min_duration_s))
            if range_reacquire_min_duration_s is not None
            else _env_float(
                "SWARM_BR_FILTER_RANGE_REACQUIRE_MIN_DURATION_S",
                0.6,
                0.1,
                5.0,
            )
        )
        self.inverse_depth_window = self._odd(
            inverse_depth_window
            if inverse_depth_window is not None
            else _env_int(
                "SWARM_BR_FILTER_INVERSE_DEPTH_WINDOW",
                15,
                3,
                31,
            )
        )
        self.inverse_depth_minimum_samples = min(
            self.inverse_depth_window,
            max(
                2,
                (
                    int(inverse_depth_minimum_samples)
                    if inverse_depth_minimum_samples is not None
                    else _env_int(
                        "SWARM_BR_FILTER_INVERSE_DEPTH_MIN_SAMPLES",
                        7,
                        2,
                        15,
                    )
                ),
            ),
        )
        self.inverse_depth_tau_s = (
            max(0.02, float(inverse_depth_tau_s))
            if inverse_depth_tau_s is not None
            else _env_float(
                "SWARM_BR_FILTER_INVERSE_DEPTH_TAU_S",
                1.5,
                0.02,
                8.0,
            )
        )
        self.inverse_depth_hampel_sigma = max(
            2.0,
            (
                float(inverse_depth_hampel_sigma)
                if inverse_depth_hampel_sigma is not None
                else _env_float(
                    "SWARM_BR_FILTER_INVERSE_DEPTH_HAMPEL_SIGMA",
                    3.0,
                    2.0,
                    10.0,
                )
            ),
        )
        self.inverse_depth_relative_gate = max(
            0.02,
            (
                float(inverse_depth_relative_gate)
                if inverse_depth_relative_gate is not None
                else _env_float(
                    "SWARM_BR_FILTER_INVERSE_DEPTH_REL_GATE",
                    0.18,
                    0.02,
                    1.0,
                )
            ),
        )
        self.inverse_depth_uncertainty_floor_fraction = (
            max(
                0.1,
                min(
                    1.0,
                    float(inverse_depth_uncertainty_floor_fraction),
                ),
            )
            if inverse_depth_uncertainty_floor_fraction is not None
            else _env_float(
                "SWARM_BR_FILTER_INVERSE_DEPTH_UNCERTAINTY_FLOOR",
                0.35,
                0.1,
                1.0,
            )
        )
        self.reset()

    def reset(self) -> None:
        self._bearing_samples: deque[np.ndarray] = deque(
            maxlen=self.bearing_window
        )
        self._bearing: np.ndarray | None = None
        self._bearing_timestamp_s: float | None = None
        self._bearing_outliers = 0
        self._range_samples: deque[float] = deque(maxlen=self.range_window)
        self._range_m: float | None = None
        self._range_rate_m_s = 0.0
        self._range_timestamp_s: float | None = None
        self._range_outliers = 0
        self._range_reacquire_candidates: deque[
            tuple[float, float, float]
        ] = deque(maxlen=self.range_reacquire_samples)
        self._range_reacquire_count = 0
        self._inverse_depth_samples: deque[float] = deque(
            maxlen=self.inverse_depth_window
        )
        self._inverse_depth_uncertainties: deque[float] = deque(
            maxlen=self.inverse_depth_window
        )
        self._inverse_depth: float | None = None
        self._inverse_depth_timestamp_s: float | None = None
        self._inverse_depth_outliers = 0
        self._last_bearing = BearingFilterResult(
            False, "uninitialized", None, None, None, False, 0
        )
        self._last_range = RangeFilterResult(
            False, "uninitialized", None, None, None, None, None, None,
            False, 0,
        )
        self._last_inverse_depth = InverseDepthFilterResult(
            False,
            "uninitialized",
            None,
            None,
            None,
            None,
            None,
            0,
        )

    def update_bearing(
        self,
        bearing_ned_unit: Sequence[float],
        timestamp_s: float,
    ) -> BearingFilterResult:
        raw = self._unit(bearing_ned_unit)
        timestamp = float(timestamp_s)
        if not math.isfinite(timestamp):
            raise ValueError("bearing timestamp must be finite")
        if (
            self._bearing_timestamp_s is not None
            and timestamp <= self._bearing_timestamp_s
        ):
            return self._last_bearing

        innovation = (
            0.0
            if self._bearing is None
            else self._angle(self._bearing, raw)
        )
        outlier = bool(
            self._bearing is not None
            and innovation > self.bearing_outlier_rad
        )
        if outlier:
            self._bearing_outliers += 1
        self._bearing_samples.append(raw)
        candidate = self._spherical_median(self._bearing_samples)

        if self._bearing is None:
            filtered = candidate
        else:
            dt = max(1e-3, timestamp - float(self._bearing_timestamp_s))
            candidate_angle = self._angle(self._bearing, candidate)
            maximum_step = self.maximum_bearing_rate_rad_s * dt
            if candidate_angle > maximum_step:
                candidate = self._nlerp(
                    self._bearing,
                    candidate,
                    maximum_step / candidate_angle,
                )
            alpha = 1.0 - math.exp(-dt / self.bearing_tau_s)
            filtered = self._nlerp(self._bearing, candidate, alpha)

        self._bearing = filtered
        self._bearing_timestamp_s = timestamp
        self._last_bearing = BearingFilterResult(
            True,
            "bearing_outlier_suppressed" if outlier else "ok",
            self._tuple(raw),
            self._tuple(filtered),
            math.degrees(innovation),
            outlier,
            len(self._bearing_samples),
        )
        return self._last_bearing

    def update_range(
        self,
        range_m: float,
        measurement_std_m: float,
        timestamp_s: float,
    ) -> RangeFilterResult:
        raw = float(range_m)
        measurement_std = float(measurement_std_m)
        timestamp = float(timestamp_s)
        if (
            not math.isfinite(raw)
            or raw <= 0.0
            or not math.isfinite(measurement_std)
            or measurement_std <= 0.0
            or not math.isfinite(timestamp)
        ):
            raise ValueError("range, standard deviation and timestamp invalid")
        if (
            self._range_timestamp_s is not None
            and timestamp <= self._range_timestamp_s
        ):
            return self._last_range
        if raw < self.minimum_range_m:
            return self._hold_range(
                "range_below_minimum",
                raw,
                measurement_std,
            )
        if raw > self.maximum_range_m:
            return self._hold_range(
                "range_above_maximum",
                raw,
                measurement_std,
            )
        maximum_allowed_std = max(
            0.15,
            min(
                self.maximum_measurement_std_m,
                raw * self.maximum_measurement_relative_std,
            ),
        )
        if measurement_std > maximum_allowed_std:
            return self._hold_range(
                "range_uncertainty_too_large",
                raw,
                measurement_std,
            )

        previous_samples = np.asarray(
            self._range_samples,
            dtype=np.float64,
        )
        if previous_samples.size:
            previous_center = float(np.median(previous_samples))
            previous_mad = float(
                np.median(np.abs(previous_samples - previous_center))
            )
            previous_sigma = 1.4826 * previous_mad
            gate = max(
                self.range_absolute_gate_m,
                self.range_relative_gate * previous_center,
                self.range_hampel_sigma * previous_sigma,
            )
            outlier = bool(abs(raw - previous_center) > gate)
        else:
            outlier = False
        if outlier:
            self._range_outliers += 1
            quarantine = np.asarray(
                [
                    candidate[1]
                    for candidate in self._range_reacquire_candidates
                ],
                dtype=np.float64,
            )
            if quarantine.size:
                quarantine_center = float(np.median(quarantine))
                quarantine_mad = float(
                    np.median(np.abs(quarantine - quarantine_center))
                )
                quarantine_sigma = max(
                    0.05,
                    1.4826 * quarantine_mad,
                )
                quarantine_gate = max(
                    self.range_reacquire_max_sigma_m * 2.0,
                    0.12 * quarantine_center,
                    3.0 * quarantine_sigma,
                )
                if abs(raw - quarantine_center) > quarantine_gate:
                    self._range_reacquire_candidates.clear()
            self._range_reacquire_candidates.append(
                (timestamp, raw, measurement_std)
            )
            candidates = tuple(self._range_reacquire_candidates)
            candidate_values = np.asarray(
                [candidate[1] for candidate in candidates],
                dtype=np.float64,
            )
            candidate_center = float(np.median(candidate_values))
            candidate_sigma = max(
                0.05,
                1.4826
                * float(
                    np.median(
                        np.abs(candidate_values - candidate_center)
                    )
                ),
            )
            candidate_duration_s = (
                candidates[-1][0] - candidates[0][0]
                if len(candidates) >= 2
                else 0.0
            )
            if (
                len(candidates) < self.range_reacquire_samples
                or candidate_duration_s
                < self.range_reacquire_min_duration_s
                or candidate_sigma > self.range_reacquire_max_sigma_m
            ):
                return self._hold_range(
                    "range_change_point_quarantine",
                    raw,
                    measurement_std,
                    outlier=True,
                )
            # Promote one consensus measurement, never the individual rejected
            # samples. Keep the previous filtered range so the ordinary rate
            # and acceleration limiters make any later transition gradual.
            raw_for_filter = candidate_center
            measurement_std_for_filter = max(
                candidate_sigma,
                float(
                    np.median(
                        np.asarray(
                            [candidate[2] for candidate in candidates],
                            dtype=np.float64,
                        )
                    )
                ),
            )
            self._range_samples.clear()
            self._range_reacquire_candidates.clear()
            self._range_rate_m_s = 0.0
            self._range_reacquire_count += 1
            outlier = False
            reacquired = True
        else:
            self._range_reacquire_candidates.clear()
            raw_for_filter = raw
            measurement_std_for_filter = measurement_std
            reacquired = False

        # Only trusted samples enter the finite window. A rejected spike can
        # therefore never corrupt the median used by later measurements.
        self._range_samples.append(raw_for_filter)
        samples = np.asarray(self._range_samples, dtype=np.float64)
        center = float(np.median(samples))
        mad = float(np.median(np.abs(samples - center)))
        robust_sigma = max(0.05, 1.4826 * mad)
        sample_count = len(self._range_samples)

        if sample_count < self.range_minimum_samples:
            self._range_timestamp_s = timestamp
            self._last_range = RangeFilterResult(
                False,
                (
                    "range_change_point_warming_up"
                    if reacquired
                    else "range_filter_warming_up"
                ),
                raw,
                None,
                max(measurement_std_for_filter, robust_sigma),
                None,
                center,
                robust_sigma,
                outlier,
                sample_count,
                False,
            )
            return self._last_range

        if self._range_m is None:
            filtered = center
            filtered_rate = 0.0
        else:
            dt = max(1e-3, timestamp - float(self._range_timestamp_s))
            alpha = 1.0 - math.exp(-dt / self.range_tau_s)
            desired_log = (
                math.log(max(center, 1e-3))
                - math.log(max(self._range_m, 1e-3))
            )
            low_pass_candidate = self._range_m * math.exp(alpha * desired_log)
            desired_rate = (low_pass_candidate - self._range_m) / dt
            acceleration_step = self.maximum_range_acceleration_m_s2 * dt
            desired_rate = float(
                np.clip(
                    desired_rate,
                    self._range_rate_m_s - acceleration_step,
                    self._range_rate_m_s + acceleration_step,
                )
            )
            filtered_rate = float(
                np.clip(
                    desired_rate,
                    -self.maximum_range_rate_m_s,
                    self.maximum_range_rate_m_s,
                )
            )
            filtered = max(0.05, self._range_m + filtered_rate * dt)

        self._range_m = filtered
        self._range_rate_m_s = filtered_rate
        self._range_timestamp_s = timestamp
        self._last_range = RangeFilterResult(
            True,
            "range_outlier_suppressed" if outlier else "ok",
            raw,
            filtered,
            max(measurement_std_for_filter, robust_sigma),
            filtered_rate,
            center,
            robust_sigma,
            outlier,
            sample_count,
            True,
        )
        return self._last_range

    def update_inverse_depth(
        self,
        inverse_depth: float,
        inverse_depth_std: float,
        timestamp_s: float,
    ) -> InverseDepthFilterResult:
        """Robust temporal filtering in MiDaS' native inverse-depth domain."""

        raw = float(inverse_depth)
        uncertainty = float(inverse_depth_std)
        timestamp = float(timestamp_s)
        if not (
            math.isfinite(raw)
            and raw > 0.0
            and math.isfinite(uncertainty)
            and uncertainty > 0.0
            and math.isfinite(timestamp)
        ):
            raise ValueError("inverse depth, uncertainty and timestamp invalid")
        if (
            self._inverse_depth_timestamp_s is not None
            and timestamp <= self._inverse_depth_timestamp_s
        ):
            return self._last_inverse_depth
        if uncertainty / raw > self.inverse_depth_relative_gate:
            return self._hold_inverse_depth(
                "inverse_depth_uncertainty_too_large",
                raw,
                uncertainty,
            )
        previous = np.asarray(
            self._inverse_depth_samples,
            dtype=np.float64,
        )
        outlier = False
        if previous.size:
            center = float(np.median(previous))
            mad = float(np.median(np.abs(previous - center)))
            sigma = max(1e-6, 1.4826 * mad)
            gate = max(
                self.inverse_depth_relative_gate * center,
                self.inverse_depth_hampel_sigma * sigma,
            )
            outlier = abs(raw - center) > gate
        if outlier:
            self._inverse_depth_outliers += 1
            return self._hold_inverse_depth(
                "inverse_depth_outlier",
                raw,
                uncertainty,
                outlier=True,
            )
        # Rejected samples never enter this window.
        self._inverse_depth_samples.append(raw)
        self._inverse_depth_uncertainties.append(uncertainty)
        samples = np.asarray(
            self._inverse_depth_samples,
            dtype=np.float64,
        )
        center = float(np.median(samples))
        mad = float(np.median(np.abs(samples - center)))
        sigma = max(1e-6, 1.4826 * mad)
        count = len(self._inverse_depth_samples)
        uncertainty_samples = np.asarray(
            self._inverse_depth_uncertainties,
            dtype=np.float64,
        )
        median_uncertainty = float(np.median(uncertainty_samples))
        uncertainty_reduction = max(
            self.inverse_depth_uncertainty_floor_fraction,
            1.0 / math.sqrt(max(1, count)),
        )
        filtered_uncertainty = max(
            1e-6,
            median_uncertainty * uncertainty_reduction,
            sigma / math.sqrt(max(1, count)),
        )
        previous_timestamp = self._inverse_depth_timestamp_s
        self._inverse_depth_timestamp_s = timestamp
        if count < self.inverse_depth_minimum_samples:
            self._last_inverse_depth = InverseDepthFilterResult(
                False,
                "inverse_depth_filter_warming_up",
                raw,
                None,
                max(uncertainty, sigma),
                center,
                sigma,
                count,
            )
            return self._last_inverse_depth
        if self._inverse_depth is None:
            filtered = center
        else:
            dt = max(
                1e-3,
                timestamp - float(previous_timestamp),
            )
            alpha = 1.0 - math.exp(-dt / self.inverse_depth_tau_s)
            filtered = (
                (1.0 - alpha) * self._inverse_depth + alpha * center
            )
        self._inverse_depth = filtered
        self._last_inverse_depth = InverseDepthFilterResult(
            True,
            "ok",
            raw,
            filtered,
            filtered_uncertainty,
            center,
            sigma,
            count,
            True,
            False,
        )
        return self._last_inverse_depth

    def status(self) -> dict[str, object]:
        return {
            "bearing": {
                "valid": self._last_bearing.valid,
                "reason": self._last_bearing.reason,
                "raw_ned_unit": self._last_bearing.raw_bearing_ned_unit,
                "filtered_ned_unit": self._last_bearing.bearing_ned_unit,
                "innovation_deg": self._last_bearing.innovation_deg,
                "outlier": self._last_bearing.outlier,
                "outlier_count": self._bearing_outliers,
                "sample_count": self._last_bearing.sample_count,
            },
            "range": {
                "valid": self._last_range.valid,
                "reason": self._last_range.reason,
                "raw_m": self._last_range.raw_range_m,
                "filtered_m": self._last_range.range_m,
                "std_m": self._last_range.range_std_m,
                "rate_m_s": self._last_range.range_rate_m_s,
                "robust_center_m": self._last_range.robust_center_m,
                "robust_sigma_m": self._last_range.robust_sigma_m,
                "outlier": self._last_range.outlier,
                "outlier_count": self._range_outliers,
                "sample_count": self._last_range.sample_count,
                "measurement_accepted": (
                    self._last_range.measurement_accepted
                ),
                "change_point_quarantine_count": len(
                    self._range_reacquire_candidates
                ),
                "change_point_required_samples": (
                    self.range_reacquire_samples
                ),
                "change_point_reacquire_count": (
                    self._range_reacquire_count
                ),
                "change_point_max_sigma_m": (
                    self.range_reacquire_max_sigma_m
                ),
            },
            "inverse_depth": {
                "valid": self._last_inverse_depth.valid,
                "reason": self._last_inverse_depth.reason,
                "raw": self._last_inverse_depth.raw_inverse_depth,
                "filtered": (
                    self._last_inverse_depth.filtered_inverse_depth
                ),
                "std": self._last_inverse_depth.inverse_depth_std,
                "robust_center": self._last_inverse_depth.robust_center,
                "robust_sigma": self._last_inverse_depth.robust_sigma,
                "sample_count": self._last_inverse_depth.sample_count,
                "measurement_accepted": (
                    self._last_inverse_depth.measurement_accepted
                ),
                "outlier": self._last_inverse_depth.outlier,
                "outlier_count": self._inverse_depth_outliers,
                "uncertainty_floor_fraction": (
                    self.inverse_depth_uncertainty_floor_fraction
                ),
                "window": self.inverse_depth_window,
                "minimum_samples": self.inverse_depth_minimum_samples,
                "tau_s": self.inverse_depth_tau_s,
            },
        }

    def _hold_range(
        self,
        reason: str,
        raw_range_m: float,
        measurement_std_m: float,
        *,
        outlier: bool = True,
    ) -> RangeFilterResult:
        samples = np.asarray(self._range_samples, dtype=np.float64)
        center = None if not samples.size else float(np.median(samples))
        robust_sigma = None
        if samples.size:
            mad = float(np.median(np.abs(samples - float(center))))
            robust_sigma = max(0.05, 1.4826 * mad)
        self._last_range = RangeFilterResult(
            self._range_m is not None,
            reason,
            float(raw_range_m),
            self._range_m,
            max(
                float(measurement_std_m),
                0.05 if robust_sigma is None else robust_sigma,
            ),
            self._range_rate_m_s if self._range_m is not None else None,
            center,
            robust_sigma,
            outlier,
            len(self._range_samples),
            False,
        )
        return self._last_range

    def _hold_inverse_depth(
        self,
        reason: str,
        raw: float,
        uncertainty: float,
        *,
        outlier: bool = False,
    ) -> InverseDepthFilterResult:
        samples = np.asarray(
            self._inverse_depth_samples,
            dtype=np.float64,
        )
        center = None if not samples.size else float(np.median(samples))
        sigma = None
        if samples.size:
            sigma = max(
                1e-6,
                1.4826 * float(np.median(np.abs(samples - float(center)))),
            )
        self._last_inverse_depth = InverseDepthFilterResult(
            self._inverse_depth is not None,
            reason,
            raw,
            self._inverse_depth,
            max(uncertainty, 1e-6 if sigma is None else sigma),
            center,
            sigma,
            len(self._inverse_depth_samples),
            False,
            outlier,
        )
        return self._last_inverse_depth

    @staticmethod
    def _odd(value: int) -> int:
        integer = max(3, int(value))
        return integer if integer % 2 else integer + 1

    @staticmethod
    def _unit(value: Sequence[float]) -> np.ndarray:
        vector = np.asarray(value, dtype=np.float64)
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            raise ValueError("bearing must contain three finite values")
        norm = float(np.linalg.norm(vector))
        if norm <= 1e-9:
            raise ValueError("bearing has zero length")
        return vector / norm

    @classmethod
    def _spherical_median(
        cls,
        samples: Sequence[np.ndarray],
    ) -> np.ndarray:
        matrix = np.asarray(samples, dtype=np.float64)
        candidate = np.median(matrix, axis=0)
        norm = float(np.linalg.norm(candidate))
        if norm <= 1e-9:
            candidate = np.mean(matrix, axis=0)
            norm = float(np.linalg.norm(candidate))
        return candidate / max(norm, 1e-9)

    @staticmethod
    def _angle(left: np.ndarray, right: np.ndarray) -> float:
        return math.acos(float(np.clip(np.dot(left, right), -1.0, 1.0)))

    @classmethod
    def _nlerp(
        cls,
        start: np.ndarray,
        end: np.ndarray,
        fraction: float,
    ) -> np.ndarray:
        alpha = float(np.clip(fraction, 0.0, 1.0))
        mixed = start * (1.0 - alpha) + end * alpha
        norm = float(np.linalg.norm(mixed))
        if norm <= 1e-9:
            return start.copy()
        return mixed / norm

    @staticmethod
    def _tuple(value: np.ndarray) -> tuple[float, float, float]:
        return tuple(float(component) for component in value)
