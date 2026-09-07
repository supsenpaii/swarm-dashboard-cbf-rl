"""Timestamp synchronization for camera bearing observations.

The tracking loop timestamps frames with ``time.monotonic()``.  This module
keeps telemetry and camera-orientation samples in that same clock domain and
interpolates them at the frame timestamp.  Long extrapolation is rejected.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import threading
from typing import Sequence


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


@dataclass(frozen=True)
class PoseSample:
    timestamp_s: float
    position_ned_m: Vector3
    velocity_ned_m_s: Vector3
    quaternion_xyzw: Quaternion


@dataclass(frozen=True)
class SynchronizedPose:
    valid: bool
    timestamp_s: float
    position_ned_m: Vector3 | None = None
    velocity_ned_m_s: Vector3 | None = None
    quaternion_xyzw: Quaternion | None = None
    sample_age_s: float | None = None
    interpolation_span_s: float | None = None
    interpolated: bool = False
    reason: str = ""
    sample_timestamp_s: float | None = None
    sample_offset_s: float | None = None
    lower_sample_delta_s: float | None = None
    upper_sample_delta_s: float | None = None


def _finite_tuple(values: Sequence[float], length: int) -> tuple[float, ...]:
    if len(values) != length:
        raise ValueError(f"expected {length} values")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError("sample contains a non-finite value")
    return result


def _normalize_quaternion(values: Sequence[float]) -> Quaternion:
    quaternion = _finite_tuple(values, 4)
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1e-9:
        raise ValueError("quaternion norm is zero")
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def slerp_quaternion(
    first: Sequence[float],
    second: Sequence[float],
    fraction: float,
) -> Quaternion:
    """Shortest-path quaternion interpolation in ``[x, y, z, w]`` order."""

    first_q = _normalize_quaternion(first)
    second_q = _normalize_quaternion(second)
    amount = max(0.0, min(1.0, float(fraction)))
    dot = sum(a * b for a, b in zip(first_q, second_q))
    if dot < 0.0:
        second_q = tuple(-value for value in second_q)  # type: ignore[assignment]
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        blended = tuple(
            first_q[index] + amount * (second_q[index] - first_q[index])
            for index in range(4)
        )
        return _normalize_quaternion(blended)
    angle = math.acos(dot)
    sine = math.sin(angle)
    first_weight = math.sin((1.0 - amount) * angle) / sine
    second_weight = math.sin(amount * angle) / sine
    return tuple(
        first_weight * first_q[index] + second_weight * second_q[index]
        for index in range(4)
    )  # type: ignore[return-value]


class TimestampedPoseBuffer:
    def __init__(
        self,
        *,
        maxlen: int = 240,
        max_interpolation_gap_s: float = 0.25,
        max_nearest_age_s: float = 0.12,
    ) -> None:
        self._samples: deque[PoseSample] = deque(maxlen=max(2, int(maxlen)))
        self._lock = threading.Lock()
        self.max_interpolation_gap_s = max(0.001, float(max_interpolation_gap_s))
        self.max_nearest_age_s = max(0.0, float(max_nearest_age_s))
        self.rejected_out_of_order = 0

    def clear(self) -> None:
        with self._lock:
            self._samples.clear()
            self.rejected_out_of_order = 0

    def append(self, sample: PoseSample) -> bool:
        timestamp = float(sample.timestamp_s)
        if not math.isfinite(timestamp):
            return False
        normalized = PoseSample(
            timestamp_s=timestamp,
            position_ned_m=_finite_tuple(sample.position_ned_m, 3),  # type: ignore[arg-type]
            velocity_ned_m_s=_finite_tuple(sample.velocity_ned_m_s, 3),  # type: ignore[arg-type]
            quaternion_xyzw=_normalize_quaternion(sample.quaternion_xyzw),
        )
        with self._lock:
            if self._samples and timestamp <= self._samples[-1].timestamp_s:
                self.rejected_out_of_order += 1
                return False
            self._samples.append(normalized)
        return True

    def sample_at(self, timestamp_s: float) -> SynchronizedPose:
        query = float(timestamp_s)
        if not math.isfinite(query):
            return SynchronizedPose(False, query, reason="invalid_timestamp")
        with self._lock:
            samples = tuple(self._samples)
        if not samples:
            return SynchronizedPose(False, query, reason="missing_samples")

        lower = next(
            (sample for sample in reversed(samples) if sample.timestamp_s <= query),
            None,
        )
        upper = next(
            (sample for sample in samples if sample.timestamp_s >= query),
            None,
        )
        if lower is not None and upper is not None:
            if lower.timestamp_s == upper.timestamp_s:
                return self._result_from_sample(lower, query, 0.0)
            span = upper.timestamp_s - lower.timestamp_s
            if span > self.max_interpolation_gap_s:
                return SynchronizedPose(
                    False,
                    query,
                    sample_age_s=min(
                        query - lower.timestamp_s,
                        upper.timestamp_s - query,
                    ),
                    interpolation_span_s=span,
                    reason="interpolation_gap_too_large",
                    lower_sample_delta_s=query - lower.timestamp_s,
                    upper_sample_delta_s=upper.timestamp_s - query,
                )
            fraction = (query - lower.timestamp_s) / span
            position = tuple(
                lower.position_ned_m[index]
                + fraction
                * (upper.position_ned_m[index] - lower.position_ned_m[index])
                for index in range(3)
            )
            velocity = tuple(
                lower.velocity_ned_m_s[index]
                + fraction
                * (
                    upper.velocity_ned_m_s[index]
                    - lower.velocity_ned_m_s[index]
                )
                for index in range(3)
            )
            return SynchronizedPose(
                True,
                query,
                position_ned_m=position,  # type: ignore[arg-type]
                velocity_ned_m_s=velocity,  # type: ignore[arg-type]
                quaternion_xyzw=slerp_quaternion(
                    lower.quaternion_xyzw,
                    upper.quaternion_xyzw,
                    fraction,
                ),
                sample_age_s=max(
                    query - lower.timestamp_s,
                    upper.timestamp_s - query,
                ),
                interpolation_span_s=span,
                interpolated=True,
                reason="interpolated",
                sample_timestamp_s=query,
                sample_offset_s=0.0,
                lower_sample_delta_s=query - lower.timestamp_s,
                upper_sample_delta_s=upper.timestamp_s - query,
            )

        nearest = min(samples, key=lambda sample: abs(sample.timestamp_s - query))
        age = abs(nearest.timestamp_s - query)
        if age > self.max_nearest_age_s:
            return SynchronizedPose(
                False,
                query,
                sample_age_s=age,
                reason="sample_stale",
                sample_timestamp_s=nearest.timestamp_s,
                sample_offset_s=nearest.timestamp_s - query,
            )
        return self._result_from_sample(nearest, query, age)

    @staticmethod
    def _result_from_sample(
        sample: PoseSample,
        query: float,
        age_s: float,
    ) -> SynchronizedPose:
        return SynchronizedPose(
            True,
            query,
            position_ned_m=sample.position_ned_m,
            velocity_ned_m_s=sample.velocity_ned_m_s,
            quaternion_xyzw=sample.quaternion_xyzw,
            sample_age_s=age_s,
            interpolation_span_s=0.0,
            interpolated=False,
            reason="exact" if age_s == 0.0 else "nearest",
            sample_timestamp_s=sample.timestamp_s,
            sample_offset_s=sample.timestamp_s - query,
        )
