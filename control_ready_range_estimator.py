"""CORE_RANGE_CONTROL_READY_OBSERVATION_PIPELINE, Phase 1.

Two independent, causal, observation-only components:

- ControlReadyRangeEstimator: a causal alpha-beta (state) filter that turns a
  raw per-frame range prediction into a smoother filtered_range_m + an
  estimated range rate, then age-compensates to "now" to produce
  predicted_current_range_m. Never uses future frames. Resets cleanly at
  session/target boundaries. No hard reject of a single noisy measurement --
  a *soft* innovation limiter bounds its influence instead.

- RangeTrendEstimator: a separate, causal, windowed (0.6/0.8/1.0s) robust
  slope estimator used ONLY for monitoring/display (APPROACHING/STABLE/
  RECEDING/UNKNOWN with hysteresis). Never used to drive the estimator above,
  and never used as a control signal -- callers must not treat trend state as
  a command input.

Both classes hold no reference to PX4/MAVLink/controller code and send no
commands anywhere; they are pure, in-process, observation-only state
machines over a scalar range signal.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from statistics import median

# --------------------------------------------------------------------------
# ControlReadyRangeEstimator -- causal alpha-beta state estimator
# --------------------------------------------------------------------------

STATUS_NOT_INITIALIZED = "NOT_INITIALIZED"
STATUS_OK = "OK"
STATUS_STALE_MEASUREMENT = "STALE_MEASUREMENT"
STATUS_OUT_OF_VALIDATED_RANGE_NEAR = "OUT_OF_VALIDATED_RANGE_NEAR"
STATUS_OUT_OF_VALIDATED_RANGE_FAR = "OUT_OF_VALIDATED_RANGE_FAR"
STATUS_MEASUREMENT_INVALID = "MEASUREMENT_INVALID"

DEFAULT_ALPHA = 0.65
DEFAULT_BETA = 0.03
DEFAULT_INNOVATION_THRESHOLD_M = 2.0
DEFAULT_MAX_EXTRAPOLATION_S = 0.5
DEFAULT_MEASUREMENT_GAP_TIMEOUT_S = 1.0  # ~5 dropped frames at the corpus's nominal 5 Hz rate
VALIDATED_RANGE_NEAR_M = 3.0
VALIDATED_RANGE_FAR_M = 12.0
MIN_DT_S = 1e-6  # guards against divide-by-zero on duplicate/near-duplicate timestamps


@dataclass
class EstimatorOutput:
    raw_model_range_m: float
    filtered_range_m: float | None
    estimated_range_rate_mps: float | None
    predicted_current_range_m: float | None
    display_range_m: float | None
    measurement_age_s: float | None
    innovation_m: float | None
    estimator_valid: bool
    estimator_status: str
    reset_count: int
    reset_reason: str | None


@dataclass
class _EstimatorState:
    filtered_range_m: float | None = None
    estimated_range_rate_mps: float = 0.0
    last_measurement_timestamp: float | None = None
    last_update_timestamp: float | None = None
    session_id: object | None = None
    model_signature: object | None = None
    initialized: bool = False
    consecutive_stale_count: int = 0
    reset_count: int = 0


class ControlReadyRangeEstimator:
    """Causal alpha-beta range/rate estimator with measurement-age compensation.

    All timestamps passed to `update()` must be from a single, consistent
    clock domain (see clock_domain_contract.json) -- never mix sim-time and
    wall-clock within one estimator instance's lifetime.
    """

    def __init__(
        self,
        alpha: float = DEFAULT_ALPHA,
        beta: float = DEFAULT_BETA,
        innovation_threshold_m: float = DEFAULT_INNOVATION_THRESHOLD_M,
        max_extrapolation_s: float = DEFAULT_MAX_EXTRAPOLATION_S,
        measurement_gap_timeout_s: float = DEFAULT_MEASUREMENT_GAP_TIMEOUT_S,
        validated_range_m: tuple[float, float] = (VALIDATED_RANGE_NEAR_M, VALIDATED_RANGE_FAR_M),
    ) -> None:
        self.alpha = float(alpha)
        self.beta = float(beta)
        self.innovation_threshold_m = float(innovation_threshold_m)
        self.max_extrapolation_s = float(max_extrapolation_s)
        self.measurement_gap_timeout_s = float(measurement_gap_timeout_s)
        self.validated_range_m = validated_range_m
        self._state = _EstimatorState()

    @property
    def reset_count(self) -> int:
        return self._state.reset_count

    def reset(self, reason: str, session_id: object | None = None, model_signature: object | None = None) -> None:
        previous_reset_count = self._state.reset_count
        self._state = _EstimatorState()
        self._state.reset_count = previous_reset_count + 1
        self._state.session_id = session_id
        self._state.model_signature = model_signature
        self._last_reset_reason = reason

    def _auto_reset_check(
        self, measurement_timestamp_s: float, session_id: object | None, model_signature: object | None
    ) -> str | None:
        state = self._state
        if not state.initialized:
            return None
        if session_id is not None and state.session_id is not None and session_id != state.session_id:
            return "session_changed"
        if model_signature is not None and state.model_signature is not None and model_signature != state.model_signature:
            return "model_or_feature_contract_changed"
        if state.last_measurement_timestamp is not None and measurement_timestamp_s < state.last_measurement_timestamp:
            return "timestamp_rollback"
        if (
            state.last_measurement_timestamp is not None
            and (measurement_timestamp_s - state.last_measurement_timestamp) > self.measurement_gap_timeout_s
        ):
            return "measurement_gap_timeout"
        return None

    def update(
        self,
        raw_candidate_range_m: float,
        measurement_timestamp_s: float,
        current_timestamp_s: float,
        measurement_valid: bool = True,
        measurement_age_s: float | None = None,
        session_id: object | None = None,
        model_signature: object | None = None,
    ) -> EstimatorOutput:
        reset_reason: str | None = None

        auto_reason = self._auto_reset_check(measurement_timestamp_s, session_id, model_signature)
        if auto_reason is not None:
            self.reset(auto_reason, session_id=session_id, model_signature=model_signature)
            reset_reason = auto_reason
        elif not self._state.initialized:
            self._state.session_id = session_id
            self._state.model_signature = model_signature

        state = self._state

        if not measurement_valid:
            state.consecutive_stale_count += 1
            if not state.initialized:
                return EstimatorOutput(
                    raw_model_range_m=raw_candidate_range_m, filtered_range_m=None,
                    estimated_range_rate_mps=None, predicted_current_range_m=None, display_range_m=None,
                    measurement_age_s=None, innovation_m=None, estimator_valid=False,
                    estimator_status=STATUS_NOT_INITIALIZED, reset_count=state.reset_count, reset_reason=reset_reason,
                )
            return self._age_compensated_output(
                raw_candidate_range_m, current_timestamp_s, measurement_age_s,
                innovation_m=None, base_status=STATUS_MEASUREMENT_INVALID, reset_reason=reset_reason,
            )

        state.consecutive_stale_count = 0

        if not state.initialized:
            state.filtered_range_m = float(raw_candidate_range_m)
            state.estimated_range_rate_mps = 0.0
            state.last_measurement_timestamp = float(measurement_timestamp_s)
            state.last_update_timestamp = float(current_timestamp_s)
            state.initialized = True
            innovation = 0.0
        else:
            dt = float(measurement_timestamp_s) - float(state.last_measurement_timestamp)
            if dt <= MIN_DT_S:
                # duplicate/near-duplicate measurement timestamp: do not update
                # filtered range/rate (would divide by ~0); pass state through.
                innovation = None
            else:
                range_pred = state.filtered_range_m + state.estimated_range_rate_mps * dt
                innovation = float(raw_candidate_range_m) - range_pred
                limited_innovation = max(-self.innovation_threshold_m, min(self.innovation_threshold_m, innovation))
                state.filtered_range_m = range_pred + self.alpha * limited_innovation
                state.estimated_range_rate_mps = state.estimated_range_rate_mps + self.beta * limited_innovation / dt
                state.last_measurement_timestamp = float(measurement_timestamp_s)
            state.last_update_timestamp = float(current_timestamp_s)

        return self._age_compensated_output(
            raw_candidate_range_m, current_timestamp_s, measurement_age_s,
            innovation_m=innovation, base_status=None, reset_reason=reset_reason,
        )

    def _age_compensated_output(
        self,
        raw_candidate_range_m: float,
        current_timestamp_s: float,
        measurement_age_s: float | None,
        innovation_m: float | None,
        base_status: str | None,
        reset_reason: str | None,
    ) -> EstimatorOutput:
        state = self._state
        if measurement_age_s is None:
            age = max(0.0, float(current_timestamp_s) - float(state.last_measurement_timestamp))
        else:
            age = max(0.0, float(measurement_age_s))

        predicted_current_range_m = state.filtered_range_m + state.estimated_range_rate_mps * age

        if base_status == STATUS_MEASUREMENT_INVALID and age > self.max_extrapolation_s:
            status = STATUS_STALE_MEASUREMENT
            valid = False
        elif base_status == STATUS_MEASUREMENT_INVALID:
            status = STATUS_MEASUREMENT_INVALID
            valid = True
        elif age > self.max_extrapolation_s:
            status = STATUS_STALE_MEASUREMENT
            valid = False
        elif predicted_current_range_m < self.validated_range_m[0]:
            status = STATUS_OUT_OF_VALIDATED_RANGE_NEAR
            valid = True
        elif predicted_current_range_m > self.validated_range_m[1]:
            status = STATUS_OUT_OF_VALIDATED_RANGE_FAR
            valid = True
        else:
            status = STATUS_OK
            valid = True

        display_range_m = max(self.validated_range_m[0], min(self.validated_range_m[1], predicted_current_range_m))

        return EstimatorOutput(
            raw_model_range_m=float(raw_candidate_range_m),
            filtered_range_m=state.filtered_range_m,
            estimated_range_rate_mps=state.estimated_range_rate_mps,
            predicted_current_range_m=predicted_current_range_m,
            display_range_m=display_range_m,
            measurement_age_s=age,
            innovation_m=innovation_m,
            estimator_valid=valid,
            estimator_status=status,
            reset_count=state.reset_count,
            reset_reason=reset_reason,
        )


# --------------------------------------------------------------------------
# RangeTrendEstimator -- monitoring-only causal windowed trend (Section 7)
# --------------------------------------------------------------------------

TREND_APPROACHING = "APPROACHING"
TREND_STABLE = "STABLE"
TREND_RECEDING = "RECEDING"
TREND_UNKNOWN = "UNKNOWN"

DEFAULT_ENTRY_THRESHOLD_MPS = 0.08
DEFAULT_EXIT_THRESHOLD_MPS = 0.04
DEFAULT_MIN_SAMPLES = 3
DEFAULT_CONFIRM_COUNT = 2


@dataclass
class TrendOutput:
    window_s: float
    state: str
    slope_mps: float | None
    n_samples: int
    time_span_s: float
    confidence: float


@dataclass
class _WindowTrackerState:
    confirmed_state: str = TREND_UNKNOWN
    pending_candidate: str | None = None
    pending_count: int = 0


def _theil_sen_slope(timestamps: list[float], values: list[float]) -> float:
    """Robust slope: median of all pairwise slopes. Causal -- only ever
    called on already-observed (timestamp, value) pairs, never future
    samples."""
    slopes: list[float] = []
    n = len(timestamps)
    for i in range(n):
        for j in range(i + 1, n):
            dt = timestamps[j] - timestamps[i]
            if dt > MIN_DT_S:
                slopes.append((values[j] - values[i]) / dt)
    if not slopes:
        return 0.0
    return median(slopes)


class RangeTrendEstimator:
    """Causal, windowed, robust-slope trend monitor over a scalar range
    signal. Observation/display only -- callers must never use `.state` from
    this class as a control-loop input."""

    def __init__(
        self,
        windows_s: tuple[float, ...] = (0.6, 0.8, 1.0),
        entry_threshold_mps: float = DEFAULT_ENTRY_THRESHOLD_MPS,
        exit_threshold_mps: float = DEFAULT_EXIT_THRESHOLD_MPS,
        min_samples: int = DEFAULT_MIN_SAMPLES,
        confirm_count: int = DEFAULT_CONFIRM_COUNT,
    ) -> None:
        self.windows_s = tuple(windows_s)
        self.entry_threshold_mps = float(entry_threshold_mps)
        self.exit_threshold_mps = float(exit_threshold_mps)
        self.min_samples = int(min_samples)
        self.confirm_count = int(confirm_count)
        self._history: list[tuple[float, float]] = []  # (timestamp, value), causal append-only
        self._trackers: dict[float, _WindowTrackerState] = {w: _WindowTrackerState() for w in self.windows_s}

    def reset(self) -> None:
        self._history = []
        self._trackers = {w: _WindowTrackerState() for w in self.windows_s}

    def _classify_candidate(self, slope: float, confirmed_state: str) -> str:
        entry, exit_ = self.entry_threshold_mps, self.exit_threshold_mps
        if confirmed_state == TREND_APPROACHING:
            if slope <= -exit_:
                return TREND_APPROACHING
            if slope >= entry:
                return TREND_RECEDING
            return TREND_STABLE
        if confirmed_state == TREND_RECEDING:
            if slope >= exit_:
                return TREND_RECEDING
            if slope <= -entry:
                return TREND_APPROACHING
            return TREND_STABLE
        if slope <= -entry:
            return TREND_APPROACHING
        if slope >= entry:
            return TREND_RECEDING
        return TREND_STABLE

    def update(self, timestamp_s: float, range_m: float, valid: bool = True) -> dict[float, TrendOutput]:
        if valid and range_m is not None:
            # causal, append-only, monotonic-timestamp history
            if self._history and timestamp_s < self._history[-1][0]:
                self.reset()
            self._history.append((float(timestamp_s), float(range_m)))
            # bound memory: no window can ever use a sample older than the
            # largest configured window, so older history is dead weight.
            # A large gap since the previous sample therefore also causes
            # every window to naturally fall below min_samples on the next
            # update (self-healing back to UNKNOWN without an explicit
            # cross-component reset signal from the range estimator).
            max_window_s = max(self.windows_s)
            cutoff = timestamp_s - max_window_s
            if self._history[0][0] < cutoff:
                self._history = [(t, v) for t, v in self._history if t >= cutoff]

        results: dict[float, TrendOutput] = {}
        for window_s in self.windows_s:
            tracker = self._trackers[window_s]
            if not valid or range_m is None:
                # "không giữ state cũ khi input invalid/stale"
                tracker.confirmed_state = TREND_UNKNOWN
                tracker.pending_candidate = None
                tracker.pending_count = 0
                results[window_s] = TrendOutput(window_s, TREND_UNKNOWN, None, 0, 0.0, 0.0)
                continue

            window_samples = [(t, v) for t, v in self._history if timestamp_s - t <= window_s]
            n_samples = len(window_samples)
            if n_samples < self.min_samples:
                tracker.confirmed_state = TREND_UNKNOWN
                tracker.pending_candidate = None
                tracker.pending_count = 0
                results[window_s] = TrendOutput(window_s, TREND_UNKNOWN, None, n_samples, 0.0, 0.0)
                continue

            ts = [t for t, _ in window_samples]
            vs = [v for _, v in window_samples]
            time_span_s = ts[-1] - ts[0]
            slope = _theil_sen_slope(ts, vs)
            confidence = min(1.0, n_samples / (self.min_samples * 2)) * min(1.0, time_span_s / window_s)

            candidate = self._classify_candidate(slope, tracker.confirmed_state)
            if candidate == tracker.confirmed_state:
                tracker.pending_candidate = None
                tracker.pending_count = 0
            else:
                if candidate == tracker.pending_candidate:
                    tracker.pending_count += 1
                else:
                    tracker.pending_candidate = candidate
                    tracker.pending_count = 1
                if tracker.pending_count >= self.confirm_count:
                    tracker.confirmed_state = candidate
                    tracker.pending_candidate = None
                    tracker.pending_count = 0

            results[window_s] = TrendOutput(window_s, tracker.confirmed_state, slope, n_samples, time_span_s, confidence)

        return results
