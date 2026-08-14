"""Unit tests for control_ready_range_estimator.py (Phase 1)."""

from __future__ import annotations

import math

import pytest

from control_ready_range_estimator import (
    STATUS_MEASUREMENT_INVALID,
    STATUS_NOT_INITIALIZED,
    STATUS_OK,
    STATUS_OUT_OF_VALIDATED_RANGE_FAR,
    STATUS_OUT_OF_VALIDATED_RANGE_NEAR,
    STATUS_STALE_MEASUREMENT,
    TREND_APPROACHING,
    TREND_RECEDING,
    TREND_STABLE,
    TREND_UNKNOWN,
    ControlReadyRangeEstimator,
    RangeTrendEstimator,
)


# --------------------------------------------------------------------------
# ControlReadyRangeEstimator
# --------------------------------------------------------------------------

def test_first_update_initializes_without_prediction_step():
    est = ControlReadyRangeEstimator()
    out = est.update(8.0, measurement_timestamp_s=0.0, current_timestamp_s=0.0)
    assert out.filtered_range_m == pytest.approx(8.0)
    assert out.estimated_range_rate_mps == pytest.approx(0.0)
    assert out.estimator_status == STATUS_OK
    assert out.estimator_valid is True
    assert out.innovation_m == pytest.approx(0.0)


def test_alpha_beta_numerical_update_matches_hand_computation():
    est = ControlReadyRangeEstimator(alpha=0.65, beta=0.03, innovation_threshold_m=2.0)
    est.update(8.0, measurement_timestamp_s=0.0, current_timestamp_s=0.0)
    # second measurement: dt=0.2s, raw=7.9m (approaching at 0.5 m/s)
    out = est.update(7.9, measurement_timestamp_s=0.2, current_timestamp_s=0.2)
    range_pred = 8.0 + 0.0 * 0.2
    innovation = 7.9 - range_pred
    limited = max(-2.0, min(2.0, innovation))
    expected_filtered = range_pred + 0.65 * limited
    expected_rate = 0.0 + 0.03 * limited / 0.2
    assert out.filtered_range_m == pytest.approx(expected_filtered)
    assert out.estimated_range_rate_mps == pytest.approx(expected_rate)
    assert out.innovation_m == pytest.approx(innovation)


def test_irregular_dt_uses_actual_measurement_interval():
    est = ControlReadyRangeEstimator()
    est.update(10.0, measurement_timestamp_s=0.0, current_timestamp_s=0.0)
    out_a = est.update(9.8, measurement_timestamp_s=0.05, current_timestamp_s=0.05)
    est2 = ControlReadyRangeEstimator()
    est2.update(10.0, measurement_timestamp_s=0.0, current_timestamp_s=0.0)
    out_b = est2.update(9.8, measurement_timestamp_s=0.5, current_timestamp_s=0.5)
    # same innovation, very different dt -> very different rate estimate
    assert out_a.innovation_m == pytest.approx(out_b.innovation_m)
    assert out_a.estimated_range_rate_mps != pytest.approx(out_b.estimated_range_rate_mps)


def test_soft_innovation_limiter_bounds_single_measurement_influence():
    est = ControlReadyRangeEstimator(alpha=0.65, beta=0.03, innovation_threshold_m=2.0)
    est.update(8.0, measurement_timestamp_s=0.0, current_timestamp_s=0.0)
    # huge single outlier: raw jumps to 20m
    out = est.update(20.0, measurement_timestamp_s=0.2, current_timestamp_s=0.2)
    # innovation itself is large but limited influence bounds the filtered jump
    assert out.innovation_m == pytest.approx(12.0)
    max_possible_move = 0.65 * 2.0  # alpha * threshold
    assert abs(out.filtered_range_m - 8.0) <= max_possible_move + 1e-9
    # does not lock/freeze output -- estimator still valid and usable
    assert out.estimator_valid is True


def test_soft_limiter_still_allows_convergence_on_genuine_change():
    est = ControlReadyRangeEstimator(alpha=0.65, beta=0.03, innovation_threshold_m=2.0)
    est.update(8.0, measurement_timestamp_s=0.0, current_timestamp_s=0.0)
    # target genuinely moved to 3m and stays there; feed consistent measurements
    t = 0.0
    last = None
    for _ in range(30):
        t += 0.2
        last = est.update(3.0, measurement_timestamp_s=t, current_timestamp_s=t)
    assert last.filtered_range_m == pytest.approx(3.0, abs=0.05)


def test_measurement_age_compensation_formula():
    est = ControlReadyRangeEstimator(alpha=0.65, beta=0.03)
    est.update(8.0, measurement_timestamp_s=0.0, current_timestamp_s=0.0)
    out = est.update(7.0, measurement_timestamp_s=0.2, current_timestamp_s=0.2)
    rate = out.estimated_range_rate_mps
    filtered = out.filtered_range_m
    # query slightly after the measurement arrived: current_timestamp_s > measurement_timestamp_s
    out2 = est.update(7.0, measurement_timestamp_s=0.2, current_timestamp_s=0.2)  # duplicate ts, dt<=0 branch
    # directly exercise age compensation via explicit measurement_age_s override
    out3 = est.update(6.9, measurement_timestamp_s=0.4, current_timestamp_s=0.45, measurement_age_s=0.05)
    expected = out3.filtered_range_m + out3.estimated_range_rate_mps * 0.05
    assert out3.predicted_current_range_m == pytest.approx(expected)
    assert out3.measurement_age_s == pytest.approx(0.05)


def test_extrapolation_cap_marks_stale_beyond_threshold():
    est = ControlReadyRangeEstimator(max_extrapolation_s=0.5)
    est.update(8.0, measurement_timestamp_s=0.0, current_timestamp_s=0.0)
    out_ok = est.update(7.5, measurement_timestamp_s=0.2, current_timestamp_s=0.45)  # age=0.25
    assert out_ok.estimator_status == STATUS_OK
    assert out_ok.estimator_valid is True

    out_stale = est.update(7.5, measurement_timestamp_s=0.2, current_timestamp_s=0.9, measurement_age_s=0.7)
    assert out_stale.estimator_status == STATUS_STALE_MEASUREMENT
    assert out_stale.estimator_valid is False


def test_stale_measurement_does_not_predict_indefinitely():
    est = ControlReadyRangeEstimator(max_extrapolation_s=0.5)
    est.update(8.0, measurement_timestamp_s=0.0, current_timestamp_s=0.0)
    out1 = est.update(0.0, measurement_timestamp_s=0.0, current_timestamp_s=1.0, measurement_valid=False, measurement_age_s=1.0)
    out2 = est.update(0.0, measurement_timestamp_s=0.0, current_timestamp_s=5.0, measurement_valid=False, measurement_age_s=5.0)
    assert out1.estimator_status == STATUS_STALE_MEASUREMENT
    assert out2.estimator_status == STATUS_STALE_MEASUREMENT
    assert out1.estimator_valid is False and out2.estimator_valid is False


def test_measurement_invalid_but_fresh_keeps_estimator_valid_with_distinct_status():
    est = ControlReadyRangeEstimator(max_extrapolation_s=0.5)
    est.update(8.0, measurement_timestamp_s=0.0, current_timestamp_s=0.0)
    out = est.update(0.0, measurement_timestamp_s=0.0, current_timestamp_s=0.1, measurement_valid=False, measurement_age_s=0.1)
    assert out.estimator_status == STATUS_MEASUREMENT_INVALID
    assert out.estimator_valid is True


def test_measurement_invalid_before_initialization_reports_not_initialized():
    est = ControlReadyRangeEstimator()
    out = est.update(0.0, measurement_timestamp_s=0.0, current_timestamp_s=0.0, measurement_valid=False)
    assert out.estimator_status == STATUS_NOT_INITIALIZED
    assert out.estimator_valid is False
    assert out.predicted_current_range_m is None


def test_out_of_validated_range_near_and_far_status_no_hard_clamp():
    est = ControlReadyRangeEstimator()
    est.update(2.0, measurement_timestamp_s=0.0, current_timestamp_s=0.0)
    out_near = est.update(2.0, measurement_timestamp_s=0.2, current_timestamp_s=0.2)
    assert out_near.estimator_status == STATUS_OUT_OF_VALIDATED_RANGE_NEAR
    assert out_near.predicted_current_range_m < 3.0  # not hard-clamped
    assert out_near.display_range_m == pytest.approx(3.0)  # display copy IS clamped

    est2 = ControlReadyRangeEstimator()
    est2.update(13.0, measurement_timestamp_s=0.0, current_timestamp_s=0.0)
    out_far = est2.update(13.0, measurement_timestamp_s=0.2, current_timestamp_s=0.2)
    assert out_far.estimator_status == STATUS_OUT_OF_VALIDATED_RANGE_FAR
    assert out_far.predicted_current_range_m > 12.0
    assert out_far.display_range_m == pytest.approx(12.0)


def test_timestamp_rollback_triggers_reset():
    est = ControlReadyRangeEstimator()
    est.update(8.0, measurement_timestamp_s=1.0, current_timestamp_s=1.0)
    est.update(7.5, measurement_timestamp_s=1.2, current_timestamp_s=1.2)
    reset_count_before = est.reset_count
    out = est.update(9.0, measurement_timestamp_s=0.5, current_timestamp_s=0.5)  # timestamp went backwards
    assert out.reset_count == reset_count_before + 1
    assert out.reset_reason == "timestamp_rollback"
    assert out.filtered_range_m == pytest.approx(9.0)  # re-initialized from this measurement


def test_measurement_gap_timeout_triggers_reset():
    est = ControlReadyRangeEstimator(measurement_gap_timeout_s=1.0)
    est.update(8.0, measurement_timestamp_s=0.0, current_timestamp_s=0.0)
    out = est.update(8.0, measurement_timestamp_s=5.0, current_timestamp_s=5.0)
    assert out.reset_reason == "measurement_gap_timeout"
    assert out.reset_count == 1


def test_session_change_triggers_reset_and_no_state_leak():
    est = ControlReadyRangeEstimator()
    est.update(3.0, measurement_timestamp_s=0.0, current_timestamp_s=0.0, session_id="session_A")
    est.update(3.0, measurement_timestamp_s=0.2, current_timestamp_s=0.2, session_id="session_A")
    out = est.update(11.0, measurement_timestamp_s=0.4, current_timestamp_s=0.4, session_id="session_B")
    assert out.reset_reason == "session_changed"
    # new session's filtered range reflects ONLY its own first measurement, not session_A's state
    assert out.filtered_range_m == pytest.approx(11.0)
    assert out.estimated_range_rate_mps == pytest.approx(0.0)


def test_model_signature_change_triggers_reset():
    est = ControlReadyRangeEstimator()
    est.update(8.0, measurement_timestamp_s=0.0, current_timestamp_s=0.0, model_signature="v1")
    out = est.update(8.0, measurement_timestamp_s=0.2, current_timestamp_s=0.2, model_signature="v2")
    assert out.reset_reason == "model_or_feature_contract_changed"


def test_explicit_reset_clears_state_and_increments_reset_count():
    est = ControlReadyRangeEstimator()
    est.update(8.0, measurement_timestamp_s=0.0, current_timestamp_s=0.0)
    est.reset("target_reacquired")
    assert est.reset_count == 1
    out = est.update(5.0, measurement_timestamp_s=1.0, current_timestamp_s=1.0)
    assert out.filtered_range_m == pytest.approx(5.0)
    assert out.estimated_range_rate_mps == pytest.approx(0.0)


def test_duplicate_timestamp_does_not_divide_by_zero():
    est = ControlReadyRangeEstimator()
    est.update(8.0, measurement_timestamp_s=1.0, current_timestamp_s=1.0)
    out = est.update(7.5, measurement_timestamp_s=1.0, current_timestamp_s=1.0)  # dt == 0
    assert math.isfinite(out.filtered_range_m)
    assert math.isfinite(out.estimated_range_rate_mps)


# --------------------------------------------------------------------------
# RangeTrendEstimator
# --------------------------------------------------------------------------

def test_trend_reports_unknown_below_min_samples():
    trend = RangeTrendEstimator(windows_s=(0.6,), min_samples=3)
    out = trend.update(0.0, 10.0)
    assert out[0.6].state == TREND_UNKNOWN
    out = trend.update(0.2, 9.9)
    assert out[0.6].state == TREND_UNKNOWN


def test_trend_detects_approaching_with_confirm_count_and_hysteresis():
    trend = RangeTrendEstimator(windows_s=(0.6,), entry_threshold_mps=0.08, exit_threshold_mps=0.04, min_samples=3, confirm_count=2)
    t = 0.0
    r = 10.0
    last_state = None
    for _ in range(10):
        out = trend.update(t, r)
        last_state = out[0.6].state
        t += 0.1
        r -= 0.02  # 0.2 m/s approaching, well above 0.08 entry threshold
    assert last_state == TREND_APPROACHING


def test_trend_detects_receding():
    trend = RangeTrendEstimator(windows_s=(0.6,), min_samples=3, confirm_count=2)
    t = 0.0
    r = 5.0
    last_state = None
    for _ in range(10):
        out = trend.update(t, r)
        last_state = out[0.6].state
        t += 0.1
        r += 0.02
    assert last_state == TREND_RECEDING


def test_trend_stop_and_hold_reports_stable():
    trend = RangeTrendEstimator(windows_s=(0.6,), min_samples=3, confirm_count=2)
    t = 0.0
    last_state = None
    # approach first, then hold
    r = 10.0
    for _ in range(8):
        trend.update(t, r)
        t += 0.1
        r -= 0.05
    for _ in range(12):
        out = trend.update(t, r)  # held constant
        last_state = out[0.6].state
        t += 0.1
    assert last_state == TREND_STABLE


def test_trend_hysteresis_requires_larger_slope_to_enter_than_to_exit():
    trend = RangeTrendEstimator(windows_s=(0.6,), entry_threshold_mps=0.08, exit_threshold_mps=0.04, min_samples=3, confirm_count=2)
    t = 0.0
    r = 10.0
    for _ in range(8):
        out = trend.update(t, r)
        t += 0.1
        r -= 0.02  # enters APPROACHING (0.2 m/s > 0.08 entry)
    assert out[0.6].state == TREND_APPROACHING
    # now slow down to 0.05 m/s (below entry 0.08 but above exit 0.04): should REMAIN approaching
    for _ in range(6):
        out = trend.update(t, r)
        t += 0.1
        r -= 0.005
    assert out[0.6].state == TREND_APPROACHING


def test_trend_invalid_input_resets_to_unknown_no_stale_state_held():
    trend = RangeTrendEstimator(windows_s=(0.6,), min_samples=3, confirm_count=2)
    t = 0.0
    r = 10.0
    for _ in range(8):
        out = trend.update(t, r)
        t += 0.1
        r -= 0.02
    assert out[0.6].state == TREND_APPROACHING
    out = trend.update(t, None, valid=False)
    assert out[0.6].state == TREND_UNKNOWN


def test_trend_irregular_sampling_still_produces_finite_slope():
    trend = RangeTrendEstimator(windows_s=(0.8,), min_samples=3, confirm_count=2)
    samples = [(0.0, 10.0), (0.05, 9.98), (0.4, 9.7), (0.75, 9.3)]
    out = None
    for t, r in samples:
        out = trend.update(t, r)
    assert out[0.8].slope_mps is not None
    assert math.isfinite(out[0.8].slope_mps)
    assert out[0.8].n_samples == 4


def test_trend_no_future_data_used():
    trend = RangeTrendEstimator(windows_s=(0.6,), min_samples=3, confirm_count=2)
    out_t2 = trend.update(0.0, 10.0)
    out_t2 = trend.update(0.2, 9.9)
    out_before_jump = trend.update(0.4, 9.8)
    slope_before = out_before_jump[0.6].slope_mps
    # a huge future jump fed afterward must not retroactively change the
    # already-returned TrendOutput for t=0.4
    trend.update(0.6, 2.0)
    assert out_before_jump[0.6].slope_mps == pytest.approx(slope_before)


def test_trend_large_temporal_gap_self_heals_to_unknown():
    trend = RangeTrendEstimator(windows_s=(0.6, 1.0), min_samples=3, confirm_count=2)
    t = 0.0
    r = 10.0
    out = None
    for _ in range(8):
        out = trend.update(t, r)
        t += 0.1
        r -= 0.02
    assert out[0.6].state == TREND_APPROACHING
    # a huge real-world gap (e.g. sim stutter) -- next sample arrives 54s later
    t += 54.0
    out = trend.update(t, r)
    assert out[0.6].state == TREND_UNKNOWN
    assert out[1.0].state == TREND_UNKNOWN
    assert out[0.6].n_samples == 1


def test_trend_multiple_windows_independent():
    trend = RangeTrendEstimator(windows_s=(0.6, 1.0), min_samples=3, confirm_count=2)
    t = 0.0
    r = 10.0
    out = None
    for _ in range(6):
        out = trend.update(t, r)
        t += 0.1
        r -= 0.02
    assert 0.6 in out and 1.0 in out
    assert out[0.6].window_s == 0.6
    assert out[1.0].window_s == 1.0
