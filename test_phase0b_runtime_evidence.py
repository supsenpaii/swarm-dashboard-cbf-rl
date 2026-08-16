from __future__ import annotations

import ast
import math
from pathlib import Path

from phase0b_runtime_evidence import (
    ClockMapping,
    EXPECTED_CAMERA_HEIGHT,
    EXPECTED_CAMERA_HORIZONTAL_FOV_RAD,
    EXPECTED_CAMERA_WIDTH,
    StatsRecorder,
    StreamRecorder,
    evaluate_api_safety,
    evaluate_gate,
    validate_camera_info,
)


def record_regular_stream(
    recorder: StreamRecorder,
    *,
    source_step_s: float,
    receipt_step_s: float,
    count: int,
) -> None:
    for index in range(count):
        recorder.record(
            index * source_step_s,
            100.0 + index * receipt_step_s,
            0.00001,
        )


def test_rtf_normalization_does_not_label_slow_sim_as_backlog() -> None:
    recorder = StreamRecorder("camera", 50.0)
    record_regular_stream(
        recorder,
        source_step_s=0.02,
        receipt_step_s=0.04,
        count=51,
    )
    mapping = ClockMapping(0.0, 1.0, 100.0, 102.0)
    summary = recorder.summary(mean_rtf=0.5, clock_mapping=mapping)
    assert summary["estimated_missing_samples"] == 0
    assert math.isclose(summary["receipt_rate_hz"], 25.0)
    assert math.isclose(summary["rtf_expected_receipt_rate_hz"], 25.0)
    assert not summary["rtf_normalized_backlog_suspected"]


def test_missing_sample_is_detected_from_source_timeline() -> None:
    recorder = StreamRecorder("camera", 50.0)
    for source_s, receipt_s in ((0.0, 10.0), (0.02, 10.02), (0.06, 10.06)):
        recorder.record(source_s, receipt_s, 0.0)
    summary = recorder.summary(mean_rtf=1.0, clock_mapping=None)
    assert summary["expected_inclusive_count"] == 4
    assert summary["estimated_missing_samples"] == 1
    assert summary["rtf_normalized_backlog_suspected"]


def test_stats_mean_rtf_uses_sim_and_real_deltas() -> None:
    recorder = StatsRecorder()
    recorder.record(740.404, 926.480291669, 10.0, 0.80)
    recorder.record(745.392, 932.675338471, 16.2, 0.81)
    summary, mapping = recorder.summary()
    assert mapping is not None
    assert math.isclose(
        summary["mean_rtf_from_real_deltas"], 0.805159373, rel_tol=1e-8
    )
    assert math.isclose(
        summary["mean_rtf_from_host_deltas"], 4.988 / 6.2, rel_tol=1e-8
    )
    assert summary["normalization_rtf"] == summary["mean_rtf_from_host_deltas"]


def matching_camera_info() -> dict[str, object]:
    fx = EXPECTED_CAMERA_WIDTH / (
        2.0 * math.tan(EXPECTED_CAMERA_HORIZONTAL_FOV_RAD / 2.0)
    )
    return {
        "width": EXPECTED_CAMERA_WIDTH,
        "height": EXPECTED_CAMERA_HEIGHT,
        "intrinsics_k": [
            fx,
            0.0,
            EXPECTED_CAMERA_WIDTH / 2.0,
            0.0,
            fx,
            EXPECTED_CAMERA_HEIGHT / 2.0,
            0.0,
            0.0,
            1.0,
        ],
        "distortion_k": [0.0, 0.0, 0.0, 0.0, 0.0],
    }


def test_camera_info_is_authoritative_when_contract_matches() -> None:
    result = validate_camera_info(matching_camera_info())
    assert result["pass"]
    assert result["authority"] == "gazebo_camera_info"


def test_camera_info_mismatch_fails_closed() -> None:
    snapshot = matching_camera_info()
    snapshot["width"] = EXPECTED_CAMERA_WIDTH + 1
    result = validate_camera_info(snapshot)
    assert not result["pass"]
    assert result["reason"] == "contract_mismatch"


def safe_api_payload(*, armed: bool = False) -> dict[str, object]:
    drones = {
        drone_id: {
            "online": True,
            "status": {"armed": armed, "failsafe": False, "nav_state": 2},
        }
        for drone_id in ("UAV-01", "UAV-02")
    }
    pose_streams = {
        drone_id: {
            "visual_follow_bridge": {
                "enabled": False,
                "publish_count": 0,
                "mode_request_attempts": 0,
                "px4_nav_state": None,
            }
        }
        for drone_id in ("UAV-01", "UAV-02")
    }
    return {
        "drones": drones,
        "tracking": {
            "active": False,
            "visual_follow_requested": False,
            "visual_follow_active": False,
            "visual_target_stream_active": False,
            "follow_mode_request_attempts": 0,
        },
        "tracking_pose_streams": pose_streams,
    }


def test_api_safety_requires_disarmed_and_inactive_follow() -> None:
    assert evaluate_api_safety(safe_api_payload())["pass"]
    unsafe = evaluate_api_safety(safe_api_payload(armed=True))
    assert not unsafe["pass"]
    assert any("armed" in violation for violation in unsafe["violations"])


def test_dual_gimbal_publishers_force_phase0b_no_go() -> None:
    stream = {
        "callbacks": 10,
        "rtf_normalized_backlog_suspected": False,
    }
    result = evaluate_gate(
        stream_summaries={"camera": stream},
        camera_contracts={"UAV-01": {"pass": True}},
        api_safety_samples=[{"pass": True}],
        gimbal_topics={
            "/model/x/command/gimbal_pitch": {"publisher_count": 2}
        },
    )
    assert result["p0b_a_result"] == "FAIL"
    assert result["phase0b_decision"] == "NO-GO"
    assert not result["p1b_authorized"]
    assert any("gimbal_publisher_count" in item for item in result["blockers"])


def test_collector_source_contains_no_write_or_publish_calls() -> None:
    source_path = Path(__file__).with_name("phase0b_runtime_evidence.py")
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    forbidden = {
        "advertise",
        "publish",
        "publish_raw",
        "param_set_send",
        "command_long_send",
        "follow_target_send",
        "write",
    }
    calls = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert not calls.intersection(forbidden)
    assert 'method="GET"' in source_path.read_text(encoding="utf-8")
