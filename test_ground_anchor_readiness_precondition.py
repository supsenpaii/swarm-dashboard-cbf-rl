import json
from pathlib import Path

import pytest

from core_range_dynamic_capture import (
    GROUND_ANCHOR_STARVED_FRAME_FRACTION,
    GROUND_ANCHOR_YIELD_FACTOR,
    ground_anchor_readiness,
)
from metric_depth_calibrator import MetricDepthCalibrator


MINIMUM_ANCHORS = MetricDepthCalibrator().minimum_anchors


def write_session(
    root: Path,
    session_id: int,
    counts: list[int],
    *,
    camera_down_m: float = -1.5,
    ground_down_m: float = 0.0,
) -> None:
    lines = []
    for index, count in enumerate(counts):
        lines.append(
            json.dumps(
                {
                    "session_id": session_id,
                    "frame_index": index,
                    "stage": "calibration_only",
                    "extrinsics": {"camera_position_ned_m": [0.0, 0.0, camera_down_m]},
                    "anchors": {
                        "accepted_ground_anchor_count": count,
                        "adapter_config": {
                            "minimum_range_m": 1.0,
                            "ground_down_m": ground_down_m,
                        },
                    },
                }
            )
        )
    (root / "physical_diagnostics.jsonl").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )


def readiness_for(tmp_path: Path, name: str, counts: list[int]) -> dict:
    session = tmp_path / name
    session.mkdir()
    write_session(session, 2, counts)
    return ground_anchor_readiness(session, 2)


def test_healthy_anchor_yield_is_accepted(tmp_path: Path) -> None:
    result = readiness_for(tmp_path, "healthy", [60] * 20)
    assert result["satisfied"] is True
    assert result["reason"] == "ok"
    assert result["minimum_anchors"] == MINIMUM_ANCHORS
    assert result["required_anchor_count"] == GROUND_ANCHOR_YIELD_FACTOR * MINIMUM_ANCHORS


def test_yield_below_floor_multiple_is_rejected(tmp_path: Path) -> None:
    # Median 28 sits over the calibrator floor of 12 but under 3x it, so normal
    # frame-to-frame variation would keep dipping under.
    result = readiness_for(tmp_path, "low_yield", [28] * 20)
    assert result["satisfied"] is False
    assert result["reason"] == "ground_anchor_yield_below_minimum"


def test_starved_frames_rejected_even_with_healthy_median(tmp_path: Path) -> None:
    # Median 60, but a fifth of the frames already fall under the floor.
    counts = [60] * 16 + [0] * 4
    result = readiness_for(tmp_path, "spiky", counts)
    assert result["median_anchor_count"] >= GROUND_ANCHOR_YIELD_FACTOR * MINIMUM_ANCHORS
    assert result["starved_frame_fraction"] == pytest.approx(0.2)
    assert result["satisfied"] is False
    assert result["reason"] == "ground_anchor_starved_frames_above_limit"


def test_missing_counts_never_silently_pass(tmp_path: Path) -> None:
    (tmp_path / "physical_diagnostics.jsonl").write_text(
        json.dumps({"session_id": 2, "stage": "calibration_only"}) + "\n",
        encoding="utf-8",
    )
    result = ground_anchor_readiness(tmp_path, 2)
    assert result["satisfied"] is False
    assert result["reason"] == "ground_anchor_counts_unavailable"


def test_altitude_is_measured_against_the_ground_plane(tmp_path: Path) -> None:
    session = tmp_path / "offset"
    session.mkdir()
    write_session(session, 2, [60] * 5, camera_down_m=1.0, ground_down_m=2.0)
    assert ground_anchor_readiness(session, 2)["camera_altitude_m"] == pytest.approx(1.0)


def test_only_the_requested_session_is_measured(tmp_path: Path) -> None:
    healthy = {
        "session_id": 2,
        "stage": "calibration_only",
        "anchors": {"accepted_ground_anchor_count": 60, "adapter_config": {}},
    }
    starved = {
        "session_id": 3,
        "stage": "calibration_only",
        "anchors": {"accepted_ground_anchor_count": 0, "adapter_config": {}},
    }
    (tmp_path / "physical_diagnostics.jsonl").write_text(
        json.dumps(healthy) + "\n" + json.dumps(starved) + "\n", encoding="utf-8"
    )
    assert ground_anchor_readiness(tmp_path, 2)["satisfied"] is True
    assert ground_anchor_readiness(tmp_path, 3)["satisfied"] is False


def build_counts(median_count: int, starved_fraction: float, frames: int = 100) -> list[int]:
    starved = round(starved_fraction * frames)
    return [0] * starved + [median_count] * (frames - starved)


# Median anchor count and starved-frame fraction measured from the 2026-08-06
# Stage 2A corpus and the four follow-up diagnostic runs.
MEASURED_ATTEMPTS = [
    ("mid_center_recede_3", 0, 0.529, False),
    ("yaw_recede_2", 0, 0.614, False),
    ("far_center_approach_1", 2, 0.800, False),
    ("yaw_recede_3", 28, 0.128, False),
    ("control_near_lateral_recede", 30, 0.294, False),
    ("yaw_recede_1", 52, 0.197, False),
    ("near_center_recede_1", 56, 0.180, False),
    ("yaw_approach_3", 60, 0.063, False),
    ("attempt_3_diagnostic", 56, 0.0, True),
    ("cpu_probe_approach", 60, 0.0, True),
    ("control_profiled_recede", 60, 0.0, True),
    ("mid_center_approach_1", 60, 0.0, True),
    ("near_lateral_approach_1", 60, 0.0, True),
    ("near_center_approach_1", 60, 0.0, True),
]


@pytest.mark.parametrize("name,median_count,starved_fraction,expected", MEASURED_ATTEMPTS)
def test_gate_matches_measured_stage_2a_outcomes(
    tmp_path: Path,
    name: str,
    median_count: int,
    starved_fraction: float,
    expected: bool,
) -> None:
    result = readiness_for(tmp_path, name, build_counts(median_count, starved_fraction))
    assert result["satisfied"] is expected, (
        f"{name}: median={median_count} starved={starved_fraction:.1%} "
        f"-> {result['reason']}"
    )


def test_gate_accepts_the_run_a_geometric_threshold_would_have_rejected(
    tmp_path: Path,
) -> None:
    """A 0.529 m camera keeps 56 of 60 anchors and must not be rejected.

    Requiring every ground ray to clear `minimum_range_m` needs roughly 0.64 m at
    this gimbal pitch, which would throw away a run whose anchor yield never came
    near the calibrator floor.
    """

    result = readiness_for(tmp_path, "attempt_3", build_counts(56, 0.0))
    assert result["satisfied"] is True
    assert result["starved_frame_fraction"] <= GROUND_ANCHOR_STARVED_FRAME_FRACTION
