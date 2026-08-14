import json
from pathlib import Path

from core_range_logging_eval import evaluate
from range_physical_diagnostics import (
    RangePhysicalDiagnosticsCollector,
    TIMESTAMP_STAGE_ORDER,
    timestamp_stage,
)


def _runtime(tmp_path: Path) -> Path:
    root = tmp_path / "runtime"
    collector = RangePhysicalDiagnosticsCollector(
        root, run_id="smoke", target_id="UAV-02"
    )
    collector._prepare()
    stages = {
        name: timestamp_stage(
            10.0 + index * 0.01,
            component="test",
            execution_context="test-thread",
            semantic=name,
        )
        for index, name in enumerate(TIMESTAMP_STAGE_ORDER[:-1])
    }
    for index in range(20):
        assert collector.record(
            {
                "diagnostics_schema_version": collector.status()["schema_version"],
                "instrumentation_only": True,
                "run_id": "smoke",
                "target_id": "UAV-02",
                "group_id": "UAV-01.2",
                "session_id": 2,
                "frame_index": index,
                "measurement_timestamp_s": 10.0,
                "source_sim_timestamp_s": 20.0,
                "stage": "raw_range_computed",
                "timestamp_stages": stages,
                "anchors": {"per_grid_point": [{} for _ in range(96)]},
                "raw_range": {"physics_slant_range_m": 5.5},
                "ground_truth": {
                    "valid": True,
                    "distance_m": 6.0,
                    "timestamp_s": 20.0,
                    "timestamp_clock_id": "gazebo_sim_time",
                },
            }
        )
    assert collector.finalize_integrity()["pass"]
    (root / "capture_events.jsonl").write_text(
        json.dumps(
            {
                "session_id": 2,
                "armed_observations_before": [["uav1", False], ["uav2", False]],
                "armed_observations_after": [["uav1", False], ["uav2", False]],
                "follow_endpoint_called": False,
                "motion_or_vehicle_control_endpoint_called": False,
            }
        )
        + "\n",
        encoding="utf-8",
    )
    return root


def test_smoke_evaluation_ready_is_deterministic(tmp_path: Path) -> None:
    summary = evaluate(_runtime(tmp_path))
    assert summary["conclusion"] == "CORE_LOGGING_READY"
    assert summary["counts"]["raw_session_rows"] == 20
    assert all(summary["gates"].values())


def test_smoke_evaluation_blocks_on_record_checksum_mismatch(tmp_path: Path) -> None:
    root = _runtime(tmp_path)
    path = root / "physical_diagnostics.jsonl"
    rows = path.read_text(encoding="utf-8").splitlines()
    first = json.loads(rows[0])
    first["raw_range"]["physics_slant_range_m"] = 50.0
    rows[0] = json.dumps(first, sort_keys=True, separators=(",", ":"))
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")
    summary = evaluate(root)
    assert summary["conclusion"] == "CORE_LOGGING_BLOCKED"
    assert not summary["gates"]["record_checksum_mismatches_zero"]
