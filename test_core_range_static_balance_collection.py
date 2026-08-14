import json
from pathlib import Path

import pytest

from core_range_collect_static_balance import archive_runtime_logs, disk_preflight
from core_range_static_balance_audit import (
    bin_contains,
    load_effective_plan,
    percentile,
    validate_precommit,
)


PLAN = Path("artifacts/core_range_3_12m/static_balance_collection/collection_plan.json")
AMENDMENT = Path("artifacts/core_range_3_12m/static_balance_collection/collection_plan_amendment_v001.json")


def test_precommitted_manifest_has_exactly_18_balanced_sessions() -> None:
    plan = json.loads(PLAN.read_text(encoding="utf-8"))
    validate_precommit(plan)
    assert len(plan["sessions"]) == 18
    assert len({row["session_id"] for row in plan["sessions"]}) == 18
    assert len({row["group_id"] for row in plan["sessions"]}) == 18


def test_gt_bin_boundaries_are_left_closed_right_open() -> None:
    assert bin_contains("3-4m", 3.0)
    assert bin_contains("3-4m", 3.999999)
    assert not bin_contains("3-4m", 4.0)
    assert bin_contains("9-10m", 9.0)
    assert not bin_contains("9-10m", 10.0)


def test_technical_replacements_preserve_effective_18_session_balance() -> None:
    effective, amendment = load_effective_plan(PLAN, AMENDMENT)
    assert amendment is not None
    assert len(effective["sessions"]) == 18
    assert {row["replacement_for"] for row in amendment["replacements"].values()} == {
        "crsb35_lateral_left_p11",
        "crsb75_center_p10",
        "crsb85_oblique_p12",
    }
    assert all(row["raw_accuracy_metrics_used"] is False for row in amendment["replacements"].values())


def test_equal_group_percentile_is_deterministic() -> None:
    assert percentile([1.0, 2.0, 3.0], 0.90) == pytest.approx(2.8)


def test_archive_is_read_tested_before_raw_logs_removed(tmp_path: Path) -> None:
    logs = tmp_path / "runtime_logs"
    logs.mkdir()
    (logs / "bounded.log").write_text("safe\n", encoding="utf-8")
    result = archive_runtime_logs(tmp_path)
    assert result["read_test"] == "PASS"
    assert not logs.exists()
    assert (tmp_path / "runtime_logs.tar.zst").is_file()


def test_disk_preflight_fails_closed_for_impossible_projection(tmp_path: Path) -> None:
    with pytest.raises(RuntimeError, match="disk_preflight_failed"):
        disk_preflight(tmp_path, 10**9)


def test_plan_forbids_training_outputs_and_keeps_residual_off() -> None:
    plan = json.loads(PLAN.read_text(encoding="utf-8"))
    assert plan["scope_guards"]["model_training"] is False
    assert plan["scope_guards"]["train_validation_test_split"] is False
    assert plan["scope_guards"]["residual_correction"] == "off"
    assert not any(PLAN.parent.glob("*.joblib"))
    assert not any(PLAN.parent.glob("*.model"))


def test_runtime_formula_and_controller_paths_are_not_changed_by_collection_patch() -> None:
    source = Path("metric_target_fusion.py").read_text(encoding="utf-8")
    assert "physics_slant_range_m=filtered_optical_depth_m*" in source
    assert "pipeline_config_fingerprint_sha256" in source
    collector = Path("core_range_collect_scenario.sh").read_text(encoding="utf-8")
    for forbidden in ("FOLLOW_TARGET", "OFFBOARD", "PARAM_SET", "takeoff"):
        assert forbidden not in collector
