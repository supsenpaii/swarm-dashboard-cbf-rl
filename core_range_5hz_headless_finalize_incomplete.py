"""Finalize CORE_RANGE_5HZ_HEADLESS_DYNAMIC_RECOLLECTION_AND_RETRAIN as an
incomplete corpus: writes the skipped-training deliverables without fitting
anything, mirroring core_range_5hz_retrain_finalize.py's discipline for the
prior (also-incomplete) 5 Hz attempt.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

from core_range_dynamic_robust_retrain import (
    BBOX_GATES, CLIP_BOUNDS, DYNAMIC_GATES, PHYSICAL_ONLY_FEATURES,
    PHYSICAL_TEMPORAL_FEATURES, STATIC_GATES, TEMPORAL_EXTRA_FEATURES, TEMPORAL_GATES,
)
from core_range_xgboost_benchmark import HYPERPARAMETERS

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "artifacts/core_range_3_12m/dynamic_5hz_headless_retrain"
RETRAIN_ID = "core_range_5hz_headless_dynamic_retrain_20260805_v001"
CONCLUSION = "DATA_COLLECTION_BLOCKED_BY_PERSISTENT_RTF_STUTTER"


def write_json(name: str, value: object) -> None:
    (OUT / name).write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def write_csv(name: str, headers: list[str]) -> None:
    with (OUT / name).open("w", newline="", encoding="utf-8") as stream:
        csv.DictWriter(stream, fieldnames=headers).writeheader()


def main() -> None:
    manifest = json.loads((OUT / "dynamic_session_manifest.json").read_text())
    accepted = manifest["accepted_sessions"]

    write_json("frozen_dataset_manifest.json", {
        "frozen": False, "training_allowed": False,
        "static_verified_group_count": 27, "static_verified_frame_count": 832,
        "dynamic_accepted_group_count": manifest["accepted_group_count"],
        "dynamic_accepted_frame_count": sum(int(r["frame_count"]) for r in accepted),
        "dynamic_accepted_scenario_counts": manifest["accepted_scenario_counts"],
        "quarantine_included": False, "historical_2hz_included": False, "old_5hz_quarantine_included": False,
        "reason": "5 Hz headless dynamic data gate failed after all three missing groups exhausted 8 attempts each; persistent collection-window RTF stutter prevented acceptance",
    })
    write_json("feature_contract.json", {
        "status": "PRECOMMITTED_NOT_FIT",
        "PHYSICAL_ONLY": {"feature_order": list(PHYSICAL_ONLY_FEATURES), "bbox_width_height_area_aspect_excluded": True},
        "PHYSICAL_TEMPORAL": {
            "feature_order": list(PHYSICAL_TEMPORAL_FEATURES), "causal_only": True,
            "temporal_features": list(TEMPORAL_EXTRA_FEATURES), "reset_policy": "reset_at_session_boundary",
        },
        "clip_m": list(CLIP_BOUNDS), "seed": 52,
        "missing_value_policy": "training-only preprocessor median; not fit",
        "forbidden_features": ["bbox_width", "bbox_height", "bbox_area", "bbox_aspect_ratio", "session_id", "scenario_label", "distance_bin", "nominal_distance", "GT-derived runtime features", "future-frame features"],
        "hyperparameter_configs_precommitted": list(HYPERPARAMETERS),
    })
    write_csv("fold_assignments.csv", ["group_id", "fold", "domain"])
    write_csv("static_metrics.csv", ["variant", "config", "group_id", "metric", "value"])
    write_csv("dynamic_metrics.csv", ["variant", "config", "scenario_type", "group_id", "metric", "value"])
    write_csv("temporal_metrics.csv", ["variant", "config", "kind", "session_id", "metric", "value"])
    write_csv("bbox_stress_metrics.csv", ["variant", "config", "perturbation", "metric", "value"])
    write_csv("prediction_rows.csv", ["variant", "config", "group_id", "ground_truth_range_m", "oof_prediction_m"])
    write_csv("model_comparison.csv", ["variant", "config", "passed_all_gates", "reason_not_trained"])

    write_json("retrain_manifest.json", {
        "retrain_id": RETRAIN_ID,
        "conclusion": CONCLUSION,
        "training_started": False, "model_files_created": False, "candidate_selected": False,
        "accepted_group_count": manifest["accepted_group_count"],
        "accepted_scenario_counts": manifest["accepted_scenario_counts"],
        "required_scenario_counts": {"approaching": 3, "receding": 3, "stop_and_hold": 2},
        "quarantined_attempt_count": len(manifest["quarantined_sessions"]),
        "maximum_attempts_per_missing_group": 8,
        "focused_tests": "21 passed",
        "full_repository_tests": "417 passed, 1 pre-existing PytestReturnNotNoneWarning",
        "full_repository_test_command": "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q --ignore=swarm_dashboard_handoff_20260729",
        "run_all_check": "PASS",
        "unscoped_pytest_discovery_note": "An initial unscoped pytest invocation collected the historical swarm_dashboard_handoff_20260729 snapshot and failed on duplicate module names; the current-source suite excludes that archival snapshot.",
        "reason_not_trained": (
            "data gate not met after continuation retries: approaching 3/3 PASS, receding 1/3 FAIL, "
            "stop_and_hold 1/2 FAIL. Each missing group exhausted 8 total attempts. Of 18 continuation "
            "attempts, 16 failed the RTF/long-stutter hard gate, one failed calibration prewarm, and one "
            "failed stop-hold evidence. The five accepted sources remain unchanged and integrity PASS; "
            "quarantine was not used for training."
        ),
        "static_gates": STATIC_GATES, "dynamic_gates": DYNAMIC_GATES,
        "temporal_gates": TEMPORAL_GATES, "bbox_gates": BBOX_GATES,
        "depth_rate_changed": False, "effective_configured_depth_rate_hz": 5.0,
        "gazebo_gui_used": False,
        "shadow": False, "active_runtime_integration": False, "follow_target": False,
    })

    (OUT / "retrain_report.md").write_text(
        "# CORE_RANGE_5HZ_HEADLESS_DYNAMIC_RETRAIN report\n\n"
        f"Conclusion: `{CONCLUSION}`\n\n"
        "The five previously accepted groups were checksum-verified and left unchanged. "
        "The three missing groups were retried with full-stack restart, calibration prewarm, "
        "immediate pre-capture RTF gate, continuous collection-window RTF monitoring, and a "
        "maximum of 8 total attempts per group. All three exhausted the budget: 16 of the 18 "
        "continuation attempts failed RTF median and/or a >=3-sample low-RTF stutter streak; "
        "one failed prewarm and one failed stop-hold evidence. Corpus remains 5/8 groups / "
        "570 accepted frames with integrity PASS. Training was not started and quarantine was "
        "not included.\n\n"
        "See docs/CORE_RANGE_5HZ_HEADLESS_DYNAMIC_RETRAIN_REPORT.md for the full account.\n",
        encoding="utf-8",
    )
    print(json.dumps({"conclusion": CONCLUSION}, indent=2))


if __name__ == "__main__":
    main()
