"""CORE_RANGE_5HZ_HEADLESS_DYNAMIC_RECOLLECTION_AND_RETRAIN training driver.

Combines the verified static corpus (27 groups / 832 frames, via
core_range_xgboost_benchmark.collect_verified_rows) with the new headless
5 Hz dynamic corpus collected by
core_range_collect_dynamic_5hz_headless_batch.py (accepted_sessions.json
under artifacts/core_range_3_12m/dynamic_5hz_headless_retrain/), then
reuses -- unmodified, as pure functions -- the training/evaluation engine
already built and gate-calibrated by core_range_dynamic_robust_retrain.py:
add_temporal_features, assign_combined_folds, train_variant,
evaluate_config, oof_bbox_stress, and the four gate functions/thresholds.

This driver only supplies its own dynamic-corpus loader (the existing
_load_dynamic_rows in core_range_direct_dynamic_replay.py expects a
different, frozen-candidate-wrapped manifest schema this task does not use)
and its own precommit/output-writing glue, scoped to the two variants this
task asks for: PHYSICAL_ONLY and PHYSICAL_TEMPORAL. BBOX_AUGMENTED is not
trained (out of scope), but the bbox-robustness *gate* is still evaluated
for both trained variants via oof_bbox_stress, exactly as
core_range_dynamic_robust_retrain.py does.

No retraining/overwriting of the frozen static baseline candidate, no
runtime/shadow/Follow Target action.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from core_range_direct_dynamic_replay import _direction_row, _stop_row
from core_range_dynamic_robust_retrain import (
    BBOX_GATES,
    CLIP_BOUNDS,
    DYNAMIC_GATES,
    PHYSICAL_ONLY_FEATURES,
    PHYSICAL_TEMPORAL_FEATURES,
    STATIC_GATES,
    TEMPORAL_EXTRA_FEATURES,
    TEMPORAL_GATES,
    _bbox_gate,
    _dynamic_gate,
    _static_gate,
    _temporal_gate,
    add_temporal_features,
    assign_combined_folds,
    equal_group_aggregate,
    evaluate_config,
    oof_bbox_stress,
    per_group_rows,
    train_variant,
)
from core_range_logging_eval import load_jsonl, sha256_file, verify_record
from core_range_xgboost_benchmark import (
    BINS,
    FEATURE_NAMES,
    HYPERPARAMETERS,
    canonical_sha256,
    collect_verified_rows,
    extract_features,
)
from range_physical_diagnostics import validate_timestamp_stages

RETRAIN_ID = "core_range_5hz_headless_dynamic_retrain_20260805_v001"
SEED = 52
FEATURE_VARIANTS = {
    "PHYSICAL_ONLY": PHYSICAL_ONLY_FEATURES,
    "PHYSICAL_TEMPORAL": PHYSICAL_TEMPORAL_FEATURES,
}
VARIANT_PRIORITY = ("PHYSICAL_TEMPORAL", "PHYSICAL_ONLY")
CONCLUSION_BY_VARIANT = {
    "PHYSICAL_TEMPORAL": "PHYSICAL_TEMPORAL_5HZ_HEADLESS_BEST_CANDIDATE",
    "PHYSICAL_ONLY": "PHYSICAL_ONLY_5HZ_HEADLESS_BEST_CANDIDATE",
    None: "NO_5HZ_HEADLESS_DYNAMIC_ROBUST_MODEL_MEETS_GATE",
}


def _json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


# ---------------------------------------------------------------------------
# Dynamic corpus loader: this task's own accepted_sessions.json schema
# (core_range_collect_dynamic_5hz_headless_batch.py), re-verified with the
# same integrity checks _load_dynamic_rows uses, but without the
# frozen-candidate/dynamic_replay_plan wrapper that module requires.
# ---------------------------------------------------------------------------

def load_dynamic_rows(workspace: Path, dynamic_output: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    session_manifest = _json(dynamic_output / "dynamic_session_manifest.json")
    accepted = session_manifest.get("accepted_sessions") or []
    if not session_manifest.get("collection_complete"):
        raise ValueError("dynamic_collection_incomplete")
    all_rows: list[dict[str, Any]] = []
    traces: set[str] = set()
    groups: set[str] = set()
    for source in accepted:
        session_id = source["session_id"]
        root = workspace / source["source_root"]
        sidecar = root / "physical_diagnostics.jsonl"
        if sha256_file(sidecar) != source["source_sidecar_sha256"]:
            raise ValueError(f"dynamic_sidecar_checksum_mismatch:{session_id}")
        loaded, malformed = load_jsonl(sidecar)
        if malformed:
            raise ValueError(f"dynamic_sidecar_malformed:{session_id}")
        runtime_session_id = int(source["runtime_session_id"])
        rows = [
            row for row in loaded
            if row.get("stage") == "raw_range_computed" and int(row.get("session_id", -1)) == runtime_session_id
        ]
        if len(rows) < 40:
            raise ValueError(f"dynamic_frame_count_failed:{session_id}")
        identities = {(str(row.get("run_id")), int(row.get("session_id")), str(row.get("group_id"))) for row in rows}
        if len(identities) != 1 or session_id in groups:
            raise ValueError(f"dynamic_group_independence_failed:{session_id}")
        groups.add(session_id)
        previous_timestamp = -math.inf
        for row in rows:
            record_ok, trace_ok = verify_record(row)
            timestamp_ok, _ = validate_timestamp_stages(row.get("timestamp_stages") or {})
            if not record_ok or not trace_ok or not timestamp_ok or row.get("ground_truth_trace_valid") is not True:
                raise ValueError(f"dynamic_integrity_failed:{session_id}")
            if len(((row.get("anchors") or {}).get("per_grid_point") or [])) != 96:
                raise ValueError(f"dynamic_anchor_count_failed:{session_id}")
            trace = str(row["trace_identity_sha256"])
            if trace in traces:
                raise ValueError(f"dynamic_duplicate_trace:{trace}")
            traces.add(trace)
            timestamp = float(row["measurement_timestamp_s"])
            if timestamp <= previous_timestamp:
                raise ValueError(f"dynamic_measurement_timestamp_nonmonotonic:{session_id}")
            previous_timestamp = timestamp
            features = extract_features(row)
            gt = float(row["ground_truth"]["distance_m"])
            raw_range = float(row["raw_range"]["physics_slant_range_m"])
            if not math.isfinite(gt) or not math.isfinite(raw_range):
                raise ValueError(f"dynamic_nonfinite_range:{session_id}")
            all_rows.append({
                "session_id": session_id, "group_id": session_id,
                "scenario_type": source["scenario_type"], "context": source["context"],
                "measurement_timestamp_s": timestamp,
                "ground_truth_range_m": gt, "raw_physical_range_m": raw_range,
                "trace_identity_sha256": trace, "record_sha256": str(row["record_sha256"]),
                "features": features,
            })
    all_rows.sort(key=lambda row: (row["session_id"], row["measurement_timestamp_s"]))
    return all_rows, session_manifest


def load_combined_rows(workspace: Path, dynamic_output: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    static_raw, static_manifest = collect_verified_rows(workspace)
    if static_manifest["group_count"] != 27 or static_manifest["frame_count"] != 832:
        raise ValueError(f"static_corpus_shape_unexpected:{static_manifest['group_count']}:{static_manifest['frame_count']}")
    dynamic_raw, dynamic_session_manifest = load_dynamic_rows(workspace, dynamic_output)
    counts = dynamic_session_manifest.get("accepted_scenario_counts") or {}
    if dynamic_session_manifest["accepted_group_count"] != 8 or counts != {"approaching": 3, "receding": 3, "stop_and_hold": 2}:
        raise ValueError(f"dynamic_corpus_shape_unexpected:{dynamic_session_manifest['accepted_group_count']}:{counts}")

    combined: list[dict[str, Any]] = []
    for row in static_raw:
        features = {name: row[name] for name in FEATURE_NAMES}
        combined.append({
            "domain": "static", "group_id": row["logical_session_id"], "session_id": row["logical_session_id"],
            "scenario_type": "static", "context": row["distance_bin"], "distance_bin": row["distance_bin"],
            "measurement_timestamp_s": row["measurement_timestamp_s"],
            "ground_truth_range_m": row["ground_truth_range_m"], "raw_physical_range_m": row["raw_physical_range_m"],
            "features": dict(features),
        })
    for row in dynamic_raw:
        combined.append({
            "domain": "dynamic", "group_id": row["session_id"], "session_id": row["session_id"],
            "scenario_type": row["scenario_type"], "context": row["context"], "distance_bin": None,
            "measurement_timestamp_s": row["measurement_timestamp_s"],
            "ground_truth_range_m": row["ground_truth_range_m"], "raw_physical_range_m": row["raw_physical_range_m"],
            "features": dict(row["features"]),
        })

    manifest = {
        "static_group_count": static_manifest["group_count"], "static_frame_count": static_manifest["frame_count"],
        "dynamic_session_manifest_sha256": canonical_sha256(dynamic_session_manifest),
        "dynamic_accepted_group_count": dynamic_session_manifest["accepted_group_count"],
        "dynamic_accepted_frame_count": len(dynamic_raw),
        "dynamic_accepted_scenario_counts": counts,
        "total_group_count": len({row["group_id"] for row in combined}),
        "total_frame_count": len(combined),
    }
    return combined, manifest


# ---------------------------------------------------------------------------
# prepare / run
# ---------------------------------------------------------------------------

def prepare(workspace: Path, output: Path, dynamic_output: Path) -> dict[str, Any]:
    combined_rows, dataset_manifest = load_combined_rows(workspace, dynamic_output)
    fold_map = assign_combined_folds(workspace, combined_rows, seed=SEED)
    fold_rows = [{"group_id": group, "fold": fold, "domain": next(r["domain"] for r in combined_rows if r["group_id"] == group)} for group, fold in sorted(fold_map.items())]
    _write_csv(output / "fold_assignments.csv", fold_rows)
    _write_json(output / "frozen_dataset_manifest.json", {
        "frozen": True, "training_allowed": True, "seed": SEED,
        **dataset_manifest,
        "quarantine_included": False, "historical_2hz_included": False, "old_5hz_quarantine_included": False,
    })
    _write_json(output / "feature_contract.json", {
        "status": "PRECOMMITTED",
        "PHYSICAL_ONLY": {"feature_order": list(PHYSICAL_ONLY_FEATURES), "bbox_width_height_area_aspect_excluded": True},
        "PHYSICAL_TEMPORAL": {
            "feature_order": list(PHYSICAL_TEMPORAL_FEATURES), "causal_only": True,
            "temporal_features": list(TEMPORAL_EXTRA_FEATURES), "reset_policy": "reset_at_session_boundary",
        },
        "clip_m": list(CLIP_BOUNDS), "seed": SEED,
        "missing_value_policy": "training-only preprocessor median (fit on train folds only) plus a missingness indicator column",
        "forbidden_features": ["bbox_width", "bbox_height", "bbox_area", "bbox_aspect_ratio", "session_id", "scenario_label", "distance_bin", "nominal_distance", "GT-derived runtime features", "future-frame features"],
        "hyperparameter_configs_precommitted": list(HYPERPARAMETERS),
    })
    plan = {
        "retrain_id": RETRAIN_ID, "seed": SEED, "created_before_fit": True,
        "variants": list(FEATURE_VARIANTS), "variant_priority": list(VARIANT_PRIORITY),
        "hyperparameter_configs": HYPERPARAMETERS,
        "static_gates": STATIC_GATES, "dynamic_gates": DYNAMIC_GATES,
        "temporal_gates": TEMPORAL_GATES, "bbox_gates": BBOX_GATES,
        "dataset_manifest_sha256": canonical_sha256(dataset_manifest),
        "fold_assignments_sha256": canonical_sha256(fold_rows),
    }
    _write_json(output / "retrain_plan.json", plan)
    return plan


def run(workspace: Path, output: Path, dynamic_output: Path) -> dict[str, Any]:
    plan = _json(output / "retrain_plan.json")
    combined_rows, dataset_manifest = load_combined_rows(workspace, dynamic_output)
    if canonical_sha256(dataset_manifest) != plan["dataset_manifest_sha256"]:
        raise ValueError("source_dataset_changed_after_precommit")
    add_temporal_features(combined_rows)
    fold_map: dict[str, int] = {}
    with (output / "fold_assignments.csv").open() as stream:
        for row in csv.DictReader(stream):
            fold_map[row["group_id"]] = int(row["fold"])
    if set(fold_map) != {row["group_id"] for row in combined_rows}:
        raise ValueError("fold_group_mapping_incomplete")

    import xgboost as xgb

    models_dir = output / "models"
    models_dir.mkdir(exist_ok=True)

    all_static_metric_rows: list[dict[str, Any]] = []
    all_bin_rows: list[dict[str, Any]] = []
    all_dynamic_metric_rows: list[dict[str, Any]] = []
    all_temporal_rows: list[dict[str, Any]] = []
    all_bbox_rows: list[dict[str, Any]] = []
    model_comparison: list[dict[str, Any]] = []
    prediction_rows: list[dict[str, Any]] = []
    per_config_state: dict[tuple[str, str], dict[str, Any]] = {}

    for variant_name, feature_names in FEATURE_VARIANTS.items():
        trained = train_variant(xgb, variant_name, feature_names, combined_rows, fold_map)
        for config_name in HYPERPARAMETERS:
            oof_prediction = trained["oof"][config_name]
            evaluation = evaluate_config(combined_rows, oof_prediction)
            bbox_stress = oof_bbox_stress(xgb, combined_rows, fold_map, trained["boosters"], trained["preprocessors"], config_name)
            bbox_gate = _bbox_gate(bbox_stress)
            for row in bbox_stress:
                all_bbox_rows.append({"variant": variant_name, "config": config_name, **row})
            for row in evaluation["static_groups"]:
                all_static_metric_rows.append({"variant": variant_name, **row, "config": config_name})
            for row in evaluation["static_bin_rows"]:
                all_bin_rows.append({"variant": variant_name, "config": config_name, **row})
            for row in evaluation["dynamic_groups"]:
                all_dynamic_metric_rows.append({"variant": variant_name, **row, "config": config_name})
            for row in evaluation["direction_rows"]:
                all_temporal_rows.append({"variant": variant_name, "config": config_name, "kind": "direction", **row})
            for row in evaluation["stop_rows"]:
                all_temporal_rows.append({"variant": variant_name, "config": config_name, "kind": "stop", **row})
            passed_all = (
                evaluation["static_gate"]["passed"] and evaluation["dynamic_gate"]["passed"]
                and evaluation["temporal_gate"]["passed"] and bbox_gate["passed"]
            )
            per_config_state[(variant_name, config_name)] = {
                "trained": trained, "evaluation": evaluation, "bbox_gate": bbox_gate, "passed_all": passed_all,
            }
            model_comparison.append({
                "variant": variant_name, "config": config_name, "passed_all_gates": passed_all,
                "static_mae_m": evaluation["static_gate"]["equal_group_mae_m"], "static_passed": evaluation["static_gate"]["passed"],
                "dynamic_mae_m": evaluation["dynamic_gate"]["equal_group_mae_m"], "dynamic_passed": evaluation["dynamic_gate"]["passed"],
                "median_absolute_lag_s": evaluation["temporal_gate"]["median_absolute_lag_s"], "temporal_passed": evaluation["temporal_gate"]["passed"],
                "bbox_pm10_catastrophic_fraction": bbox_gate["worst_pm10_catastrophic_error_fraction"], "bbox_passed": bbox_gate["passed"],
            })
            for index, row in enumerate(combined_rows):
                prediction_rows.append({
                    "variant": variant_name, "config": config_name, "group_id": row["group_id"], "domain": row["domain"],
                    "scenario_type": row["scenario_type"], "fold": fold_map[row["group_id"]],
                    "ground_truth_range_m": row["ground_truth_range_m"], "raw_physical_range_m": row["raw_physical_range_m"],
                    "oof_prediction_m": float(oof_prediction[index]),
                })

    selected_variant = None
    selected_config = None
    for variant_name in VARIANT_PRIORITY:
        candidates = [
            (config_name, per_config_state[(variant_name, config_name)])
            for config_name in HYPERPARAMETERS if per_config_state[(variant_name, config_name)]["passed_all"]
        ]
        if candidates:
            candidates.sort(key=lambda item: (
                item[1]["evaluation"]["dynamic_gate"]["equal_group_mae_m"],
                item[1]["evaluation"]["static_gate"]["equal_group_mae_m"], item[0],
            ))
            selected_variant, (selected_config, _state) = variant_name, candidates[0]
            break
    conclusion = CONCLUSION_BY_VARIANT[selected_variant]

    frozen_model_checksum = None
    if selected_variant is not None:
        state = per_config_state[(selected_variant, selected_config)]
        trained = state["trained"]
        for fold in range(3):
            booster = trained["boosters"][(selected_config, fold)]
            preprocessor = trained["preprocessors"][(selected_config, fold)]
            booster.save_model(str(models_dir / f"{selected_variant}_{selected_config}_fold_{fold}.json"))
            train_groups_for_fold = sorted({row["group_id"] for row in combined_rows if fold_map[row["group_id"]] != fold})
            (models_dir / f"{selected_variant}_{selected_config}_fold_{fold}_preprocessing.json").write_text(
                json.dumps(preprocessor.as_json(train_groups_for_fold), indent=2, sort_keys=True) + "\n", encoding="utf-8",
            )
        model_files = sorted(models_dir.glob(f"{selected_variant}_{selected_config}_*"))
        frozen_model_checksum = canonical_sha256([sha256_file(p) for p in model_files])

    _write_csv(output / "static_metrics.csv", all_static_metric_rows)
    _write_csv(output / "per_bin_metrics.csv", all_bin_rows)
    _write_csv(output / "dynamic_metrics.csv", all_dynamic_metric_rows)
    _write_csv(output / "temporal_metrics.csv", all_temporal_rows)
    _write_csv(output / "bbox_stress_metrics.csv", all_bbox_rows)
    _write_csv(output / "model_comparison.csv", model_comparison)
    _write_csv(output / "prediction_rows.csv", prediction_rows)

    report_lines = [
        "# CORE_RANGE_5HZ_HEADLESS_DYNAMIC_RETRAIN report", "",
        f"Conclusion: `{conclusion}`", "",
        (f"Selected: `{selected_variant}` / `{selected_config}`" if selected_variant else "No configuration passed all gates."),
        "", "## Model comparison", "",
        "| Variant | Config | Static MAE | Dynamic MAE | Median lag (s) | BBox pm10 catastrophic | All gates |",
        "|---|---|---:|---:|---:|---:|---|",
    ]
    for row in model_comparison:
        report_lines.append(
            f"| {row['variant']} | {row['config']} | {row['static_mae_m']:.3f} | {row['dynamic_mae_m']:.3f} | "
            f"{row['median_absolute_lag_s']:.3f} | {100*row['bbox_pm10_catastrophic_fraction']:.2f}% | "
            f"{'PASS' if row['passed_all_gates'] else 'FAIL'} |"
        )
    report_lines += ["", "## Scope guards", "", "No baseline-candidate overwrite; no runtime, shadow or Follow Target action.", ""]
    (output / "retrain_report.md").write_text("\n".join(report_lines), encoding="utf-8")

    output_files = [p for p in output.rglob("*") if p.is_file() and p.name != "retrain_manifest.json"]
    manifest = {
        "retrain_id": RETRAIN_ID, "conclusion": conclusion,
        "selected_variant": selected_variant, "selected_config": selected_config,
        "frozen_model_checksum": frozen_model_checksum,
        "model_comparison": model_comparison, "counts": dataset_manifest,
        "scope_guards": {
            "additional_data_collection": False, "runtime_modified": False, "controller_effect": False,
            "shadow": False, "follow_target": False, "final_holdout": False, "gazebo_gui_used": False,
            "depth_rate_hz": 5.0,
        },
        "artifacts": {str(p.relative_to(output)): sha256_file(p) for p in sorted(output_files)},
    }
    _write_json(output / "retrain_manifest.json", manifest)
    print(json.dumps({"conclusion": conclusion, "selected_variant": selected_variant, "selected_config": selected_config}, indent=2))
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    for command in ("prepare", "run"):
        child = sub.add_parser(command)
        child.add_argument("--workspace", type=Path, required=True)
        child.add_argument("--output", type=Path, required=True)
        child.add_argument("--dynamic-output", type=Path, required=True)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    output = args.output.resolve()
    dynamic_output = args.dynamic_output.resolve()
    if args.command == "prepare":
        prepare(workspace, output, dynamic_output)
    else:
        run(workspace, output, dynamic_output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
