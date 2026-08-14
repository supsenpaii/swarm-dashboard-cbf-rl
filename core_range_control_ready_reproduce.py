"""CORE_RANGE_CONTROL_READY_OBSERVATION_PIPELINE, Phase 0.

Verifies the Candidate B (CANDIDATE_B_GROUP_AND_DISTANCE_BALANCED) artifact
from the prior stable_relative_tracking task is intact and reproducible:
loads the 3 saved fold boosters + fold preprocessors (no retraining), rebuilds
the frozen corpus + fold assignment exactly as before, recomputes group-disjoint
out-of-fold predictions, and compares them bit-for-bit (within float tolerance)
against the already-published prediction_rows.csv / raw_candidate_metrics.csv.

Never guesses model file names -- reads training_precommit.json /
feature_contract.json for the feature order and hyperparameter spec, and
enumerates the models/ directory for the actual saved artifact names.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import xgboost as xgb

import core_range_stable_relative_tracking as s
from core_range_dynamic_robust_retrain import CLIP_BOUNDS, VariantPreprocessor

WORKSPACE = Path(__file__).resolve().parent
SRC = WORKSPACE / "artifacts/core_range_3_12m/stable_relative_tracking"
OUT = WORKSPACE / "artifacts/core_range_3_12m/control_ready_observation"
CANDIDATE = "CANDIDATE_B_GROUP_AND_DISTANCE_BALANCED"
TOLERANCE_M = 1e-4


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_preprocessor(path: Path) -> VariantPreprocessor:
    payload = json.loads(path.read_text())
    feature_names = tuple(payload["feature_names"])
    medians = np.asarray([payload["medians"][name] for name in feature_names], dtype=np.float64)
    return VariantPreprocessor(feature_names, medians)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    checksums: dict[str, str] = {}

    # --- locate and checksum artifact files (no guessing: enumerate + read manifest) ---
    models_dir = SRC / "models"
    model_files = sorted(models_dir.glob(f"{CANDIDATE}_fold_*.json"))
    fold_model_files = [p for p in model_files if "_preprocessing" not in p.name]
    fold_preproc_files = [p for p in model_files if "_preprocessing" in p.name]
    if len(fold_model_files) != 3 or len(fold_preproc_files) != 3:
        raise SystemExit(f"CANDIDATE_B_REPRODUCTION_FAILED:model_file_count:{len(fold_model_files)}:{len(fold_preproc_files)}")
    for p in fold_model_files + fold_preproc_files:
        checksums[p.name] = sha256_file(p)

    feature_contract = json.loads((SRC / "feature_contract.json").read_text())
    training_precommit = json.loads((SRC / "training_precommit.json").read_text())
    if tuple(feature_contract["feature_order"]) != s.STABLE_FEATURES:
        raise SystemExit("CANDIDATE_B_REPRODUCTION_FAILED:feature_contract_mismatch")
    if tuple(training_precommit["feature_order"]) != s.STABLE_FEATURES:
        raise SystemExit("CANDIDATE_B_REPRODUCTION_FAILED:training_precommit_feature_order_mismatch")

    forbidden = set(feature_contract["excluded_by_task_spec"])
    if forbidden & set(s.STABLE_FEATURES):
        raise SystemExit("CANDIDATE_B_REPRODUCTION_FAILED:forbidden_feature_present")

    # --- rebuild corpus + fold assignment exactly as the training task did ---
    combined_rows, fold_map = s.load_corpus()

    # --- reload (not retrain) fold boosters/preprocessors, recompute OOF ---
    boosters: dict[int, xgb.Booster] = {}
    preprocessors: dict[int, VariantPreprocessor] = {}
    for fold in range(3):
        booster = xgb.Booster()
        booster.load_model(str(models_dir / f"{CANDIDATE}_fold_{fold}.json"))
        boosters[fold] = booster
        preprocessors[fold] = load_preprocessor(models_dir / f"{CANDIDATE}_fold_{fold}_preprocessing.json")

    n = len(combined_rows)
    oof = np.full(n, np.nan, dtype=np.float64)
    for fold in range(3):
        test_idx = [i for i, r in enumerate(combined_rows) if fold_map[r["group_id"]] == fold]
        test_rows = [combined_rows[i] for i in test_idx]
        x = preprocessors[fold].transform(test_rows)
        pred = np.clip(np.asarray(boosters[fold].predict(xgb.DMatrix(x)), dtype=np.float64), *CLIP_BOUNDS)
        oof[test_idx] = pred
    if np.any(~np.isfinite(oof)):
        raise SystemExit("CANDIDATE_B_REPRODUCTION_FAILED:oof_incomplete")

    # --- compare against published prediction_rows.csv (candidate B, calibration=none) ---
    import csv
    published: dict[tuple[str, float], float] = {}
    with (SRC / "prediction_rows.csv").open() as stream:
        for row in csv.DictReader(stream):
            if row["candidate"] != CANDIDATE:
                continue
            key = (row["group_id"], float(row["measurement_timestamp_s"]))
            published[key] = float(row["pred_none_m"])

    if len(published) != n:
        raise SystemExit(f"CANDIDATE_B_REPRODUCTION_FAILED:published_row_count_mismatch:{len(published)}:{n}")

    max_abs_diff = 0.0
    mismatches = 0
    for i, row in enumerate(combined_rows):
        key = (row["group_id"], float(row["measurement_timestamp_s"]))
        if key not in published:
            mismatches += 1
            continue
        diff = abs(float(oof[i]) - published[key])
        max_abs_diff = max(max_abs_diff, diff)
        if diff > TOLERANCE_M:
            mismatches += 1

    gt = np.asarray([float(r["ground_truth_range_m"]) for r in combined_rows])
    abs_err = np.abs(oof - gt)
    reproduced_metrics = {
        "mae_m": round(float(np.mean(abs_err)), 4),
        "medae_m": round(float(np.median(abs_err)), 4),
        "p90_m": round(float(np.percentile(abs_err, 90)), 4),
        "bias_m": round(float(np.mean(oof - gt)), 4),
        "std_m": round(float(np.std(oof - gt)), 4),
    }
    raw_metrics_rows = list(csv.DictReader((SRC / "raw_candidate_metrics.csv").open()))
    published_metrics = next(r for r in raw_metrics_rows if r["candidate"] == CANDIDATE and r["calibration"] == "none")
    metrics_match = all(
        abs(float(published_metrics[k]) - reproduced_metrics[k]) <= TOLERANCE_M for k in reproduced_metrics
    )

    reproduction_ok = mismatches == 0 and metrics_match
    if not reproduction_ok:
        result = {
            "reproduced": False, "mismatches": mismatches, "max_abs_diff_m": max_abs_diff,
            "reproduced_metrics": reproduced_metrics, "published_metrics": published_metrics,
        }
        (OUT / "candidate_b_reproduction.json").write_text(json.dumps(result, indent=2) + "\n")
        raise SystemExit("CANDIDATE_B_REPRODUCTION_FAILED:prediction_or_metric_mismatch")

    result = {
        "reproduced": True,
        "candidate": CANDIDATE,
        "n_rows": n,
        "n_groups": len(fold_map),
        "max_abs_diff_m": max_abs_diff,
        "tolerance_m": TOLERANCE_M,
        "reproduced_metrics": reproduced_metrics,
        "published_metrics": {k: float(v) for k, v in published_metrics.items() if k in reproduced_metrics},
        "feature_order": list(s.STABLE_FEATURES),
        "no_forbidden_feature_present": True,
        "no_future_feature": "per-frame features only, no lookahead in feature extraction (unchanged from stable_relative_tracking task)",
        "no_session_scenario_group_id_feature": True,
        "no_bbox_size_feature": True,
        "session_group_mapping": "group_id == session_id for both static and dynamic rows (core_range_5hz_headless_retrain.load_combined_rows)",
        "timestamp_field": "measurement_timestamp_s (sim time, monotonic within session/group, no live capture/consume timestamp present in frozen corpus)",
        "measurement_age_field_available_in_corpus": False,
        "measurement_age_note": "Frozen corpus records a single measurement_timestamp_s per frame (sim time at capture); no separate capture-vs-consume timestamp is present. Offline replay (Phase 2) must synthesize measurement_age_s per a precommitted, disclosed methodology -- see offline_precommit.json.",
    }
    (OUT / "candidate_b_reproduction.json").write_text(json.dumps(result, indent=2) + "\n")
    (OUT / "artifact_checksums.json").write_text(json.dumps({
        "candidate": CANDIDATE,
        "model_and_preprocessing_files": checksums,
        "source_prediction_rows_csv_sha256": sha256_file(SRC / "prediction_rows.csv"),
        "source_raw_candidate_metrics_csv_sha256": sha256_file(SRC / "raw_candidate_metrics.csv"),
        "source_feature_contract_json_sha256": sha256_file(SRC / "feature_contract.json"),
        "source_training_precommit_json_sha256": sha256_file(SRC / "training_precommit.json"),
    }, indent=2) + "\n")

    print("CANDIDATE_B_REPRODUCED_OK")
    print(json.dumps(reproduced_metrics, indent=2))
    print(f"max_abs_diff_m={max_abs_diff:.8f}")


if __name__ == "__main__":
    main()
