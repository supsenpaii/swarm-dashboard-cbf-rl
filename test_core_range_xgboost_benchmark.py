from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path

import numpy as np
import pytest

import core_range_xgboost_benchmark as bench


def synthetic_rows() -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for lower in range(3, 12):
        label = f"{lower}-{lower + 1}m"
        for group_index in range(3):
            for frame in range(2):
                row: dict[str, object] = {
                    "logical_session_id": f"g{lower}_{group_index}",
                    "distance_bin": label,
                    "ground_truth_range_m": lower + 0.5,
                    "raw_physical_range_m": lower + 1.0 + 0.1 * frame,
                    "residual_gt_m": -0.5 - 0.1 * frame,
                    "direct_target_m": lower + 0.5,
                    "source_collection": "synthetic",
                }
                for index, name in enumerate(bench.FEATURE_NAMES):
                    row.setdefault(name, float(index + 1))
                rows.append(row)
    return rows


def test_group_disjoint_stratified_fold_assignment() -> None:
    rows = synthetic_rows()
    first = bench.assign_folds(rows)
    second = bench.assign_folds(list(reversed(rows)))
    assert first == second
    assert len(first) == 27
    mapping = {row["logical_session_id"]: row["fold"] for row in first}
    assert set(mapping) == {str(row["logical_session_id"]) for row in rows}
    for fold in range(3):
        selected = [row for row in first if row["fold"] == fold]
        assert len(selected) == 9
        assert {row["distance_bin"] for row in selected} == set(bench.BINS)
        assert all(sum(item["distance_bin"] == row["distance_bin"] for item in selected) == 1 for row in selected)


def test_no_duplicate_trace_between_train_and_test() -> None:
    rows = synthetic_rows()
    mapping = {row["logical_session_id"]: int(row["fold"]) for row in bench.assign_folds(rows)}
    for fold in range(3):
        train = {str(row["logical_session_id"]) for row in rows if mapping[str(row["logical_session_id"])] != fold}
        test = {str(row["logical_session_id"]) for row in rows if mapping[str(row["logical_session_id"])] == fold}
        assert train.isdisjoint(test)


def test_feature_contract_has_no_blacklisted_leakage() -> None:
    contract = bench._feature_contract()
    assert not (set(contract["feature_names"]) & bench.FEATURE_BLACKLIST)
    assert "ground_truth_range_m" not in contract["feature_names"]
    assert "distance_bin" not in contract["feature_names"]
    assert "logical_session_id" not in contract["feature_names"]


def test_preprocessing_fit_is_train_only_and_missing_is_explicit() -> None:
    train = synthetic_rows()[:4]
    test = synthetic_rows()[4:5]
    name = bench.FEATURE_NAMES[0]
    for row, value in zip(train, (1.0, 2.0, 3.0, None), strict=True):
        row[name] = value
    test[0][name] = 9999.0
    processor = bench.FoldPreprocessor.fit(train)
    assert processor.medians[0] == 2.0
    transformed = processor.transform([train[-1], test[0]])
    assert transformed[0, 0] == 2.0
    assert transformed[0, len(bench.FEATURE_NAMES)] == 1.0
    assert transformed[1, 0] == 9999.0
    assert transformed[1, len(bench.FEATURE_NAMES)] == 0.0


def test_seed_and_hyperparameters_are_frozen() -> None:
    assert bench.SEED == 52
    assert set(bench.HYPERPARAMETERS) == {"A_conservative", "B_balanced", "C_shallow"}
    assert bench.HYPERPARAMETERS["A_conservative"]["n_estimators"] == 200
    assert bench.HYPERPARAMETERS["B_balanced"]["max_depth"] == 3
    assert bench.HYPERPARAMETERS["C_shallow"]["max_depth"] == 1


def test_label_construction_and_residual_reconstruction() -> None:
    truth = 8.25
    raw = 12.0
    residual = truth - raw
    assert raw + residual == truth
    variants = bench._method_predictions(np.asarray([raw]), np.asarray([residual]), "residual")
    assert variants["residual_xgboost_unclipped"][0] == truth
    assert variants["residual_xgboost_clipped"][0] == truth


def test_clipping_policy_is_fair_and_legacy_is_diagnostic() -> None:
    raw = np.asarray([1.0, 13.0, 8.0])
    assert np.array_equal(np.clip(raw, 3.0, 12.0), np.asarray([3.0, 12.0, 8.0]))
    residual = bench._method_predictions(raw, np.asarray([10.0, -10.0, 10.0]), "residual")
    assert np.all((residual["residual_xgboost_clipped"] >= 3.0) & (residual["residual_xgboost_clipped"] <= 12.0))
    assert residual["residual_xgboost_legacy_clamp"][2] == 10.0
    direct = bench._method_predictions(raw, np.asarray([1.0, 13.0, 8.0]), "direct")
    assert np.array_equal(direct["direct_xgboost_clipped"], np.asarray([3.0, 12.0, 8.0]))


def test_equal_group_aggregation_not_frame_weighted() -> None:
    group_rows = [
        {key: 1.0 for key in ("signed_bias_m", "median_absolute_error_m", "mae_m", "rmse_m", "p90_abs_error_m", "p95_abs_error_m", "mean_absolute_relative_error", "error_gt_1m_fraction", "error_gt_2m_fraction", "error_gt_3m_fraction")},
        {key: 3.0 for key in ("signed_bias_m", "median_absolute_error_m", "mae_m", "rmse_m", "p90_abs_error_m", "p95_abs_error_m", "mean_absolute_relative_error", "error_gt_1m_fraction", "error_gt_2m_fraction", "error_gt_3m_fraction")},
    ]
    assert bench.equal_group_aggregate(group_rows)["mae_m"] == 2.0


def test_per_group_and_per_bin_isolation() -> None:
    rows = synthetic_rows()
    truth = np.asarray([float(row["ground_truth_range_m"]) for row in rows])
    groups = bench.per_group_metric_rows(rows, truth.copy(), "perfect")
    assert len(groups) == 27
    assert all(row["mae_m"] == 0.0 for row in groups)
    for label in bench.BINS:
        selected = [row for row in groups if row["distance_bin"] == label]
        assert len(selected) == 3
        assert bench.equal_group_aggregate(selected)["mae_m"] == 0.0


def test_model_artifact_checksum_and_prediction_traceability(tmp_path: Path) -> None:
    artifact = tmp_path / "model.json"
    artifact.write_text("offline-model", encoding="utf-8")
    assert bench.sha256_file(artifact) == hashlib.sha256(b"offline-model").hexdigest()
    assert "trace_identity_sha256" in bench.TRACE_COLUMNS
    assert "record_sha256" in bench.TRACE_COLUMNS
    assert "frame_index" in bench.TRACE_COLUMNS


def test_precommit_checksum_change_aborts(tmp_path: Path) -> None:
    (tmp_path / "dataset_manifest.json").write_text("{}\n")
    (tmp_path / "feature_contract.json").write_text("{}\n")
    (tmp_path / "fold_assignments.csv").write_text("logical_session_id,distance_bin,fold,role_in_fold\n")
    plan = {
        "benchmark_id": bench.BENCHMARK_ID, "seed": 52, "created_before_fit": True,
        "source_contracts": {
            "dataset_manifest_sha256": bench.sha256_file(tmp_path / "dataset_manifest.json"),
            "feature_contract_sha256": bench.sha256_file(tmp_path / "feature_contract.json"),
        },
        "cross_validation": {"fold_assignments_sha256": bench.sha256_file(tmp_path / "fold_assignments.csv")},
    }
    (tmp_path / "benchmark_plan.json").write_text(json.dumps(plan))
    (tmp_path / "dataset_manifest.json").write_text('{"changed":true}\n')
    with pytest.raises(ValueError, match="dataset_manifest_changed_after_precommit"):
        bench._validate_precommit(tmp_path)


def test_group_balanced_training_weights() -> None:
    rows = synthetic_rows()[:6]
    # First three synthetic groups each have two frames and therefore equal totals.
    weights = bench.group_weights(rows)
    totals: dict[str, float] = {}
    for row, weight in zip(rows, weights, strict=True):
        group = str(row["logical_session_id"])
        totals[group] = totals.get(group, 0.0) + float(weight)
    assert len(set(round(value, 12) for value in totals.values())) == 1


def test_offline_scope_and_residual_default_remain_off() -> None:
    source = Path(bench.__file__).read_text(encoding="utf-8")
    assert "metric_target_fusion" not in source
    assert "mavlink" not in source.lower()
    assert os.environ.get("SWARM_RANGE_RESIDUAL_MODE", "off").lower() == "off"
    assert bench._feature_contract()["temporal_features"]["status"].startswith("NOT_USED")


def test_completed_benchmark_artifacts_are_traceable_and_checksums_match() -> None:
    root = Path(__file__).resolve().parent / "artifacts/core_range_3_12m/xgboost_benchmark"
    manifest = json.loads((root / "benchmark_manifest.json").read_text())
    assert manifest["counts"] == {"groups": 27, "frames": 832, "folds": 3, "bins": 9}
    assert manifest["scope_guards"]["runtime_modified"] is False
    assert manifest["scope_guards"]["controller_effect"] is False
    assert manifest["scope_guards"]["residual_runtime_default"] == "off"
    for relative, checksum in manifest["artifacts"].items():
        assert bench.sha256_file(root / relative) == checksum
    model_paths = sorted((root / "models").glob("*.json"))
    assert [path.name for path in model_paths] == [
        "direct_fold_0.json", "direct_fold_1.json", "direct_fold_2.json",
        "residual_fold_0.json", "residual_fold_1.json", "residual_fold_2.json",
    ]
    with (root / "prediction_rows.csv").open() as stream:
        prediction_rows = list(__import__("csv").DictReader(stream))
    assert len(prediction_rows) == 832
    assert len({row["trace_identity_sha256"] for row in prediction_rows}) == 832
    fold_map = {}
    with (root / "fold_assignments.csv").open() as stream:
        for row in __import__("csv").DictReader(stream):
            fold_map[row["logical_session_id"]] = int(row["fold"])
    for path in (root / "preprocessing").glob("*.json"):
        fold = int(path.stem.rsplit("_", 1)[1])
        preprocessor = json.loads(path.read_text())
        assert preprocessor["fit_partition"] == "train_only"
        assert len(preprocessor["train_groups"]) == 18
        assert all(fold_map[group] != fold for group in preprocessor["train_groups"])
