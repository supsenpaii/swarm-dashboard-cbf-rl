from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from range_residual_correction import (
    FEATURE_NAMES,
    RangeResidualCorrector,
    RangeResidualFeatures,
)
from range_residual_dataset import (
    DATASET_SCHEMA_VERSION,
    RangeResidualDatasetCollector,
    load_training_samples,
)
from range_residual_training import (
    _applicability_group_reports,
    _group_balanced_weights,
    applicability_stratified_grouped_split,
    deterministic_applicability_samples,
    grouped_split,
    regression_metrics,
    train_range_residual_bundle,
)


def feature(index: int) -> RangeResidualFeatures:
    return RangeResidualFeatures(
        m52_anchor_median_m=10.0 + 0.1 * index,
        m52_anchor_quality=0.75,
        midas_target_inverse_depth_median=0.4 + 0.001 * index,
        midas_target_inverse_depth_spread=0.02,
        bbox_width_px=80.0 + index,
        bbox_height_px=120.0 + index,
        bbox_area_fraction=0.05,
        previous_physics_distance_m=10.0 + 0.05 * index,
        delta_time_s=0.1,
        bbox_center_x_fraction=0.45 + 0.001 * index,
        bbox_center_y_fraction=0.40,
        image_ray_x=-0.10 + 0.002 * index,
        image_ray_y=-0.20,
        target_bearing_down=0.05,
        camera_optical_axis_down=0.17,
        calibration_scale=0.0002 + 1.0e-7 * index,
        calibration_offset=0.02,
        calibration_residual_m_inv=0.002,
        calibration_condition_number_log10=3.5,
        calibration_inlier_fraction=0.70,
        anchor_spatial_coverage_fraction=0.50,
        target_anchor_extrapolation_iqr=0.0,
        ray_range_relative_std=0.10,
    )


def build_dataset(root: Path) -> RangeResidualDatasetCollector:
    collector = RangeResidualDatasetCollector(
        root,
        run_id="sitl_run_01",
        target_id="UAV-02",
    )
    collector._prepare()
    corrector = RangeResidualCorrector(mode="off")
    frame = 0
    for session_id, group in enumerate(("UAV-01.1", "UAV-01.2", "UAV-01.3"), 1):
        for offset in range(4):
            physics = 8.0 + session_id + offset * 0.2
            residual = 0.5 if offset < 2 else 4.0
            features = feature(frame + 1)
            correction = corrector.correct(features, physics)
            assert collector.record(
                features=features,
                physics_distance_m=physics,
                correction=correction,
                group_id=group,
                session_id=session_id,
                frame_index=frame,
                measurement_timestamp_s=frame * 0.1,
                source_sim_timestamp_s=100.0 + frame * 0.1,
                ground_truth_distance_m=physics + residual,
                ground_truth_valid=True,
                ground_truth_reason="ok",
            )
            frame += 1
    return collector


def test_collector_is_disabled_without_path_and_fails_closed() -> None:
    collector = RangeResidualDatasetCollector()
    features = feature(1)
    correction = RangeResidualCorrector(mode="off").correct(features, 10.0)

    assert not collector.enabled
    assert not collector.record(
        features=features,
        physics_distance_m=10.0,
        correction=correction,
        group_id="UAV-01.1",
        session_id=1,
        frame_index=1,
        measurement_timestamp_s=1.0,
        source_sim_timestamp_s=1.0,
        ground_truth_distance_m=10.5,
        ground_truth_valid=True,
        ground_truth_reason="ok",
    )
    assert collector.status()["last_reason"] == "disabled"


def test_memory_write_mode_counts_without_creating_dataset_files(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("SWARM_RANGE_DATASET_WRITE_MODE", "memory")
    collector = RangeResidualDatasetCollector(
        tmp_path, run_id="sitl_memory", target_id="UAV-02"
    )
    collector._prepare()
    correction = RangeResidualCorrector(mode="off").correct(feature(1), 10.0)
    assert collector.record(
        features=feature(1), physics_distance_m=10.0,
        correction=correction, group_id="UAV-01.1", session_id=1,
        frame_index=1, measurement_timestamp_s=1.0,
        source_sim_timestamp_s=1.0, ground_truth_distance_m=10.5,
        ground_truth_valid=True, ground_truth_reason="ok",
    )
    assert collector.status()["record_count"] == 1
    assert collector.status()["write_mode"] == "memory"
    assert not list(tmp_path.iterdir())


def test_environment_collector_requires_explicit_target_identity(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("SWARM_RANGE_DATASET_DIR", str(tmp_path / "dataset"))
    monkeypatch.setenv("SWARM_RANGE_DATASET_RUN_ID", "sitl_run_01")
    monkeypatch.delenv("SWARM_RANGE_DATASET_TARGET_ID", raising=False)

    collector = RangeResidualDatasetCollector.from_environment()

    assert not collector.enabled
    assert collector.status()["load_error"] == "target_id_invalid"
    assert not (tmp_path / "dataset").exists()


def test_environment_collector_locks_explicit_rtk_source(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    root = tmp_path / "rtk_dataset"
    monkeypatch.setenv("SWARM_RANGE_DATASET_DIR", str(root))
    monkeypatch.setenv("SWARM_RANGE_DATASET_RUN_ID", "hardware_run_01")
    monkeypatch.setenv("SWARM_RANGE_DATASET_TARGET_ID", "UAV-02")
    monkeypatch.setenv(
        "SWARM_RANGE_DATASET_GROUND_TRUTH_SOURCE",
        "rtk_fixed_camera_to_target_center",
    )

    collector = RangeResidualDatasetCollector.from_environment()

    assert collector.enabled
    assert collector.ground_truth_source == (
        "rtk_fixed_camera_to_target_center"
    )
    manifest = json.loads(
        (root / "manifest.json").read_text(encoding="utf-8")
    )
    assert manifest["label"]["ground_truth_source"] == (
        "rtk_fixed_camera_to_target_center"
    )


def test_environment_collector_rejects_unknown_ground_truth_source(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    root = tmp_path / "invalid_dataset"
    monkeypatch.setenv("SWARM_RANGE_DATASET_DIR", str(root))
    monkeypatch.setenv("SWARM_RANGE_DATASET_RUN_ID", "hardware_run_01")
    monkeypatch.setenv("SWARM_RANGE_DATASET_TARGET_ID", "UAV-02")
    monkeypatch.setenv(
        "SWARM_RANGE_DATASET_GROUND_TRUTH_SOURCE",
        "gps_single",
    )

    collector = RangeResidualDatasetCollector.from_environment()

    assert not collector.enabled
    assert collector.load_error == "range_dataset_ground_truth_source_invalid"
    assert not root.exists()


def test_rtk_collector_requires_matching_quality_metadata(
    tmp_path: Path,
) -> None:
    collector = RangeResidualDatasetCollector(
        tmp_path / "rtk_dataset",
        run_id="hardware_run_01",
        target_id="UAV-02",
        ground_truth_source="rtk_fixed_camera_to_target_center",
    )
    collector._prepare()
    features = feature(1)
    correction = RangeResidualCorrector(mode="off").correct(features, 10.0)

    assert not collector.record(
        features=features,
        physics_distance_m=10.0,
        correction=correction,
        group_id="UAV-01.1",
        session_id=1,
        frame_index=1,
        measurement_timestamp_s=1.0,
        source_sim_timestamp_s=None,
        ground_truth_distance_m=10.2,
        ground_truth_valid=True,
        ground_truth_reason="ok",
        ground_truth_uncertainty_m=0.05,
        ground_truth_time_offset_ms=20.0,
        ground_truth_quality="simulation_exact",
        ground_truth_lever_arm_corrected=True,
    )
    assert collector.last_reason == "ground_truth_quality_source_mismatch"

    assert collector.record(
        features=features,
        physics_distance_m=10.0,
        correction=correction,
        group_id="UAV-01.1",
        session_id=1,
        frame_index=1,
        measurement_timestamp_s=1.0,
        source_sim_timestamp_s=None,
        ground_truth_distance_m=10.2,
        ground_truth_valid=True,
        ground_truth_reason="ok",
        ground_truth_uncertainty_m=0.05,
        ground_truth_time_offset_ms=20.0,
        ground_truth_quality="rtk_fixed",
        ground_truth_lever_arm_corrected=True,
    )
    samples, _ = load_training_samples([tmp_path / "rtk_dataset"])
    assert len(samples) == 1
    assert samples[0].ground_truth_quality == "rtk_fixed"


def test_collector_writes_versioned_grouped_labeled_samples(
    tmp_path: Path,
) -> None:
    collector = build_dataset(tmp_path / "dataset")
    samples, provenance = load_training_samples([tmp_path / "dataset"])

    assert collector.status()["record_count"] == 12
    assert collector.status()["labeled_count"] == 12
    assert provenance["dataset_schema_version"] == DATASET_SCHEMA_VERSION
    assert provenance["labeled_sample_count"] == 12
    assert len(samples) == 12
    assert samples[0].residual_target_m == 0.5
    assert samples[0].ground_truth_quality == "simulation_exact"
    assert samples[0].ground_truth_uncertainty_m == 0.0
    assert samples[0].ground_truth_lever_arm_corrected
    assert list(samples[0].features.as_dict()) == list(FEATURE_NAMES)


def test_training_loader_rejects_low_quality_ground_truth(
    tmp_path: Path,
) -> None:
    dataset = tmp_path / "dataset"
    build_dataset(dataset)
    samples_path = dataset / "samples.jsonl"
    records = [
        json.loads(line)
        for line in samples_path.read_text(encoding="utf-8").splitlines()
    ]
    records[0]["ground_truth_quality"] = "gps_single"
    samples_path.write_text(
        "\n".join(
            json.dumps(record, sort_keys=True)
            for record in records
        )
        + "\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="ground_truth_quality_invalid"):
        load_training_samples([dataset])


def test_grouped_split_never_leaks_session_between_partitions(
    tmp_path: Path,
) -> None:
    build_dataset(tmp_path / "dataset")
    samples, _ = load_training_samples([tmp_path / "dataset"])

    split = grouped_split(samples, seed=52)

    groups = [
        set(split.train_groups),
        set(split.validation_groups),
        set(split.test_groups),
    ]
    assert all(groups)
    assert not groups[0] & groups[1]
    assert not groups[0] & groups[2]
    assert not groups[1] & groups[2]
    assert len(split.train) + len(split.validation) + len(split.test) == 12


def test_applicability_stratified_split_has_both_classes_per_partition(
    tmp_path: Path,
) -> None:
    build_dataset(tmp_path / "dataset")
    samples, _ = load_training_samples([tmp_path / "dataset"])

    split = applicability_stratified_grouped_split(
        samples,
        maximum_absolute_residual_m=3.0,
        seed=52,
    )

    for partition in (split.train, split.validation, split.test):
        labels = [
            abs(sample.residual_target_m or 0.0) <= 3.0
            for sample in partition
        ]
        assert any(labels)
        assert not all(labels)


def test_training_deterministic_filter_matches_runtime_envelope(
    tmp_path: Path,
) -> None:
    build_dataset(tmp_path / "dataset")
    samples, _ = load_training_samples([tmp_path / "dataset"])

    accepted, report = deterministic_applicability_samples(samples)

    assert len(accepted) == len(samples)
    assert report["accepted_samples"] == len(samples)
    assert report["rejected_samples"] == 0


def test_regression_metrics_compare_physical_distances() -> None:
    metrics = regression_metrics([10.0, 12.0], [9.0, 14.0])

    assert metrics["count"] == 2
    assert metrics["mae_m"] == 1.5
    assert np.isclose(metrics["rmse_m"], np.sqrt(2.5))


class FakeScaler:
    last_instance = None

    def __init__(self) -> None:
        FakeScaler.last_instance = self
        self.n_features_in_ = len(FEATURE_NAMES)
        self.fit_rows = 0
        self.mean_: np.ndarray | None = None
        self.scale_: np.ndarray | None = None

    def fit_transform(self, values: np.ndarray) -> np.ndarray:
        self.fit_rows = len(values)
        self.mean_ = np.mean(values, axis=0)
        self.scale_ = np.std(values, axis=0)
        self.scale_[self.scale_ == 0.0] = 1.0
        return (values - self.mean_) / self.scale_

    def transform(self, values: np.ndarray) -> np.ndarray:
        assert self.mean_ is not None and self.scale_ is not None
        return (values - self.mean_) / self.scale_


class FakeDMatrix:
    def __init__(
        self,
        data: np.ndarray,
        label: np.ndarray | None = None,
        weight: np.ndarray | None = None,
    ) -> None:
        self.data = np.asarray(data)
        self.label = None if label is None else np.asarray(label)
        self.weight = None if weight is None else np.asarray(weight)


class FakeBooster:
    def __init__(self, residual: float) -> None:
        self.residual = residual

    def predict(self, matrix: FakeDMatrix) -> np.ndarray:
        return np.full(len(matrix.data), self.residual, dtype=np.float64)

    def save_model(self, path: str) -> None:
        Path(path).write_text("fake-xgboost-model", encoding="utf-8")


class FakeXGBoost:
    __version__ = "test"
    DMatrix = FakeDMatrix

    @staticmethod
    def train(_parameters, train_matrix, **_kwargs):
        if _parameters.get("objective") == "binary:logistic":
            return FakeBooster(1.0)
        return FakeBooster(float(np.mean(train_matrix.label)))


class FakeJoblib:
    __version__ = "test"

    @staticmethod
    def dump(_object, path: Path) -> None:
        Path(path).write_text("fake-standard-scaler", encoding="utf-8")


def fake_dependencies():
    return (
        FakeJoblib,
        SimpleNamespace(__version__="test"),
        FakeXGBoost,
        FakeScaler,
    )


def test_group_balanced_weights_equalize_independent_group_totals(
    tmp_path: Path,
) -> None:
    dataset = tmp_path / "dataset"
    build_dataset(dataset)
    samples, _ = load_training_samples([dataset])
    selected = tuple(samples[:4] + samples[4:6] + samples[8:9])

    weights = _group_balanced_weights(selected)
    totals: dict[str, float] = {}
    for sample, weight in zip(selected, weights, strict=True):
        totals[sample.group_id] = totals.get(sample.group_id, 0.0) + weight

    assert np.isclose(np.mean(weights), 1.0)
    assert len(totals) == 3
    assert np.allclose(list(totals.values()), [7.0 / 3.0] * 3)


def test_per_group_promotion_is_class_aware_and_fail_closed(
    tmp_path: Path,
) -> None:
    dataset = tmp_path / "dataset"
    build_dataset(dataset)
    samples, _ = load_training_samples([dataset])
    accepted = np.asarray(
        [True, False, False, False] * 3,
        dtype=bool,
    )
    correctable = np.asarray(
        [True, True, False, False] * 3,
        dtype=bool,
    )

    reports = _applicability_group_reports(
        samples,
        accepted,
        correctable,
        minimum_recall=0.5,
        maximum_false_accept_rate=0.05,
    )

    assert set(reports) == {
        "sitl_run_01/UAV-01.1",
        "sitl_run_01/UAV-01.2",
        "sitl_run_01/UAV-01.3",
    }
    assert all(report["passed"] for report in reports.values())
    assert all(report["applicability_recall"] == 0.5 for report in reports.values())
    assert all(
        report["uncorrectable_false_accept_rate"] == 0.0
        for report in reports.values()
    )

    failed = _applicability_group_reports(
        samples[:4],
        np.asarray([False, False, True, False]),
        correctable[:4],
        minimum_recall=0.5,
        maximum_false_accept_rate=0.05,
    )
    report = failed["sitl_run_01/UAV-01.1"]
    assert report["applicability_recall"] == 0.0
    assert report["uncorrectable_false_accept_rate"] == 0.5
    assert not report["passed"]


def test_trainer_fits_scaler_on_training_groups_only_and_exports_bundle(
    tmp_path: Path,
) -> None:
    dataset = tmp_path / "dataset"
    output = tmp_path / "bundle"
    build_dataset(dataset)

    manifest = train_range_residual_bundle(
        [dataset],
        output,
        dependency_loader=fake_dependencies,
    )

    counts = manifest["training_manifest"]["sample_counts"]
    assert FakeScaler.last_instance is not None
    assert FakeScaler.last_instance.fit_rows == counts["train"]
    assert counts == {"train": 4, "validation": 4, "test": 4}
    assert manifest["model_input"] == "scaled"
    assert manifest["minimum_applicability_probability"] == 0.8
    assert manifest["ood"]["method"] == (
        "zscore_and_shrunk_mahalanobis"
    )
    assert manifest["training_manifest"]["evaluation"]["test"][
        "mae_improvement_m"
    ] > 0.0
    weight_policy = manifest["training_manifest"]["sample_weight_policy"]
    assert weight_policy["method"] == (
        "equal_total_weight_per_run_session_group"
    )
    assert not manifest["training_manifest"]["promotion_gate"][
        "validation_groups_passed"
    ]
    test_evaluation = manifest["training_manifest"]["evaluation"]["test"]
    assert test_evaluation["applicability_precision"] == 0.5
    assert test_evaluation["uncorrectable_false_accept_rate"] == 1.0
    assert not manifest["candidate_promotion_gate_passed"]
    assert (output / "manifest.json").is_file()
    assert (output / "scaler.joblib").is_file()
    assert (output / "model.json").is_file()
    assert (output / "applicability_model.json").is_file()
