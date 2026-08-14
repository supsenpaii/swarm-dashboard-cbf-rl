import csv
import hashlib
import json
from pathlib import Path

from range_residual_correction import RangeResidualFeatures, feature_schema
from range_v2_dataset_audit import run_audit


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _features() -> dict[str, float]:
    return RangeResidualFeatures(
        m52_anchor_median_m=4.0,
        m52_anchor_quality=0.8,
        midas_target_inverse_depth_median=100.0,
        midas_target_inverse_depth_spread=2.0,
        bbox_width_px=40.0,
        bbox_height_px=30.0,
        bbox_area_fraction=0.01,
        previous_physics_distance_m=7.5,
        delta_time_s=0.1,
        bbox_center_x_fraction=0.5,
        bbox_center_y_fraction=0.5,
        image_ray_x=0.0,
        image_ray_y=0.0,
        target_bearing_down=0.0,
        camera_optical_axis_down=0.2,
        calibration_scale=0.001,
        calibration_offset=0.1,
        calibration_residual_m_inv=0.01,
        calibration_condition_number_log10=3.0,
        calibration_inlier_fraction=0.8,
        anchor_spatial_coverage_fraction=0.6,
        target_anchor_extrapolation_iqr=0.2,
        ray_range_relative_std=0.1,
    ).as_dict()


def _make_dataset(
    workspace: Path,
    name: str,
    *,
    run_id: str,
    session_id: int,
    distance_m: float = 8.0,
    frame_count: int = 3,
) -> Path:
    root = workspace / "artifacts" / name
    root.mkdir(parents=True)
    manifest = {
        "created_utc": "2026-08-04T00:00:00+00:00",
        "dataset_schema_version": "m52_midas_range_residual_dataset_v2",
        "feature_schema": feature_schema(),
        "grouping": {
            "run_id": run_id,
            "group_key": "run_id/session_id",
            "group_ids_are_features": False,
        },
        "label": {
            "name": "range_residual_m",
            "definition": "ground_truth_distance_m - physics_distance_m",
            "ground_truth_source": "gazebo_camera_to_target_center",
            "ground_truth_target_id": "UAV-02",
            "distance_semantics": "camera_center_to_target_center_slant_range",
            "quality_metadata_required": True,
        },
        "samples_file": "samples.jsonl",
    }
    (root / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    records = []
    for index in range(frame_count):
        physics = 7.0
        records.append(
            {
                "dataset_schema_version": "m52_midas_range_residual_dataset_v2",
                "run_id": run_id,
                "target_id": "UAV-02",
                "group_id": f"UAV-01.{session_id}",
                "session_id": session_id,
                "frame_index": index,
                "measurement_timestamp_s": 100.0 + index,
                "source_sim_timestamp_s": 200.0 + index,
                "features": _features(),
                "physics_distance_m": physics,
                "ground_truth_distance_m": distance_m,
                "ground_truth_valid": True,
                "ground_truth_reason": "ok",
                "ground_truth_source": "gazebo_camera_to_target_center",
                "ground_truth_uncertainty_m": 0.0,
                "ground_truth_time_offset_ms": 0.0,
                "ground_truth_quality": "simulation_exact",
                "ground_truth_lever_arm_corrected": True,
                "range_residual_m": distance_m - physics,
                "correction_mode": "disabled",
                "candidate_distance_m": None,
            }
        )
    (root / "samples.jsonl").write_text(
        "".join(
            json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n"
            for record in records
        ),
        encoding="utf-8",
    )
    return root


def _run(workspace: Path, roots: list[Path]):
    docs = workspace / "docs"
    docs.mkdir()
    spec = docs / "RANGE_V2_FROZEN_SPEC.md"
    spec.write_text("frozen test spec\n", encoding="utf-8")
    return run_audit(
        workspace=workspace,
        dataset_roots=roots,
        output_dir=workspace / "artifacts" / "range_v2",
        frozen_spec_path=spec,
    )


def test_audit_is_read_only_and_every_row_is_traceable(tmp_path):
    source = _make_dataset(
        tmp_path,
        "range_v2_test_a",
        run_id="test_a",
        session_id=2,
    )
    before = {
        path.name: _sha256(path)
        for path in (source / "manifest.json", source / "samples.jsonl")
    }
    result = _run(tmp_path, [source])
    after = {
        path.name: _sha256(path)
        for path in (source / "manifest.json", source / "samples.jsonl")
    }
    assert before == after
    assert result["source_originals_read_only_verified"] is True

    output = tmp_path / "artifacts" / "range_v2"
    rows = [
        json.loads(line)
        for line in (output / "derived_labels.jsonl").read_text().splitlines()
    ]
    assert len(rows) == 3
    assert all(row["source_manifest_sha256"] == before["manifest.json"] for row in rows)
    assert all(row["source_samples_sha256"] == before["samples.jsonl"] for row in rows)
    assert [row["source_line_number"] for row in rows] == [1, 2, 3]
    assert all(len(row["source_record_sha256"]) == 64 for row in rows)


def test_legacy_metadata_stays_unknown_instead_of_being_inferred(tmp_path):
    source = _make_dataset(
        tmp_path,
        "range_v2_test_b",
        run_id="test_b",
        session_id=3,
    )
    _run(tmp_path, [source])
    row = json.loads(
        (tmp_path / "artifacts" / "range_v2" / "derived_labels.jsonl")
        .read_text()
        .splitlines()[0]
    )
    assert row["validity_gt"] == "unknown"
    assert row["direction_gt"] == "unknown"
    assert row["location_id"] == "unknown"
    assert row["background_regime"] == "unknown"
    assert row["pitch_regime"] == "unknown"
    assert row["calibration_epoch"] == "unknown"
    assert row["track_epoch"] == "unknown"


def test_audit_reports_atomic_group_collision_without_creating_split(tmp_path):
    first = _make_dataset(
        tmp_path,
        "range_v2_test_c1",
        run_id="shared_run",
        session_id=2,
    )
    second = _make_dataset(
        tmp_path,
        "range_v2_test_c2",
        run_id="shared_run",
        session_id=2,
    )
    _run(tmp_path, [first, second])
    leakage = json.loads(
        (tmp_path / "artifacts" / "range_v2" / "leakage_audit.json").read_text()
    )
    assert leakage["status"] == "FAIL"
    assert leakage["duplicate_identity_count"] == 3
    assert leakage["r2_partition_leakage_status"] == "not_applicable_no_r2_split_created"


def test_audit_artifact_records_no_training_or_runtime_actions(tmp_path):
    source = _make_dataset(
        tmp_path,
        "range_v2_test_d",
        run_id="test_d",
        session_id=2,
    )
    _run(tmp_path, [source])
    manifest = json.loads(
        (tmp_path / "artifacts" / "range_v2" / "r2_audit_manifest.json").read_text()
    )
    assert manifest["scope_guards"] == {
        "calibrator_fitted": False,
        "final_holdout_opened": False,
        "model_trained": False,
        "runtime_modified": False,
        "scaler_fitted": False,
        "shadow_run": False,
        "simulation_run": False,
        "threshold_fitted": False,
    }


def test_coverage_csv_keeps_fields_that_only_later_rows_use(tmp_path):
    source = _make_dataset(
        tmp_path,
        "range_v2_test_e",
        run_id="test_e",
        session_id=2,
    )
    _run(tmp_path, [source])
    with (
        tmp_path / "artifacts" / "range_v2" / "coverage_status.csv"
    ).open(encoding="utf-8", newline="") as stream:
        coverage = list(csv.DictReader(stream))
    severe_ood = next(
        row
        for row in coverage
        if row["scope"] == "invalid_regime"
        and row["name"] == "severe_ood_geometry"
    )
    assert "policy_flag_frame_count" in severe_ood
    assert "policy_flag_group_count" in severe_ood
