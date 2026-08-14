from collections import Counter
import json
import math
import os
from pathlib import Path
import time

import numpy as np
import pytest

from depth_model_adapter import CallableDepthAdapter, DepthMap
from depth_worker import DepthJob, DepthResult, LatestDepthWorker
from m52_adapter import M52GroundAnchorAdapter
from metric_target_fusion import FusionFrameContext, MetricTargetFusion
from range_physical_diagnostics import (
    DIAGNOSTICS_FILENAME,
    DIAGNOSTICS_MANIFEST_FILENAME,
    DIAGNOSTICS_SCHEMA_VERSION,
    RangePhysicalDiagnosticsCollector,
    TIMESTAMP_STAGE_ORDER,
    canonical_sha256,
    timestamp_stage,
    validate_timestamp_stages,
)
from visual_follow_target import CameraRayProjector


def collector(root: Path) -> RangePhysicalDiagnosticsCollector:
    subject = RangePhysicalDiagnosticsCollector(
        root,
        run_id="run-a",
        target_id="UAV-02",
    )
    subject._prepare()
    return subject


def minimal_payload() -> dict:
    return {
        "diagnostics_schema_version": DIAGNOSTICS_SCHEMA_VERSION,
        "instrumentation_only": True,
        "run_id": "run-a",
        "target_id": "UAV-02",
        "group_id": "UAV-01.2",
        "session_id": 2,
        "frame_index": 10,
        "measurement_timestamp_s": 12.5,
        "stage": "test",
    }


def complete_payload(start: float = 10.0) -> dict:
    payload = minimal_payload()
    payload["stage"] = "raw_range_computed"
    payload["source_sim_timestamp_s"] = 20.0
    payload["ground_truth"] = {
        "valid": True,
        "distance_m": 6.0,
        "timestamp_s": 20.0,
        "timestamp_clock_id": "gazebo_sim_time",
    }
    payload["timestamp_stages"] = {
        name: timestamp_stage(
            start + index * 0.01,
            component="test",
            execution_context="test-thread",
            semantic=f"test_{name}",
        )
        for index, name in enumerate(TIMESTAMP_STAGE_ORDER[:-1])
    }
    return payload


def test_sidecar_manifest_and_trace_record_are_append_only(tmp_path: Path) -> None:
    subject = collector(tmp_path)
    payload = minimal_payload()

    assert subject.record(payload)
    assert subject.record({**payload, "frame_index": 11})

    manifest = json.loads(
        (tmp_path / DIAGNOSTICS_MANIFEST_FILENAME).read_text()
    )
    assert manifest["runtime_semantics"] == {
        "changes_raw_range": False,
        "changes_calibration": False,
        "changes_runtime_output": False,
        "changes_controller": False,
    }
    rows = [
        json.loads(line)
        for line in (tmp_path / DIAGNOSTICS_FILENAME).read_text().splitlines()
    ]
    assert [row["frame_index"] for row in rows] == [10, 11]
    for row in rows:
        digest = row.pop("record_sha256")
        assert digest == canonical_sha256(row)
    assert subject.status()["record_count"] == 2


def test_sidecar_nonfinite_or_identity_error_is_fail_soft(tmp_path: Path) -> None:
    subject = collector(tmp_path)
    assert not subject.record({**minimal_payload(), "measurement_timestamp_s": math.nan})
    assert not subject.record({**minimal_payload(), "session_id": 0})
    assert subject.status()["rejected_count"] == 2
    assert not (tmp_path / DIAGNOSTICS_FILENAME).exists()


def test_manifest_mismatch_does_not_overwrite_existing_sidecar(tmp_path: Path) -> None:
    collector(tmp_path)
    before = (tmp_path / DIAGNOSTICS_MANIFEST_FILENAME).read_bytes()
    conflicting = RangePhysicalDiagnosticsCollector(
        tmp_path,
        run_id="run-b",
        target_id="UAV-02",
    )
    with pytest.raises(ValueError, match="manifest_mismatch"):
        conflicting._prepare()
    assert (tmp_path / DIAGNOSTICS_MANIFEST_FILENAME).read_bytes() == before


def test_environment_fallback_creates_only_sidecar_files(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.delenv("SWARM_RANGE_PHYSICAL_DIAGNOSTICS_DIR", raising=False)
    monkeypatch.setenv("SWARM_RANGE_DATASET_DIR", str(tmp_path))
    monkeypatch.setenv("SWARM_RANGE_DATASET_RUN_ID", "run-a")
    monkeypatch.setenv("SWARM_RANGE_DATASET_TARGET_ID", "UAV-02")

    subject = RangePhysicalDiagnosticsCollector.from_environment()

    assert subject.enabled
    assert (tmp_path / DIAGNOSTICS_MANIFEST_FILENAME).is_file()
    assert not (tmp_path / "manifest.json").exists()
    assert not (tmp_path / "samples.jsonl").exists()


def test_environment_flag_disables_sidecar_without_disabling_dataset(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("SWARM_RANGE_PHYSICAL_DIAGNOSTICS_ENABLED", "false")
    monkeypatch.setenv("SWARM_RANGE_DATASET_DIR", str(tmp_path))

    subject = RangePhysicalDiagnosticsCollector.from_environment()

    assert not subject.enabled
    assert not list(tmp_path.iterdir())


def test_memory_write_mode_is_bounded_and_never_touches_disk(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("SWARM_RANGE_DIAGNOSTICS_WRITE_MODE", "memory")
    monkeypatch.setenv("SWARM_RANGE_DIAGNOSTICS_MEMORY_MAX_RECORDS", "1")
    subject = collector(tmp_path)

    assert subject.record(complete_payload())
    assert not subject.record({**complete_payload(), "frame_index": 11})
    status = subject.status()
    assert status["write_mode"] == "memory"
    assert status["memory_buffer_depth"] == 1
    assert status["dropped_record_count"] == 1
    assert not list(tmp_path.iterdir())


def test_configuration_fingerprint_is_deterministic() -> None:
    first = canonical_sha256({"fx": 200.0, "resolution": [160, 120]})
    second = canonical_sha256({"resolution": [160, 120], "fx": 200.0})
    assert first == second
    assert first != canonical_sha256({"fx": 201.0, "resolution": [160, 120]})


def test_depth_worker_preserves_generation_and_queue_timestamps() -> None:
    worker = LatestDepthWorker(
        CallableDepthAdapter(
            lambda frame: np.ones(frame.shape[:2], dtype=np.float32),
            source="timestamp_test",
        )
    )
    worker.submit(
        DepthJob(
            frame_bgr=np.zeros((8, 10, 3), dtype=np.uint8),
            measurement_timestamp_s=time.monotonic(),
            frame_index=7,
        )
    )
    deadline = time.monotonic() + 1.0
    result = None
    while time.monotonic() < deadline:
        _, result = worker.latest()
        if result is not None:
            break
        time.sleep(0.005)
    worker.stop()

    assert result is not None and result.valid
    assert result.generation == 0
    assert result.submitted_timestamp_s is not None
    assert result.inference_started_timestamp_s is not None
    assert result.submitted_timestamp_s <= result.inference_started_timestamp_s
    assert result.inference_started_timestamp_s <= result.completed_timestamp_s
    assert result.result_publish_timestamp_s is not None
    assert result.completed_timestamp_s <= result.result_publish_timestamp_s
    assert result.worker_thread_name == "metric-depth-worker"
    assert result.publisher_thread_name == "metric-depth-worker"
    assert result.depth_map is not None
    assert result.depth_map.source == "timestamp_test"


def test_monotonic_stage_contract_and_sidecar_completion(tmp_path: Path) -> None:
    subject = collector(tmp_path)
    assert subject.record(complete_payload())
    subject.finalize_integrity()
    row = json.loads((tmp_path / DIAGNOSTICS_FILENAME).read_text().strip())
    valid, reason = validate_timestamp_stages(row["timestamp_stages"])
    assert valid and reason == "ok"
    assert row["timestamp_order_valid"] is True
    assert row["ground_truth_trace_valid"] is True
    assert row["diagnostic_complete"] is True
    assert row["reason_code"] == "ok"


def test_timestamp_order_is_not_repaired_or_sorted(tmp_path: Path) -> None:
    subject = collector(tmp_path)
    payload = complete_payload()
    payload["timestamp_stages"]["consume"]["timestamp_s"] = 1.0
    assert subject.record(payload)
    subject.finalize_integrity()
    row = json.loads((tmp_path / DIAGNOSTICS_FILENAME).read_text().strip())
    assert row["timestamp_order_valid"] is False
    assert row["diagnostic_complete"] is False
    assert row["reason_code"] == "timestamp_order_invalid"
    assert row["timestamp_stages"]["consume"]["timestamp_s"] == 1.0


def test_disk_preflight_fails_closed(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    subject = RangePhysicalDiagnosticsCollector(
        tmp_path, run_id="run-a", target_id="UAV-02"
    )
    subject.minimum_free_bytes = 1_000
    subject.projected_record_bytes = 1_000
    subject.projected_record_count = 1
    usage_type = type(__import__("shutil").disk_usage(tmp_path))
    monkeypatch.setattr(
        "range_physical_diagnostics.shutil.disk_usage",
        lambda _path: usage_type(2_000, 1_500, 500),
    )
    with pytest.raises(ValueError, match="disk_preflight_failed"):
        subject._prepare()


def test_simulated_enospc_rolls_back_partial_json_line(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    subject = collector(tmp_path)
    subject.minimum_free_bytes = 0

    def partial_then_fail(fd: int, encoded: bytes) -> None:
        os.write(fd, encoded[: max(1, len(encoded) // 3)])
        raise OSError("simulated_ENOSPC")

    monkeypatch.setattr(subject, "_write_bytes", partial_then_fail)
    assert not subject.record(complete_payload())
    path = tmp_path / DIAGNOSTICS_FILENAME
    assert path.exists()
    assert path.stat().st_size == 0
    assert subject.last_reason == "simulated_ENOSPC"


def test_rotation_and_manifest_reconciliation(tmp_path: Path) -> None:
    subject = collector(tmp_path)
    subject.maximum_file_bytes = 1
    subject.fsync_every_records = 1
    assert subject.record(complete_payload())
    assert subject.record({**complete_payload(11.0), "frame_index": 11})
    report = subject.finalize_integrity()
    assert (tmp_path / DIAGNOSTICS_FILENAME).is_file()
    assert (tmp_path / "physical_diagnostics.000001.jsonl").is_file()
    assert report["segment_count"] == 2
    assert report["parsed_record_count"] == 2
    assert report["checksum_mismatches"] == 0
    assert report["malformed_json_lines"] == 0
    assert report["counts_reconciled"] is True
    assert report["pass"] is True


def synthetic_scene(projector: CameraRayProjector):
    height, width = 120, 160
    camera = (0.0, 0.0, -10.0)
    pitch = np.deg2rad(45.0)
    quaternion = (0.0, np.sin(pitch / 2.0), 0.0, np.cos(pitch / 2.0))
    inverse = np.full((height, width), np.nan, dtype=np.float32)
    for v in range(height):
        for u in range(width):
            ray = projector.pixel_to_ned_ray(u, v, width, height, quaternion)
            if ray[2] <= 1e-4:
                continue
            slant = -camera[2] / ray[2]
            forward = 1.0 / np.sqrt(
                1.0
                + ((u - width / 2.0) / projector.fx) ** 2
                + ((v - height / 2.0) / projector.fy) ** 2
            )
            optical = slant * forward
            inverse[v, u] = (1.0 / optical - 0.03) / 0.2
    bbox = (62.0, 42.0, 36.0, 40.0)
    inverse[48:76, 68:92] = (1.0 / 12.0 - 0.03) / 0.2
    return inverse, camera, quaternion, bbox


def test_per_anchor_diagnostics_cover_every_grid_point_without_changing_arrays() -> None:
    projector = CameraRayProjector(200.0, 200.0)
    inverse, camera, quaternion, bbox = synthetic_scene(projector)
    adapter = M52GroundAnchorAdapter(projector)

    anchors = adapter.anchors(
        inverse,
        camera_position_ned_m=camera,
        camera_quaternion_xyzw=quaternion,
        ground_down_m=0.0,
        excluded_bbox_xywh=bbox,
    )

    assert len(anchors.diagnostics) == adapter.grid_columns * adapter.grid_rows
    accepted = [item for item in anchors.diagnostics if item.accepted]
    assert len(accepted) == anchors.accepted_ground_anchor_count
    assert np.array_equal(
        anchors.relative_inverse_depth,
        np.asarray([item.relative_inverse_depth for item in accepted]),
    )
    assert np.array_equal(
        anchors.metric_optical_depth_m,
        np.asarray([item.metric_optical_depth_m for item in accepted]),
    )
    rejected = Counter(
        item.reason for item in anchors.diagnostics if not item.accepted
    )
    assert rejected == Counter(dict(anchors.rejected_reason_counts))
    assert all(item.ray_ned_unit is not None for item in accepted)
    assert all(item.local_sample_count >= 4 for item in accepted)


def run_direct_fusion(
    diagnostics: RangePhysicalDiagnosticsCollector | None,
) -> tuple[MetricTargetFusion, list[dict]]:
    projector = CameraRayProjector(200.0, 200.0)
    inverse, camera, quaternion, bbox = synthetic_scene(projector)
    fusion = MetricTargetFusion(
        projector,
        enabled=False,
        physical_diagnostics_collector=diagnostics,
    )
    context = FusionFrameContext(
        bbox_xywh=bbox,
        bbox_center_px=(80.0, 62.0),
        camera_position_ned_m=camera,
        camera_quaternion_xyzw=quaternion,
        tracking_score=0.99,
        frame_width=160,
        frame_height=120,
        ground_down_m=0.0,
        filtered_bearing_ned_unit=projector.pixel_to_ned_ray(
            80.0, 62.0, 160, 120, quaternion
        ),
        dataset_group_id="UAV-01.2",
        dataset_session_id=2,
        source_sim_timestamp_s=5.0,
        instrumentation_context={
            "raw_geometry_center_source": "synthetic_test_camera_center",
            "camera_info": {
                "camera_info_source": "synthetic",
                "distortion_model": "none",
                "distortion_coefficients": [],
                "rectified": True,
                "unavailable_fields": [],
            },
            "extrinsics": {
                "vehicle_position_ned_m": [0.0, 0.0, -10.0],
                "camera_offset_body_frd_m": [0.0, 0.0, 0.0],
            },
            "timestamps": {"sensor_capture_timestamp_s": 5.0},
        },
    )
    for index in range(16):
        timestamp = 1.0 + index * 0.2
        result = DepthResult(
            valid=True,
            reason="ok",
            measurement_timestamp_s=timestamp,
            completed_timestamp_s=timestamp + 0.01,
            frame_index=index,
            inference_ms=10.0,
            depth_map=DepthMap(inverse.copy(), "synthetic_midas"),
            context=context,
            submitted_timestamp_s=timestamp + 0.002,
            inference_started_timestamp_s=timestamp + 0.004,
            result_publish_timestamp_s=timestamp + 0.012,
            worker_thread_name="synthetic-worker",
            publisher_thread_name="synthetic-worker",
        )
        fusion._consume_depth_result(
            result,
            timestamp + 0.02,
            consumer_receive_timestamp_s=timestamp + 0.014,
            consume_timestamp_s=timestamp + 0.016,
        )
    rows = []
    if diagnostics is not None:
        rows = [
            json.loads(line)
            for line in (
                diagnostics.root / DIAGNOSTICS_FILENAME
            ).read_text().splitlines()
        ]
    return fusion, rows


def test_instrumentation_enabled_and_disabled_have_identical_pipeline_state(
    tmp_path: Path,
) -> None:
    disabled, _ = run_direct_fusion(None)
    enabled_collector = collector(tmp_path)
    enabled, rows = run_direct_fusion(enabled_collector)

    assert enabled._last_physics_distance_m == pytest.approx(
        disabled._last_physics_distance_m, abs=0.0
    )
    assert enabled._last_target_range_m == pytest.approx(
        disabled._last_target_range_m, abs=0.0
    )
    assert enabled.calibrator.scale == pytest.approx(disabled.calibrator.scale, abs=0.0)
    assert enabled.calibrator.offset == pytest.approx(disabled.calibrator.offset, abs=0.0)
    assert enabled.bearing_range_filter.status() == disabled.bearing_range_filter.status()
    assert enabled.ekf.status() == disabled.ekf.status()
    assert rows
    raw_rows = [row for row in rows if row["stage"] == "raw_range_computed"]
    assert raw_rows
    last = raw_rows[-1]
    assert len(last["anchors"]["per_grid_point"]) == 96
    assert last["calibration"]["fit"]["raw_scale"] is not None
    assert last["calibration"]["fit"]["filtered_scale"] is not None
    assert last["calibration"]["applied"]["filtered_scale"] is not None
    assert last["calibration"]["source"] in {"live", "cached"}
    assert "cache_age_s" in last["calibration"]
    assert last["calibration"]["cache_ttl_s"] == pytest.approx(
        enabled.calibration_cache_ttl_s
    )
    assert last["calibration"]["recovery"]["state"] == "stable"
    assert "reseed_count" in last["calibration"]["recovery"]
    assert last["track_epoch"] >= 1
    assert last["calibration_epoch"] >= 1
    assert last["timestamps"]["depth_inference_ms"] == 10.0
    assert last["timestamp_stages"]["depth_complete"]["timestamp_s"] <= (
        last["timestamp_stages"]["result_publish"]["timestamp_s"]
    )
    assert last["timestamp_stages"]["result_publish"]["timestamp_s"] <= (
        last["timestamp_stages"]["consumer_receive"]["timestamp_s"]
    )
    assert last["timestamp_stages"]["consumer_receive"]["timestamp_s"] <= (
        last["timestamp_stages"]["consume"]["timestamp_s"]
    )
    assert last["camera_info"]["fingerprint_sha256"]
    assert last["reference_centers"]["raw_geometry_center_source"] == (
        "synthetic_test_camera_center"
    )
    assert last["correction_observation"]["mode"] == "off"
    assert not last["correction_observation"]["applied"]


def test_track_and_calibration_epochs_observe_reset_without_changing_calibration() -> None:
    fusion = MetricTargetFusion(CameraRayProjector(200.0, 200.0), enabled=False)
    relative = np.linspace(0.1, 1.2, 80)
    metric = 1.0 / (0.18 * relative + 0.035)
    for _ in range(fusion.calibrator.stable_samples_required):
        fusion.calibrator.fit(relative, metric)
    fusion._last_valid_calibration_timestamp_s = 10.0
    track_before = fusion._track_epoch
    calibration_before = fusion._calibration_epoch
    scale_before = fusion.calibrator.scale

    fusion.reset(preserve_calibration=True)

    assert fusion._track_epoch == track_before + 1
    assert fusion._calibration_epoch == calibration_before
    assert fusion.calibrator.scale == scale_before
    fusion.reset(preserve_calibration=False)
    assert fusion._track_epoch == track_before + 2
    assert fusion._calibration_epoch == calibration_before + 1
    assert fusion.calibrator.scale is None
