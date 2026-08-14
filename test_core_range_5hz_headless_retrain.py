from __future__ import annotations

import json
from pathlib import Path

import pytest

import core_range_5hz_headless_retrain as retrain


def test_feature_variants_scope_matches_task_spec():
    assert set(retrain.FEATURE_VARIANTS) == {"PHYSICAL_ONLY", "PHYSICAL_TEMPORAL"}
    assert retrain.VARIANT_PRIORITY == ("PHYSICAL_TEMPORAL", "PHYSICAL_ONLY")
    bbox_features = {
        "bbox_center_x_fraction", "bbox_center_y_fraction",
        "bbox_width_fraction", "bbox_height_fraction",
        "bbox_area_fraction", "bbox_aspect_ratio", "bbox_tracking_score",
    }
    assert not (set(retrain.FEATURE_VARIANTS["PHYSICAL_ONLY"]) & bbox_features)
    assert not (set(retrain.FEATURE_VARIANTS["PHYSICAL_TEMPORAL"]) & bbox_features)
    assert set(retrain.FEATURE_VARIANTS["PHYSICAL_TEMPORAL"]) - set(retrain.FEATURE_VARIANTS["PHYSICAL_ONLY"]) == set(retrain.TEMPORAL_EXTRA_FEATURES)


def test_conclusion_strings_match_task_gate_outcomes():
    assert retrain.CONCLUSION_BY_VARIANT["PHYSICAL_TEMPORAL"] == "PHYSICAL_TEMPORAL_5HZ_HEADLESS_BEST_CANDIDATE"
    assert retrain.CONCLUSION_BY_VARIANT["PHYSICAL_ONLY"] == "PHYSICAL_ONLY_5HZ_HEADLESS_BEST_CANDIDATE"
    assert retrain.CONCLUSION_BY_VARIANT[None] == "NO_5HZ_HEADLESS_DYNAMIC_ROBUST_MODEL_MEETS_GATE"


def test_load_dynamic_rows_rejects_incomplete_collection(tmp_path):
    dynamic_output = tmp_path / "dynamic"
    dynamic_output.mkdir()
    (dynamic_output / "dynamic_session_manifest.json").write_text(json.dumps({
        "collection_complete": False, "accepted_sessions": [],
    }))
    with pytest.raises(ValueError, match="dynamic_collection_incomplete"):
        retrain.load_dynamic_rows(tmp_path, dynamic_output)


def test_load_dynamic_rows_rejects_sidecar_checksum_mismatch(tmp_path):
    workspace = tmp_path
    session_root = workspace / "session_a"
    session_root.mkdir()
    (session_root / "physical_diagnostics.jsonl").write_text('{"stage": "raw_range_computed"}\n')

    dynamic_output = workspace / "dynamic"
    dynamic_output.mkdir()
    (dynamic_output / "dynamic_session_manifest.json").write_text(json.dumps({
        "collection_complete": True,
        "accepted_sessions": [{
            "session_id": "session_a", "source_root": "session_a",
            "source_sidecar_sha256": "0" * 64, "runtime_session_id": 1,
            "scenario_type": "approaching", "context": "center",
        }],
    }))
    with pytest.raises(ValueError, match="dynamic_sidecar_checksum_mismatch"):
        retrain.load_dynamic_rows(workspace, dynamic_output)


def test_write_csv_handles_empty_rows(tmp_path):
    path = tmp_path / "empty.csv"
    retrain._write_csv(path, [])
    assert path.read_text() == ""


def test_write_csv_unions_keys_across_rows(tmp_path):
    path = tmp_path / "rows.csv"
    retrain._write_csv(path, [{"a": 1}, {"a": 2, "b": 3}])
    lines = path.read_text().splitlines()
    assert lines[0] == "a,b"
