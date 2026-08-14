from __future__ import annotations

import json
import os

import camera_source_trace


def test_record_is_noop_when_env_unset(monkeypatch, tmp_path):
    monkeypatch.delenv("SWARM_CAMERA_TRACE_JSONL", raising=False)
    assert camera_source_trace.trace_enabled() is False
    camera_source_trace.record("camera_source_receipt", drone_id="UAV-01")
    assert list(tmp_path.iterdir()) == []


def test_record_writes_jsonl_row_when_env_set(monkeypatch, tmp_path):
    trace_path = tmp_path / "trace.jsonl"
    monkeypatch.setenv("SWARM_CAMERA_TRACE_JSONL", str(trace_path))
    assert camera_source_trace.trace_enabled() is True

    camera_source_trace.record(
        "camera_source_receipt",
        drone_id="UAV-01",
        frame_seq=1,
        sim_timestamp_s=12.5,
        monotonic_receipt_s=100.0,
    )
    camera_source_trace.record(
        "camera_mailbox_dequeue",
        drone_id="UAV-01",
        raw_frame_version=1,
    )

    lines = trace_path.read_text().strip().splitlines()
    assert len(lines) == 2

    row0 = json.loads(lines[0])
    assert row0["event"] == "camera_source_receipt"
    assert row0["drone_id"] == "UAV-01"
    assert row0["frame_seq"] == 1
    assert row0["sim_timestamp_s"] == 12.5
    assert "trace_monotonic_s" in row0

    row1 = json.loads(lines[1])
    assert row1["event"] == "camera_mailbox_dequeue"


def test_record_never_raises_on_unwritable_path(monkeypatch, tmp_path):
    blocking_file = tmp_path / "not_a_directory"
    blocking_file.write_text("x")
    bad_path = blocking_file / "nested" / "trace.jsonl"
    monkeypatch.setenv("SWARM_CAMERA_TRACE_JSONL", str(bad_path))
    camera_source_trace.record("camera_source_receipt", drone_id="UAV-01")


def test_record_disabled_after_env_cleared_mid_process(monkeypatch, tmp_path):
    trace_path = tmp_path / "trace.jsonl"
    monkeypatch.setenv("SWARM_CAMERA_TRACE_JSONL", str(trace_path))
    camera_source_trace.record("camera_source_receipt", drone_id="UAV-01")

    monkeypatch.delenv("SWARM_CAMERA_TRACE_JSONL", raising=False)
    camera_source_trace.record("camera_source_receipt", drone_id="UAV-01")

    lines = trace_path.read_text().strip().splitlines()
    assert len(lines) == 1
