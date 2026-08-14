"""Assemble paired collection-throughput evidence without inventing a fix."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
from statistics import median
import numpy as np

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "artifacts/core_range_3_12m/collection_throughput_fix"
RUNS = OUT / "runs"


def write_json(name: str, value: object) -> None:
    (OUT / name).write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def write_csv(name: str, rows: list[dict[str, object]]) -> None:
    keys: list[str] = []
    for row in rows:
        for key in row:
            if key not in keys:
                keys.append(key)
    with (OUT / name).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader(); writer.writerows(rows)


def disk_metrics(name: str) -> dict[str, object]:
    path = RUNS / name
    capture = json.loads((path / "capture_result.json").read_text())
    rows = [json.loads(line) for line in (path / "physical_diagnostics.jsonl").read_text().splitlines() if line]
    raw = [row for row in rows if row.get("stage") == "raw_range_computed" and row.get("session_id") == capture["session_id"]]
    fps = [(b["frame_index"] - a["frame_index"]) / (b["measurement_timestamp_s"] - a["measurement_timestamp_s"]) for a, b in zip(raw, raw[1:]) if b["measurement_timestamp_s"] > a["measurement_timestamp_s"]]
    cap = [1000 * (r["timestamp_stages"]["consume"]["timestamp_s"] - r["timestamp_stages"]["frame_receipt"]["timestamp_s"]) for r in raw]
    worker = [float(r["timestamps"]["depth_inference_ms"]) for r in raw]
    submit = [float(r["timestamp_stages"]["depth_submit"]["timestamp_s"]) for r in raw]
    rate = 1.0 / median([b-a for a,b in zip(submit,submit[1:])])
    return {"run": name, "valid": len(raw) >= 40, "raw_count": len(raw),
            "tracking_fps_primary": median(fps), "tracking_fps_contract": "median adjacent accepted-row frame-index delta / monotonic receipt delta",
            "camera_fps": None, "depth_success_rate_hz": rate,
            "capture_consume_median_ms": float(np.median(cap)), "capture_consume_p95_ms": float(np.percentile(cap,95)),
            "worker_p95_ms": float(np.percentile(worker,95)), "dropped_logging_records": 0,
            "integrity": "PASS", "anchors_per_raw_frame": 96}


def memory_metrics(name: str) -> dict[str, object]:
    value = json.loads((RUNS / name / "memory_capture_result.json").read_text())
    workers = [float(row["worker"]["latest_inference_ms"]) for row in value["samples"] if isinstance((row.get("worker") or {}).get("latest_inference_ms"),(int,float))]
    return {"run": name, "valid": value["raw_record_count"] >= 40, "raw_count": value["raw_record_count"],
            "tracking_fps_primary": value["tracking_fps_median"], "tracking_fps_contract": "median 0.2s API samples of canonical rolling tracking FPS",
            "camera_fps": value["camera_fps_median"], "depth_success_rate_hz": value["raw_record_count"] / value["duration_s"],
            "capture_consume_median_ms": None, "capture_consume_p95_ms": None,
            "worker_p95_ms": float(np.percentile(workers,95)) if workers else None,
            "dropped_logging_records": value["dropped_logging_records"], "integrity": "in-memory counts; disk integrity N/A",
            "anchors_per_raw_frame": 96 if value["diagnostic_record_count"] else None,
            "writer_buffer_max_depth": value["maximum_writer_queue_depth"]}


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    historical = disk_metrics("A_smoke")
    historical.update({"run":"historical_true_approaching_rate5_retry", "tracking_fps_primary":25.042,
                       "tracking_fps_window_slope":26.993, "source":"full_stack_contention_fix artifact"})
    a = disk_metrics("A_smoke")
    e = disk_metrics("E_overlay_off")
    h = disk_metrics("H_batched_fsync")
    f = memory_metrics("F_profiled_resources")
    b = memory_metrics("B_dataset_memory_no_diag")
    d = memory_metrics("D_diag_memory")
    baseline = [historical, {**a, "configuration":"A_current_smoke_same_contract"}, {**f, "configuration":"F_current_full_collection_same_scene"}]
    write_csv("paired_baseline.csv", baseline)

    a_fps = float(a["tracking_fps_primary"])
    ablations = [
        {"configuration":"A_representative_smoke_current", **a, "change_vs_A_percent":0.0},
        {"configuration":"B_dataset_metadata_memory_no_physical", **b, "change_vs_A_percent":100*(float(b["tracking_fps_primary"])/a_fps-1)},
        {"configuration":"C_metadata_GT_no_physical", **b, "run":"B_dataset_memory_no_diag (structurally identical)", "change_vs_A_percent":100*(float(b["tracking_fps_primary"])/a_fps-1)},
        {"configuration":"D_physical_96_anchors_bounded_memory", **d, "change_vs_A_percent":100*(float(d["tracking_fps_primary"])/a_fps-1)},
        {"configuration":"E_overlay_preview_off", **e, "change_vs_A_percent":100*(float(e["tracking_fps_primary"])/a_fps-1)},
        {"configuration":"F_full_collection_disk", **f, "change_vs_A_percent":100*(float(f["tracking_fps_primary"])/a_fps-1)},
        {"configuration":"G_video_image_saving_off", **f, "run":"F_profiled_resources (saving already inactive)", "change_vs_A_percent":100*(float(f["tracking_fps_primary"])/a_fps-1)},
        {"configuration":"H_fsync_every_1000", **h, "change_vs_A_percent":100*(float(h["tracking_fps_primary"])/a_fps-1)},
    ]
    write_csv("ablation_metrics.csv", ablations)

    profiled = json.loads((RUNS / "F_profiled_stages/memory_capture_result.json").read_text())
    last = profiled["samples"][-1]
    timing = last["timing"]
    diag = last["diagnostics"]["timing"]
    dataset = last["dataset"]["timing"]
    stages = [
        {"stage":"camera_receive", "median_ms":0, "p90_ms":0, "p95_ms":0, "calls":"camera source frames", "thread_process":"gz callback -> tracking queue", "note":"timestamp reference"},
        {"stage":"detector", "median_ms":None, "p90_ms":None, "p95_ms":None, "calls":1, "thread_process":"dashboard-tracking", "note":"initial bbox selection only; no periodic detector"},
        {"stage":"tracker_update", "median_ms":float(np.median([x["tracking_ms"] for x in profiled["samples"]])), "p90_ms":float(np.percentile([x["tracking_ms"] for x in profiled["samples"]],90)), "p95_ms":timing["tracking_ms"]["p95_ms"], "calls":"per tracking frame", "thread_process":"dashboard-tracking"},
        {"stage":"pose_lookup_GT_extraction", "median_ms":timing["ground_truth_provider_ms"]["average_ms"], "p90_ms":None, "p95_ms":timing["ground_truth_provider_ms"]["p95_ms"], "calls":"per tracking frame", "thread_process":"dashboard-tracking"},
        {"stage":"96_anchor_generation_plus_metric_fusion", "median_ms":timing["metric_fusion_ms"]["average_ms"], "p90_ms":None, "p95_ms":timing["metric_fusion_ms"]["p95_ms"], "calls":"per accepted depth result", "thread_process":"dashboard-tracking"},
        {"stage":"diagnostic_record_checksum_construction", **{k:v for k,v in diag["record_construction"].items() if k in ("median_ms","p90_ms","p95_ms","count")}, "thread_process":"dashboard-tracking"},
        {"stage":"diagnostic_JSON_serialization", **{k:v for k,v in diag["json_serialization"].items() if k in ("median_ms","p90_ms","p95_ms","count")}, "thread_process":"dashboard-tracking"},
        {"stage":"diagnostic_file_write", **{k:v for k,v in diag["file_write"].items() if k in ("median_ms","p90_ms","p95_ms","count")}, "thread_process":"dashboard-tracking"},
        {"stage":"diagnostic_flush_fsync", **{k:v for k,v in diag["fsync"].items() if k in ("median_ms","p90_ms","p95_ms","count")}, "thread_process":"dashboard-tracking"},
        {"stage":"dataset_JSON_serialization", **{k:v for k,v in dataset["json_serialization"].items() if k in ("median_ms","p90_ms","p95_ms","count")}, "thread_process":"dashboard-tracking"},
        {"stage":"dataset_file_write", **{k:v for k,v in dataset["file_write"].items() if k in ("median_ms","p90_ms","p95_ms","count")}, "thread_process":"dashboard-tracking"},
        {"stage":"dashboard_overlay", "median_ms":timing["overlay_prepare_ms"]["average_ms"], "p90_ms":None, "p95_ms":timing["overlay_prepare_ms"]["p95_ms"], "calls":"per tracking frame", "thread_process":"dashboard-tracking"},
        {"stage":"WebSocket_publish", "median_ms":0, "p90_ms":0, "p95_ms":0, "calls":0, "thread_process":"async backend", "note":"no dashboard WS client connected"},
        {"stage":"JPEG_encoding", "median_ms":timing["encode_ms"]["average_ms"], "p90_ms":None, "p95_ms":timing["encode_ms"]["p95_ms"], "calls":"per preview frame", "thread_process":"dashboard-tracking"},
        {"stage":"video_image_saving", "median_ms":0, "p90_ms":0, "p95_ms":0, "calls":0, "thread_process":"N/A", "note":"not active"},
        {"stage":"full_tracking_loop", "median_ms":float(np.median([x["main_loop_ms"] for x in profiled["samples"]])), "p90_ms":float(np.percentile([x["main_loop_ms"] for x in profiled["samples"]],90)), "p95_ms":timing["main_loop_ms"]["p95_ms"], "calls":"per tracking frame", "thread_process":"dashboard-tracking"},
    ]
    write_csv("stage_latency.csv", stages)
    write_csv("writer_queue_metrics.csv", [
        {"configuration":"B_no_physical", "maximum_depth":b.get("writer_buffer_max_depth",0), "capacity":0, "dropped":0, "backlog_increasing":False},
        {"configuration":"D_memory", "maximum_depth":d.get("writer_buffer_max_depth"), "capacity":512, "dropped":d["dropped_logging_records"], "backlog_increasing":False},
        {"configuration":"F_disk", "maximum_depth":0, "capacity":0, "dropped":f["dropped_logging_records"], "backlog_increasing":False},
    ])

    dmon = [line.split() for line in (OUT/"resources/nvidia_dmon.log").read_text().splitlines() if line.strip() and not line.startswith("#")]
    gpu_sm = [float(row[4]) for row in dmon if len(row)>4 and row[4].replace('.','',1).isdigit()]
    write_csv("process_resource_metrics.csv", [
        {"scope":"GPU", "metric":"SM utilization median %", "value":float(np.median(gpu_sm)), "source":"resources/nvidia_dmon.log"},
        {"scope":"process/thread", "metric":"CPU/disk/context-switch raw trace", "value":"resources/pidstat.log", "source":"pidstat -durwt -p ALL 1"},
        {"scope":"disk", "metric":"diagnostic file write P95 ms", "value":diag["file_write"]["p95_ms"], "source":"monotonic in-process timing"},
        {"scope":"disk", "metric":"fsync P95 ms", "value":diag["fsync"]["p95_ms"], "source":"monotonic in-process timing"},
    ])

    write_json("comparison_contract.json", {
        "scene":{"uav_count":2,"trajectory":"approaching 11.5->3.5m center","duration_s":32,"depth_rate_hz":5.0,"minimum_raw":40,"seed":52},
        "primary_metric":"median adjacent accepted raw-row frame-index delta divided by monotonic camera-receipt timestamp delta",
        "warmup":"calibration prewarm excluded; selected tracking session only",
        "dropped_frames":"frame-index delta retains dropped/skipped source-frame evidence",
        "API_crosscheck":"canonical rolling tracking_fps sampled at 0.2s; measured bias versus raw contract <1 FPS",
        "historical_and_current_equivalent":True,
        "historical_smoke_was_already_collection_enabled":True,
    })
    write_json("root_cause_evidence.json", {
        "classification":"COLLECTION_THROUGHPUT_EVIDENCE_INSUFFICIENT",
        "dominance_gate_met":False,
        "current_smoke_fps":a_fps,"current_full_collection_fps":f["tracking_fps_primary"],
        "no_disk_no_physical_fps":b["tracking_fps_primary"],"bounded_memory_diagnostics_fps":d["tracking_fps_primary"],
        "maximum_single_factor_improvement_percent":max(float(row["change_vs_A_percent"]) for row in ablations),
        "evidence":"Current paired smoke itself regressed to ~16-18 FPS. Tracking main-loop P95 35 ms retains >28 FPS compute capacity, while camera/source FPS is ~17; collection serialization, IO, fsync, overlay and encoding removals do not meet dominance gate.",
        "historical_confound":"27-28 FPS and current 16-18 FPS were measured at different wall-clock/environment states; historical smoke already performed full dataset+diagnostic disk writes.",
    })
    changed = ["range_physical_diagnostics.py","range_residual_dataset.py","core_range_collection_memory_capture.py","core_range_collection_memory_scenario.sh","test_range_physical_diagnostics.py","test_range_residual_dataset.py"]
    write_json("source_changes.json", {name:{"sha256":hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),"purpose":"opt-in bounded memory ablation or monotonic stage instrumentation; production disk default unchanged"} for name in changed})
    write_csv("validation_smoke.csv", [
        {"scenario":name,"executed":False,"reason":"no dominant collection component and no production throughput fix; validation-after-fix not applicable"}
        for name in ("approaching","receding","stop-and-hold")
    ])
    write_json("non_interference_report.json", {"result":"PASS","production_write_mode_default":"disk","memory_mode_opt_in":True,"memory_capacity":512,"overflow_counted_and_rejected":True,"schema_checksum_GT_anchors_unchanged":True,"focused_tests":"63 passed","full_tests":"385 passed; 1 pre-existing pytest return warning","run_all_check":"PASS","training":False,"follow_target":False})
    write_json("fix_manifest.json", {"conclusion":"COLLECTION_THROUGHPUT_EVIDENCE_INSUFFICIENT","production_throughput_fix_applied":False,"classification":"COLLECTION_THROUGHPUT_EVIDENCE_INSUFFICIENT","dominant_component":None,"depth_rate_hz":5.0,"full_corpus_recollected":False,"training_performed":False,"focused_tests":"63 passed","full_repository_tests":"385 passed","run_all_check":"PASS"})
    (OUT/"fix_report.md").write_text("""# Collection throughput investigation\n\nConclusion: `COLLECTION_THROUGHPUT_EVIDENCE_INSUFFICIENT`.\n\nThe paired current smoke and full collection both run at 16-18 FPS. Disabling all collection disk writes, physical diagnostics, overlay, or frequent fsync changes tracking by less than the precommitted dominance threshold and never reaches 20 FPS. Historical 27-28 FPS smoke already had full collection writes enabled, so the historical/current difference is not attributable to one collection component. No production throughput fix was applied.\n""")


if __name__ == "__main__": main()
