"""CORE_RANGE_CONTROL_READY_OBSERVATION_PIPELINE, Phase 2 (replay driver).

Replays the frozen 8/8 dynamic corpus (3 approaching, 3 receding, 2
stop-and-hold) through:

  Candidate B raw OOF prediction
  -> ControlReadyRangeEstimator (causal alpha-beta + age compensation)
  -> RangeTrendEstimator (causal windowed trend, 0.6/0.8/1.0s, monitor-only)

No model retraining, no dataset modification, no frame deletion, no
alpha/beta re-tuning after seeing results (alpha=0.65/beta=0.03/threshold=2.0m
kept as the precommitted reference from the stable_relative_tracking task).

Measurement-age methodology (precommitted BEFORE running -- see
clock_domain_contract.json): the frozen corpus stores a single
measurement_timestamp_s per frame (sim time at capture) with no separate
capture/consume timestamp, so a live "age" is not present in the dataset.
Primary replay pass uses a constant, small, realistic per-frame processing
age (0.05s, representative of one inference cycle) for all Section 9.1/9.3/
9.4/10 gate metrics. A secondary, deterministic 5-bucket age-sweep
(0.05/0.15/0.25/0.4/0.6s midpoints of the Section 9.2 buckets, cycled by
row index within each group) is applied post-hoc to the SAME filtered
state (the alpha-beta recursion itself does not depend on the assigned
query age -- only the final age-compensation step does) to produce
age_compensation_metrics.csv.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import core_range_stable_relative_tracking as srt
from control_ready_range_estimator import ControlReadyRangeEstimator, RangeTrendEstimator

WORKSPACE = Path(__file__).resolve().parent
SRC = WORKSPACE / "artifacts/core_range_3_12m/stable_relative_tracking"
OUT = WORKSPACE / "artifacts/core_range_3_12m/control_ready_observation"
CANDIDATE = "CANDIDATE_B_GROUP_AND_DISTANCE_BALANCED"
CALIBRATION = "none"

PRIMARY_AGE_S = 0.05
AGE_BUCKETS = [
    ("0-0.1s", 0.05),
    ("0.1-0.2s", 0.15),
    ("0.2-0.3s", 0.25),
    ("0.3-0.5s", 0.4),
    (">0.5s", 0.6),
]
TREND_WINDOWS_S = (0.6, 0.8, 1.0)
MODEL_SIGNATURE = f"{CANDIDATE}/{CALIBRATION}"


def load_dynamic_rows() -> list[dict]:
    with (SRC / "prediction_rows.csv").open() as f:
        rows = [r for r in csv.DictReader(f) if r["candidate"] == CANDIDATE and r["domain"] == "dynamic"]
    for r in rows:
        r["measurement_timestamp_s"] = float(r["measurement_timestamp_s"])
        r["ground_truth_range_m"] = float(r["ground_truth_range_m"])
        r["raw_physical_range_m"] = float(r["raw_physical_range_m"])
        r["raw_model_range_m"] = float(r["pred_none_m"])
    return rows


def replay() -> tuple[list[dict], list[dict]]:
    rows = load_dynamic_rows()
    by_group: dict[str, list[dict]] = {}
    for r in rows:
        by_group.setdefault(r["group_id"], []).append(r)

    scenario_counts: dict[str, int] = {}
    prediction_rows: list[dict] = []
    age_rows: list[dict] = []

    for group_id, group_rows in sorted(by_group.items()):
        ordered = sorted(group_rows, key=lambda r: r["measurement_timestamp_s"])
        scenario_type = ordered[0]["scenario_type"]
        scenario_counts[scenario_type] = scenario_counts.get(scenario_type, 0) + 1

        estimator = ControlReadyRangeEstimator()
        trend = RangeTrendEstimator(windows_s=TREND_WINDOWS_S)

        for idx, row in enumerate(ordered):
            t = row["measurement_timestamp_s"]
            est_out = estimator.update(
                raw_candidate_range_m=row["raw_model_range_m"],
                measurement_timestamp_s=t,
                current_timestamp_s=t + PRIMARY_AGE_S,
                measurement_valid=True,
                measurement_age_s=PRIMARY_AGE_S,
                session_id=group_id,
                model_signature=MODEL_SIGNATURE,
            )
            trend_out = trend.update(t, est_out.predicted_current_range_m, valid=est_out.estimator_valid)

            record = {
                "group_id": group_id,
                "scenario_type": scenario_type,
                "row_index_in_group": idx,
                "measurement_timestamp_s": t,
                "ground_truth_range_m": row["ground_truth_range_m"],
                "raw_physical_range_m": row["raw_physical_range_m"],
                "raw_model_range_m": row["raw_model_range_m"],
                "filtered_range_m": est_out.filtered_range_m,
                "estimated_range_rate_mps": est_out.estimated_range_rate_mps,
                "predicted_current_range_m": est_out.predicted_current_range_m,
                "display_range_m": est_out.display_range_m,
                "measurement_age_s": est_out.measurement_age_s,
                "innovation_m": est_out.innovation_m,
                "estimator_valid": est_out.estimator_valid,
                "estimator_status": est_out.estimator_status,
                "reset_count": est_out.reset_count,
                "reset_reason": est_out.reset_reason,
            }
            for w in TREND_WINDOWS_S:
                to = trend_out[w]
                record[f"trend_state_{w}s"] = to.state
                record[f"trend_slope_{w}s"] = to.slope_mps
                record[f"trend_n_samples_{w}s"] = to.n_samples
                record[f"trend_confidence_{w}s"] = to.confidence
            prediction_rows.append(record)

            # secondary age-bucket sweep (post-hoc on already-computed filtered
            # state; does not affect or reuse the primary estimator's own
            # recursive state, so this pass cannot leak future information)
            bucket_label, bucket_age = AGE_BUCKETS[idx % len(AGE_BUCKETS)]
            if est_out.filtered_range_m is not None:
                predicted_bucket = est_out.filtered_range_m + est_out.estimated_range_rate_mps * bucket_age
                age_rows.append({
                    "group_id": group_id, "scenario_type": scenario_type,
                    "measurement_timestamp_s": t, "ground_truth_range_m": row["ground_truth_range_m"],
                    "age_bucket": bucket_label, "assigned_age_s": bucket_age,
                    "filtered_range_m": est_out.filtered_range_m,
                    "filtered_range_error_m": abs(est_out.filtered_range_m - row["ground_truth_range_m"]),
                    "predicted_current_range_bucket_m": predicted_bucket,
                    "predicted_current_range_bucket_error_m": abs(predicted_bucket - row["ground_truth_range_m"]),
                })

    assert scenario_counts == {"approaching": 3, "receding": 3, "stop_and_hold": 2}, scenario_counts
    return prediction_rows, age_rows


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    keys: list[str] = []
    for row in rows:
        for k in row:
            if k not in keys:
                keys.append(k)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    prediction_rows, age_rows = replay()
    write_csv(OUT / "offline_prediction_rows.csv", prediction_rows)
    write_csv(OUT / "age_compensation_metrics_raw.csv", age_rows)
    print(f"replayed {len(prediction_rows)} dynamic frames across 8 groups")
    print(f"age-bucket rows: {len(age_rows)}")


if __name__ == "__main__":
    main()
