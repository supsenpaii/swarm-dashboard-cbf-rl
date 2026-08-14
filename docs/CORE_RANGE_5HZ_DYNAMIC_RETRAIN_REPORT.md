# CORE_RANGE 5 Hz dynamic recollection and retrain

Date: 2026-08-05  
Conclusion: `FIVE_HZ_DYNAMIC_CORPUS_INCOMPLETE`

## Preflight

Preflight passed before collection:

- `metric_target_fusion.py` production default and the dataset fallback in
  `main.py` were both 5.0 Hz.
- The pose-history optimization was active. The tracking loop performs an
  ordered bracket search and deep-copies only one or two selected snapshots;
  the 600-entry history is not copied.
- No stale PX4, Gazebo, ROS2, backend, capture, profiler or training process
  existed.
- `/` had 19 GB free and `/mnt/px4ssd` had 13 GB free; mounts were writable.
- Focused GT/pose/checksum/96-anchor/ROI/fusion tests: 56 passed.
- `./run_all.sh --check`: PASS.

The static corpus was independently reverified as 27 groups / 832 frames.

## Precommitted collection

`collection_plan.json` was written before any capture. It specifies the new
dataset role `core_range_dynamic_5hz_post_contention_fix`, three approaching,
three receding and two stop-and-hold groups, independent Gazebo `set_pose`
target motion, and the unchanged 5.0 Hz scheduler configuration.

The previously proven recovery policy was precommitted only for
`cdr_recede_left_yaw_5hz`:
`SWARM_TARGET_ROI_RECOVERY_POLICY=central_quantile_region`. All other groups
used the default foreground-half+MAD ROI statistic. No ROI or bbox parameter
was changed after inspecting a capture, GT, or model prediction.

## Collection result

Two independent attempts were made for every planned group. All 16 failed
attempts were moved unchanged into `quarantine/`; none is accepted or eligible
for training.

| Runtime metric across attempts | Minimum | Median | Maximum | Gate |
|---|---:|---:|---:|---:|
| raw rows | 44 | 129.5 | 142 | >=40 |
| tracking median FPS | 15.59 | 16.14 | 17.65 | >=20 |
| configured scheduler rate | 5.0 Hz | 5.0 Hz | 5.0 Hz | 5.0 Hz |
| successful submit cadence | 4.02 Hz | 4.33 Hz | 4.40 Hz | latest-value worker |
| capture→consume median | 70.9 ms | 82.4 ms | 98.1 ms | <=200 ms |
| capture→consume P95 | 144.2 ms | 172.5 ms | 215.1 ms | <=300 ms |
| MiDaS worker P95 | 51.0 ms | 89.4 ms | 122.5 ms | <=100 ms |

All 16 attempts passed capture median/P95 and no-growing-backlog gates. Ten of
16 passed MiDaS P95; six exceeded 100 ms. Every attempt failed the mandatory
tracking gate. Live API observation during collection agreed with the sidecar
frame-index evidence: tracking was approximately 16.2 FPS and camera source
approximately 16.7 FPS, while controller/main-loop work remained short. Thus
these captures cannot be accepted even though the corrected depth path stayed
far below the old 1.2-second contention latency.

The left-yaw recovery did produce >=40 valid raw ranges in both attempts, so
the historical ROI inverse-depth rejection was successfully avoided. Those
attempts still failed tracking, and one also failed worker P95.

## Integrity and data gate

Across the 1,888 quarantined raw rows:

- malformed records: 0;
- duplicate traces: 0;
- checksum failures: 0;
- timestamp ordering failures: 0;
- anchor-count failures: 0; every raw row logged exactly 96 anchors;
- nonfinite raw or GT ranges: 0.

This integrity does not make quarantine eligible for training. Accepted data
gate: approaching 0/3, receding 0/3, stop-and-hold 0/2. Required: 3/3, 3/3,
2/2. The corpus is therefore incomplete.

## Training and evaluation

Training was not started. The verified static corpus was not combined with
quarantine, the historical 2 Hz corpus was not used, no fold preprocessing was
fit, and no XGBoost model was created. `feature_contract.json` records the
precommitted PHYSICAL_ONLY and causal PHYSICAL_TEMPORAL contracts, while fold,
accuracy, temporal, bbox-stress, prediction and comparison CSV files contain
headers only to explicitly record that their phase was not reached.

No model was frozen and there was no shadow, active runtime integration,
Follow Target, OFFBOARD, arm/takeoff or hardware action.

## Artifacts

Evidence is under `artifacts/core_range_3_12m/dynamic_5hz_retrain/`:

- collection and quarantine: `collection_plan.json`,
  `accepted_sessions.json`, `quarantined_sessions.json`;
- raw gates: `integrity_report.json`, `runtime_metrics.csv`,
  `latency_metrics.csv`, `tracking_fps.csv`;
- skipped-training contracts: `frozen_dataset_manifest.json`,
  `feature_contract.json`, metric CSVs and `models/`;
- final state: `retrain_manifest.json`, `retrain_report.md`.

