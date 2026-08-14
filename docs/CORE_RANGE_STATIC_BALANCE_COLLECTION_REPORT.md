# Core Range 3–12 m — Static Balance Collection Report

## Outcome

```text
STATIC_3_12M_COVERAGE_READY_FOR_MODEL_BENCHMARK
```

The six requested 1 m bins now each contain three independent, integrity-clean
static measurement groups. This closes the static session-balance gate only.
No XGBoost model, scaler, threshold, split, shadow run or runtime correction was
created or enabled.

## Frozen collection inputs

- Base precommit: `collection_plan.json`
  - SHA-256: `84dec822470ca2542732f23d6b724cb5678c156781650d46b5076cd1d9c2f285`
- Technical replacement amendment: `collection_plan_amendment_v001.json`
  - SHA-256: `650e30d1742728a3bd5a06b119ca9dbc23abe04610e2fc6921f6f00981cb8224`
- Dataset role: `core_range_static_development`
- Logging schema: `core_range_logging_3_12m_v001`
- Seed recorded in the plan: `52`

The amendment was frozen before replacement capture. It replaced only three
technical failures: one attempt with zero raw rows and two attempts that had 24
and 26 rows, below the frozen 30-frame minimum. It explicitly records
`raw_accuracy_metrics_used=false`. The three failed attempts remain unchanged
under `quarantine/` and are excluded.

## Coverage

| Bin | Independent groups | Frames | Equal-group bias | Equal-group MAE | Equal-group P90 | Equal-group P95 | Status |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 3–4 m | 3 | 93 | +4.662 m | 4.794 m | 5.220 m | 5.254 m | `STATIC_COVERED` |
| 4–5 m | 3 | 100 | -0.135 m | 0.610 m | 0.732 m | 0.742 m | `STATIC_COVERED` |
| 5–6 m | 3 | 93 | +3.767 m | 5.823 m | 6.610 m | 6.691 m | `STATIC_COVERED` |
| 7–8 m | 3 | 91 | -2.325 m | 2.325 m | 2.519 m | 2.539 m | `STATIC_COVERED` |
| 8–9 m | 3 | 90 | -0.312 m | 2.952 m | 3.320 m | 3.343 m | `STATIC_COVERED` |
| 9–10 m | 3 | 91 | -5.253 m | 5.253 m | 5.521 m | 5.537 m | `STATIC_COVERED` |

Together with the prior accepted 6–7 m, 10–11 m and 11–12 m priority batch,
every static 1 m bin from 3 m through 12 m has the requested minimum of three
independent sessions. The large context-dependent raw errors remain visible and
must not be hidden by a frame-weighted aggregate.

## Integrity

- Accepted groups: 18/18 effective planned groups.
- Accepted raw frames: 558; every group has at least 30.
- Exactly one raw measurement run/session/group per accepted dataset.
- Timestamp-order pass: 558/558.
- GT trace/checksum pass: 558/558.
- Record/trace checksum pass: 558/558.
- Anchor completeness: exactly 96 anchors on 558/558 frames.
- Malformed JSON lines: 0.
- Duplicate trace identities: 0.
- Runtime archives: 18/18 created, read-tested and checksummed before the raw
  `runtime_logs/` directories were removed.
- Quarantined attempts: 3, preserved and excluded.

The sidecar now also records a deterministic pipeline/config fingerprint built
from schema, CameraInfo, extrinsics, anchor-grid/range, cache TTL, raw formula
and residual-mode metadata. This is instrumentation only and does not change an
estimate or state transition.

## Safety and non-interference

- Both UAVs were verified disarmed before and after every accepted session.
- Static pose setup occurred before the measurement window.
- No Follow, OFFBOARD, arm, takeoff, flight-mode, motion or controller command
  was sent.
- Residual correction remained `off`.
- Raw formula, calibration, anchor selection, filter, EKF, runtime output,
  controller and PX4 were not changed.
- No dynamic collection, training, split, scaler/threshold fit, shadow or final
  holdout operation occurred.

## Verification

```text
Focused collection/instrumentation tests: 55 passed
Repository tests: 334 passed, 1 known PytestReturnNotNoneWarning
./run_all.sh --check: PASS
Final stale PX4/Gazebo/backend/train process check: PASS
```

The system `python3` test command needs the workspace virtualenv site-packages
on `PYTHONPATH`; running it without that path fails at missing `fastapi`. With
the repository dependency path, the complete suite passes as shown above.

## Artifacts

```text
artifacts/core_range_3_12m/static_balance_collection/
  collection_plan.json
  collection_plan_amendment_v001.json
  session_manifest.json
  accepted_sessions.json
  quarantined_sessions.json
  per_group_audit.csv
  per_bin_coverage.csv
  integrity_report.json
  training_readiness_report.json
  static_balance_collection_manifest.json
  static_balance_collection_report.md
  runtime_sessions/
  quarantine/
```

## Decision

The next permitted proposal is an offline benchmark of:

```text
raw physical baseline
vs XGBoost residual regression
vs XGBoost direct-range regression
```

It must use group-disjoint partitions and seed 52. This report does not start
that benchmark automatically and does not establish dynamic readiness or
controller readiness.
