# Core Range 3–12 m — Minimal Logging Implementation Report

## Outcome

```text
CORE_LOGGING_READY
```

The minimal logging gate is complete. This outcome authorizes planning and
collection of balanced 3–12 m development data. It does not authorize model
training, promotion, controller integration or PX4 Follow.

## Changes

- Added worker publish timestamp at the actual latest-result publication.
- Added consumer receive and consume timestamps at the consumer boundary.
- Added a monotonic eight-stage timestamp record through sidecar write.
- Added GT trace identity and independent record checksums.
- Added disk-space preflight, file rotation, transactional append rollback,
  periodic fsync and full-file integrity reconciliation.
- Added diagnostic-only target ROI quantiles/statistic and anchor target-bbox
  exclusion/contribution fields.
- Added an offline smoke evaluator and deterministic integrity tests.

No raw-range formula, anchor selection, calibration fit/filter/cache/reseed,
range filter, EKF, runtime range output, controller or PX4 source was changed.

## Verification

```text
Focused tests:   52 passed
Repository:      318 passed, 1 known PytestReturnNotNoneWarning
Runtime check:   PASS
```

The system-Python command requires the repository virtualenv site-packages on
`PYTHONPATH`; without it, collection stops at missing `fastapi`. This was an
environment invocation issue, not a code-test failure.

## Smoke capture

One static center-view smoke was collected around 6 m:

- 20 raw diagnostic frames;
- 20/20 diagnostic-complete;
- 20/20 timestamp-order pass;
- 20/20 GT-trace pass;
- 96 anchors on every raw frame;
- zero malformed JSON lines;
- zero record or trace checksum mismatch;
- both UAVs observed disarmed before and after;
- no Follow or motion/vehicle-control endpoint called;
- residual correction off.

Instrumentation non-interference is covered by the enabled/disabled pipeline
state regression test and remains PASS.

The 5.977107 m GT scene produced raw range from 3.777216 m to 3.876559 m. This
is expected evidence of the known raw accuracy problem; the smoke validates
logging, not estimator accuracy.

## Disk handling

The short stack run produced about 1.4 GB of repetitive PX4 console logs. All
eight raw log files were checksummed, archived to `runtime_logs.tar.zst`, and
the archive passed both zstd integrity test and tar listing before the raw log
files were removed. The logs remain recoverable from the verified archive.
Measurement sidecars and manifests were not deleted or overwritten.

## Artifacts

- `artifacts/core_range_3_12m/logging_smoke/runtime/`
- `artifacts/core_range_3_12m/logging_smoke/smoke_summary.json`
- `artifacts/core_range_3_12m/logging_smoke/smoke_manifest.json`
- `artifacts/core_range_3_12m/logging_smoke/smoke_report.md`
- `artifacts/core_range_3_12m/logging_smoke/runtime_logs.tar.zst`

## Next decision

Prepare a small collection manifest for 3–12 m. Prioritize 10–12 m and 6–7 m.
Do not start XGBoost training until independent-session coverage is adequate.

## 2026-08-04 static collection use

The repaired logging contract was exercised on 18 accepted static balance
groups (558 raw frames). All frames passed timestamp ordering, GT trace and
record checksums, and exact 96-anchor completeness. A deterministic
pipeline/config fingerprint was added to the instrumentation sidecar; this did
not alter the estimator or runtime output.

All 18 runtime-log directories were archived, read-tested and checksummed
before their raw copies were removed. Three failed attempts remain preserved in
quarantine. Repository verification is now 334 passed with the one known
warning; runtime check remains PASS. See
`docs/CORE_RANGE_STATIC_BALANCE_COLLECTION_REPORT.md`.
