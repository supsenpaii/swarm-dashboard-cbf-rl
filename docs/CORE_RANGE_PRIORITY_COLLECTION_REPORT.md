# Core Range 3–12 m — Priority Static Collection Report

## Outcome

```text
PRIORITY_STATIC_COVERAGE_COLLECTED
```

```text
Training gate: NO_GO_DYNAMIC_AND_FULL_BIN_BALANCE_MISSING
```

The priority static development collection is complete for the previously
missing 6–7 m, 10–11 m and 11–12 m bins. This is a data-coverage result, not a
model or physical-range PASS.

## Accepted data

| Bin | Independent groups | Frames | Equal-group raw bias | Equal-group raw MAE | Static status |
| --- | ---: | ---: | ---: | ---: | --- |
| 6–7 m | 3 | 92 | +0.268 m | 0.906 m | COVERED |
| 10–11 m | 3 | 91 | −5.377 m | 5.377 m | COVERED |
| 11–12 m | 3 | 91 | −8.772 m | 8.772 m | COVERED |

All nine accepted roots satisfy:

- exactly one raw run/session group;
- at least 30 finite raw/GT frames;
- GT lies in the precommitted 1 m bin;
- 100% record and trace checksum validity;
- 100% monotonic timestamp ordering and GT trace validity;
- exactly 96 anchor diagnostics on every accepted raw frame;
- both UAVs disarmed before and after capture;
- no Follow, OFFBOARD, arm, takeoff, mode, motion or controller call;
- residual correction off and no training/fit/tuning.

The large far-core bias is consistent across the new independent static groups
and must not be hidden by aggregate averaging. It is evidence that raw physical
range is not usable as-is near 10–12 m.

## Collection controls

Each scenario used an isolated output root and one independent session/group.
The simulator was observation-only. Zero gravity and direct static model poses
were applied only before each measurement window, after both UAVs were observed
disarmed. Tracking/follow/motion callbacks remained disabled.

Runtime logs were capped by the bounded writer to avoid the prior multi-GB log
failure. Accepted logs are still present because there is 19 GB free space;
they were not deleted or falsely reported as archived. Measurement sidecars,
manifests and aggregate checksums were verified before this report.

## Quarantine

Failed attempts were moved intact under
`artifacts/core_range_3_12m/priority_collection/quarantine/` and are excluded
from all coverage and future training. Reasons include early readiness failure,
wrong bbox, insufficient frames, multiple raw sessions, unstable disarmed
physics before the static setup, calibration rejection and tracker reacquire.

No quarantined row was merged into an accepted root.

## Tooling changes

- `bounded_log_writer.py`: bounded, rotated process logs.
- `run_all.sh`: bounded process logging and process-group cleanup.
- `range_v2_r3_6a_capture.py`: explicit disarmed readiness, strict minimum-row
  failure and rotated-sidecar loading.
- `core_range_collect_scenario.sh`: isolated observation-only static capture.
- `core_range_collect_priority_batch.py`: frozen-manifest sequential runner.
- `core_range_logging_eval.py`: strict one-session evaluation and truthful
  per-run manifest metadata.
- `core_range_priority_collection_audit.py`: aggregate source/checksum/bin/group
  audit.

These changes do not alter raw range, anchors, calibration, runtime range
output, controller or PX4.

## Verification

```text
Focused collection/audit tests: 10 passed
Repository suite:               326 passed, 1 known warning
./run_all.sh --check:           PASS
Old PX4/Gazebo/backend/train:   none
```

The workspace still is not a valid Git repository (`.git` lacks usable
repository metadata), so this gate is anchored by SHA-256 manifests rather
than a commit.

## Artifacts and checksums

```text
artifacts/core_range_3_12m/priority_collection/collection_manifest.json
  f78b9d333fd39d62a6ff18de8add74815264df99d742bc83a35f55e8ed4bad6b
artifacts/core_range_3_12m/priority_collection/collection_manifest_amendment_v005.json
  6fcd59e19937945c1815997604234354d37e1949d2965ac0f55bbdf32826957c
artifacts/core_range_3_12m/priority_collection/audit/priority_collection_manifest.json
  a8bad6d77c26a89e119597403d4ce3fce77e9c4bdcd7171d9384215ebe039c9c
artifacts/core_range_3_12m/priority_collection/audit/priority_collection_report.md
  2213398a6b7fa9e4aa21530c2cad841f8695f8f56d7da48ee68e7c85dbf9988a
artifacts/core_range_3_12m/priority_collection/audit/per_group_audit.csv
  942e96af4660dd1f107764ee9a90f7b6ec0f7c495624e98ca8567ff0f3298617
artifacts/core_range_3_12m/priority_collection/audit/per_bin_coverage.csv
  2be266a5212f35d8f40bf19f1a38abdbabd6374cf3e8faf2c564d34ab3e2e9ed
```

## Next decision

Do not train a promotion candidate from static-only data. The next bounded
step is a new precommitted observation-only collection for dynamic approaching
and receding sequences, followed by balanced independent-session coverage in
the remaining 3–10 m bins. After that audit passes, benchmark raw versus
XGBoost direct-range versus residual regression using group-disjoint splits
and seed 52.

