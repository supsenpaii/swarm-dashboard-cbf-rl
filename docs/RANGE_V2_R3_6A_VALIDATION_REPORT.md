# Range V2 R3.6A-V Controlled Validation Report

Date: 2026-08-04  
Validation ID: `range_v2_r3_6a_validation_v001`  
Dataset role: `physical_diagnostic_development`  
Conclusion: **`DIAGNOSTIC_EVIDENCE_INSUFFICIENT`**

## Outcome

R3.6A-V collected 11 independent static observation-only measurement groups
at nominal 4, 6, 8, 10 and 7 m geometry variants. The selected groups contain
389 raw-range frames and 37,344 anchor records. Both UAVs were observed
disarmed before and after every accepted scenario. Follow, OFFBOARD, vehicle
motion, arm/takeoff/mode and controller endpoints were not called. Residual
correction remained off.

The capture provides useful controlled evidence, but it does not satisfy the
precommitted required-field contract: **0/389** raw-range frames are complete,
and one source sidecar has a malformed, unselected tail record caused by disk
exhaustion. Therefore this patch cannot select a physical fix. It does not
start R3.6B, R4 or R5.

## Scope and integrity

- R1 frozen spec, R2, R3, R3.5, R3.6A and all 64 locked source files were
  checksum-verified before capture. The five R3.6A production/instrumentation
  source files still match the R3.6A manifest exactly after capture.
- Diagnostics were opt-in and append-only under
  `artifacts/range_v2/r3_6a_validation/runtime_sessions/`; no original dataset,
  manifest, R1-R3.6A artifact or current Range V2 development corpus was
  overwritten.
- Each selected `run_id/session_id` is one independent group. Setup, prewarm,
  aborted and measurement sessions remain separately identified.
- To keep both disarmed models static above the ground-anchor plane, Gazebo
  gravity was set to zero and model poses were set directly outside measurement
  windows. This is recorded in `session_manifest.json`; it is not an estimator,
  calibration, PX4 or controller change.
- After capture, no PX4, Gazebo, backend, capture or training process remained.
  Sidecar diagnostics are again default-off because the opt-in environment
  variable is absent.

The first 10 m source ended with one truncated JSON line when runtime logs
filled the disk. The 36 selected `center_10m_repeat_a` records precede that
line and their per-record checksums pass. The source-root integrity result
nonetheless remains `FAIL`; the tail is not silently repaired. Runtime logs
were checksummed, archived and read-tested before the original bulky log
directories were removed. The two retained archives have SHA-256:

```text
r3_6a_v_runtime_logs_part1_20260804.tar.gz
  ecae669a4e41f82ad1dd628918d898eabefc073bd6478b2ed5a19b72101f54e1
r3_6a_v_runtime_logs_part2_20260804.tar.gz
  956eb55eef57644af71b4de051cc2f1515d9e3a75a45e233f274006082cdcc17
```

## Scenario coverage

| Scenario | Frames | GT range (m) | Live MAE (m) | Live std (m) | Freeze MAE (m) | Freeze std (m) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| center 4 m, prewarm | 34 | 3.980 | 2.715 | 0.151 | 2.538 | 0.001 |
| center 4 m, nominal cold start | 35 | 3.980 | 3.543 | 0.047 | 3.610 | 0.002 |
| center 6 m, stable live | 40 | 5.977 | 3.717 | 0.754 | 3.372 | 0.519 |
| center 6 m, cache perturbation | 30 | 5.977 | 2.651 | 0.476 | 2.812 | 0.415 |
| center 8 m, reacquire role | 36 | 7.976 | 5.123 | 0.443 | 5.152 | 0.055 |
| center 8 m, reseed/recovery | 35 | 7.976 | 4.252 | 0.428 | 3.171 | 0.185 |
| center 10 m, repeat A | 36 | 9.975 | 0.944 | 1.015 | 0.826 | 0.804 |
| center 10 m, repeat B | 36 | 9.975 | 2.371 | 1.814 | 2.584 | 0.340 |
| 7 m lateral left | 36 | 7.136 | 9.754 | 1.098 | 9.137 | 1.103 |
| 7 m lateral right | 35 | 7.137 | 7.975 | 0.999 | 7.698 | 0.620 |
| 7 m oblique, pitch -12° | 36 | 7.045 | 10.888 | 2.960 | 14.197 | 1.139 |

“Cold start” is only the planned role: the setup session had 17 calibration
rows before bbox, so a truly clean process-level cold start was not captured.
The tracker-reacquire event was deliberately not forced because the helper did
not have a verified safe direct visibility-event path. These lifecycle cells
remain missing, not claimed as observed.

## Offline experiments

The four precommitted variants are fully reported per session in
`per_session_metrics.csv`:

| Variant | Evaluated frames | Median group MAE (m) | Median group std (m) | Causal |
| --- | ---: | ---: | ---: | --- |
| current live/applied | 389 | 3.717 | 0.754 | yes |
| stable-window freeze | 306 | 3.372 | 0.415 | yes |
| group median oracle | 389 | 3.765 | 0.584 | **no — `NON_CAUSAL_ORACLE`** |
| anchor refit replay | 389 | 3.717 | 0.754 | diagnostic replay |

Stable freeze reduced output standard deviation in 10/11 groups, but improved
MAE in only 6/11 and made the oblique MAE materially worse. This supports a
calibration-lifecycle contribution to jitter, but also shows that freezing
`a/b` is not a demonstrated bias or quantity-domain fix.

Full-run, same-config anchor replay reproduces every logged raw fit:

```text
maximum |offline raw a - logged raw a| = 6.505213034913027e-19
```

Using the logged applied `a/b`, filtered target q and ray scale reproduces every
live raw-range value:

```text
maximum absolute reconstruction error = 3.552713678800501e-15 m
```

These deterministic checks support the analyzer and recorded formula; they do
not validate the physical quantity contract.

## Calibration and anchor evidence

- 24 source/epoch transitions and 4 naturally occurring reseed records were
  observed; 17 event rows use live calibration and 11 use cache.
- Applied `a`/`b` vary in all groups. The largest observed standard deviations
  are `1.829e-4` for `a` and `0.02676` for `b`.
- Nine of 11 groups have exactly zero accepted-anchor membership churn while
  range and `a/b` still vary. Membership churn is therefore not supported as
  the primary driver in those groups. Per-anchor q variation, depth-domain
  behavior and fit state remain candidates.
- Center-8 groups have lower metric-depth spans (about 3.0-3.3 m) and the
  greatest `a/b` variation. This is association, not proof of causality.
- Conditioning medians are about 1,601-2,359, below the configured `1e5`
  rejection threshold. That does not prove the affine model or anchor domain
  is physically correct.

## Quantity and geometry evidence

GT is the Euclidean range from Gazebo `camera_link` origin to target model
origin at the recorded simulation timestamp. The raw target is an
eroded-bbox foreground inverse-depth statistic and raw geometry uses the
vehicle position plus configured body-FRD offset. The visible-surface to
target-center offset and verified optical-center transform are not recorded.

The observed drone-center versus `camera_link` distance contribution is only
0.0187-0.0287 m, so that lever arm alone does not support the multi-metre
errors. It does not eliminate the unverified optical-center or target-surface
quantity mismatch.

Geometry is strongly context dependent in this capture: live mean errors are
about +9.75 m left, +7.97 m right and +10.89 m oblique, while the two center
10 m repeats differ markedly. This controlled comparison supports further
quantity/extrinsic/domain investigation; it is not by itself causal proof.

## Timestamp evidence

GT source simulation timestamp and GT pose timestamp match with recorded
offset 0 ms in selected examples, but the authoritative sensor-capture
timestamp remains unavailable. Moreover 133/389 frames record
`consume_now_monotonic_s < depth_completed_timestamp_s`. These frames fail the
precommitted ordering contract. Measurement-to-consume delay has a median of
about 0.999 s, but no dynamic-latency conclusion is made from static data.

## Required-field completeness

Every selected raw-range frame is missing or cannot verify:

- per-anchor target-exclusion result;
- calibration age;
- distortion model and rectification state;
- extrinsics fingerprint;
- verified optical center;
- target ROI q quantiles and selected foreground-statistic name.

Additionally 133 frames fail timestamp ordering. Values remain `NOT_RECORDED`,
`UNKNOWN` or incomplete; none are inferred. The source sidecar stays fail-soft
as designed, while this offline audit fails the frame completeness gate.

## Hypothesis decisions

| Hypothesis | Status | Evidence boundary |
| --- | --- | --- |
| Quantity/domain mismatch | `SUPPORTED` | raw foreground-surface statistic and GT model center differ; offset is missing |
| Vehicle-center vs camera_link explains multi-m error | `NOT_SUPPORTED` | observed contribution ≤0.0287 m |
| Calibration lifecycle variation | `SUPPORTED` | a/b and event-aligned range vary; freeze usually reduces jitter |
| Anchor membership churn is primary | `NOT_SUPPORTED` | zero churn in 9/11 varying groups |
| Offline refit differs from runtime fit | `NOT_SUPPORTED` | full-run replay is numerically exact |
| Timestamp/frame association is valid | `NOT_SUPPORTED` | 133 ordering failures and no sensor-capture timestamp |
| Camera/extrinsic evidence is complete | `NOT_SUPPORTED` | required fields absent on all selected frames |
| Required diagnostics are complete | `NOT_SUPPORTED` | 0/389 complete frames |

No single causal fix is claimed from correlation. The permitted conclusion is
therefore `DIAGNOSTIC_EVIDENCE_INSUFFICIENT`, even though calibration,
quantity-domain and timestamp problems are supported targets for a future
instrumentation-completeness patch and recapture.

## Verification

Preflight before capture:

```text
Focused R3.6A tests: 59 passed
Repository tests: 303 passed, 1 known PytestReturnNotNoneWarning
./run_all.sh --check: Runtime check passed
No old PX4/Gazebo/backend/train process
Diagnostics default off and residual correction default/effective off
Instrumentation on/off non-interference: PASS
```

R3.6A-V analyzer tests before final repository regression:

```text
python3 -m pytest -q test_range_v2_r3_6a_validation.py \
  test_range_physical_diagnostics.py
Result: 17 passed
```

Final full-suite and runtime-check results are frozen in
`r3_6a_validation_manifest.json` after this report is complete.

## Files and rollback

New implementation files are offline/capture-only:

```text
range_v2_r3_6a_capture.py
range_v2_r3_6a_validation.py
test_range_v2_r3_6a_validation.py
docs/RANGE_V2_R3_6A_VALIDATION_REPORT.md
artifacts/range_v2/r3_6a_validation/
```

No R3.6A production instrumentation source was changed. Rollback removes only
the three R3.6A-V files, this report section, and the isolated validation
artifact directory. Original datasets, manifests, runtime estimator and
controller require no restoration.

## Decision

```text
DIAGNOSTIC_EVIDENCE_INSUFFICIENT
```

R3.6B, R4, R5, physical fixes, training, shadow inference and controller
integration remain **NO-GO / not started** pending explicit review.
