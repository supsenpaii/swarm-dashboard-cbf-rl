# CORE RANGE DYNAMIC REPAIR AND REPLAY REPORT

Conclusion: `DIRECT_CANDIDATE_FAILS_BBOX_ROBUSTNESS`

This is the follow-on to `DYNAMIC_REPLAY_BLOCKED_BY_DATA_INTEGRITY`
(`docs/CORE_RANGE_DIRECT_DYNAMIC_REPLAY_REPORT.md`). It repaired the near-start
lateral collection blocker, collected the four missing dynamic groups, and
replayed the frozen direct `C_shallow` candidate on the complete 8-group
corpus.

## 1. Root cause

The two quarantined `cdr_recede_left_yaw` attempts (original and
center-preserving 80% bbox) both showed `target_depth.valid = true` and
`valid_fraction = 1.0` on every rejected frame — the ROI/bbox extraction and
foreground-pixel selection were working correctly, with full pixel coverage
and no background contamination. The rejection
(`inverse_depth_uncertainty_too_large`) came from the temporal filter's
relative-uncertainty gate (`bearing_range_filter.py`,
`inverse_depth_relative_gate = 0.18`, hardcoded in `metric_target_fusion.py`
and matching the env-var default): the MAD-gated foreground-half statistic
(`median_of_median_or_mad_gated_foreground_half`) produced a per-frame
`std / raw` ratio of 0.20-0.40 at this specific geometry, above gate on 100%
of measurement-attempt frames in both attempts.

Classification: `HIGH_TARGET_DEPTH_VARIATION` / `THRESHOLD_TOO_STRICT_FOR_GEOMETRY`,
not `ROI_BACKGROUND_CONTAMINATION` or `TARGET_PARTIALLY_OUTSIDE_BBOX`.

## 2. Offline ROI/statistic recovery search

Eight deterministic, non-GT, non-model-prediction variants (A original bbox,
B/C/D centered 90/80/70%, E/F erosion 10/20%, G current foreground-half
statistic, H central-IQR-band statistic) were compared on live probe captures
per geometry (`SWARM_TARGET_ROI_VARIANT_DIAGNOSTICS=1`, opt-in, added to
`TargetDepthExtractor`). Results (full data in
`artifacts/core_range_3_12m/dynamic_repair_and_replay/roi_variant_results*.csv`):

| Geometry | Best variant | Availability | Ratio (min-median-max) |
|---|---|---|---|
| `cdr_recede_left_yaw` (yaw 15→-15) | H central-quantile | 100% | 0.043-0.047-0.060 |
| `cdr_recede_left_yaw` default (G) | — | 0% | 0.198-0.272-0.394 |
| `cdr_recede_right` (no yaw) | G foreground-half (default) | 100% | 0.029-0.041-0.048 |
| `cdr_recede_right` H (recovery) | — | 0% | 0.251-0.335-0.380 |
| `cdr_stop_recede_9` | G foreground-half (default) | 100% | 0.011-0.034-0.073 |

The recovery policy is **not universal**: it fixes the yaw-varying receding
geometry and actively breaks the non-yaw receding and stop-recede geometries.
This was discovered the hard way — an initial official capture of
`cdr_recede_right` with the policy globally enabled failed 46/50 frames
before the per-geometry probe revealed the reversal (quarantined as
`cdr_recede_right_r4_attempt_...`).

## 3. Recovery policy

`target_depth_extractor.py`: opt-in, default-off
`SWARM_TARGET_ROI_RECOVERY_POLICY=central_quantile_region` (also settable via
constructor kwarg `recovery_policy`). When unset, `TargetDepthExtractor`
behavior, output fields and byte-for-byte statistic name are unchanged from
before this branch (verified by
`test_target_depth_extractor_roi_variants.py::test_recovery_policy_default_off_is_byte_identical_to_before`
and a same-input off-vs-on field comparison test). When enabled, only the
foreground-pixel-selection statistic changes (median of the central IQR band
instead of the MAD-gated near-half); bbox/erosion geometry, calibration, the
temporal filter, `inverse_depth_relative_gate` (still 0.18), EKF, runtime
output, controller and PX4 are untouched. Applied per session:
`cdr_recede_left_yaw_r4` only. See
`artifacts/core_range_3_12m/dynamic_repair_and_replay/recovery_policy.json`
for full per-geometry justification.

## 4. Smoke and missing-session collection

Smoke (`cdr_recede_left_yaw_smoke_v3`, 45 s, policy on): 88 measurement-attempt
frames, 90.9% raw availability, 0 `inverse_depth_uncertainty_too_large`
rejections (down from 100%). PASS on all gates
(`artifacts/core_range_3_12m/dynamic_repair_and_replay/smoke_report.json`).

Four missing sessions collected and integrity-audited via the project's own
`core_range_logging_eval.py` → `audit_session()` → `archive_runtime_logs()`
pipeline (same tooling the original four accepted sessions went through):

| Session | Frames | Recovery policy |
|---|---|---|
| `cdr_recede_left_yaw_r4` | 112 | on |
| `cdr_recede_right_r5` | 147 | off (default) |
| `cdr_stop_recede_9_r2` | 92 | off (default) |
| `cdr_stop_approach_6_r1` | 47 | off (default) |

Two attempts were quarantined and superseded by one retry each, per the
project's established one-retry precedent (`dynamic_replay_plan_amendment_v003/v004.json`):
`cdr_recede_right_r3` (calibration never converged — infrastructure timeout,
unrelated to ROI) and `cdr_recede_right_r4` (recovery policy applied to the
wrong geometry, per §2).

`dynamic_replay_plan_amendment_v006.json` records the final accepted session
identities and reverts all three near-lateral bboxes to their original
(pre-v005) geometry — the accepted fix is the per-geometry statistic swap,
not a bbox scale change.

## 5. Coverage gate

8/8 groups accepted, all integrity `PASS`: 3/3 approaching, 3/3 receding,
2/2 stop-and-hold, 844 total frames.
(`artifacts/core_range_3_12m/dynamic_repair_and_replay/dynamic_coverage_report.json`,
`accepted_sessions.json`, `quarantined_sessions.json`.)

## 6. Dynamic replay result

Frozen direct `C_shallow` three-fold ensemble replayed unchanged on all 8
groups, comparing raw physical range, direct ensemble unsmoothed and causal
smoothed (`tau = 0.20 s`), no future-frame use.

```json
{
  "mae_m": 1.984, "median_absolute_error_m": 1.726,
  "p90_abs_error_m": 4.151, "p95_abs_error_m": 4.218,
  "signed_bias_m": 0.510, "error_gt_3m_fraction": 0.289
}
```

Gate checks: accuracy (equal-group MAE, P90, worst-session MAE, error>3m) all
FAIL; temporal (median lag, stop settling, stationary std) all FAIL — median
absolute lag sat at the 2.0 s search ceiling; bbox robustness FAILS on the
±10% catastrophic-error check (±5% degradation/shift gates PASS).

Per-group detail
(`artifacts/core_range_3_12m/direct_dynamic_replay/per_group_metrics.csv`):
the model measurably improves on raw physical range in 6/8 groups, and both
stop-and-hold sessions individually clear the 1.0 m MAE gate
(`cdr_stop_approach_6_r1` 0.51 m, `cdr_stop_recede_9_r2` 0.68 m,
settling time 0.76 s). The three approaching/receding groups captured before
this branch (`cdr_approach_left_r2`, `cdr_approach_right_yaw_r3`,
`cdr_recede_center_r3`) and the two new receding groups all show 1.4-4.9 m MAE
during continuous motion, well above gate. `cdr_stop_recede_9`'s model
prediction never re-enters the settling tolerance band (`NEVER_SETTLED`)
despite a very low stationary std, i.e. it converges to a stable but biased
value.

## 7. What this means

The ROI/collection blocker is fixed and will not recur for these three
geometries. The candidate genuinely helps during stop-and-hold and partially
helps during continuous motion, but does not meet the precommitted dynamic
accuracy, temporal-response or bbox-robustness gates as a whole. This is a
legitimate `DIRECT_CANDIDATE_FAILS_BBOX_ROBUSTNESS` result on real, integrity-verified
dynamic data — not a data-integrity block and not a partial-corpus artifact.

## Scope guards

No retraining, hyperparameter or feature change; no gate change; candidate
checksum `8c3f2bb450439e6141143ae63ca021ac1fbbc76f25cd62e655aa434c652671fc`
unchanged; residual correction remains default-off; no Follow/OFFBOARD/arm/
takeoff/mode-change/motion-or-controller-endpoint call; no shadow or runtime
integration.

## Verification

```text
Focused + repository test suite: 373 passed, 1 known PytestReturnNotNoneWarning
./run_all.sh --check: PASS
Candidate checksum: unchanged, reconfirmed
dynamic_session_manifest.json: 8/8 accepted, integrity_status=PASS for all
```

Key SHA-256 values at gate close:

```text
dynamic_replay_manifest.json
  580ad432d6ecf0df3b6169d0caf00b8ce14b122aa83158ce3b82d235f796983e

dynamic_session_manifest.json
  94382ea0c952875cc15e624a38b638187df3026d3189e6a7cdc5f25ffc9987f9

dynamic_replay_plan_amendment_v006.json
  0cc286431b77700cae7b58fd5c2c176d7356254b2073b967aa590844fe004002
```

Work stops here. No shadow evaluation or Follow Target/runtime integration is
authorized regardless of this conclusion; the next step requires explicit
review and a new prompt.
