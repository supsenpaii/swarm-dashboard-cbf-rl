# CORE RANGE TEMPORAL LAG ROOT CAUSE AUDIT

Audit ID: `core_range_temporal_lag_root_cause_audit_20260805_v001`

## Conclusion

```
TEMPORAL_EVIDENCE_INSUFFICIENT
```

This is a genuinely informative negative result, not a shrug — see "Why
insufficient" below. No retrain, no additional data collection, no
quarantine use, no MiDaS/M52/calibration/ROI/filter/EKF/controller/PX4 code
change, no shadow, no Follow Target. The `[-3, +3]s` shift sweep is a
diagnostic oracle only (it queries ground truth to find the best alignment)
and was never used to correct predictions at runtime.

## Data used (unchanged from the dynamic-robust retrain)

8 accepted dynamic groups / 844 frames, reloaded via the existing frozen
`_load_dynamic_rows` loader (same integrity checks: timestamp-stage
validation, trace-identity dedup, checksum verification against
`dynamic_session_manifest.json`). Model prediction signal:
`PHYSICAL_TEMPORAL` / `B_balanced` OOF predictions from
`artifacts/core_range_3_12m/dynamic_robust_retrain/prediction_rows.csv`,
positionally joined back to the timestamped frames (both derive from the
same deterministic `(session_id, measurement_timestamp_s)`-sorted order).

## 1. Timestamp chain

Full per-frame chain in `timestamp_chain.csv` (844 rows). Three of the
twelve requested concepts are genuinely **not recorded anywhere** in the
pipeline — left blank, not inferred:

| Concept | Status | Why |
|---|---|---|
| Sensor/frame capture | NOT RECORDED | `tracking_web.py` explicitly sets `sensor_capture_timestamp_s=None`, `sensor_capture_clock="unavailable"`. The closest proxy is `frame_receipt` (python_monotonic, captured immediately after BGR conversion), used as "capture" throughout this audit. |
| Feature creation | NOT RECORDED | No wall-clock "features assembled at" field; features inherit the row's `measurement_timestamp_s`. Only the *derived* cadence field `delta_time_s` exists, and it is not a timestamp. |
| Model prediction | NOT RECORDED | `FrozenDirectEnsemble.predict()` is a pure offline batch call; no timestamp is attached to a prediction anywhere, including in `prediction_rows.csv`. |

The other nine are recorded on **two distinct clock domains that are never
reconciled onto one axis anywhere in the pipeline**:

- `python_monotonic` — measurement/depth-submit/worker-start/depth-complete/
  result-publish/consumer-receive/consume/sidecar-write (all 8 stages of
  `TIMESTAMP_STAGE_ORDER`). Values are on the process-uptime scale
  (~30,000s in the audited sessions).
- `gazebo_sim_time` — ground truth (`ground_truth.timestamp_s`), interpolated
  from a separately-recorded pose trace onto the frame's
  `source_sim_timestamp_s`. Values are on the simulation-clock scale
  (~10–30s in the audited sessions).

All lag/shift computation (both the original retrain gate and this audit)
stays entirely within the `python_monotonic`-indexed `measurement_timestamp_s`
axis, treating each frame's `ground_truth_range_m` as a value already
correctly paired to that frame at capture. `ground_truth_pose_age_ms` /
`ground_truth_interpolation_span_ms` / `ground_truth_time_offset_ms`
(present per-frame in `timestamp_chain.csv`) quantify the residual GT-side
misalignment budget — in the sampled data these were consistently small
(0–20ms), so GT-to-frame pairing itself is not a source of second-scale lag.

One additional finding worth flagging: two numerically distinct "consume"
fields exist in the sidecar (`timestamp_stages.consume.timestamp_s` vs
`timestamps.consume_now_monotonic_s`), differing by ~0.15–0.2s per frame.
Both are reported separately (`c08` / `c08b`) rather than assumed identical.

## 2. Latency decomposition (844 frames, all 8 groups)

| Stage | Median (s) | P90 (s) | P95 (s) |
|---|---:|---:|---:|
| capture → submit | 0.248 | 0.355 | 0.379 |
| submit → worker start | 0.022 | 0.223 | 0.270 |
| worker inference duration | 0.612 | 0.881 | 0.963 |
| complete → publish | 0.000 | 0.000 | 0.000 |
| publish → consume | 0.212 | 0.291 | 0.332 |
| **capture → consume** | **1.131** | **1.479** | **1.566** |

Worst group by median capture→consume: `cdr_stop_recede_9_r2`. Full per-frame
figures in `latency_decomposition.csv`.

`consume → prediction`, `capture → prediction`, and `prediction age at use`
are **not computable** — no model-prediction timestamp exists (see above);
left blank in `latency_decomposition.csv`, not inferred.

**Worker inference duration (MiDaS-side depth computation) is the single
largest measured, real, causal contributor** — median 0.61s, P95 1.12s —
roughly 54% of total measured capture-to-consume latency (0.61s of 1.13s),
with queue-wait and consumer-side stages together contributing only ~0.23s
(21%). This is solid, non-oracle, directly-measured evidence: it is not
where the diagnosis stops, but it is not zero either — real pipeline latency
alone already accounts for roughly half to all of a ~1.1–1.9s lag budget,
before any model or motion effects are considered.

## 3. Cross-correlation shift sweep, [-3, +3]s (diagnostic oracle)

Two signals were swept independently per group: `raw_physical_range_m`
(model-independent, single-frame, unfiltered physical estimate) and the
`PHYSICAL_TEMPORAL`/`B_balanced` model prediction. Full sweep in
`shift_sweep.csv` (1936 rows = 8 groups × 2 signals × 121 shift steps);
best-per-group in `per_group_lag_metrics.csv`.

**Key finding: the raw physical signal's MAE-vs-shift landscape is
unstable.** 6 of 8 groups saturate at the ±3.0s search boundary (best shift
= ±3.0 or ±2.85 exactly), with inconsistent sign across groups (some -3.0,
some +3.0) and even negative correlation in several groups
(`cdr_approach_right_yaw_r3`: -0.95; `cdr_recede_right_r5`: -0.92). This
means the single-frame, unfiltered physical range signal is too noisy for
this MAE-minimization search to identify a stable, physically-meaningful
temporal offset — the search is essentially hitting its window limit rather
than converging on a real optimum for most groups.

The model-prediction signal is better-behaved on two of three measures: only
3 of 8 groups saturate at the boundary (vs. 6 of 8 for the physical signal),
and correlation is consistently positive and often high (0.31–0.92, vs.
several negative values for the physical signal). Median **absolute** best
shift across groups is **2.85s**. But the **sign** of the best shift is
*not* consistently positive: 5 of 8 groups shift positive (all 3
"approach" groups, plus `cdr_recede_right_r5` and `cdr_stop_approach_6_r1`)
and 3 of 8 shift negative (`cdr_recede_center_r3`, `cdr_recede_left_yaw_r4`,
`cdr_stop_recede_9_r2`) — median **signed** best shift is only 1.575s, with
std 2.619s spanning the full ±3.0s window. The positive/negative split
correlates with scenario type (approach vs. recede) rather than looking like
a clean, scenario-independent "prediction always lags truth in absolute
time" signal — consistent with a roughly monotonic range-vs-time curve
letting the MAE-minimization search find similar scores at shifts of either
sign, which is itself evidence this specific oracle method cannot cleanly
separate "true processing lag" from "curve-shape/trend sensitivity" at this
window width, on top of the physical signal's outright instability.

What can still be said with the sign-inconsistency caveat in mind: the
**magnitude** of the best-fit shift (2.85s median absolute, for the
better-behaved of the two signals) is larger, not smaller, than the
1.93–2.00s previously reported by the retrain gate. That original figure
used the same `alignment_lag_s` search capped at **±2.0s**, and its
per-config values (1.925–2.000s) cluster suspiciously close to that cap.
This audit's wider window shows the original gate metric was very likely
truncated at its own search boundary — and even ±3.0s does not fully bound
it, since 3 of 8 groups (`cdr_approach_center_r1`, `cdr_approach_right_yaw_r3`,
`cdr_stop_recede_9_r2`) still saturate on the model signal.

## 4. Causal compensation replay (no XGBoost retrain)

Median MAE per option/signal (`causal_compensation_metrics.csv`, full
per-group detail there):

| Option | Physical signal MAE (m) | Model signal MAE (m) |
|---|---:|---:|
| A — current prediction, unmodified | 4.007 | 1.411 |
| B — re-timestamp by measured causal latency only | 3.982 | 1.449 |
| C — constant-velocity forward projection | 4.028 | 1.864 |

- **Option B** (shifting each prediction's effective timestamp forward by
  its own group-median measured `capture_to_consume_s` — a purely causal,
  currently-available correction, no ground truth used) leaves residual
  best-alignment lag essentially unchanged for both signals (physical:
  saturates at 3.0s both before and after; model: still 1.9–3.0s in
  magnitude, same sign pattern as before). **This robustly rules out a pure
  timestamp-bookkeeping/association bug as the (sole) explanation** — if
  frames were simply mislabeled with the wrong clock reading, correcting for
  the known, measured real latency should have collapsed the residual lag
  toward zero. It did not.
- **Option C** (constant-velocity projection using the causal
  `causal_raw_range_rate_m_s` feature, projected forward by the same
  group-median latency used as "age") made both signals modestly **worse**,
  not better (model MAE 1.411 → 1.864). This specific, simple
  implementation of motion compensation is not a quick fix; it does not by
  itself indicate the true cause is motion-compensation-shaped, only that
  this particular causal-rate/median-latency formulation does not help as
  implemented.

## Why "insufficient evidence" rather than a specific cause

The classification decision tree was fixed in `audit_plan.json` before any
result was computed (see that file for the exact precommitted order and
thresholds) and applied mechanically:

1. Option B did not reduce residual lag to ≤0.30s → **not**
   `TIMESTAMP_ASSOCIATION_ERROR`.
2. The physical (model-independent) signal's shift search saturated at the
   ±3.0s boundary for 6 of 8 groups, exceeding the precommitted limit of 2
   → **`TEMPORAL_EVIDENCE_INSUFFICIENT`**, per the plan's rule 2, evaluated
   before the pipeline-latency-share and model-vs-physical-gap rules.

This threshold was not tuned after seeing the result; it was fixed at
`prepare` time specifically because a saturating majority of groups means
the oracle itself cannot reliably bound the true lag for the
model-independent signal, so no downstream comparison built on it (pipeline
latency's share of "the true lag," or "how much of the true lag is
model-specific") can be trusted quantitatively, even though the model
signal and the real measured-latency numbers are individually informative.

**What we do know, with confidence, independent of the sweep's instability:**
- The lag is **not** a simple timestamp-domain/bookkeeping bug (Option B
  rules this out robustly, on both signals).
- Real, measured, causal pipeline latency (capture→consume, dominated by
  MiDaS worker inference duration) is on the order of **1.1s median, up to
  ~1.8s at P95** — a large fraction of any plausible true lag figure, and
  not attributable to queueing or the consumer side (which together are
  only ~0.23s median).
- The previously reported ~1.93s gate figure was very likely itself
  capped/truncated by its own ±2.0s search window, not a clean measurement
  of the true lag magnitude.

## Recommended next step (not performed here)

Per scope, this audit does not perform fixes. If further root-cause
narrowing is wanted, the most direct next step suggested by this evidence is
to re-run the shift-sweep oracle against a **smoothed** or **model-based**
signal only (the raw physical signal is not fit for this kind of search),
with a window wide enough to stop saturating (start from ≥±4s given 3 of 8
groups still saturate at ±3.0s here), before concluding whether the
remaining ~0.8–1.7s gap beyond measured pipeline latency is
model-response lag, cadence/backlog effects, or something not yet
instrumented.

## Full test verification

- Focused tests (`test_core_range_xgboost_benchmark.py`,
  `test_core_range_direct_dynamic_replay.py`, `test_core_range_logging_eval.py`,
  `test_core_range_static_balance_collection.py`,
  `test_core_range_priority_collection_audit.py`): 43 passed.
- Full repository suite (excluding the unrelated stale
  `swarm_dashboard_handoff_20260729/` snapshot bundle): 373 passed.
- `./run_all.sh --check`: passed.
- No PX4/Gazebo/backend/capture/training/shadow process was started or left
  running at any point in this task.

## Deliverables

```
artifacts/core_range_3_12m/temporal_lag_audit/
  audit_plan.json
  timestamp_chain.csv
  latency_decomposition.csv
  shift_sweep.csv
  per_group_lag_metrics.csv
  causal_compensation_metrics.csv
  missing_timestamp_fields.json
  audit_manifest.json
  audit_report.md
```
