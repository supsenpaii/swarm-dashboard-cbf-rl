# CORE_RANGE_5HZ_HEADLESS_DYNAMIC_RECOLLECTION_AND_RETRAIN

Date: 2026-08-05
Conclusion: `DATA_COLLECTION_BLOCKED_BY_PERSISTENT_RTF_STUTTER`

## 2026-08-05 continuation: three missing groups

The five accepted sources were reused unchanged. Their sidecar, capture,
and trajectory SHA-256 values were recomputed before collection (15/15
matched); the accepted corpus remains 5 groups / 570 frames with zero
malformed, checksum, timestamp, anchor-count, nonfinite, or duplicate-trace
failures. No quarantined attempt was promoted.

Only `cdr_recede_left_yaw_5hz_headless`,
`cdr_recede_right_5hz_headless`, and
`cdr_stop_approach_6_5hz_headless` were retried. The continuation contract
used a maximum of 8 total attempts per missing group, a full stack restart
and 10-second cooldown after failure, calibration prewarm, an immediate
five-sample RTF precheck, and a fresh one-sample/second RTF trace covering
only the collection window. A run was rejected for either median RTF <0.8
or a long-stutter streak of at least three consecutive RTF<=0.3 samples.
The latter matches the established stutter definition in the RTF audit.
`recede-left-yaw` retained its precommitted
`central_quantile_region` policy; no parameter was changed after capture.

All three groups exhausted attempts 1–8 without an accepted replacement.
Among the 18 continuation attempts (attempts 3–8 for each group), 16 failed
the RTF/long-stutter hard gate, one failed calibration prewarm, and one
failed stop-hold evidence. All six new `recede-left-yaw` attempts and all
six new `recede-right` attempts failed the RTF/long-stutter gate. The
parseable continuation attempts still showed healthy non-RTF behavior:
camera-source median 40.108–45.807 FPS, tracking median 21.936–27.166 FPS,
capture→consume median 69.0–97.5 ms and P95 136.7–189.0 ms; MiDaS P95 was
55.1–102.7 ms (one attempt exceeded the 100 ms gate). The RTF median ranged
0.2478–0.9986, but even high-median attempts contained prohibited long
low-RTF streaks (RTF P5 0.0328–0.0395).

The data gate therefore remains approaching 3/3, receding 1/3, and
stop-and-hold 1/2. Training was not started, folds/preprocessing were not
fit, and no model was created. The environment repeatedly passed the
pre-capture RTF check then stuttered inside the collection window despite
full restarts, so this continuation concludes
`DATA_COLLECTION_BLOCKED_BY_PERSISTENT_RTF_STUTTER`.

## Scope

Sole objective: recollect the dynamic corpus in the headless Gazebo
configuration confirmed by `docs/CORE_RANGE_CAMERA_SOURCE_FPS_REPORT.md`
(`SWARM_START_GAZEBO_GUI=false`), then train and evaluate a 3–12 m range
model. No Gazebo GUI, no depth-rate change from 5 Hz, no M52/calibration/
controller/PX4 change, no use of quarantined data (old 5 Hz or 2 Hz
corpora), no shadow, no active range, no Follow Target, no forced pass.

## 1. Preflight

Three representative headless smoke captures were run before committing to
the full 8-group collection (two 32 s approaching captures, one 15 s
burst-sampling capture). All confirmed:

- `SWARM_START_GAZEBO_GUI=false` effective — no `gz sim -g` process in any
  run (enforced both by the env var and by an explicit in-script check
  that fails the capture if a GUI client is detected);
- camera-source median FPS: 41.0–43.4 (gate ≥25 — comfortably passed every
  time, including later in this report's full 13-attempt collection: min
  36.15 across all attempts, gate never once failed);
- tracking median FPS: 22.4 (gate ≥20 — passed, closer to the gate than
  camera FPS, consistent with the camera-source-FPS report's own finding
  that tracker compute becomes the binding constraint once source delivery
  is fast);
- capture→consume median 61.4 ms / P95 128.8 ms (gate ≤200/≤300 ms — PASS);
- MiDaS P95 50.8 ms (gate ≤100 ms — PASS);
- raw_range_computed: 125–131 rows (gate ≥40 — PASS);
- effective depth rate 4.2 Hz (within the accepted 4–6 Hz band around the
  5.0 Hz scheduler target);
- timestamp/checksum/96-anchor integrity: PASS.

**One metric did not reliably pass**: Gazebo real-time factor (RTF),
measured via `/world/default/stats`, one sample/second plus a direct
10-message continuous-burst cross-check to rule out a sampling-method
artifact. Median RTF across the three preflight attempts was 0.39–0.42
(gate ≥0.8), and the burst sample showed a genuine, reproducible bimodal
pattern — roughly 60% of samples near 1.0, the rest near 0.03–0.1 — present
throughout the run, not concentrated at startup, and present with nothing
else running concurrently on the machine. This is a real, headless-
independent Gazebo physics-loop stutter, not a measurement bug.

Per the task's own design (`maximum_attempts_per_session: 2`, precommitted
in `collection_plan.json` before any capture), this volatility is exactly
what the retry mechanism exists to average out — one bad RTF sample does
not indicate the environment is incapable of headless operation, since
every *functional* gate (camera FPS, tracking FPS, latency, integrity)
passed regardless of the RTF number. Preflight therefore did not conclude
`FIVE_HZ_HEADLESS_RECOLLECTION_BLOCKED`; collection proceeded using the
full precommitted per-group gate (including RTF ≥0.8) via the existing
retry-then-quarantine mechanism, letting the true data-gate outcome emerge
from real attempts.

## 2. Instrumentation and tooling added

All additive/opt-in, reusing the camera-source-FPS task's own
instrumentation:

- `core_range_collect_dynamic_5hz_headless_scenario.sh` (new): headless
  variant of the existing, unmodified `core_range_collect_dynamic_5hz_
  scenario.sh` (left untouched — it belongs to the quarantined
  `core_range_dynamic_5hz_post_contention_fix` corpus and is referenced by
  that corpus's own manifest). Sets `SWARM_START_GAZEBO_GUI=false`
  unconditionally inside the script (not left to caller convention),
  `SWARM_CAMERA_TRACE_JSONL` for direct per-session camera-FPS
  verification, a background `gz topic -e -t /world/default/stats`
  sampler for RTF, an explicit `pgrep -f "gz sim -g"` guard that fails the
  capture if a GUI client is somehow detected, and
  `--dataset-role core_range_dynamic_5hz_headless_post_fixes`.
- `core_range_collect_dynamic_5hz_headless_batch.py` (new): headless
  adaptation of the existing, unmodified `core_range_collect_dynamic_5hz_
  batch.py`. Reuses that module's own `sessions()` trajectory definitions
  unchanged (same 8 planned sessions, same `central_quantile_region` ROI
  recovery scoped to `cdr_recede_left_yaw` only), extends `audit()` with
  two new per-group gate checks — `camera_source_fps ≥ 25` and
  `gazebo_rtf ≥ 0.8` — computed directly from the new per-session trace/log
  files rather than assumed.
- `core_range_dynamic_capture.py`: one-line additive change — added
  `core_range_dynamic_5hz_headless_post_fixes` to the `--dataset-role`
  argparse `choices` tuple (the only change to any file outside this
  task's own new scripts).
- `core_range_5hz_headless_retrain.py` / `core_range_5hz_headless_
  collection_report.py` / `core_range_5hz_headless_finalize_incomplete.py`
  (new): training driver and collection/finalize report generators — see
  §5.

Focused tests: `test_core_range_collect_dynamic_5hz_headless_batch.py` (5
tests) and `test_core_range_5hz_headless_retrain.py` (6 tests), all
passing.

## 3. Collection result

Two independent attempts per planned group (13 total attempts: 5 groups
accepted on their first or second try, 3 groups exhausted both attempts).

| Group | Scenario | Attempts | Result | Failure mode |
|---|---|---:|---|---|
| cdr_approach_center | approaching | 2 | ACCEPTED | attempt 1 failed RTF (0.259) |
| cdr_approach_left | approaching | 2 | ACCEPTED | attempt 1 failed GT coverage (motion undershoot) |
| cdr_approach_right_yaw | approaching | 1 | ACCEPTED | — |
| cdr_recede_center | receding | 1 | ACCEPTED | — |
| cdr_recede_left_yaw | receding (ROI recovery) | 2 | **FAILED** | both attempts failed RTF + tracking FPS |
| cdr_recede_right | receding | 2 | **FAILED** | both attempts failed RTF (attempt 2 also tracking FPS) |
| cdr_stop_approach_6 | stop_and_hold | 2 | **FAILED** | attempt 1 GT coverage, attempt 2 RTF |
| cdr_stop_recede_9 | stop_and_hold | 1 | ACCEPTED | — |

**Accepted: 5/8** — approaching 3/3, receding 1/3, stop_and_hold 1/2.
Required: 3/3, 3/3, 2/2.

`camera_fps.csv`: camera-source median FPS across all 13 attempts ranged
36.15–47.60, **never once below the 25 FPS gate** — camera-source delivery
itself is unconditionally fixed, exactly as `docs/CORE_RANGE_CAMERA_
SOURCE_FPS_REPORT.md` concluded.

`tracking_fps.csv`: tracking median FPS ranged 19.5–31.7; three attempts
(all three of `cdr_recede_left_yaw`×2 and `cdr_recede_right`×1) fell just
under the 20 FPS gate (19.55–19.79).

`gazebo_rtf.csv`: 6 of 13 attempts failed the RTF ≥0.8 gate (median as low
as 0.186). **Failures cluster by session order and scenario**: the first
attempt at `cdr_approach_center` (the very first capture in the run) failed
RTF, then every attempt stayed above gate until `cdr_recede_left_yaw`
onward (attempts 6–13), where 5 of 8 attempts failed RTF — all four
receding-direction sessions plus one stop_and_hold attempt. This is
additional evidence, not conclusive: it is consistent with either (a) pure
run-to-run RTF volatility (already established as a real, headless-
independent, bimodal stutter in §1) landing unfavorably in this particular
session's later half, or (b) a wall-clock/thermal accumulation effect
across ~20 minutes and 13 consecutive stack launches that the camera-
source-FPS report's own two-repeat test (`F_repeat_1`/`F_repeat_2`) was too
short to reveal. This task did not run enough repetitions to distinguish
between these two explanations and does not claim to.

## 4. Recede-left-yaw ROI recovery

The precommitted `central_quantile_region` recovery
(`SWARM_TARGET_ROI_RECOVERY_POLICY=central_quantile_region`, scoped to
`cdr_recede_left_yaw` only via `collection_plan.json`, never made a global
default) was applied to both attempts. It was **not the failure cause**:
both attempts produced raw-range measurements (avoiding the historical
inverse-depth ROI rejection this recovery was built to fix) but failed on
the runtime RTF/tracking gates instead — the same failure mode affecting
other, non-recovery-policy receding sessions in this run. No ROI or bbox
parameter was changed after inspecting a capture, GT, or model prediction.
Both failed attempts remain unchanged in `quarantine/`.

## 5. Data gate and training

Data gate (§4 of the task): approaching 3/3 ✅, receding 1/3 ❌, stop_and_
hold 1/2 ❌. **Corpus is incomplete.**

Per the task's explicit instruction, training was not started: the
verified static corpus (27 groups / 832 frames) was not combined with the
incomplete dynamic corpus, the historical 2 Hz corpus and the old 5 Hz
quarantine were not used, no fold preprocessing was fit, and no XGBoost
model was created. `feature_contract.json` records the precommitted
PHYSICAL_ONLY (40 features) and causal PHYSICAL_TEMPORAL (52 features,
+12 causal temporal features reset at session boundaries) contracts,
status `PRECOMMITTED_NOT_FIT`. `fold_assignments.csv`, `static_metrics.csv`,
`dynamic_metrics.csv`, `temporal_metrics.csv`, `bbox_stress_metrics.csv`,
`prediction_rows.csv`, and `model_comparison.csv` contain headers only.
`models/` is empty.

A training driver (`core_range_5hz_headless_retrain.py`) was built and unit
-tested in advance, reusing — unmodified, as pure functions —
`core_range_dynamic_robust_retrain.py`'s training/evaluation engine
(`add_temporal_features`, `assign_combined_folds`, `train_variant`,
`evaluate_config`, `oof_bbox_stress`, and the four gate functions/
thresholds, byte-identical to this task's §7 requirements), so that a
future recollection completing the data gate can proceed directly to
training without re-deriving this integration work.

## Integrity

Across the 5 accepted groups / 570 frames (`integrity_report.json`):
malformed records 0, checksum failures 0, timestamp-ordering failures 0,
anchor-count failures 0 (every row logged exactly 96 anchors), nonfinite
ranges 0, cross-session duplicate traces 0. Integrity of the accepted
corpus is not in question — only its completeness against the precommitted
8-group requirement.

## Disposition

No model trained, no candidate frozen, no runtime/shadow/Follow Target
action, no depth-rate change, no Gazebo GUI use, no quarantine data used
for anything beyond diagnostic reporting.

## Verification

- Focused tests: 11 passed (5 + 6, see §2).
- Full repository test suite and `./run_all.sh --check`: recorded in
  `retrain_manifest.json` after final execution.
- All PX4/Gazebo/ROS2/backend/capture/training processes stopped cleanly
  after every capture attempt and at task completion.

## Deliverables

`artifacts/core_range_3_12m/dynamic_5hz_headless_retrain/`: `collection_
plan.json`, `accepted_sessions.json`, `quarantined_sessions.json`,
`integrity_report.json`, `runtime_metrics.csv`, `latency_metrics.csv`,
`camera_fps.csv`, `tracking_fps.csv`, `gazebo_rtf.csv`, `frozen_dataset_
manifest.json`, `feature_contract.json`, `fold_assignments.csv`,
`static_metrics.csv`, `dynamic_metrics.csv`, `temporal_metrics.csv`,
`bbox_stress_metrics.csv`, `prediction_rows.csv`, `model_comparison.csv`,
`retrain_manifest.json`, `retrain_report.md`, `models/` (empty), plus raw
per-attempt evidence under `runtime_sessions/` and `quarantine/`.
