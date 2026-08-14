# Core Range Optimization 3–12 m

## Objective

Optimize stable metric range for the tracked target only between 3 m and
12 m using the existing tracker, MiDaS, M52 geometry and XGBoost infrastructure.
Near/far zone models, far direction estimation and the full Range V2 state
machine are deferred.

## Runtime architecture

```text
RGB + target bbox
  -> MiDaS inverse depth
  -> M52 metric calibration
  -> raw range + quality/context features
  -> XGBoost direct and residual benchmarks
  -> confidence gate and light causal temporal stabilization
  -> camera ray + accepted range
  -> target geolocation
  -> PX4 FollowTarget only after replay/shadow/safety gates
```

XGBoost remains part of the plan. Direct range regression is the primary
candidate because the raw baseline has large, context-dependent signed bias;
residual regression remains a required comparison.

## Frozen practical sequence

1. Minimal logging integrity — complete (`CORE_LOGGING_READY`).
2. Collect independent static development sessions across 3–12 m — complete.
3. Benchmark raw, residual XGBoost and direct XGBoost with seed 52 and
   session-group splits.
4. Evaluate every 1 m bin plus stationary jitter and dynamic lag.
5. Add only a light causal filter to the selected candidate.
6. Offline replay, then shadow with zero controller effect.
7. Propose PX4 Follow integration only through a separate safety gate.

## Collection priorities

The static corpus is now balanced to the minimum three-session gate in every
1 m bin. The priority and balance batches completed in this order:

1. 10–11 m and 11–12 m, currently missing.
2. 6–7 m, currently represented by only one independent group.
3. Balance the other 1 m bins.

Each bin now has at least three independent static sessions with center,
lateral and pitch/background contexts. Dynamic approaching/receding coverage
is still absent. One run/session remains one group; no group may cross a future
train/validation/test partition.

## Required frame contract

- run/session/group/frame identity;
- measurement, source and GT timestamps;
- record and trace checksums;
- raw physical range and GT range;
- calibration scale/offset and quality;
- target inverse depth and ROI quality;
- image ray, bbox geometry and pitch/context;
- deterministic gate result;
- no GT field used as a runtime feature.

## Model evaluation

Report frame-weighted, equal-group, per-bin and per-group metrics:

- bias, median absolute error, MAE, P90 and P95;
- errors greater than 2 m and 3 m;
- stationary standard deviation, P95 frame delta and drift;
- dynamic lag and range-rate response where dynamic GT exists.

Initial development targets are aggregate MAE at most 1.0 m, P90 at most
1.5–2.0 m, no bin MAE above 1.5 m and stationary jitter at most 0.3 m. These
targets must be frozen before evaluating a locked test partition.

## Scope guards

- Residual correction remains off in runtime by default.
- Do not random-split frames.
- Do not train before 10–12 m and 6–7 m coverage is repaired.
- Do not use a temporal filter to hide excessive lag.
- Do not publish to PX4 Follow before offline replay and shadow pass.
- Do not reopen or overwrite old Range V2 artifacts or old NO-GO models.

## 2026-08-04 static balance gate

`CORE_RANGE_STATIC_BALANCE_COLLECTION` completed with:

```text
STATIC_3_12M_COVERAGE_READY_FOR_MODEL_BENCHMARK
```

The balance batch added 18 accepted independent groups and 558 raw frames in
3–4, 4–5, 5–6, 7–8, 8–9 and 9–10 m. Every requested bin has three groups and
every frame passes timestamp, GT trace/checksum, record checksum and 96-anchor
integrity. Three failed technical attempts remain quarantined.

The raw estimate is still strongly context-dependent: equal-group MAE ranges
from 0.610 m at 4–5 m to 5.823 m at 5–6 m. Coverage PASS is not an accuracy
PASS. No model was trained. The next gate, after explicit review, is the offline
raw-versus-residual-versus-direct XGBoost benchmark with group-disjoint
partitions and seed 52.

## 2026-08-04 offline XGBoost benchmark

`CORE_RANGE_XGBOOST_OFFLINE_BENCHMARK` completed with:

```text
DIRECT_XGBOOST_BEST_DEVELOPMENT_CANDIDATE
```

The benchmark used 832 static development frames from 27 independent groups,
three groups in every 1 m bin, and frozen three-fold group-disjoint CV with
seed 52. Direct `C_shallow` achieved 0.427 m equal-group MAE, 0.446 m mean
per-group P90, 1.399 m worst-bin MAE and zero errors over 3 m. Residual
`B_balanced` reached 1.294 m MAE and failed three development gates.

This result is not a promotion. Bbox width dominates direct-model importance
in all folds, 18/27 static groups have near-zero prediction variation, and
dynamic behavior remains unmeasured. Runtime residual correction stays off;
no backend/controller integration or shadow was performed. See
`docs/CORE_RANGE_XGBOOST_BENCHMARK_REPORT.md`.

Verification completed with 14 focused tests, 348 repository tests and the one
known warning; `./run_all.sh --check` passed. The branch stops here pending an
explicitly approved replay/dynamic-evidence prompt.

## 2026-08-04 direct dynamic replay gate

`CORE_RANGE_DIRECT_DYNAMIC_REPLAY_GATE` stopped fail-closed with:

```text
DYNAMIC_REPLAY_BLOCKED_BY_DATA_INTEGRITY
```

The direct `C_shallow` candidate, features, preprocessing, clipping, smoothing,
seed and gates were frozen before collection. Four of eight sessions passed
integrity (three approaching and one receding; 446 total frames). The
near-start lateral-left receding scenario then produced zero accepted raw-range
rows twice, including after a checksummed collection-only ROI correction. The
repeated rejection was `inverse_depth_uncertainty_too_large`.

No candidate prediction or partial-corpus metric was generated. Dynamic
accuracy, temporal response and bbox robustness remain N/A. All failed attempts
are quarantined; runtime/controller/PX4 were unchanged and residual correction
remains default-off. See `docs/CORE_RANGE_DIRECT_DYNAMIC_REPLAY_REPORT.md`.

Verification: 16 focused tests and 364 repository tests passed with the one
known warning; `./run_all.sh --check` passed. No stale process remained at gate
close. Work stops here; neither partial replay nor shadow is authorized.

## 2026-08-05 dynamic ROI recovery and replay

`CORE_RANGE_DYNAMIC_REPAIR_AND_REPLAY` completed the blocked branch above with:

```text
DIRECT_CANDIDATE_FAILS_BBOX_ROBUSTNESS
```

Root-cause analysis of the two quarantined near-start lateral-left receding
attempts found the ROI/bbox extraction itself was not the problem (target
`valid_fraction` was 1.0 in every rejected frame); the MAD-gated
foreground-half statistic was inflating the relative inverse-depth
uncertainty ratio past the unchanged 0.18 gate for that specific geometry. An
opt-in, default-off `SWARM_TARGET_ROI_RECOVERY_POLICY=central_quantile_region`
statistic was added to `TargetDepthExtractor` (`target_depth_extractor.py`),
selected from a deterministic, non-GT 8-variant offline comparison. A
per-geometry probe on `cdr_recede_right` and `cdr_stop_recede_9` then showed
the *opposite*: the unchanged default statistic was already correct there,
and the recovery policy actively made those geometries worse — so the policy
was applied only where its own geometry-specific evidence justified it, not
globally. See `artifacts/core_range_3_12m/dynamic_repair_and_replay/recovery_policy.json`.

All 8 dynamic groups (3 approaching, 3 receding, 2 stop-and-hold, 844 frames)
reached integrity PASS; `dynamic_replay_plan_amendment_v006.json` records the
final accepted session identities. The frozen direct `C_shallow` candidate was
then replayed unchanged: equal-group MAE 1.98 m (gate 1.0 m), P90 4.15 m (gate
2.0 m), median lag at the 2.0 s search ceiling, and a ±10% bbox width
perturbation produced a catastrophic (>3 m) error on at least one session.
The model measurably beats raw physical range on most groups and both
stop-and-hold sessions individually clear the accuracy gate, but the
aggregate dynamic-motion gates and bbox-robustness gate are not met.

No retraining, feature, gate or candidate-checksum change occurred; residual
correction remains default-off; no shadow or Follow Target integration was
performed. See
`docs/CORE_RANGE_DYNAMIC_REPAIR_AND_REPLAY_REPORT.md`.

Verification: 373 repository tests passed with the one known warning;
`./run_all.sh --check` passed; candidate checksum, session-manifest integrity
and scope guards were reconfirmed after the run. Work stops here pending
explicit review of the FAILS_BBOX_ROBUSTNESS result before any further step.

## 2026-08-05 temporal/contention audit and depth scheduler fix

Temporal-lag audit found capture→consume median ≈1.13s across the frozen
8-group corpus, while MiDaS itself runs 5-14ms standalone. A staged
contention isolation (`CORE_RANGE_LIVE_STACK_CONTENTION_ISOLATION`,
configs A-F) ruled out model compute, worker-thread/GIL overhead, and
Gazebo+PX4 rendering alone, landing on `ROS_CALLBACK_OR_THREAD_BLOCKING_
DOMINANT`: full-stack worker inference at the 7.5Hz production depth-submit
rate averaged ~779ms, dropping to ~25ms at 2.0Hz in that isolated smoke test.

`CORE_RANGE_DEPTH_SCHEDULER_CONTENTION_FIX` confirmed the existing
`LatestDepthWorker`/`depth_rate_hz` architecture already decouples depth
submission from camera/tracker FPS (single-slot mailbox, non-blocking
submit, no unbounded queue — no new scheduler code was needed) and swept
2/3/4/5/7.5Hz capture→consume latency. Only 2.0Hz met the ≤200ms
median/≤300ms P95 gate in that sweep (median 42.8ms); the production default
`SWARM_METRIC_TARGET_DEPTH_RATE_HZ` was changed from 7.5 to 2.0 on that
basis. See `artifacts/core_range_3_12m/depth_scheduler_contention_fix/`.

## 2026-08-05 2Hz dynamic recollection and retrain — corpus incomplete

`CORE_RANGE_2HZ_DYNAMIC_RECOLLECTION_AND_RETRAIN` set out to recollect the
8-group dynamic corpus at the new 2.0Hz default and retrain/evaluate
`PHYSICAL_ONLY`/`PHYSICAL_TEMPORAL` XGBoost variants against it. Preflight
passed (2 pre-existing test regressions from the 2.0Hz default were fixed —
`test_metric_target_fusion.py`'s two convergence tests had implicitly relied
on the old 7.5Hz cadence; fixed by pinning `depth_rate_hz` explicitly in the
tests, no production/gate change). Collection reached **7/8 groups** (3
approaching, 2/3 receding, 2 stop-and-hold) — `cdr_recede_left_yaw_2hz`
failed 5 collection attempts (2 precommitted + 3 additional, identical
unmodified spec) with the target's ROI never producing a valid inverse-depth
extraction; this same session geometry also needed repeated recovery in the
original 7.5Hz-era corpus (see `dynamic_repair_and_replay/`), so it is a
known-hard scenario, not a 2.0Hz regression per se.

More importantly, the **latency gate did not hold** under a realistic full
two-vehicle, 32s-trajectory collection: capture→consume median came back at
**1.226s** (P95 1.849s) across the 7 accepted groups — essentially
unchanged from the old 7.5Hz-era corpus's 1.131s median, and nowhere near
the depth-scheduler-contention-fix task's 42.8ms figure. Root cause: that
figure was derived from a short, single-vehicle-adjacent isolated smoke
test (config F) whose depth pipeline never once produced a successful raw
range measurement; every measured row in that run happened to have low
actual MiDaS latency, which does not generalize to this task's realistic
multi-group collection, where MiDaS worker inference itself is again taking
0.5-1.3s per call — the same symptom the contention-fix task set out to
resolve.

Per task instructions the depth rate was left at 2.0Hz (no rate change was
in scope here), no gate was lowered, and no model-based decision was made
about session acceptance. Training (Phase 4 onward) was not started.
**Conclusion: `TWO_HZ_DYNAMIC_CORPUS_INCOMPLETE`.** 380 repository tests
passed; `./run_all.sh --check` passed; no stray PX4/Gazebo/backend
processes remained. See `docs/CORE_RANGE_2HZ_DYNAMIC_RETRAIN_REPORT.md` and
`artifacts/core_range_3_12m/dynamic_2hz_retrain/`. Recommended follow-up
(not performed here, out of this task's scope): re-investigate the ROS/
thread contention itself rather than relying on submission-rate throttling
alone, and separately root-cause the `cdr_recede_left_yaw` geometry.
## 2026-08-05 full-stack contention correction

The earlier 2 Hz scheduler result is superseded for representative dynamic
load.  The actual dominant fault was a per-tracking-frame deepcopy of all 600
Gazebo pose-history snapshots under the Python GIL.  Copying only the timestamp
bracket reduced representative capture-to-consume median by 94.9% and MiDaS
worker median by 95.3%, without changing M52/calibration/range semantics or raw
availability.  Three observation-only dynamic smokes pass; the post-fix sweep
selects 5 Hz as the highest fully valid rate meeting the complete gate; 7.5 Hz
failed stable-calibration prewarm and was excluded.  See
`docs/CORE_RANGE_FULL_STACK_CONTENTION_FIX_REPORT.md` and
`artifacts/core_range_3_12m/full_stack_contention_fix/`.
## 2026-08-05 5 Hz dynamic recollection outcome

The post-contention 5 Hz recollection preflight passed, but the new corpus did
not pass its per-group runtime gate. Sixteen attempts (two for each of eight
planned groups) produced 44--142 valid raw rows each with 0 checksum,
timestamp, duplicate, 96-anchor or nonfinite failures. Capture→consume P95 was
144--215 ms, but tracking was consistently 15.59--17.65 FPS versus the >=20
FPS gate; six attempts also exceeded MiDaS P95 100 ms. All attempts are
quarantined, accepted coverage is 0/8, and training was not started. The
geometry-specific central-quantile recovery did restore raw production for
recede-left-yaw, but did not cure the runtime tracking failure. See
`docs/CORE_RANGE_5HZ_DYNAMIC_RETRAIN_REPORT.md`.
## 2026-08-05 collection-throughput paired investigation

The 27–28 FPS historical smoke and 15.6–17.7 FPS collection metrics are
definition-compatible, but the historical smoke already performed full
dataset and 96-anchor diagnostic disk writes. In the current paired window the
smoke baseline itself ran at 16.48 FPS and full collection at 17.6 FPS.
Disabling all collection disk writes/diagnostics, using bounded memory,
disabling overlay, and batching fsync changed tracking by at most +7.4% and
never reached 20 FPS. Full-loop P95 was 35.17 ms while camera source remained
17–18 FPS. No collection component met the dominance gate, so no production
throughput fix or three-scenario post-fix validation was performed. See
`docs/CORE_RANGE_COLLECTION_THROUGHPUT_FIX_REPORT.md`.
## 2026-08-05 camera-source FPS root cause and fix

The camera-source FPS question the collection-throughput investigation left
open (`COLLECTION_THROUGHPUT_EVIDENCE_INSUFFICIENT`) is now root-caused.
New hop-level tracing (Gazebo sensor sim-time publish → gz-transport
wall-clock receipt → tracking mailbox → tracker output) found the camera
sensor publishes at a rock-solid 50 Hz in *sim time* in every run (never
the bottleneck); the loss to wall-clock FPS tracks Gazebo's real-time
factor directly, which measured a median of only 0.20–0.32 (5th percentile
as low as 0.03) against a configured target of 1.0 while the Gazebo GUI
client process was running. Disabling it
(`SWARM_START_GAZEBO_GUI=false`, a new additive opt-in `run_all.sh`
env-gate, default unchanged) raised RTF to a median of ~1.0 and
camera-source FPS from ~29 to ~45 FPS in the same representative scenario.
GPU/MiDaS compute contention, the (nonexistent) ROS2 image bridge, the
dashboard client, and static config/code drift were all ruled out with
direct evidence. Three post-fix validation scenarios
(approaching/receding/stop-and-hold) pass camera-source ≥25 FPS, tracking
≥20 FPS, capture→consume, and MiDaS-P95 gates with ~1.8× margin; one
downstream, out-of-scope tracker-lock-duty-cycle metric does not pass and
is flagged as a separate follow-up. Conclusion:
`CAMERA_SOURCE_FPS_FIXED_READY_FOR_RECOLLECTION`. See
`docs/CORE_RANGE_CAMERA_SOURCE_FPS_REPORT.md` and
`artifacts/core_range_3_12m/camera_source_fps/`.
## 2026-08-05 headless 5 Hz dynamic recollection attempt

Recollection at `SWARM_START_GAZEBO_GUI=false` confirmed camera-source FPS
is unconditionally fixed: across 13 real capture attempts, camera-source
median FPS never once fell below the 25 FPS gate (range 36.15-47.60). But
a second, RTF-specific instability surfaced: Gazebo real-time factor,
measured directly via `/world/default/stats` and cross-checked with a
continuous-burst sample to rule out a measurement artifact, showed a real,
headless-independent bimodal stutter (roughly 60% of samples near 1.0, the
rest near 0.03-0.1, throughout each run) that intermittently drops the
per-session RTF median below the 0.8 gate. 6 of 13 attempts failed on RTF
(sometimes with tracking FPS, which also dipped just under 20 FPS on the
three worst attempts); failures clustered in the second half of the
~20-minute, 13-attempt collection run, mostly on receding-direction
sessions -- consistent with either ordinary RTF volatility landing
unfavorably or a thermal/resource accumulation effect this task did not
run enough repetitions to distinguish. Collection reached 5/8 accepted
groups (approaching 3/3, receding 1/3, stop_and_hold 1/2) -- short of the
required 8/8, so training was not started. Conclusion:
`FIVE_HZ_HEADLESS_DYNAMIC_CORPUS_INCOMPLETE`. See
`docs/CORE_RANGE_5HZ_HEADLESS_DYNAMIC_RETRAIN_REPORT.md` and
`artifacts/core_range_3_12m/dynamic_5hz_headless_retrain/`.
## 2026-08-05 Gazebo RTF stutter root-cause and gate validation

Clock-domain audit found no sim-time/wall-clock mixing bug: the target is
externally teleported via wall-clock-paced gz set_pose (not physically
integrated), and ground truth is matched to camera frames entirely in
Gazebo sim time -- both correct regardless of RTF. But a 28-minute
persistent-stack long run (never restarted, 1780 one-second samples, 76
stable<->stutter transitions, typical run length 3-7s) showed real
raw-range accuracy degrades in low-RTF windows: median absolute error vs
ground truth rose from 1.27m (RTF>=0.8, n=225) to 2.08m (RTF<=0.3, n=298),
a 63% increase, on real captured data -- checksum/timestamp/anchor
integrity stayed 100% in both regimes (records are not corrupted, just
less accurate). Severe stutter also caused 5 of 13 reps to fail outright
on gz set_pose service-call timeouts. No single resource factor explains
the stutter (all correlations |r|<0.22 across CPU frequency, thermal,
memory, disk, context switches, GPU). Because raw accuracy demonstrably
degrades in low-RTF windows, RTF cannot be safely downgraded to a
diagnostic-only signal, and no root-cause fix was found; restart-per-session
is not adopted since the prior 5Hz-headless task's 13 attempts were already
full-stack restarts every session and still saw the same pattern. All other
functional gates (camera FPS, tracking FPS, latency, MiDaS P95, integrity)
remain solid and pass in fresh approaching/receding/stop-and-hold
validation. Conclusion: `RTF_REMAINS_HARD_GATE_STUTTER_UNRESOLVED`. See
docs/CORE_RANGE_GAZEBO_RTF_STUTTER_REPORT.md and
artifacts/core_range_3_12m/gazebo_rtf_stutter/.

## 2026-08-05 missing-group collection continuation

The five accepted headless groups were hash-verified and left unchanged.
Only the two missing receding groups and one missing stop-and-hold group
were retried, up to 8 total attempts each, with headless full-stack restart,
calibration prewarm, pre-capture RTF gate, continuous collection-window RTF
monitoring, and unchanged scenario parameters. All three exhausted the
budget. Of 18 continuation attempts, 16 failed median RTF and/or a
three-sample low-RTF stutter streak; one failed prewarm and one lacked
stop-hold evidence. Functional throughput stayed healthy on parseable runs
(camera 40.1–45.8 FPS, tracking 21.9–27.2 FPS, capture→consume P95
136.7–189.0 ms), so no FPS optimization was performed. Corpus remains
5/8 groups / 570 clean frames; training did not start and quarantine was
not used. Conclusion: `DATA_COLLECTION_BLOCKED_BY_PERSISTENT_RTF_STUTTER`.

## 2026-08-05/06 deterministic sim-time driver: root-caused, fixed, validated, then blocked on a new GT race

A later session traced the sim-time trajectory plugin's non-loading to
`GZ_SIM_SERVER_CONFIG_PATH` being a no-op fallback when the world SDF
already declares its own system plugins (PX4's `default.sdf` does) --
fixed by generating a per-run overlay world SDF with the plugin embedded
directly (`sim_time_trajectory_plugin/build_overlay_world.py`,
`SWARM_GZ_WORLD_SDF` additive override in `run_all.sh`). End-to-end
validated (subscriber present, publish delivered, `trajectory_events.jsonl`
written continuously). A follow-up session ran the formal
CORE_RANGE_SIM_TIME_VALIDATE_COLLECT_AND_TRAIN Phase 1 validation
(approaching x2, receding, stop_and_hold, production bbox presets): found
and fixed a real driver bug (`gz topic -p`'s one-shot CLI publish racing
the plugin's async gz-transport subscriber discovery, silently dropping
the start command -- fixed with a bounded `gz topic -i` subscriber-wait
before publish, verified by a clean re-run). Pose determinism between two
independent runs of the same scenario/seed was exact (160/160 matched
elapsed-time pairs, 0.0 m diff). But 3 of 4 runs each showed exactly one
frame with `main.py`'s Gazebo pose-history bracket returning "unavailable"
(~0.5-0.6%/frame, not RTF-correlated, absent from the historical
legacy-driver corpus's quarantine reasons) -- Phase 1's GT-alignment gate
has no insufficient-evidence exception, so this is not waved through.
`stop_and_hold` also showed a real 46% raw-error increase in low-RTF
windows, reinforcing (not contradicting) the standing RTF-hard-gate
conclusion. The 5 accepted groups were independently re-hash-verified and
remain valid/unchanged. Phase 3 (collect the 3 missing groups) and
training were not started per the task's own phase-gating rule. Conclusion:
`GROUND_TRUTH_ALIGNMENT_FAILED`. See
docs/CORE_RANGE_SIM_TIME_DYNAMIC_RETRAIN_REPORT.md and
artifacts/core_range_3_12m/sim_time_dynamic_retrain/.

## 2026-08-06 GT pose-bracket race fixed and validated

Traced the exact race: `simulation_target_ground_truth` (`main.py`) does a
synchronous bracket lookup against `pose_history`, appended from a
separate Gazebo pose-subscription callback thread; a camera frame newer
than every buffered sample so far failed immediately even though the
needed sample was about to arrive. Fixed with a bounded
(`threading.Condition`, 120ms cap) wait/retry on that specific case only
-- a camera frame *older* than the oldest buffered sample still fails
immediately, since no future sample can fill that in. Times out to a
distinct `GT_BRACKET_TIMEOUT` (never a fabricated/nearest/extrapolated
GT); a new `reset_ground_truth_pending()` hook abandons any in-flight wait
at session boundaries (start/stop/bbox reselect). No M52, calibration,
PX4, controller, or trajectory-semantics change. 10 unit tests added/
updated in `test_gazebo_ground_truth_history.py`; full suite 432 passed.

Re-ran all four Phase 1 validation scenarios live: **0 bad GT frames
across 752 raw frames** (previously 3/4 runs each had exactly 1), with 5
frames directly observed being recovered via one retry cycle each (proof
the fix does real work, not just luck). Pose determinism stayed exact
(0.0 m). One run (`receding`) still failed the RTF-window raw-error gate
on a real stutter (37% increase) -- expected, unrelated to this fix, RTF
remains a separate hard gate. Conclusion:
`GT_BRACKET_RACE_FIXED_READY_FOR_MISSING_GROUPS`. Missing-group collection
and training were not attempted in this task. See
docs/CORE_RANGE_GT_BRACKET_RACE_FIX_REPORT.md and
artifacts/core_range_3_12m/sim_time_dynamic_retrain/gt_bracket_fix_validation/.

## 2026-08-06 overnight: missing groups collected (8/8), trained, no candidate meets gate

A multi-role (single-agent, sequential) overnight session collected the 2
missing receding groups and 1 missing stop-and-hold group using the fixed
`sim_time_plugin` driver, production bbox/geometry, and the unmodified
RTF hard gate: 3 accepted (1 quarantined for RTF long-stutter first) in
only 4 total attempts across ~13 minutes -- much faster than the prior
session's 24 exhausted attempts, read as favorable transient machine load
rather than any fix to the still-unresolved RTF stutter itself. Corpus
reached 8/8 (approaching 3, receding 3, stop_and_hold 2), frozen with the
27-group/832-frame static corpus into a 35-group/2081-frame combined
dataset. Trained PHYSICAL_ONLY and PHYSICAL_TEMPORAL (3 precommitted
XGBoost configs each, reusing the prior session's unmodified training
driver): all 6 configs failed static, dynamic, temporal, and bbox
robustness gates simultaneously (best dynamic MAE 1.25m vs the 1.0m gate).
Independent review found no leakage or calculation error that would flip
the conclusion; one measurement-fidelity anomaly (temporal-lag search
saturating at its 2.0s boundary in unmodified shared evaluation code) was
documented but does not change the outcome, since three other gate
families already fail independently. No candidate frozen. Conclusion:
`NO_SIM_TIME_DYNAMIC_ROBUST_MODEL_MEETS_GATE`. See
docs/CORE_RANGE_OVERNIGHT_SIM_TIME_TRAIN_REPORT.md and
artifacts/core_range_3_12m/overnight_sim_time_train/.

## 2026-08-06 offline failure analysis and targeted retrain: no candidate, but two evaluation bugs found

Purely offline (no Gazebo/PX4/ROS2/backend) follow-up: reproduced the
overnight six-config baseline bit-exact (every artifact checksum
matched), then built a full per-frame failure matrix joining out-of-fold
predictions with every runtime-quality feature. Found static error is
concentrated (group MAE 0.02-3.60m across 27 groups; worst bin 11-12m at
2.04m) rather than diffuse, and a moderate but systematic static/dynamic
distribution shift in depth/calibration/ROI features (several
standardized-mean-differences 0.34-0.62). Traced two evaluation-gate
artifacts by reading the actual perturbation/lag code: `perturb_bbox_features()`
never recomputes `image_ray_x/y`/ROI/anchor stats, so the bbox_stress
gate cannot move PHYSICAL_ONLY/PHYSICAL_TEMPORAL predictions at all
(confirmed 0.0 shift empirically) and is really just re-measuring
baseline accuracy; `alignment_lag_s()` saturates at its 2.0s search
boundary even for the untouched raw physical-range signal (100% of
sessions), so the reported temporal lag number is not a faithful
model-lag measurement. Feature ablation showed the complete feature sets
substantially outperform any subset (no starvation) and nearest-neighbor
analysis found no near-duplicate feature vectors (no ambiguity problem).
Precommitted and trained 3 targeted candidates (robust pseudo-Huber
objective; monotonic constraint on raw_physical_range_m; both combined):
2 of 3 collapsed to predicting a constant 12.0m (training failure under
that objective/hyperparameter combination, inconclusive on the
hypothesis); the third (monotonic constraint) showed a corrected,
like-for-like improvement of only +0.013m static / -0.003m dynamic --
noise-level. No candidate frozen. Conclusion:
`TARGETED_MODEL_IMPROVES_BUT_STILL_FAILS`. See
docs/CORE_RANGE_OFFLINE_FAILURE_ANALYSIS_REPORT.md and
artifacts/core_range_3_12m/offline_targeted_retrain/.

## 2026-08-06 stable relative tracking: direction agreement is the hard limiter

A follow-up offline task changed the optimization target from raw MAE to
tracking *stability* (direction agreement, rank correlation, stationary
jitter, absence of catastrophic jumps) and built a new
`features -> Direct XGBoost -> calibration -> causal alpha-beta filter ->
clamp[3,12]m` pipeline. Two findings from the feature/monotonic audit:
(1) `raw_physical_range_m` and every ROI-depth feature FLIP sign between
"center" and lateral/yaw-varying dynamic groups (global correlation with
GT is actually -0.34, the opposite of the naive physical prior) --
explains why the prior task's monotonic-constraint candidate showed no
improvement; (2) `ray_scale` is the one feature with a consistent
(negative) sign specifically in the dynamic regime, used for a new
monotonic candidate. All 3 candidates trained without collapse. Swept the
full precommitted calibration x alpha-beta-threshold grid (171
combinations): 0 pass the practical stability gate. Root cause: true
frame-to-frame target motion at this corpus's 5Hz/0.25 m/s regime has
median magnitude 0.035m, but the raw model's own frame-to-frame noise is
0.148m -- over 4x larger -- so no calibration (bias-only) or in-grid
causal filter setting can recover >=90% consecutive-frame direction
agreement (best achieved: 0.57). No candidate frozen; per the task's own
rule, observation-only runtime integration and live Gazebo smoke testing
were correctly not attempted. Conclusion:
`STABLE_RELATIVE_RANGE_STILL_UNSTABLE`. See
docs/CORE_RANGE_STABLE_RELATIVE_TRACKING_REPORT.md and
artifacts/core_range_3_12m/stable_relative_tracking/.

## 2026-08-06 control-ready observation pipeline: windowing did not fix the direction problem

Follow-up offline task: instead of chasing per-frame accuracy, built a
practical control-usability pipeline -- `Candidate B raw prediction ->
causal alpha-beta state estimator -> measurement-age compensation ->
predicted_current_range_m`, plus an independent monitor-only causal
windowed trend estimator (0.6/0.8/1.0s, Theil-Sen slope, hysteresis) --
and scored it against control-usability metrics (windowed direction
agreement over 0.6-1.0s instead of consecutive-frame direction, stop-and-
hold jitter/settling, age-compensation quality, control usability) rather
than the old 4-gate accuracy criteria. Phase 0 bit-exact reproduced
Candidate B (max diff 0.0). Replayed all 8/8 dynamic groups: global MAE
1.158m and worst-bin MAE 1.885m both passed, catastrophic jumps stayed 0,
but windowed direction agreement only reached 0.516 at the best (1.0s)
window -- barely above chance, vs. an 0.80 target -- and stationary std
during stop-and-hold hold phases (0.588m) narrowly missed its 0.55m
target. Root cause, confirmed by direct inspection of a monotonically-
approaching group: some of Candidate B's raw errors are not
high-frequency noise that a window averages away, but multi-frame-
persistent regional bias (up to ~3.5m sustained over dozens of
consecutive frames in one trajectory's 4.5-8m portion) -- a windowed
slope estimator cannot tell that apart from genuine motion. This extends
rather than contradicts the prior task's per-frame SNR finding. Offline
gate failed; per the task's own rule, Phase 3 (observation-only runtime
integration) and Phase 4 (live Gazebo smoke test) were correctly not
attempted -- no Gazebo/PX4/ROS2/backend process was started. Conclusion:
`OFFLINE_CONTROL_READY_RANGE_FAILED`. See
docs/CORE_RANGE_CONTROL_READY_OBSERVATION_REPORT.md and
artifacts/core_range_3_12m/control_ready_observation/.

## 2026-08-06 residual bias correction: audit confirms bias, corrector does not generalize

Follow-up Giai đoạn 1 of the user's own roadmap: audit Candidate B's
residual by geometry/distance context, then train a small residual
corrector (Candidate B backbone + runtime features -> residual -> corrected
range) instead of tuning the downstream filter further. Bias audit
confirmed the premise: 20 (domain x geometry_context x distance_bin) cells
show >=1m persistent mean bias, up to 3.13m in the worst cell; only 4 of 17
audited features keep a consistent correlation sign across center/lateral/
yaw_oblique geometry contexts (ray_scale, target_inverse_depth,
target_roi_q_min/p10). Trained 3 precommitted configs (Ridge baseline,
shallow XGBoost strong-reg, shallow XGBoost on the 4 stable features),
group-disjoint 3-fold CV over the full 35-group corpus, none collapsed --
but held-out correlation between predicted and true residual was
weak-to-negative for all 3 (Spearman 0.20 / -0.30 / -0.14), while an
in-sample control check confirmed the models fit training data fine
(R^2=0.53), pinning this as small-corpus overfitting rather than a
formulation bug. All 3 failed the Giai đoạn 1 gate (bias reduction target
>=50%, best achieved 20.2%; direction agreement target >=0.80, best
achieved 0.533 -- essentially unchanged from the raw baseline). Per the
roadmap's own logic this is the trigger for Giai đoạn 2 (targeted small
data collection), not further corrector tuning; no new collection was
started autonomously. Conclusion: `RESIDUAL_BIAS_CORRECTOR_GATE_FAILED`.
See docs/CORE_RANGE_RESIDUAL_BIAS_CORRECTION_REPORT.md and
artifacts/core_range_3_12m/residual_bias_correction/.

## 2026-08-06 targeted dynamic pilot (Stage 2A): blocked at Phase 0 by disk space

Attempted Giai đoạn 2A of the roadmap: collect 14 targeted, independent
dynamic groups to test whether Candidate B's persistent regional bias
reproduces outside the original 8-group corpus. Phase 0 preflight caught
the issue before any collection started: Candidate B reproduction
(bit-exact) and runtime source verification (GT-bracket fix, sim-time
trajectory plugin, overlay world, subscriber-ready wait, headless GUI
gate -- all 10 checks) passed, but the disk-space preflight failed --
13.54GB available on the workspace filesystem vs. a 15GB minimum required
by the task itself. `artifacts/` alone is 29GB of the 35GB workspace, the
accumulated historical record of every prior task in this chain; per the
task's own rules, no historical artifact was deleted and no gate was
lowered to force a pass. No Gazebo/PX4/backend process was started.
Conclusion: `PILOT_BLOCKED_BY_STORAGE`. See
docs/CORE_RANGE_TARGETED_DYNAMIC_PILOT_REPORT.md and
artifacts/core_range_3_12m/targeted_dynamic_pilot/.
