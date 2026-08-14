# CORE RANGE DEPTH PIPELINE LATENCY OPTIMIZATION REPORT

Optimization ID: `core_range_depth_pipeline_latency_optimization_20260805_v001`

## Conclusion

```
DEPTH_PIPELINE_LATENCY_OPTIMIZATION_INSUFFICIENT
```

A safe, correctly-implemented optimization was applied and measured a real
2.33x speedup in isolation, but a live SITL smoke replay shows it does not
meaningfully reduce real-world capture→consume latency, because the true
bottleneck lives elsewhere. No retrain, no official dataset collection, no
GT/M52/calibration/controller/PX4 change, no Follow Target.

## Baseline (from the prior temporal-lag audit)

capture→consume median 1.13s / P95 1.57s; MiDaS `worker_inference_s` median
0.61s; queue/consumer stages median 0.21s; timestamp-association error
already ruled out; naive constant-velocity compensation already shown to
make things worse. This task treats those as fixed reference points and
targets the *pipeline* latency specifically — see
`docs/CORE_RANGE_TEMPORAL_LAG_ROOT_CAUSE_AUDIT.md`.

## 1. Pipeline profile (8 frozen baseline dynamic sessions, 844 frames)

| Stage | Median (s) | P90 (s) | P95 (s) |
|---|---:|---:|---:|
| frame receipt → submit | 0.248 | 0.355 | 0.379 |
| submit → worker start | 0.022 | 0.223 | 0.270 |
| **worker inference** | **0.612** | **0.881** | **0.963** |
| complete → publish | ~0.00002 | ~0.00004 | ~0.00004 |
| publish → consumer receive | 0.212 | 0.291 | 0.332 |
| consumer receive → consume | ~0.0000007 | ~0.0000015 | ~0.0000017 |
| **measurement age at consume** | **1.131** | **1.479** | **1.566** |

Full per-frame data: `artifacts/core_range_3_12m/depth_latency_optimization/baseline_profile.csv`.

`pending-frame replacements`, `dropped frames`, and true `stale-result count`
(worker-level counters, `dropped`/`discarded_generation_results` in
`depth_worker.py`) are **not retroactively recoverable** from these
historical sessions — they are live counters, and no `status()` snapshot was
taken during the original captures. What *is* recoverable and reported
(`queue_drop_metrics.csv`): per-session rejection-stage counts
(`calibration_rejected`, `inverse_depth_filter_rejected`,
`anchor_support_rejected`), and a derived proxy — **under the collection-mode
staleness threshold (3.0s, deliberately relaxed from the 0.75s production
default for these captures — see `main.py:2908-2911`), zero frames were
recorded as `depth_result_stale`.** Critically, at the **production
default** of 0.75s, the majority of these already-accepted frames *would*
have been rejected as stale (`baseline_profile.csv`'s
`would_be_stale_under_production_default_0_75s` column; per-group fractions
in `baseline_profile_summary.json`) — the current latency is not just "some
extra lag," it would silently discard most measurements under the real
production staleness gate.

## 2. Latest-frame policy — audit only, no gap found

Checked `depth_worker.py` (`LatestDepthWorker`) and `metric_target_fusion.py`
in detail:

- **Single pending slot, not a queue**: `submit()` overwrites `self._pending`;
  an overwritten unstarted job increments `self.dropped`
  (`depth_worker.py:117-120`). No backlog is possible.
- **Generation-gated publish**: results from a job submitted before a
  `reset()` are discarded, not published (`depth_worker.py:229-233`,
  counter `discarded_generation_results`).
- **Version-gated consumption**: the consumer never reprocesses an
  already-seen result (`_consumed_depth_version`,
  `metric_target_fusion.py:612-619`).
- **Age-gated staleness rejection — already real, not just diagnostic**:
  `maximum_depth_age_s` (default 0.75s, env `SWARM_METRIC_TARGET_MAX_DEPTH_AGE_S`)
  is enforced in `_consume_depth_result`
  (`metric_target_fusion.py:1580-1597`); a stale result is rejected
  (`accepted=False`) and a `depth_result_stale` diagnostic stage is written,
  not silently used.

**No code was changed for this step.** The "latest-frame, no backlog,
drop-stale" policy this task asked for was already fully and correctly
implemented before this task started.

## 3. MiDaS inference benchmark (standalone, synthetic frame, GPU)

RTX 4060, CUDA-enabled torch confirmed available. Camera frame shape
480×270 (synthetic uint8 input, since no raw frames are cached from prior
captures — MiDaS latency depends on resolution/architecture, not pixel
content). Full data: `inference_benchmark.csv`.

| Config | Resolution | Precision | Steady-state median (s) | P95 (s) | Malformed | Max abs deviation vs baseline |
|---|---:|---|---:|---:|---:|---:|
| current_256_fp32 (baseline) | 256 | fp32 | 0.01433 | 0.01477 | 0 | 0.0 |
| cudnn_benchmark_256_fp32 | 256 | fp32 | **0.00615** | 0.00673 | 0 | **0.0** |
| res224_fp32 (exploratory) | 224 | fp32 | 0.01438 | 0.01526 | 0 | 386.2 |
| res192_fp32 (exploratory) | 192 | fp32 | 0.00738 | 0.01610 | 0 | 518.1 |
| res160_fp32 (exploratory) | 160 | fp32 | 0.01462 | 0.01503 | 0 | 447.1 |
| fp16_256 (exploratory) | 256 | fp16 | — | — | — | FAILED (adapter dtype bug, see below) |

No lighter production backend exists in this repo besides MiDaS-small itself
(`create_depth_adapter_from_environment` only recognizes `midas`/
`midas_small`; the only alternative, `CallableDepthAdapter`, is a test/replay
stub, not a real model) — resolution was therefore the only other lever
tried.

`fp16_256` failed with a dtype mismatch inside the half-precision benchmark
harness (an issue in this task's benchmark patching, not in the shipped
adapter, which was never modified for fp16). Since fp16 was
`latency_exploratory_only` and not selection-eligible regardless (see
below), this was not pursued further.

**Static MAE / raw jitter / raw-range availability / calibration validity**
could only be evaluated for the **current** (256/fp32) configuration, by
reusing the existing frozen static-corpus results — no raw camera frames are
cached anywhere in this repo to re-run the physical-range pipeline at a
different resolution or precision without a new live capture, and this task
does not authorize new official dataset collection. Reported as
`NOT_MEASURABLE_WITHOUT_LIVE_CAPTURE` for every other configuration in
`configuration_comparison.csv` — not fabricated.

The huge `max_abs_deviation_from_baseline` values for the resolution
variants (386–518, in raw inverse-depth units) confirm resolution changes
the raw depth output substantially — exactly why the precommitted rule
(below) excludes them from selection without real accuracy evidence.

## 4. Configuration selection (rule precommitted before any benchmark ran)

Rule (full text in `optimization_plan.json`): only a configuration at the
**same resolution and precision** as production (i.e., numerically
output-preserving by construction) is eligible for selection, since no
cached frames exist to validate accuracy at any other setting. Among
eligible configurations, pick the fastest one with zero malformed output,
steady-state latency ≤ baseline, and max output deviation < 1e-2.

`cudnn_benchmark_256_fp32` passed all three gates (0 malformed, 0.00615s ≤
0.01433s, deviation 0.0) and was selected. **Applied**: one line in
`depth_model_adapter.py` (`MidasSmallAdapter.load()`):
`torch.backends.cudnn.benchmark = True`. No resolution or precision change,
no retrain, output numerically identical on the synthetic test input.

## 5. Live smoke replay (post-optimization, 3 new SITL sessions, observation-only)

New, non-official sessions (separate from the frozen 8/844 corpus),
reusing the existing scenario-capture script with shortened durations:
`smoke_approach` (16s, 21 frames), `smoke_recede` (16s, 33 frames),
`smoke_stop_hold` (11s move + 6s hold, 22 frames). No Follow Target, no
arming beyond what the existing observation-only capture harness already
does (both vehicles remained unarmed throughout, per `capture_result.json`).

| Session | worker_inference_s median | capture→consume median | capture→consume P95 | Raw dynamic MAE (m) | Frozen model dynamic MAE (m) | Best lag shift (s) | Saturated at ±6.0s? |
|---|---:|---:|---:|---:|---:|---:|---|
| smoke_approach | 0.854 | 1.389 | 1.736 | 3.98 | 2.13 | 6.00 | **yes** |
| smoke_recede | 0.619 | 1.084 | 1.528 | 2.50 | 0.86 | 1.35 | no |
| smoke_stop_hold | 1.076 | 1.966 | 2.440 | 3.90 | 0.89 | 1.75 | no |

Full data: `smoke_dynamic_metrics.csv`, `lag_sweep.csv`. `stale_result_count`
and `invalid_result_count` were 0 for all three (collection-mode threshold
3.0s was not exceeded by any frame, consistent with baseline). Small sample
sizes (21–33 frames per session) mean the MAE/lag figures carry real
variance — they are reported as smoke evidence, not a new accuracy
validation.

**The key result: `worker_inference_s` (median 0.62–1.08s across the three
post-optimization sessions) shows no improvement over the pre-optimization
baseline median of 0.61s** — despite the isolated benchmark's 2.33x speedup.
The lag sweep, even widened to ±6.0s (double the prior ±3.0s audit window),
still saturates at the boundary for one of the three sessions
(`smoke_approach`), reinforcing the earlier audit's finding that even wider
windows may not fully bound the true lag for every scenario.

## Why the optimization was insufficient

Isolated MiDaS-small forward-pass latency on an **idle** GPU is 6–14ms —
roughly **1–2% of the ~0.6–1.1s `worker_inference_s` measured in the live
pipeline**, both before and after this change. This is the central finding:
**MiDaS's algorithmic/FLOP cost is not the real-world bottleneck.** The
~600ms+ gap between idle-GPU inference and live measured inference duration
is not explained by the model's compute cost at all, and is most plausibly
explained by resource contention in the live environment — the same
physical GPU renders Gazebo's simulation while MiDaS runs inference, and
the `LatestDepthWorker` background thread shares CPU/Python-GIL time with
ROS2 and the dashboard's own event loop. A resolution- or
precision-preserving inference-path change cannot address a
contention-dominated cost, and this task's methodology (idle-GPU synthetic
benchmark) cannot isolate or fix contention — only the live smoke replay
could reveal this, which is exactly why step 5 existed.

## Disposition

The applied change (`cudnn.benchmark=True`) is safe, real, and kept — it is
strictly output-preserving and measurably faster in isolation, so there is
no reason to revert it, but it should not be reported as having solved the
latency problem. **No retrain and no runtime-mode integration follow from
this task**, per scope. Any further depth-pipeline latency work would need
to target contention/scheduling in the live environment (e.g., profiling
where CPU/GPU time actually goes during a live session with a proper
profiler, not a synthetic-frame idle-GPU benchmark) — out of scope here and
not attempted.

## Full test verification

- Full repository suite (excluding the unrelated stale
  `swarm_dashboard_handoff_20260729/` snapshot bundle): 373 passed, after
  the `depth_model_adapter.py` change.
- `./run_all.sh --check`: passed.
- No PX4/Gazebo/ROS2/dashboard process left running after any of the three
  smoke captures or at task completion (verified via `ps aux` and GPU
  utilization after each session).

## Deliverables

```
artifacts/core_range_3_12m/depth_latency_optimization/
  optimization_plan.json
  baseline_profile.csv / baseline_profile_summary.json
  inference_benchmark.csv
  queue_drop_metrics.csv
  configuration_comparison.csv / selected_configuration.json
  smoke_dynamic_metrics.csv
  lag_sweep.csv
  optimization_manifest.json
  optimization_report.md
  smoke_sessions/{smoke_approach,smoke_recede,smoke_stop_hold}/  (raw capture data)
```
