# CORE_RANGE collection throughput investigation

Date: 2026-08-05  
Conclusion: `COLLECTION_THROUGHPUT_EVIDENCE_INSUFFICIENT`

## Metric contract audit

The historical smoke and 5 Hz collection were recomputed with the same
contract: selected tracking session only, calibration prewarm excluded,
monotonic camera-receipt timestamps, and the median of adjacent accepted-row
frame-index delta divided by timestamp delta. Frame-index gaps remain visible
rather than being discarded.

The aggregation difference previously suspected is small, not explanatory:

| Evidence | Median adjacent FPS | Whole-window slope FPS |
|---|---:|---:|
| historical 5 Hz smoke | 25.04 | 26.99 |
| quarantined collection attempt 1 | 15.59 | 16.71 |
| quarantined collection attempt 2 | 16.10 | 17.23 |

The live API agreed with the collection-side result. Both paths used two UAVs,
the same tracker and detector configuration, a 32-second post-prewarm scene and
5 Hz depth scheduler. There is no `COLLECTION_METRIC_DEFINITION_MISMATCH`.

An important contract finding is that the historical “representative smoke”
already set both `SWARM_RANGE_DATASET_DIR` and the physical-diagnostics root.
It serialized dataset rows and complete 96-anchor diagnostic records to disk.
It therefore was not a logging-off reference.

## Current paired baseline

The historical 27–28 FPS state could not be reproduced in the present paired
window. The current A smoke configuration itself measured 16.48 FPS. Full
collection on the identical center approaching scene measured 17.6 FPS; camera
source was 17.5 FPS. Thus the current A/F pair does not show a collection-mode
penalty.

All runs used two UAVs, 32 seconds after stable prewarm, the same trajectory,
seed/config, detector/tracker, 5 Hz scheduler and at least 40 raw rows.

| Configuration | Tracking FPS | Raw rows | Change vs current A |
|---|---:|---:|---:|
| A current representative smoke | 16.48 | 143 | baseline |
| B dataset metadata in memory, no diagnostics | 17.20 | 125 | +4.3% |
| C metadata/GT, no diagnostics | 17.20 | 125 | +4.3% |
| D full diagnostics/checksum/96 anchors, bounded memory | 17.70 | 126 | +7.4% |
| E overlay/preview disabled | 16.71 | 136 | +1.4% |
| F full collection disk | 17.60 | 128 | +6.8% |
| G image/video saving disabled | 17.60 | 128 | +6.8% |
| H diagnostics fsync interval 10→1000 | 16.83 | 138 | +2.1% |

C shares B's run because those configurations are structurally identical in
this code: the dataset collector performs metadata/GT record construction in
memory, while physical diagnostics is disabled. G shares F because no
image/video-saving facility was active. No single factor improved tracking by
25% or reached 20 FPS, so none meets the precommitted dominance rule.

## Monotonic stage profiling

Opt-in instrumentation measured the full disk path on the dashboard tracking
thread:

- tracker update P95: 11.9 ms;
- pose lookup/GT provider P95: 1.57 ms;
- metric fusion including accepted-result anchors P95: 22.95 ms;
- diagnostic record/checksum construction P95: 3.60 ms;
- diagnostic JSON serialization P95: 1.50 ms;
- diagnostic file write P95: 0.23 ms;
- fsync P95: 4.92 ms;
- dataset serialization P95: 0.12 ms;
- dataset file write P95: 0.42 ms;
- overlay P95: 1.73 ms;
- JPEG encode P95: 0.66 ms;
- full tracking-loop P95: 35.17 ms.

No WebSocket client was connected and image/video saving was inactive. Full
stage distributions and execution contexts are in `stage_latency.csv`.
The loop P95 still has approximately 28 FPS compute capacity, while measured
camera/source FPS was only 17–18. Disabling all collection writes did not lift
that source rate. GPU and per-process/thread CPU, disk and context-switch raw
traces are preserved under `resources/`.

## Source changes and safety

No production throughput fix was applied because no collection component met
the dominance rule. The only source additions are profiling/ablation support:

- monotonic serialization/write/fsync timing exposed in collector status;
- opt-in `memory` write modes for dataset and diagnostics;
- bounded diagnostics memory capacity (512 records), explicit overflow count,
  and fail-soft rejection on overflow;
- an observation-only paired capture harness.

Production defaults remain `disk`. Record schema, checksum, GT and 96-anchor
contracts are unchanged. The memory mode is not a production collection mode
and cannot silently drop: a full buffer increments `dropped_record_count` and
rejects the record.

## Validation decision

Approaching/receding/stop-and-hold validation-after-fix was not run because no
production fix was justified or applied. Running three trajectories and
calling them “post-fix” would misrepresent the evidence. This task also did not
run the full eight-group recollection, train a model, change depth rate, or run
Follow Target.

The remaining 27–28→16–18 FPS difference is confounded by wall-clock/runtime
environment state outside the paired collection factors. Under the current
paired state, camera/source delivery—not serialization, diagnostic I/O, fsync,
overlay or encoding—limits tracking, but this task did not isolate a safe,
in-scope source-side root cause. Therefore the only supported conclusion is
`COLLECTION_THROUGHPUT_EVIDENCE_INSUFFICIENT`.

