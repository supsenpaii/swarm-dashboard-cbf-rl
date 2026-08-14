# CORE_RANGE RTF Stutter Root Cause Report

Date: 2026-08-06  
Conclusion: `RTF_STUTTER_REPRODUCED_CAUSE_UNRESOLVED`

## Scope and safety

This investigation ran observation/runtime tracing only. It did not collect a
Stage 2A group, train or replace a model, alter Candidate B, change M52/MiDaS
semantics or production rates, lower the RTF gate, arm/take off/enter OFFBOARD,
run Follow Target, shadow a controller, or publish a controller effect.
`SWARM_START_GAZEBO_GUI=false` was used throughout.

## Failure modes reproduced

- `COLD_START_LOW_RTF`: reproduced by the prior clean-restart Stage 2A
  precheck (median RTF 0.244) and again in `L4_run1`, which had low streaks at
  1–4 s, 8–11 s, and 31–33 s.
- `WARM_PERIODIC_STUTTER`: reproduced by the prior 410 s full-stack soak
  (median 0.9987; seven 3–5 s streaks, first near 100 s; starts near 100, 120,
  198, 257, 261, 359, and 389 s). New runs `X_OFF_run1_retry1` and
  `PX4_LOG_OFF_run1_retry1` reproduced warm failing streaks, including a 12 s
  streak after 200 s.

The older full-stack soak contains intervals in the requested 40–70 s band.
The new accepted traces did not reproduce that period consistently: their
successive failing-event intervals were 4–189 s and none was in 40–70 s.
This variability is part of why no causal claim is accepted.

## Staged isolation

All complete runs were clean restarts of the same headless world. The refined
ladder kept launcher/model poses/logging policy constant and added one component
group at a time:

| Level | Added component | Result |
|---|---|---|
| L0 | Gazebo + PX4 x2 | PASS; median 0.99996, 2 isolated low seconds |
| L1 | MicroXRCE | PASS; median 0.99998, 0 low seconds |
| L2 | ROS2 two-UAV telemetry | PASS; median 0.99997, 0 low seconds |
| L3 run 1/2 | MAVLink bridge | PASS/PASS; median 0.99984/0.99994, 0 low seconds |
| L4 run 1/2 | idle web backend with eager Gazebo camera subscriptions | FAIL/PASS; 32/18 low seconds, longest streak 4/1 |

The first configuration to show a failing streak was L4. However, L4 failed
only 1/2 repeats, so the L3→L4 boundary does not meet the required two-pair
root-cause rule. The strongest component-level association is therefore the
idle backend's eager image/IMU/lidar subscription set: it increased low seconds
from 0/0 in L3 to 32/18 in L4, but this is a suspect, not an accepted cause.
Event windows did not show a repeatable dominant CPU, memory, disk, thermal,
GPU, fault, or context-switch mechanism. PX4 CPU was only about six percentage
points higher in one run's low windows, which is weak and may be an effect of
the scheduling disturbance.

## Hypotheses rejected

- Periodic rotation `fsync`: rejected. `X_OFF_run1_retry1` disabled every
  rotation fsync (timestamped trace recorded only shutdown fsyncs) yet had 52
  low seconds and six failing streaks, longest five seconds.
- PX4 bounded stdout rotation: rejected as a sufficient cause.
  `PX4_LOG_OFF_run1_retry1` discarded only the two PX4 log streams but still
  had 48 low seconds and five failing streaks, longest 12 seconds.
- MicroXRCE, ROS telemetry, and MAVLink bridge: each passed its adjacent staged
  addition; L3 also passed its required repeat.
- MiDaS, M52, tracker, and diagnostics are not necessary for the L4 cold-start
  failure because none was active in idle L4. This does not prove they can
  never contribute in an active capture.

## Fix and validation decision

No production fix was applied. All source changes are additive, opt-in
instrumentation or staged launcher gates; historical defaults remain unchanged.
Changing camera subscription policy based on a 1/2 boundary would violate the
acceptance rule. Consequently, three “post-fix” soaks and six “post-fix” smokes
were not run or labeled PASS; `post_fix_gate_result.json` records
`NOT_RUN_NO_JUSTIFIED_FIX`. Stage 2A remains blocked.

Candidate B was reproduced in the new task directory without overwriting the
old control-ready artifact: `max_abs_diff_m=0.0`.

## Verification

- Focused: 24/24 passed.
- Full root repository contract (`test_*.py`): 484 passed, 1 skipped, one
  pre-existing `PytestReturnNotNoneWarning`.
- A default recursive pytest discovery attempt is recorded as invalid because
  the archived `swarm_dashboard_handoff_20260729/` snapshot contains duplicate
  test/module names; no snapshot/cache/data was deleted to hide it.
- `./run_all.sh --check`: PASS.
- Final cleanup: no Gazebo, PX4, MicroXRCE, ROS launch, backend, bridge,
  collector, profiler, or bounded-writer process; no listeners on ports 8000,
  8001, or 8888. The pre-existing Mosquitto listener on 1883 remains.

## Artifacts

Primary evidence is under
`artifacts/core_range_3_12m/rtf_stutter_root_cause/`, especially
`run_manifest.csv`, `rtf_events.csv`, `event_aligned_metrics.csv`,
`periodicity_analysis.csv`, `hypothesis_matrix.csv`,
`boundary_reproduction.json`, `root_cause_decision.json`, and
`candidate_b_reproduction/candidate_b_reproduction.json`.
