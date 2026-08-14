# CORE_RANGE_SAFE_STORAGE_RECOVERY

## Outcome

```text
STORAGE_RECOVERY_PARTIAL_NEEDS_USER_APPROVAL
```

Read-only inventory of `/home/sup` (13.54GB free vs. this task's own
20GB minimum / 22GB priority target) found **zero** existing
archive-plus-duplicate-extraction pairs anywhere in the repository —
every reclaimable item requires creating a new archive first. Per this
task's own Section 4 rule, that disqualifies everything above a few
hundred KB from automatic execution. The only unambiguous, zero-risk
actions (regenerable Python/pytest caches, ~349MB) were executed
automatically. The single action capable of closing the gap with wide
margin — compressing and relocating ~23GB of old, pre-core-range PX4
console logs to `/mnt/px4ssd` — is fully planned, verified safe by
sampling, and left for explicit user approval before execution, per this
task's own MEDIUM-risk handling rule.

## Phase 1 — Read-only inventory

Surveyed `artifacts/` top-level and every `artifacts/core_range_3_12m/*`
subdirectory. Two clusters account for nearly all reclaimable space:

1. **`artifacts/run_20260803_*/` (4 directories, ~23GB total)** — PX4 SITL
   stdout console logs (`px4_uav_01.log`/`px4_uav_02.log`) written by
   `run_all.sh`'s standard per-run log archive
   (`${SWARM_LOG_DIR:-artifacts/run_${run_stamp}}`), from a session on
   2026-08-03 that predates the entire core-range task chain (the earlier
   M52/MiDaS `SAFE_FOLLOW` line, per 4 old markdown reports that cite these
   paths only as directory pointers, never as embedded content). Sampled
   the largest file's first 200MB and gzip-compressed it: **187:1 ratio**.
   A tail read confirmed the content is a repeating interactive PX4
   shell-prompt redraw loop (`pxh> [2K...` control-character spam), not
   meaningful flight telemetry.
2. **`artifacts/core_range_3_12m/*/{quarantine,runtime_sessions,runs,long_run,configs,validation,config_*}/`
   (~4.3GB combined)** — raw diagnostic/quarantine data from concluded,
   already-reported RTF/throughput/contention investigation tasks
   (`gazebo_rtf_stutter`, `camera_source_fps`, `live_stack_contention`,
   `full_stack_contention_fix`, `collection_throughput_fix`,
   `priority_collection`, `direct_dynamic_replay`, and several superseded
   `dynamic_*hz_retrain`/`static_balance_collection` quarantine dirs).

`duplicate_archive_pairs.csv` confirms **no** `.tar.gz`/`.zip` archive
exists anywhere under `artifacts/core_range_3_12m/` — this repository has
never archived its own raw runtime data, so this task's Section 3
categories A/B (raw dir with an already-verified archive duplicate) are
empty. Everything found falls under category C/D (needs a *new* archive
created first).

All protected paths (`docs/`, Candidate B models/calibration, the frozen
8/8 dynamic corpus, `residual_bias_correction/`,
`control_ready_observation/`, `targeted_dynamic_pilot/`, the compiled
sim-time-trajectory Gazebo plugin `.so` required by Stage 2A itself, and
every `final_manifest.json`/feature-contract/fold-assignment file) were
enumerated in `protected_paths.json` and excluded from consideration.

## Phase 4 — Execution

**Executed automatically (LOW risk)**: deleted all `__pycache__`
directories (1604, repo + `.venv`) and `.pytest_cache/` — pure, fully
regenerable build caches, ~349MB freed. `/home/sup` free space: 13.54GB →
13.83GB.

**Not executed — held for approval (MEDIUM risk)**: the ~23GB PX4 log
cluster (`candidate_actions.csv` B1-B4) and the optional ~4.3GB diagnostic
cluster (C1/D1). Neither meets this task's own literal auto-execute
criterion (`có bản archive thay thế` — an archive must already exist), and
both involve a large enough amount of data that a deliberate approval
step is warranted even though the safety case (sampled content, zero
core-range relevance, full checksum-verified move contract) is strong.
`recovery_plan.md` documents the exact procedure ready to run the moment
it is approved: stream-compress directly to
`/mnt/px4ssd/swarm_dashboard_archive/` (never holding two large copies at
once), checksum before and after, read-test the archive, and only then
delete the source.

## Phase 7 — Verification (of what was executed)

- All 7 spot-checked protected paths confirmed present after cleanup.
- Candidate B fold-0 model checksum unchanged
  (`cb581208a3e7c3b8c0db837e332bbe1d592c67a5ec2b06cea86e21a35d1c5cac`,
  matches the value recorded in the prior task's
  `control_ready_observation/artifact_checksums.json`).
- Candidate B reproduction: `CANDIDATE_B_REPRODUCED_OK`, max_abs_diff_m =
  0.0.
- Full repository test suite: 459/459 passed.
- `./run_all.sh --check`: PASS.
- No Gazebo/PX4/backend process running.
- `/mnt/px4ssd` unchanged at ~11GB free (no move was executed).

## Deliverables

`artifacts/storage_recovery/`: `storage_inventory.csv`, `largest_files.csv`,
`protected_paths.json`, `duplicate_archive_pairs.csv`,
`candidate_actions.csv`, `executed_actions.csv`, `move_manifest.json`,
`checksum_verification.json`, `post_recovery_disk_status.json`,
`recovery_plan.md`, `final_summary.md`.

## Single next step

Approve actions B1-B4 (and optionally C1/D1) in `candidate_actions.csv`.
Once approved, re-run the compress/verify/delete-source sequence exactly
as documented in `recovery_plan.md`, then re-run this task's Phase 7
verification block and update the conclusion to
`STORAGE_RECOVERY_COMPLETE_READY_FOR_STAGE_2A` before resuming
`CORE_RANGE_TARGETED_DYNAMIC_PILOT_COLLECTION_STAGE_2A`.
