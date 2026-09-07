# CBF-RL completion roadmap

The validated architecture is fixed: an RL policy may propose only a nominal
ENU velocity. The existing CBF gate and emergency supervisor remain downstream
and are the only path to an applied command. RL must never arm, change mode,
alter CBF constants, or bypass missing/stale-state holds.

## Frozen Phase 0 contract

- Scope: two UAVs, deterministic offline simulation, no runtime authority.
- Action: one normalized three-vector per UAV in `[-1, 1]`; scaled to the CBF
  maximum velocity before filtering.
- Observation: the 20 finite fields in `cbf_rl_env.OBSERVATION_FIELDS`, in that
  exact order and SI units. Missing covariance is encoded as zero plus an
  explicit validity bit; it is still passed to the CBF as missing and holds.
- Dynamics: simultaneous point-mass integration at 20 Hz using only the
  CBF-returned safe velocities.
- Reward per UAV: `5*goal_progress - 0.01 - 0.1*cbf_intervention
  - 2*cbf_hold - 5*negative_margin + 10*first_goal_arrival`.
- Safety configuration: separation 4 m, maximum velocity 2 m/s, barrier gain
  2/s, command latency 0.65 s, covariance sigma 0.10, covariance required.
- Runtime remains unchanged and RL remains disabled.

## Gate status

1. `CBF_RL_OFFLINE_TRAINING_BASELINE`: **PASS**. Seeded dependency-free CEM
   trains the three-parameter proximity policy in `cbf_rl_policy_v1.json`.
   All three training scenarios complete through the production-equivalent CBF
   path; the artifact carries its training report and parameter checksum.
2. `CBF_RL_OFFLINE_EVALUATION`: **PASS**. Four unseen geometries complete with
   at least 5.26 m separation while the goal-only baseline completes two.
   Missing covariance, stale state and invalid source all hold at zero; a
   one-step covariance gap recovers on the next valid sample.
3. `CBF_RL_RUNTIME_SHADOW`: **PASS**. `SWARM_CBF_RL_MODE=shadow` requires a
   whole-artifact SHA-256 and the model's internal parameter checksum. A
   disarmed two-UAV live canary produced 1,276 valid samples per UAV (the last
   200/200 consecutive samples valid), with zero transmit/arm/mode attempts.
   The trace publishes nominal and CBF-shielded deltas; `applied` and
   `transmit_authority` are hard false. Runtime default remains `off`.
4. `CBF_RL_SITL_NON_CONFLICT`: **PASS**. The previously validated parallel
   trajectory completed and landed/disarmed with 797/797 covariance samples,
   no negative/infeasible/watchdog/supervisor event. The authenticated shadow
   produced 439 valid trajectory samples on UAV-01 and 419 on UAV-02, never
   applied or authorized its output; both actual and shadow minimum CBF margin
   stayed at least 0.509 m.
5. `CBF_RL_SITL_CROSSING_AND_FAULTS`: **PASS**. The first shadow-only
   retry reused the old sigma-zero geometry and correctly aborted at a -0.07 m
   reported margin; RL was never applied and both vehicles landed/disarmed.
   Its point-mass replay gap is now covered by an optional, default-neutral
   command-delay/velocity-lag stress. The replacement spawn-aligned legs
   genuinely cross at (0,8,9); production sigma/100 ms peer age and a
   300 ms + 0.75 s plant stress both complete with at least 0.30 m reported
   margin, 0.60 m physical slack, real CBF intervention, and no infeasibility.
   The replacement flight then completed both intersecting legs and cleanly
   landed/disarmed: minimum actual margins were 0.366/0.369 m, CBF intervened
   on 11.8%/5.7% of samples, and all 2,234/2,233 authenticated shadow samples
   were valid, bounded, CBF-clean, and never applied/authorized. The existing
   active server-loss flight remains `FLIGHT_PASS` (30 s outage, recovery,
   landing/disarm), while the runtime shadow is non-authoritative and its
   missing/stale/invalid-state fault suite fails closed.
6. `CBF_RL_PRODUCTION_ENABLEMENT`: **PASS**. Active mode is explicit,
   authenticated, CBF-contract pinned, downstream of the existing CBF and
   emergency supervisor, and falls back to the deterministic command on any
   load/evaluation/contract error. The first flight exposed an authority-label
   integration bug: the sender correctly withheld every unrecognized frame and
   PX4 entered failsafe; both vehicles landed/disarmed. Keeping the existing
   companion-safety authority label while reporting RL as the internal nominal
   source fixed the boundary, with an end-to-end regression. The retry flew a
   25 s two-UAV active formation hold and landed/disarmed cleanly. Every one of
   538/518 station-keeping samples was authenticated, bounded, CBF-clean,
   selected, and transmitted through companion safety; minimum margins were
   1.318/1.327 m, with no infeasible, supervisor, watchdog, or sender-latch
   event. Default remains `off`; rollback is `SWARM_CBF_RL_MODE=shadow`
   (observe) or `off`, followed by a stack restart.
7. `CBF_RL_ACTIVE_TRAJECTORY`: **PASS**. A 20 s and 35 s parallel-leg
   flight kept the active policy authenticated, applied, transmitted and
   CBF-clean, but neither UAV reached its endpoint. Extending the same run to
   50 s exposed a real latency-inclusive margin excursion (-0.070/-0.046 m);
   the flight guard aborted, both vehicles landed/disarmed, and the resting
   configuration was restored. `cbf_rl_sitl_evaluate.py` now refuses to call
   a trajectory artifact PASS unless both UAV summaries report
   `reached_trajectory_end=true`. The new v2 policy keeps v1's learned
   avoidance gain/radius fixed and learns only goal gain plus a goal-distance
   taper, so the passing bias vanishes at the endpoint instead of creating a
   limit cycle. Its seeded training completes all four scenarios; the old
   unseen/fault gate remains PASS, and the new 20 m active-trajectory gate is
   PASS in 700 steps with 1.019 m minimum margin, 5.343 m minimum physical
   distance, zero hold and zero CBF intervention. Model SHA-256 is
   `962a1989c2cf413a1555786f0f805a23c634187aa6bb7000242b119f88a9d285`.
   The authenticated v2 shadow trajectory then completed both legs and
   landed/disarmed cleanly: 419/419 valid samples per UAV, every candidate
   bounded, CBF-clean and never applied/authorized, with minimum shadow/actual
   margins 1.168/1.073 m. The separately authorized 50 s active v2 attempt
   then landed/disarmed cleanly and kept every one of 1017/1036 trajectory
   samples authenticated, applied, transmitted and CBF-clean; minimum actual
   margins were 0.698/0.700 m. UAV-02 reached its endpoint, but UAV-01 stopped
   at 0.415 m error and never entered the configured 0.25 m arrival radius, so
   the evaluator correctly returned FAIL. This exposes an optimistic offline
   assumption: the 700-step gate integrates requested velocity directly and
   does not cover the slower SITL velocity response. Runtime/default remains
   `off`; the next gate is an offline v3 robustness update using the measured
   response before any new shadow or active flight.

   Closed 2026-08-19 on the Sparrow airframe rather than the v3 rebuild the
   paragraph above anticipated: the arrival failure was never policy
   robustness, it was two runtime defects. The barrier was missing its
   `-2*R*R_dot` term, and the coordinator engaged on a fixed trigger DISTANCE,
   which buys less and less lead TIME as the rung speed rises (2.69 s at
   10 m/s, 0.62 s at 20 m/s). With both fixed, `cbf_rl_sitl_evaluate.py`
   returns PASS on the 20 m/s corridor swap with every check true on both
   vehicles, `trajectory_completed=true`, and minimum actual CBF margins of
   14.341 / 14.331 m -- against -0.596 m on the same case before the fixes.
   The 15 m/s rung passes the same way at 10.045 m. Evidence:
   `artifacts/gate7_sparrow_20ms.json`, `artifacts/leadfix_corridor_20ms_flight.json`.

   Two operational notes this gate depends on. The evaluator has no run
   splitting and reads a whole appended trace, so it must be pointed at one
   isolated run or it will report an older flight's breach; the trace above is
   the extracted final trajectory block. And the 20 m/s rung must be flown with
   `SWARM_START_GAZEBO_GUI=false` -- own-state telemetry age peaks at 99.8 ms
   headless against a 100 ms staleness latch, and with the GUI it reaches
   100.1 ms and aborts the flight on a single dropped MAVLink message.

Vision, MiDaS and range-estimation work are outside this roadmap.
