# CORE RANGE DIRECT DYNAMIC REPLAY REPORT

Conclusion: `DYNAMIC_REPLAY_BLOCKED_BY_DATA_INTEGRITY`

The frozen direct `C_shallow` development candidate was not replayed. Dynamic
collection stopped fail-closed with 4/8 accepted independent sessions because
the near-start lateral-left receding scenario repeatedly produced zero accepted
raw-range rows (`inverse_depth_uncertainty_too_large`), including after a
precommitted center-preserving 80% collection-ROI correction.

Accepted coverage is 3/3 approaching, 1/3 receding and 0/2 stop-and-hold.
Dynamic accuracy, lag, stop response and bbox robustness therefore remain
`N/A`; partial evaluation was deliberately not used to select or modify the
candidate.

All technical failures remain quarantined. Candidate models, feature contract,
preprocessing, clipping `[3,12]`, causal smoothing `tau=0.20 s`, seed `52` and
development gates remain frozen. No model was retrained or loaded for
prediction. No runtime/controller/PX4 integration, shadow, Follow, OFFBOARD,
arm, takeoff or closed-loop action was performed. Residual correction remains
default-off.

Machine-readable evidence and checksums are in
`artifacts/core_range_3_12m/direct_dynamic_replay/dynamic_replay_manifest.json`.

## Verification

```text
Focused dynamic replay tests: 16 passed
Full repository suite: 364 passed, 1 known PytestReturnNotNoneWarning
./run_all.sh --check: PASS
Prediction rows: 0
Accepted source checksum revalidation: PASS for 4/4 accepted sessions
```

Key SHA-256 values at gate close:

```text
dynamic_replay_plan.json
  40dc2b1e7c691e99972f99799c127e4d57823a0df32805087b1996aebc416744
candidate_model_manifest.json
  25bbca13b76eb7ac8bb977f8142ac836a87600e72d1a273a56ee918f5d4b382d
dynamic_session_manifest.json
  2b80e76ea44bdd507e88a1777a9f51d4d047d7efb201fb5f5cf2c65a803ceafb
dynamic_replay_manifest.json
  def70df00ffeba99506af92690433f595f690c15e74a0362e85ae6c71ec1b4cf
dynamic_replay_report.md
  d71882413380d8defbebce1080b7ace29ce16cef0f15fc8ee83f1c75213935f9
```
