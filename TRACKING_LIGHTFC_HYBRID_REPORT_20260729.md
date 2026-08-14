# LightFC tracking integration report

Date: 2026-07-29

## Integrated behavior

The default `SWARM_TRACKING_BACKEND=hybrid` path now combines KCF's fast
per-frame correlation with robustness mechanisms from:

`/home/sup/ws_px4/src/lfc_gimbal_gazebo`

Specifically:

- constant-velocity Kalman prediction;
- temporal appearance, color, and texture memory;
- motion and bbox shape/scale scoring;
- hard gates for implausible center jumps and scale changes;
- peak-to-sidelobe-ratio (PSR) validation during re-detection;
- separated-peak ambiguity/distractor detection;
- motion-direction scoring using recent target velocity;
- local re-detection around the Kalman prediction, expanding to the complete
  frame only after a configurable lost-frame threshold;
- multi-frame reacquisition confirmation and blacklist rejection.

Tracker diagnostics expose PSR, shape/motion scores, candidate count,
ambiguity, rejection reason, re-detection mode, and Kalman velocity.

## UniDepth/control safety

A tracker result is not submitted as valid to the UniDepth worker when:

- it is still re-detecting;
- the tracker reports an ambiguous target;
- the tracker reports a quality rejection;
- the existing state, score, bbox stability, drift, or gimbal gates fail.

The depth worker may still receive the latest frame as an invalid job so its
state is reset safely; invalid tracking cannot retain forward/backward motion
authority.

## Neural LightFC status

The neural LightFC ONNX core was not enabled automatically:

- the package's `onnx/lightfc_core.onnx` is a broken symlink to another user's
  workspace;
- `onnxruntime` is not installed in the dashboard virtual environment.

The default hybrid therefore reports:

- `lightfc_robust_gates=true`;
- `neural_core=false`.

This is deliberate. Switching silently to a missing or unverified model would
make tracking unavailable and reduce, rather than improve, runtime safety.
When a valid model and `onnxruntime` are installed, the existing
`SWARM_TRACKING_BACKEND=onnx` path can be benchmarked separately before it is
considered for the default configuration.

## Configuration

The new controls are documented in `.env.example`:

- `SWARM_REID_PSR_THRESHOLD`
- `SWARM_REID_AMBIGUITY_RATIO`
- `SWARM_REID_DIRECTION_WEIGHT`
- `SWARM_REID_LOCAL_SEARCH_FACTOR`
- `SWARM_REID_FULL_FRAME_AFTER`
- `SWARM_TRACKING_MAX_SCALE_CHANGE`
- `SWARM_TRACKING_MAX_CENTER_JUMP_RATIO`

All defaults are active through the existing `hybrid` backend.

## Verification

- Python compile check: passed.
- Full unit test suite: 127 tests passed.
- Added tests cover PSR, shape penalty, scale/jump rejection, local-to-full
  search expansion, motion direction, diagnostics, and a synthetic KCF frame.
- Runtime safety check: UAV-01 and UAV-02 disarmed, no failsafe, tracking
  inactive.
- One Uvicorn backend and one `mavlink_manual_bridge.py` process.

No arm, takeoff, landing, flight command, or live tracking session was
started during this integration.

## Runtime validation after PX4 respawn

Both `x500_custom_0` and `x500_custom_1` are spawned and the UAV-02 camera
topic has an active publisher. The source frame is confirmed as RGB
1280x720. Measured Gazebo camera rate is approximately 17-20 FPS.

The camera pipeline was also corrected so `raw_frames` retains 1280x720 for
tracking and UniDepth. Browser JPEG previews alone are reduced to 640x360.
Before this correction, the common conversion function resized the image
before publishing it to the tracker, which discarded the intended accuracy
gain.

Remaining live validation, still with both UAVs disarmed:

- tracking FPS and p95 update time;
- false-reacquisition rate with distractors;
- bbox center/scale error under occlusion;
- invalid-depth rate caused by tracking quality gates;
- comparison of `hybrid` against `onnx` only after a valid LightFC model is
  provided.
