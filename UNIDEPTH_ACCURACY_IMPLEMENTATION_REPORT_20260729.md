# UniDepth V2 accuracy implementation report

Date: 2026-07-29

## Outcome

The deployable accuracy path has been implemented without arming or commanding
either UAV:

- Gazebo camera profile changed from 640x360, HFOV 2.0 rad to 1280x720,
  HFOV 1.35 rad, 30 FPS.
- The corresponding pinhole intrinsics are now:
  `fx=fy=799.57528277`, `cx=640`, `cy=360`.
- Horizontal focal length in pixels increased by approximately 3.89x, so a
  target at the same range occupies substantially more useful pixels.
- UniDepth V2 inference uses configurable `resolution_level`, with the target
  runtime value set to `9.0`.
- The 8-12 m evaluator grouping was corrected; no remaining 8-10 m group is
  used by the evaluation code changed in this work.
- Calibration remains disabled. The existing candidate is not production
  recommended and is not enabled implicitly.

## Files changed

- `configure_gazebo_camera.py`: validates and atomically applies a versioned
  camera profile to the Gazebo SDF.
- `camera_profiles/unidepth_accuracy_1280x720.json`: versioned camera profile.
- `metric_depth_estimator.py`: configurable UniDepth V2 resolution level.
- `metric_depth_evaluation.py`: correct 8-12 m safe-band evaluation groups.
- `tracking_web.py`: exposes the active inference resolution level.
- `static/index.html`: shows the resolution level in camera diagnostics.
- `.env.example`: target camera and UniDepth settings.
- `README.md`: deployment, validation, and rollback/check instructions.
- `test_camera_profile.py`, `test_metric_depth_estimator.py`: camera profile,
  geometry, evaluator, and inference configuration tests.

The external PX4 Gazebo model file was updated by the profile tool:

`/mnt/px4ssd/PX4-Autopilot/Tools/simulation/gz/models/gimbal/model.sdf`

## Verification

- `python -m unittest discover -p 'test_*.py'`: 127 tests passed.
- Python compile check for the affected runtime modules: passed.
- Profile `--check`: `matches=true`.
- Backend API after restart: 1280x720, HFOV 1.35 rad, valid intrinsics,
  UniDepth preloaded, resolution level 9.0, safe band 8-12 m.
- UAV-01 and UAV-02: disarmed, no failsafe.
- Tracking: inactive.

## Runtime validation after PX4 respawn

Both PX4 SITL models were respawned successfully. The UAV-02 camera topic has
an active publisher and its message metadata confirms RGB 1280x720 with a
3840-byte row stride. The measured Gazebo source rate is approximately
17-20 FPS on the current host.

Runtime inspection then found and corrected a pipeline issue: the shared
camera conversion function resized every frame to 640x360 before storing
`raw_frames`, so tracking and UniDepth did not receive the physical camera's
full resolution. `raw_frames` now retains 1280x720 and only the browser JPEG
preview is resized to 640x360. Unit tests enforce both properties.

## Accuracy status

Previous held-out evaluation remained insufficient for flight use:

- no-calibration MAE: 8.09 m;
- scale-only held-out MAE: 2.365 m;
- observed response was not reliably monotonic.

The higher-resolution/narrower-FOV change addresses the dominant target-pixel
problem, but accuracy has not been proven until a new synchronized dataset is
collected at 5, 8, 9, 10, 12, and 15 m, including off-axis targets and varied
backgrounds. Do not enable the candidate calibration or perform a real flight
test solely because the unit tests pass.

## Re-run

Check or apply the physical camera profile:

```bash
python configure_gazebo_camera.py \
  --profile camera_profiles/unidepth_accuracy_1280x720.json \
  --sdf /mnt/px4ssd/PX4-Autopilot/Tools/simulation/gz/models/gimbal/model.sdf \
  --check
```

```bash
python configure_gazebo_camera.py \
  --profile camera_profiles/unidepth_accuracy_1280x720.json \
  --sdf /mnt/px4ssd/PX4-Autopilot/Tools/simulation/gz/models/gimbal/model.sdf \
  --apply
```

Start Gazebo first, then restart both PX4 SITL instances so that both UAV
models and their sensors are spawned. Start the backend with the values in
`.env.example`. Confirm both UAVs remain disarmed before selecting a bbox.

Run verification:

```bash
python -m compileall -q \
  main.py tracking_web.py tracking_hybrid.py metric_depth_estimator.py \
  metric_depth_accuracy.py metric_depth_evaluation.py visual_follow_target.py \
  body_attitude_recenter.py mavlink_manual_bridge.py \
  configure_gazebo_camera.py

python -m unittest discover -p 'test_*.py'
```

Then collect a new ground-truth dataset and compare no-calibration,
scale-only, affine, and monotonic piecewise profiles on a separate validation
split. Enable a profile only if validation error and monotonicity meet the
flight acceptance threshold.

## Fine-tuning note

The official UniDepth repository currently documents public training support
for V1, not V2. This implementation therefore does not ship an unverified
"UniDepth V2 fine-tuning" trainer. If V2 training code or a supported training
recipe becomes available, fine-tuning on synchronized Gazebo/real-camera
samples is the next model-level step after the optical pipeline and dataset
have been validated.
