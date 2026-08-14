# M52/MiDaS Range Applicability v2 Implementation — 2026-08-03

## Outcome

The range path now uses a deterministic applicability gate before the range
filter/EKF, and the optional XGBoost path uses a two-stage
applicability-classifier plus bounded residual-regressor contract. GPS/RTK or
simulation truth remains label-only and is never a runtime model feature.

The deterministic gate defaults to `active`. Residual correction remains
`off` until a new v2 bundle passes independent grouped holdouts and shadow
validation. Archived v1 bundles are intentionally incompatible with the v2
loader.

## Implemented containment

- `range_applicability_gate.py` checks image-ray geometry, camera/target
  bearing geometry, calibration health, anchor coverage/extrapolation and
  relative range uncertainty before the measurement reaches the range filter.
- `range_residual_correction.py` locks the 23-feature
  `m52_midas_range_features_v2` contract, shrunk-covariance Mahalanobis OOD
  detection, applicability probability threshold and bounded correction.
- `metric_target_fusion.py` records the uncorrected physics history, collects
  training observations before an enforced abstention, and prevents rejected
  measurements from entering the range filter.
- `range_residual_dataset.py` locks camera-center-to-target-center slant-range
  labels and requires source, uncertainty, timestamp offset, quality and
  lever-arm metadata.
- `range_residual_training.py` trains the applicability classifier first and
  the residual regressor only on correctable samples. Evaluation applies the
  same z-score, multivariate OOD, classifier and bounded-output contract used
  at runtime.

## Safe SITL evidence

Both verification runs kept both UAVs disarmed. Follow, body motion, gimbal
tracking and residual correction were disabled. The simulated gimbal was held
at -10 degrees only to provide ground anchors. No arm, takeoff, mode, follow or
motion endpoint was invoked.

### Oblique negative case

- Dataset: `artifacts/range_v2_gate_oblique_final_20260803_141340`
- Final samples: 14 labeled v2 observations, each with all 23 features.
- Runtime snapshot: 13 gate requests, 13 enforced rejections, zero applicable;
  last reason `image_ray_x_outside_validated_envelope`.
- Range-filter sample count stayed zero; metric measurement remained rejected.
- Ground-truth slant range: 4.4543 m. Raw physics range in recorded samples:
  6.3222–8.4408 m.
- Manifest SHA-256:
  `625afa9220d49d148447043c4467265954aafca8622cad458faa0a1b88a3220e`
- Samples SHA-256:
  `07d08ca2fb24376ea68c61776427f16d7488a273d6a6b7fffd1e8ddfc1a6065d`

### Front positive case

- Dataset: `artifacts/range_v2_gate_front_final_20260803_143000`
- Final samples: 59 labeled v2 observations, each with all 23 features.
- Runtime snapshot: 45 gate requests, 45 applicable, zero rejected; last reason
  `applicable`.
- The range filter became valid and accepted measurements. The metric EKF was
  not exercised because follow/control remained intentionally inactive.
- Ground-truth slant range: 3.9801 m. Raw physics range in recorded samples:
  5.0491–6.1028 m. This confirms the gate is an applicability boundary, not a
  substitute for a learned bias correction.
- Manifest SHA-256:
  `807071ae4b7c4290cd3d23ef28c21f9e9e04bde4607b07a7c4428b3c33965b49`
- Samples SHA-256:
  `d91cec32580c3e7a947546b3e96ba3e2ced05d32354340f404422d3b17082148`

Temporary SITL logs for these two runs were permanently deleted after
verification (8.0 GiB total). The compact versioned datasets above were kept.

## Verification

- `220 passed` with `python3 -m pytest -q test_*.py`.
- `run_all.sh --check` passed.
- Updated Python modules passed `py_compile`.
- Loading `range_residual_xgboost_candidate_v3_20260803` fails with
  `bundle_bundle_version_mismatch`, as required.
- No Swarm/PX4/Gazebo/backend runtime processes remained after cleanup.

The single pytest warning is pre-existing: `test_bearing_target_estimator.py`
returns a config object from a test function.

## Promotion state and next action

Do not train or activate a v2 production candidate from these two static
groups. They prove the new contract and positive/negative data path, but do not
provide enough independent variation for leakage-resistant grouped
train/validation/test partitions.

Collect at least one additional independent v2 group (three is the bare split
minimum; more is preferred), covering different ranges, angles, bbox sizes,
backgrounds and motion. For real hardware, use time-synchronized,
lever-arm-corrected RTK-fixed camera-center-to-target-center labels with stated
uncertainty. Then train a fresh v2 bundle, evaluate independent front and
oblique holdouts, run it in `shadow`, and only consider `active` after the
coverage and safety gates pass. Do not reuse or relabel the archived v1
datasets/bundles.

## Continuation: classifier promotion audit

Seven initial v2 groups produced a candidate whose hash-only validation/test
partitions contained only one applicability class each. Training was therefore
changed to require class-representative, group-disjoint partitions and to emit
precision/recall/specificity/false-accept metrics plus an explicit promotion
gate.

Candidate `range_residual_xgboost_candidate_v5_v2_20260803` is retained as a
rejected diagnostic. It failed promotion: validation precision was 0.1667,
test precision was 0, both uncorrectable false-accept rates were 1.0, and held
out residual prediction standard deviation was 4.9557 m. Residual correction
remains `off`.

Training now also reapplies the deterministic runtime envelope before fitting
the classifier. Across the expanded eleven-group dataset, 312 of 472 samples were
inside the envelope; 160 oblique samples were correctly excluded because the
runtime gate would reject them first. The remaining in-envelope split was:

- train: 189 samples (143 correctable, 46 uncorrectable);
- validation: 82 samples (44 correctable, 38 uncorrectable);
- test: 41 samples (29 correctable, 12 uncorrectable).

At the locked classifier threshold 0.8—and also at 0.5—the model rejected the
entire validation partition, so no v6 bundle was created. Direct probability
diagnostics showed that 9 m correctable and 10 m uncorrectable validation
samples both received approximately 0.16; threshold tuning cannot separate
them. This is group/range generalization failure, not a threshold issue.

Additional independent front-facing datasets retained by this continuation:
5 m (44 correctable), 9 m (44 correctable), 10 m (38 uncorrectable), and an
independent 8 m repeat (29 correctable, 12 uncorrectable). Further collection
must repeat conditions across independent sessions so each distance/background
appears on both sides of grouped holdouts. Do not change the seed to select a
more favorable split. Hardware RTK-fixed labels remain the preferred next
source when synchronized camera-center-to-target-center measurements are
available.

## Continuation: repeated 9 m / 10 m session audit

Two additional front-facing SITL sessions were collected with both vehicles
disarmed, the camera fixed at -10 degrees, and follow, motion, automatic gimbal
tracking and residual correction disabled. No arm, takeoff, mode, follow or
motion endpoint was invoked.

- `artifacts/range_v2_front9_repeat_20260803`: 105 labeled samples, all inside
  the deterministic envelope; 25 correctable and 80 uncorrectable at the
  locked 3.0 m residual limit. Ground truth was 8.9751 m and residuals ranged
  from 2.0219 m to 5.1490 m. Manifest SHA-256:
  `47498a053dff083106df0492a8c8eb56b19d0f438ce66a57ba3b66cdae9dc6db`;
  samples SHA-256:
  `0eaf95b0a3d398157e7c33216bc506a6089b14c89a52db8257aab9203a29ff55`.
- `artifacts/range_v2_front10_repeat_20260803`: 121 labeled samples, all inside
  the deterministic envelope; 22 correctable and 99 uncorrectable. Ground
  truth was 9.9748 m and residuals ranged from 2.7664 m to 3.8730 m. Manifest
  SHA-256:
  `c60e4c4ae09b6c826124bb36efeb4d28ef7bde28ec73606665a4e4bffc46c1a6`;
  samples SHA-256:
  `29d0a2542ddb73e72cf93e7e845704c89a402d12cac1c678e93668822d155af2`.

Across all thirteen retained groups, 538 of 698 samples were inside the locked
runtime envelope. The seed-52 group-disjoint split contained:

- train: 392 samples (209 correctable, 183 uncorrectable);
- validation: the new 9 m session, 105 samples (25/80);
- test: the independent 8 m repeat, 41 samples (29/12).

Candidate v7 was trained with the unchanged 0.8 classifier threshold and
unchanged seed. Validation improved to precision 0.625 and recall 1.0, but its
uncorrectable false-accept rate was 0.1875, above the 0.05 promotion limit. The
classifier rejected every test sample, giving test recall 0. No v7 artifact
directory was created because training failed closed with
`all_partition_predictions_rejected`. Residual correction remains `off`.

The result is evidence that the failure is session generalization, not a lack
of same-range frames or a threshold-selection problem. The next useful data
source is another independent 8 m session that can expose both classes outside
the current held-out group, followed by grouped cross-session evaluation. Real
RTK-fixed, time-synchronized, lever-arm-corrected measurements remain preferred
over accumulating nearly identical SITL frames.

Current verification: `222 passed` with the archived handoff directory ignored,
one pre-existing `PytestReturnNotNoneWarning`; `run_all.sh --check` and
`py_compile` passed. No PX4, Gazebo, XRCE, backend, MAVLink bridge or ROS launch
process remained after collection. The 5.5 GiB of temporary logs from the two
accepted sessions was permanently deleted; the compact datasets were retained.

## Continuation: second independent 8 m session and v8 audit

A further front-facing 8 m SITL session was collected with both vehicles
disarmed, residual correction off, automatic gimbal tracking off, and no arm,
takeoff, mode, follow or motion command. The manual gimbal pose remained -10
degrees. Tracking/calibration was reset when depth anchoring stalled, without
changing the experiment pose or training parameters.

- Dataset: `artifacts/range_v2_front8_repeat2_20260803`.
- 78 labeled samples; all 78 passed the deterministic runtime envelope.
- Ground-truth camera-center-to-target-center slant range: median 7.9756 m.
- All 78 samples were uncorrectable at the locked 3.0 m residual limit.
- Absolute residual range: 3.6609–4.4535 m; median 3.8229 m, mean 3.9275 m,
  population standard deviation 0.2276 m.
- Manifest SHA-256:
  `d2afd4b152dcc1ad72ade88b22e5f149a83c2511b78fc7efd9b12fe2c2f68d0c`.
- Samples SHA-256:
  `266fa7a9d80823fc6212a38377cd6884cc35ce86507329ee0b47cdcb3866f316`.

Across all 14 dataset directories, 616 of 776 labeled samples passed the
deterministic envelope; the other 160 were rejected for
`image_ray_x_outside_validated_envelope`. The unchanged seed-52 grouped split
contained:

- train: 388 samples (165 correctable, 223 uncorrectable);
- validation: 149 samples (69 correctable, 80 uncorrectable);
- test: 79 samples (29 correctable, 50 uncorrectable).

Candidate v8 used the unchanged classifier threshold 0.8, residual limit 3.0 m
and seed 52. It fit the train partition perfectly at that threshold, but only
accepted 1 of 69 validation-correctable samples and accepted no test sample.
Validation recall was 0.0145; test recall was 0. Test correctable probabilities
were only 0.0113–0.0267, while uncorrectable probabilities ranged from 0.0103
to 0.6328. The trainer therefore failed closed with
`all_partition_predictions_rejected`; no v8 artifact directory exists and
residual correction remains `off`.

This repeat adds an independent uncorrectable 8 m group but does not resolve
cross-session generalization. Further identical static 8 m frames have low
expected value. The preferred next dataset is real, time-synchronized,
lever-arm-corrected RTK-fixed ground truth with controlled variation across
sessions, backgrounds, angles and ranges. If hardware is unavailable, create
equivalent independent SITL variation rather than tuning the locked seed or
threshold.

Post-audit verification remained green: `222 passed` with one pre-existing
`PytestReturnNotNoneWarning`; `run_all.sh --check` and `py_compile` passed.
No PX4, Gazebo, XRCE, backend, MAVLink bridge or ROS launch process remained.
The exact 5.26 GB temporary runtime-log directory for this session was
permanently deleted after dataset verification; the compact dataset was kept.

## Continuation: online RTK-fixed ground-truth collector

The hardware-label path is now implemented without changing the model feature
schema or control path. `range_ground_truth.py` computes camera-center to
target-center slant range in WGS84/ECEF from the two vehicles' global
positions. It rotates explicit antenna-to-center lever arms from each body FRD
frame into NED/ECEF using the corresponding attitude.

The provider is fail-closed and label-only:

- both receivers must report MAVLink/PX4 GPS fix type 6 (`RTK_FIXED`);
- camera-frame/receiver and receiver/receiver offsets must be at most 100 ms;
- the quadrature combination of both horizontal/vertical GPS uncertainties
  and lever-arm calibration uncertainty must be at most 0.20 m;
- camera and target antenna-to-center body-FRD vectors plus their combined
  calibration uncertainty must be explicitly configured;
- runtime source `rtk_fixed_camera_to_target_center` is locked to quality
  `rtk_fixed` in both the manifest and every valid sample;
- GPS-only, RTK-float, stale, incomplete, excessive-uncertainty or mismatched
  source/quality observations are rejected before becoming training labels.

`tracking_web.py` now carries source, uncertainty, timestamp offset, quality
and lever-arm status through `FusionFrameContext`; `metric_target_fusion.py`
records those values in the existing v2 JSONL format. Provider counters and
the last rejection reason are exposed as `tracking.range_ground_truth`.
Gazebo remains the default source and receives the same locked metadata, so
existing SITL collection behavior is preserved.

The real collection configuration and fixed-gimbal lever-arm constraint are
documented in `README.md` and `.env.example`. Verification after this change:
`232 passed`, one pre-existing `PytestReturnNotNoneWarning`,
`run_all.sh --check` passed, and the changed Python modules passed
`py_compile`. No real hardware labels have been claimed or synthesized; the
next action is physical lever-arm calibration followed by independent RTK
collection sessions.

## Continuation: unchanged-data v9 retrain

At the user's request, the 14 retained Gazebo dataset directories were trained
again as candidate v9 with the unchanged seed 52, applicability threshold 0.8
and residual limit 3.0 m. The deterministic result repeated v8: all 165
train-correctable samples were accepted, only 1 of 69 validation-correctable
samples was accepted, and none of the 29 test-correctable samples was accepted.
The trainer failed closed with `all_partition_predictions_rejected`; no v9
bundle directory was created and residual correction remains `off`.

## Continuation: lateral SITL variation and candidates v10–v12

The user confirmed that only simulation is currently available. Three passive
Gazebo sessions were therefore collected with both UAVs disarmed, residual
correction off, automatic gimbal tracking off, and no arm, takeoff, mode,
follow or motion command:

- `range_v2_lateral7_pos_20260803`: target pose `(7, +1.5, 0)`, gimbal -10°,
  target center near `(174, 78)` px; 126/126 samples inside the deterministic
  envelope, 28 correctable and 98 uncorrectable. Median truth 7.1357 m,
  absolute residual 2.4193–3.8280 m. Manifest SHA-256
  `3e3633a46720b157909d3395b323bed02f4eb50a29adecf7f62a7e1b11e1d7e2`;
  samples SHA-256
  `c47c0d39df13069496a8215c5d2241973a50b995a0b564425068a3e18be6b9ab`.
- `range_v2_lateral7_neg_20260803`: target pose `(7, -1.5, 0)`, gimbal -10°,
  target center near `(306, 78)` px; 152/152 samples inside the envelope and
  all 152 correctable. Median truth 7.1357 m, absolute residual
  0.2958–2.3609 m. Manifest SHA-256
  `aac0b8888fd3064365ec26edc9c639300c58d6280ad7034d946175a240738f41`;
  samples SHA-256
  `b87fe5be381829171edfe95d2817487eb5245a8607220464882cbcefcc07e505`.
- `range_v2_lateral7_pos_repeat_20260803`: target pose `(7, +1.5, 0)`, gimbal
  -12°, target center near `(173, 65)` px; 140/140 samples inside the envelope
  and all 140 uncorrectable. Median truth 7.1300 m, absolute residual
  4.3707–9.9081 m. Manifest SHA-256
  `354569b718f40d1534b0e47f26560484deac6ffdf80863446e554130776413dc`;
  samples SHA-256
  `4d2a79360434da3c112da83ebaa2fdb67a8c1ccf91c196dfa142537300072a9a`.

Candidate v10 was the first training run with lateral data. Its validation
partition passed, but the lateral test partition had zero correctable recall;
five uncorrectable samples were accepted and corrected MAE was 1.048 m, so the
promotion gate remained false. Candidate v11 held both ±1.5 m lateral groups
in test while train contained only front-facing groups. It accepted the 152
right-side correctable samples but its regressor worsened their MAE from
1.424 m to 2.542 m; validation also failed.

Candidate v12 added the -12° lateral-negative group to train. Across all 17
dataset directories, 1,034 of 1,194 samples passed the deterministic envelope.
The fixed seed-52 split was train 677 (234/443), validation 79 (29/50), and
test 278 (180/98). False accepts on test fell to zero, but test correctable
recall was only 0.0889 and validation recall was zero. Test corrected MAE was
1.986 m versus the 1.104 m baseline, and residual prediction standard
deviation was 1.119 m above the 1.0 m limit. Candidate v12 manifest SHA-256 is
`35ba88979ef08ebff139a065b804e1d56b2f81830ebed0e2b66ace0637dcfd4a`.
Candidates v10–v12 are retained as rejected diagnostics only; residual mode
remains `off`.

The three exact temporary runtime-log directories (7.66 GB total) were
permanently deleted after verification; compact datasets were retained. No
runtime process remained. The next useful simulated condition is an
independent correctable lateral repeat that can place positive lateral support
in train without changing the locked seed or threshold.

## Continuation: shifted lateral repeat and candidate v13

The first shifted-background lateral-right collection produced two tracking
session groups inside one run after a tracking reset. Because the deterministic
seed-52 split placed those groups in different partitions, using it would have
created run-level session leakage. No candidate was trained on that input. The
compact dataset was moved intact, not deleted, to
`artifacts/quarantine_range_v2_lateral7_neg_shifted_20260803_multisession_leak`
and is excluded from the retained `artifacts/range_v2_*` training glob. Its
exact 3.68 GB temporary runtime-log directory was deleted after the audit.

A replacement collection pre-warmed tracking and calibration before setting
the target bbox, then recorded one uninterrupted session. Both UAVs remained
disarmed; residual correction, automatic gimbal tracking, follow and motion
remained off. No arm, takeoff or mode endpoint was invoked. UAV world poses
were shifted to `(3, 2, 0)` and `(10, 0.5, 0)`, preserving the relative target
offset `(7, -1.5, 0)` while changing the rendered background. The manual
gimbal pitch remained -10 degrees.

- Dataset: `artifacts/range_v2_lateral7_neg_shifted_clean_20260803`.
- Exactly one group: `UAV-01.2`; 220 labeled samples.
- All 220 samples passed the deterministic runtime envelope and all were
  correctable at the locked 3.0 m residual limit.
- Median truth was 7.1357 m. Absolute residual ranged 0.5649–2.5675 m, with
  median 1.7105 m, mean 1.6278 m and population standard deviation 0.5634 m.
- Manifest SHA-256:
  `5c59500cd5eaf912dc89cf3e0afbb8d9696b2c7e929b761e4a887aff42bd5304`.
- Samples SHA-256:
  `e63ce34aa4ffa50af022c9c647b0f6368e024a9ae7f9415843391c378e6e3dfa`.

Across the 18 retained dataset directories, 1,254 of 1,414 samples passed the
deterministic envelope. The unchanged seed-52 group-disjoint split placed the
new clean group wholly in train:

- train: 897 samples (454 correctable, 443 uncorrectable), 10 groups;
- validation: 79 samples (29 correctable, 50 uncorrectable), 2 groups;
- test: 278 samples (180 correctable, 98 uncorrectable), 2 lateral groups.

Candidate v13 used the unchanged seed 52, classifier threshold 0.8 and residual
limit 3.0 m. The independent lateral test improved substantially: precision
1.0, recall 0.8444, zero false accepts, and corrected MAE 0.3627 m versus the
1.4244 m physics baseline. The validation partition still failed: it accepted
four uncorrectable samples, no correctable sample, giving recall 0 and a 0.08
false-accept rate; corrected validation MAE was 1.0995 m. Residual prediction
standard deviation was 1.0997 m, above the 1.0 m promotion limit. Therefore the
overall promotion gate remains false even though the test partition passed.
Candidate v13 manifest SHA-256 is
`a119efe024ca819f6dd09aa01764fdec7aebc7a1c9fdc4f4f54a18dbe27da815`.
It is retained as a rejected diagnostic only; residual mode remains `off`.

The replacement session's exact 3.74 GB temporary runtime logs were deleted
after the compact dataset was verified. The next useful simulated evidence is
independent front-facing variation across sessions, backgrounds and pitch to
address the remaining validation-domain failure without changing the locked
seed or thresholds.

Post-v13 verification remained green: `232 passed` with one pre-existing
`PytestReturnNotNoneWarning`; `run_all.sh --check` and `py_compile` passed.
No PX4, Gazebo, XRCE, backend, MAVLink bridge or ROS launch process remained.

## Continuation: shifted front-facing evidence and candidate v14

Candidate v13's per-group audit localized the remaining failure to the fixed
front-facing validation partition. Its 8 m mixed group contained 29
correctable and 12 uncorrectable samples, but the classifier rejected every
sample. The independent 10 m validation group was entirely uncorrectable; four
of its 38 samples were false accepts. Two new passive Gazebo runs were therefore
collected without changing the seed, classifier threshold or residual limit.
Both UAVs remained disarmed and residual correction, automatic gimbal tracking,
follow and motion remained off. No arm, takeoff or mode endpoint was invoked.

The first retained run held UAV1 at `(3, -3, 0)` and UAV2 at `(11, -3, 0)`,
preserving a front-facing 8 m relative pose while shifting the world position.
It pre-warmed calibration before bbox selection and recorded one uninterrupted
tracking group:

- Dataset: `artifacts/range_v2_front8_shifted_clean_20260803`.
- Exactly one group/session, `UAV-01.2`; 78 labeled samples.
- All 78 passed the deterministic envelope and all were uncorrectable.
- Median truth was 7.9756 m. Absolute residual ranged 3.4407–4.9501 m, with
  median 3.9051 m, mean 4.0421 m and population standard deviation 0.5409 m.
- Manifest SHA-256:
  `47e97de0c0a598ab9d81709d9f8d21dd3749e8603d25a48898156fe12db60d32`.
- Samples SHA-256:
  `2d97bc16bee772920610e0ff01ac2fda2f448eb33bafa3dccf951652e4bf1262`.

An attempted front-7 run at world poses `(-3, 3, 0)` and `(4, 3, 0)` could not
obtain ground anchors at either -10 or -12 degrees gimbal pitch. It produced no
sample and was moved intact to
`artifacts/quarantine_range_v2_front7_shifted_clean_20260803_no_anchors`.
It is excluded from the retained training glob.

The replacement front-7 run used the already validated background with UAV1 at
`(3, -3, 0)` and UAV2 at `(10, -3, 0)`. Calibration was pre-warmed at -10
degrees before bbox selection. It recorded one uninterrupted mixed group:

- Dataset: `artifacts/range_v2_front7_shifted2_clean_20260803`.
- Exactly one group/session, `UAV-01.2`; 168 labeled samples.
- All 168 passed the deterministic envelope: 16 correctable and 152
  uncorrectable at the locked 3.0 m residual limit.
- Median truth was 6.9763 m. Absolute residual ranged 2.8487–6.1169 m, with
  median 3.3696 m, mean 3.7867 m and population standard deviation 0.9742 m.
- Manifest SHA-256:
  `3433ed19f2a5fdd4eb90f771b11abd8babe018abaa7f5862da039dbe87992095`.
- Samples SHA-256:
  `89c031cb2bfc73ff61de4c50a7af63ae82aea94f07b7b6d8a1d7621ff295a9fb`.

Across all 20 retained dataset directories, 1,500 of 1,660 samples passed the
deterministic envelope. The unchanged seed-52 group-disjoint split kept both
new groups wholly in train: train 1,143 samples (470/673), validation 79
(29/50), and test 278 (180/98). Validation and test groups remained identical
to v13, so the comparison has no holdout leakage.

Candidate v14 retained the strong lateral test result: applicability precision
1.0, recall 0.8444, zero false accepts, and corrected MAE 0.3952 m versus the
1.4244 m physics baseline. Residual prediction standard deviation improved from
1.0997 m in v13 to 1.0431 m, but remained above the 1.0 m limit. The front
validation partition still accepted no correctable sample and falsely accepted
five uncorrectable samples, giving recall 0 and false-accept rate 0.10.
Candidate v14 manifest SHA-256 is
`6234557aac48539af55795eab597efe163ebe2d377bf449ffda584f59b5b92b1`.
The overall promotion gate is false; v14 is diagnostic only and residual mode
remains `off`.

The exact temporary runtime-log directories for the retained front-8 and
front-7 runs (3.07 GB and 2.83 GB) and the rejected zero-anchor run (4.32 GB)
were deleted after audit; compact datasets/quarantine evidence were retained.
Static same-pose accumulation is no longer justified. The next useful simulated
experiment is a controlled multi-run front-facing sweep around 7.5–8.5 m with
independent backgrounds and pitch values, followed by grouped cross-session
evaluation without changing locked seed or thresholds.

Post-v14 verification remained green: `232 passed` with one pre-existing
`PytestReturnNotNoneWarning`; `run_all.sh --check` and `py_compile` passed.
No PX4, Gazebo, XRCE, backend, MAVLink bridge or ROS launch process remained.

## Continuation: controlled front-facing sweep before v15

A three-run front-facing sweep was collected around the unresolved 8 m class
boundary. Every retained run pre-warmed calibration before bbox selection,
contained exactly one tracking session/group, and kept both UAVs disarmed.
Residual correction, automatic gimbal tracking, follow and motion remained off;
no arm, takeoff or mode endpoint was invoked.

### Sweep A — 7.5 m, shifted background, -10 degrees

- Dataset: `artifacts/range_v2_front75_sweep_a_20260803`.
- World poses: UAV1 `(3, -3, 0)`, UAV2 `(10.5, -3, 0)`.
- 56/56 samples passed the deterministic envelope: 42 correctable and 14
  uncorrectable.
- Median truth 7.4759 m; absolute residual 2.4913–4.1304 m, median 2.6179 m,
  mean 2.8692 m, population standard deviation 0.4950 m.
- Manifest SHA-256:
  `60b1b2bb6660aa2ed1589abb68fa35da664ae773e6befa697f59a5798255abf7`.
- Samples SHA-256:
  `c0c806ce7eeade4536ad6e2b640e34bb5472fcd05a15724a3621b1bfe450ca91`.

### Sweep B — 8.0 m, default background, -12 degrees

- Dataset: `artifacts/range_v2_front80_sweep_b_20260803`.
- World poses: UAV1 `(0, 0, 0)`, UAV2 `(8, 0, 0)`.
- 42/42 samples passed the deterministic envelope: 34 correctable and 8
  uncorrectable.
- Median truth 7.9698 m; absolute residual 0.2543–3.6746 m, median 2.2593 m,
  mean 2.4534 m, population standard deviation 0.7058 m.
- Manifest SHA-256:
  `4f806cc1c7a944b30bdc5e7053cf6179264415c377808e83387598b5856ffa8f`.
- Samples SHA-256:
  `03ddd64f9f2b317feb1173c4617a314a99a20b3075fc7d1c872ab5e09f8a18fb`.

### Sweep C — 8.5 m, default background, -12 degrees

The initial shifted-background attempt failed closed: pitch -8 degrees did not
reach RANSAC consensus, pitch -10 degrees lost all anchors after bbox selection,
and no sample was recorded. Its manifest was moved intact to
`artifacts/quarantine_range_v2_front85_sweep_c_20260803_no_samples` and is
excluded from training.

The replacement retained run used UAV1 `(0, 0, 0)` and UAV2 `(8.5, 0, 0)`:

- Dataset: `artifacts/range_v2_front85_sweep_c2_20260803`.
- 33/33 samples passed the deterministic envelope; all 33 were uncorrectable.
- Median truth 8.4696 m; absolute residual 3.8747–4.3049 m, median 4.0343 m,
  mean 4.0555 m, population standard deviation 0.1394 m.
- Manifest SHA-256:
  `9efb6685d7eee21c1ed793fba4a3b68ab6e4882f84ca3f5c43c6488469c45acd`.
- Samples SHA-256:
  `7d5c134a6b5ea35770341dac7830079d4906f768c21d836fa4452a0595170acf`.

Across all 23 retained dataset directories, 1,631 of 1,791 samples passed the
deterministic envelope. With the unchanged seed 52, the applicability-stratified
group split is now:

- train: 1,151 samples (497 correctable/654 uncorrectable), 13 groups;
- validation: 248 samples (72/176), 3 groups — shifted front-8 negative,
  front-9 correctable, and lateral-left mixed;
- test: 232 samples (186/46), 3 groups — front-10 negative, the new front-8
  mixed sweep, and lateral-right correctable.

This is a more diverse group-disjoint evaluation than v14 and was obtained by
the locked split policy, not by changing the seed or assigning groups manually.
Candidate v15 has not yet been trained. The next action is exactly one v15
train/evaluation with the locked seed 52, classifier threshold 0.8 and residual
limit 3.0 m; no additional static collection is justified before that result.

The exact temporary logs for sweep A (1.64 GB), sweep B (1.93 GB), failed sweep
C (3.06 GB), and retained sweep C2 (2.84 GB) were deleted after audit. Compact
datasets and quarantine evidence were retained. No runtime process remained.

## Continuation: candidate v15 evaluation

Candidate v15 was trained exactly once on all 23 retained datasets with the
locked seed 52, classifier threshold 0.8 and residual limit 3.0 m. The split
remained train 1,151 samples (497/654), validation 248 (72/176), and test 232
(186/46).

Validation passed every promotion threshold: applicability precision 1.0,
recall 0.5417, zero false accepts, and corrected MAE 0.0960 m versus the 2.3718
m physics baseline. Residual prediction standard deviation improved to 0.1216
m, below the 1.0 m limit.

The aggregate test metrics also improved substantially: precision 0.9682,
recall 0.8172, corrected MAE 0.4336 m versus the 1.5015 m physics baseline.
However, five of 46 uncorrectable test samples were accepted, producing a
0.1087 false-accept rate above the locked 0.05 maximum. The test partition and
therefore the overall promotion gate failed.

Per-group inspection exposed a stronger containment reason than the aggregate
gate alone:

- the lateral-right group provided all 152 test true positives;
- the new front-8 mixed sweep had 34 correctable and 8 uncorrectable samples,
  but the classifier rejected all 42, giving front-8 correctable recall 0;
- all five false accepts came from the final five observations of the front-10
  negative group, with absolute residual about 3.845–3.847 m and classifier
  probabilities 0.805–0.844.

Candidate v15 manifest SHA-256 is
`7a822b731e498f33ab29abf4ad41bec39485388355f97d5c2d1b15cca81283af`.
It remains a rejected diagnostic and residual mode remains `off`. The next
decision must address front-domain per-group generalization; aggregate recall
must not be used to justify activation while front-8 recall is zero. Seed and
threshold tuning against these holdouts remains prohibited.

Post-v15 verification remained green: `232 passed` with one pre-existing
`PytestReturnNotNoneWarning`; `run_all.sh --check` and `py_compile` passed.
No PX4, Gazebo, XRCE, backend, MAVLink bridge or ROS launch process remained.

## Continuation: feature-drift audit, group containment and v16

The v15 failures were audited read-only before changing the trainer. The 13
train groups had a 6.67:1 largest-to-smallest sample-count ratio. The 220-sample
lateral-right correctable group alone contributed 220 of 497 train positives.
The 34 correctable front-8 test samples were closer in scaled feature space to
train uncorrectables than to train correctables; their largest positive-domain
shifts were camera optical-axis down, bbox/ray y, target-anchor extrapolation,
MiDaS target inverse depth and calibration offset. The five front-10 false
accepts were exactly the final five observations after a calibration transition,
with probabilities 0.805–0.844 and absolute residual about 3.845–3.847 m.

Two offline-only containments were then implemented without changing the
23-feature schema, runtime inference, seed or thresholds:

- XGBoost classifier and residual-regressor training matrices now use weights
  normalized to mean one while giving every run/session group equal total
  weight. Validation matrices use the same group-balanced policy for early
  stopping. The exact group totals are persisted in the training manifest.
- Promotion now reports and requires class-aware per-group recall and
  uncorrectable false-accept gates in both validation and test. Correctable-only
  groups do not invent a false-accept denominator; uncorrectable-only groups do
  not invent a recall denominator. Aggregate gates remain mandatory as well.

Candidate v16 was trained once as a diagnostic on the unchanged 23 datasets and
seed-52 split. Group weighting improved the residual regressor but did not solve
the classifier domain gap:

- residual prediction standard deviation: 0.5475 m, within limit;
- validation: precision 0.6984, recall 0.6111, false-accept 0.1080, corrected
  MAE 0.3357 m; aggregate and per-group gates failed;
- test: precision 0.9682, recall 0.8172, false-accept 0.1087, corrected MAE
  0.3571 m; classification was effectively unchanged from v15;
- validation per-group failures were 19/78 false accepts on shifted front-8
  negative and 0/28 recall on lateral-left correctable;
- test per-group failures remained 5/38 false accepts on front-10 negative and
  0/34 recall on front-8 correctable.

Candidate v16 manifest SHA-256 is
`7572a28f460d0f38986af3d4f9ee342792315deaf976211fee09138048a22c49`.
The overall gate is false; v16 is diagnostic only and residual remains `off`.
Because both the old validation and test groups have now influenced development,
they are no longer acceptable as final evidence. The training policy is frozen
at this point. Any final claim requires newly collected holdouts stored outside
the `artifacts/range_v2_*` training glob and left label-blind until the next
model is frozen.

Verification after containment: `14 passed` in the targeted dataset/trainer
suite and `234 passed` in full regression, with one pre-existing
`PytestReturnNotNoneWarning`; `run_all.sh --check` and `py_compile` passed.

## Continuation: precommitted frozen-v16 holdout evaluation

Before collecting any further labels, candidate v16 and its training policy
were frozen. A holdout precommit was written with SHA-256
`6e1b9f6600986bfb4a96ecd2a2bfd9b45711d405b0c41233427867310fe6a1d4`.
Both datasets were stored under `artifacts/holdout_range_v2_*`, outside the
`artifacts/range_v2_*` training glob. During collection only record count,
group/session identity, calibration and safety state were inspected. Residuals
and class labels were not opened until both compact datasets were complete and
checksummed.

Both runs kept the vehicles disarmed, residual correction off, automatic
gimbal tracking/follow/motion disabled, and invoked no arm, takeoff or mode
endpoint.

### Fresh H1 — front 8.2 m, pitch -11 degrees

- Dataset: `artifacts/holdout_range_v2_front82_pitch11_20260803`.
- World poses `(0, 0, 0)` and `(8.2, 0, 0)`; one group `UAV-01.4`.
- 24/24 samples passed the deterministic envelope; all 24 were correctable.
- V16 accepted none: recall 0. Seven samples were OOD; the remaining
  classifier probabilities were also below threshold, with overall range
  0.0611–0.1863 and median 0.1690.
- Manifest SHA-256:
  `d08b14a719235bcdba683d237d9a249ec78afabb71aa2bea5358a7bc3652d7c9`.
- Samples SHA-256:
  `9b760d70031eae7f144a982994040c0406bf1278f2e204c3a7f15f19f656e8dc`.

### Fresh H2 — shifted front 10 m, pitch -10 degrees

- Dataset: `artifacts/holdout_range_v2_front100_shifted_20260803`.
- World poses `(3, -3, 0)` and `(13, -3, 0)`; one group `UAV-01.2`.
- 60/60 samples passed the deterministic envelope; all 60 were uncorrectable.
- All 60 were rejected by the multivariate OOD contract, producing zero runtime
  false accepts even though raw classifier probabilities were high.
- Manifest SHA-256:
  `7adfd73aa63660dde61c9a0a5ffd2cce910e47c16692e55b93fe2e75097d318c`.
- Samples SHA-256:
  `dfe3a9aa592a47575a3562113ab51a6bf9de856b3b11c9f14f1f4680e9b34f56`.

Combined frozen-v16 holdout coverage was 0/84: 24 correctable false negatives
and 60 true negatives. This is safely fail-closed but unusable for correction,
so the final result is NO-GO. The immutable evaluation artifact is
`artifacts/range_v16_frozen_holdout_evaluation_20260803.json`.

Both holdouts have now been opened and must not serve as final evidence for a
future model. Further progress requires development data that covers
front-facing pitch/background variation, followed by another newly
precommitted untouched holdout after the next model is frozen. V16 remains off.
The exact H1 (6.98 GB) and H2 (2.15 GB) runtime logs were deleted after compact
artifact verification; no runtime process remained.

## Continuation: v17 development-only retrain

After the frozen-v16 evaluation was irreversibly opened, H1 and H2 ceased to be
final holdouts and were explicitly reclassified as development data. The v17
input list contained the 23 retained `range_v2_*` datasets plus both
`holdout_range_v2_*` directories. The unchanged seed-52 splitter placed both
new groups wholly in train, leaving the existing validation and test groups
unchanged and disjoint:

- train: 1,235 samples (521 correctable/714 uncorrectable), 15 groups;
- validation: 248 samples (72/176), 3 groups;
- test: 232 samples (186/46), 3 groups.

Candidate v17 used the frozen equal-total group weights, per-group promotion
gates, seed 52, classifier threshold 0.8 and residual limit 3.0 m. It improved
some aggregate metrics but did not resolve generalization:

- residual prediction standard deviation improved to 0.0727 m;
- validation aggregate precision 1.0, recall 0.5278, false-accept 0 and
  corrected MAE 0.0470 m passed, but lateral-left per-group recall remained
  0/28, so the validation group gate failed;
- test precision 0.9745, recall 0.8226 and corrected MAE 0.3522 m improved, but
  four front-10 false accepts produced a 0.0870 rate above the 0.05 limit;
- front-8 per-group recall improved only from 0/34 to 1/34, far below 0.5;
  front-10 false-accept rate remained 4/38 = 0.1053.

Candidate v17 manifest SHA-256 is
`b8b7248baccbb7b1b2c5ac19fd6d897bf303d7ef0fd9721a10b3e9755e6e15e5`.
The overall promotion gate is false and residual remains `off`. No new final
holdout should be consumed while v17 still fails these development group gates.
Further progress requires multiple independent correctable front-8/pitch groups,
lateral-left correctable support, and front-10 calibration-transition negatives
before freezing another model and precommitting another untouched holdout.
