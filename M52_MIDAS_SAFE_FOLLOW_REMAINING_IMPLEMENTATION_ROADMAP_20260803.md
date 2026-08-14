---
title: "M52 + MiDaS Safe Follow - Remaining Implementation Roadmap"
project: "Swarm Dashboard"
date: "2026-08-03"
status: "planning-only-phase-0b-no-go"
scope: "Những việc còn phải triển khai để hoàn thành dự án"
---

# M52 + MiDaS Safe Follow — Kế hoạch triển khai phần còn lại

## 1. Mục đích

Tài liệu này tổng hợp những việc còn phải thực hiện để đưa M52 + MiDaS Safe
Follow từ trạng thái safety containment hiện tại đến Definition of Done.

Đây là tài liệu kế hoạch, không phải xác nhận rằng các phase chưa chạy đã PASS.
Mỗi phase phải có evidence, test, metrics và báo cáo gate riêng trước khi chuyển
sang phase tiếp theo.

## 2. Trạng thái hiện tại

| Hạng mục | Trạng thái |
|---|---|
| P1A safety containment | PASS static/unit |
| P1A targeted tests | 68 passed |
| Full regression | 192 passed, 1 warning đã biết |
| Phase 0B runtime evidence | NO-GO |
| P1B / Phase 1 | Chưa triển khai |
| Phase 2–12 | Chưa được phép tuyên bố hoàn thành |
| PX4 Follow mode trong lần audit 2026-08-03 | Không được bật bởi phiên audit |
| PX4 source/firmware | Không sửa |

P1A đã khóa các đường nguy hiểm mặc định: không tự bật Follow, không tự ghi
`FLW_TGT_*`, không phát target giả, không tự re-enter Follow và không dùng
OFFBOARD thay native Follow.

Phase 0B hiện NO-GO vì timing backlog, dual gimbal authority, frame/origin và
altitude convention chưa đóng, chưa có measured-target shadow stream, chưa có
jerk fixture và phiên đo trước bị UI/QGroundControl bên ngoài làm nhiễu.

## 3. Tổng số gate còn lại

Còn **13 gate lớn**:

1. Đóng Phase 0B runtime evidence.
2. Phase 1 — Structured logging và deterministic replay.
3. Phase 2 — Observation, time synchronization và camera calibration.
4. Phase 3 — M52 ground anchors và independent target candidate.
5. Phase 4 — MiDaS metric calibration và target candidate.
6. Phase 5 — Rule-based fusion baseline.
7. Phase 6 — Target EKF và send-time prediction.
8. Phase 7 — NED to WGS84 và MAVLink `FOLLOW_TARGET`.
9. Phase 8 — Native PX4 Follow handover và loss handling.
10. Phase 9 — Dataset chuẩn cho XGBoost.
11. Phase 10 — StandardScaler, XGBoost và OOD.
12. Phase 11 — Covariance Intersection và shadow runtime.
13. Phase 12 — UI, observability và operational safety.

Luồng phụ thuộc chính:

```text
Phase 0B
  -> Phase 1
  -> Phase 2
  -> Phase 3 + Phase 4
  -> Phase 5
  -> Phase 6
  -> Phase 7
  -> Phase 8
  -> Phase 9
  -> Phase 10
  -> Phase 11
  -> Phase 12
  -> Definition of Done
```

Phase 3 và Phase 4 có thể được phát triển độc lập sau khi contract của Phase 2
đã ổn định. Các gate vẫn phải được review theo thứ tự để tránh thay đổi
semantics dữ liệu giữa chừng.

## 4. Nguyên tắc an toàn áp dụng cho mọi phase

- Không sửa PX4 source hoặc firmware.
- Không gửi `PARAM_SET` để thay đổi `FLW_TGT_*`.
- Không dùng OFFBOARD để thay native PX4 Follow.
- Không phát beacon zero, NaN/Inf, stale hoặc không đúng measured-target
  semantics.
- Không tự động arm, takeoff, land hoặc vào Follow.
- Không tự re-enter Follow sau target loss hoặc nav-state loss.
- Chỉ một controller được quyền ghi gimbal tại một thời điểm.
- Mọi feature mới phải mặc định fail-closed.
- Không dùng dữ liệu tương lai hoặc ground truth trong runtime features.
- Không triển khai phase sau khi gate trước đang NO-GO.
- Mỗi hành động bật Follow phải được trình trước bằng exact command, UAV, current
  nav state, điều kiện vào, safe-exit command và ảnh hưởng dự kiến; chỉ chạy sau
  khi người dùng phê duyệt rõ ràng.

## 5. Phase 0B — Đóng runtime evidence gate

### 5.1 Tạo baseline SITL sạch

Việc cần làm:

- Đưa cả hai SITL UAV về trạng thái an toàn và disarm.
- Stop tracking.
- Đóng hoặc cô lập QGroundControl và browser control clients.
- Tạo session/log directory mới, không ghi đè evidence cũ.
- Xác minh native Follow, Follow request, OFFBOARD tracking và legacy automation
  đều off.
- Ghi process identity, PX4 binary/commit/SHA và tất cả client có khả năng phát
  command.

Gate PASS:

- Baseline không có external control contamination.
- Trạng thái đầu vào và ownership được ghi lại đủ để tái hiện.

### 5.2 Đóng camera, clock, rate và delay contract

Việc cần làm:

- Xác định artifact intrinsics có thẩm quyền: runtime `CameraInfo` hay lens
  introspection; mismatch phải fail-closed.
- Ghi resolution, FOV, `fx/fy/cx/cy`, distortion và camera profile ID.
- Dùng latest-frame/non-blocking recorder để đo source timestamp, receipt
  monotonic, processing start/end, queue age, drops và actual consumption rate.
- Đo camera, camera IMU, vehicle pose, gimbal state và MAVLink bridge timing trong
  cùng session sạch.
- Chứng minh recorder không tự tạo backlog.

Gate PASS:

- Không còn clock domain mơ hồ.
- Queue/drop policy và end-to-end age được đo.
- Runtime không có offset tăng vô hạn do subscriber không theo kịp.

### 5.3 Đóng frame, transform, origin và altitude contract

Việc cần làm:

- Chứng minh optical axes, pixel-center convention và quaternion order.
- Chứng minh camera-to-body transform và ENU↔NED signs bằng known rays/points.
- Thu synchronized Gazebo pose, PX4 local position, global position, home và
  altitude ở các điểm đứng yên đã biết.
- Ghi per-instance local origin epoch và home/reset behavior.
- Xác định rõ Gazebo Z, local Down, ellipsoid altitude và AMSL mapping.
- Kiểm tra round trip NED → WGS84 → NED với tolerance được công bố.

Gate PASS:

- Không còn axis/sign, origin hoặc altitude convention chưa xác định.
- Cùng một contract áp dụng đúng cho cả hai SITL instances.

### 5.4 Đóng gimbal authority

Việc cần làm:

- Liệt kê tất cả publisher của roll/pitch/yaw command topics.
- Chọn một authority duy nhất hoặc thiết kế arbitration có state rõ ràng.
- Chứng minh authority transfer không tạo command jump và không làm mất bbox.
- Nếu phải sửa orchestration/control để giải quyết, lập patch riêng và xin review
  trước; không lén workaround trong runtime audit.

Gate PASS:

- Mỗi axis chỉ có một effective writer tại mọi thời điểm.
- Ownership và transition được log và test.

### 5.5 Measured-target shadow stream

Việc cần làm:

- Tạo target đúng canonical contract: session, frame, source, semantics,
  timestamp, position, velocity, covariance và quality hợp lệ.
- Inspect `FOLLOW_TARGET` fields, coordinate frame, capabilities, rate, gaps,
  duplicates và out-of-order behavior khi chưa vào Follow mode.
- Không dùng selection/provisional/apparent/bootstrap target làm measured target.
- Không bật Follow trong bước này.

Vì đây là outbound MAVLink write dù chưa đổi mode, phải trình protocol cụ thể và
được phê duyệt trước khi chạy.

Gate PASS:

- Field mapping đúng, không có invalid/zero packet.
- Rate/gap đạt contract và không burst bù sau event-loop stall.

### 5.6 Controlled Follow handover/jerk fixture

Chỉ thực hiện sau khi 5.1–5.5 PASS.

Trước khi chạy phải trình người dùng:

- Exact command/API action.
- UAV được chọn và current nav state.
- Preconditions: SITL only, armed/in-air/local-position/home/target warm-up.
- Expected motion và safety envelope.
- Exact safe-exit command và abort conditions.
- Log directory và metrics sẽ thu.

Metrics bắt buộc:

- First PX4 position/velocity/acceleration setpoint delta sau mode switch.
- Vehicle peak velocity và acceleration.
- Overshoot và settling time.
- Target position/range/velocity jitter.
- Bbox retention và bbox-lost timing.
- Mode request, `COMMAND_ACK`, actual nav state và safe-exit time.

Gate PASS:

- Không tái diễn mode-entry jerk trong SITL acceptance set.
- Bbox được giữ theo threshold đã duyệt.
- Target loss dẫn đến safe exit trong timeout hữu hạn.

### 5.7 Đầu ra Phase 0B

- Phase 0B gate report cập nhật.
- Session log sạch, không overwrite session cũ.
- Timing/frame/altitude/gimbal contracts đã được chốt.
- Replayable handover/jerk fixture hoặc blocker có evidence.
- Quyết định GO/NO-GO rõ ràng cho P1B.

## 6. Phase 1 — Structured logging và deterministic replay

Triển khai:

- Schema/version cho observation, pose sync, calibration, source candidates,
  fusion, EKF, WGS84 beacon, mode command/ACK/nav state và vehicle response.
- Log capture timestamp, processing timestamps, age, validity và reason codes.
- Gắn đúng `session_or_track_id` và `frame_index` cho mọi record.
- Replay runner không cần bay lại và không dùng dữ liệu tương lai.
- Fixture cố định cho sự cố mode-entry jerk.
- Validation và rejection rõ ràng cho schema mismatch.

Gate PASS:

- Replay cùng fixture cho output deterministic trong tolerance đã định.
- Có thể chạy current, M52-only, MiDaS-only, baseline, ML và CI trên cùng session.

## 7. Phase 2 — Observation, time sync và camera calibration

Triển khai:

- Canonical observation contract theo session/frame/capture timestamp.
- Pose buffer lookup theo capture time.
- Linear interpolation position và quaternion SLERP attitude.
- Stale/extrapolation rejection và reason codes.
- Full intrinsics/profile validation; distortion/undistortion policy rõ ràng.
- Pixel center, four corners, horizon ray và known-ground-intersection tests.

Gate PASS:

- Không dùng pose/depth của frame khác.
- Không có axis/sign swap.
- Profile/FOV mismatch fail-closed.

## 8. Phase 3 — M52 ground anchors và target candidate độc lập

Triển khai:

- Ground-anchor sampling loại trừ target bbox/mask.
- Kiểm tra valid fraction, spatial coverage, horizon/geometry condition và
  optical-depth consistency.
- Robust contact point/bottom-center projection xuống ground plane.
- Reject near-horizon, behind-camera, negative và ill-conditioned intersection.
- Propagate covariance từ pixel, bbox/contact, pose, intrinsics và ground plane.
- M52-only logging/replay và reason codes.

Gate PASS:

- Candidate có semantics, age, validity và covariance đầy đủ.
- Error/covariance được đánh giá theo range và camera attitude.

## 9. Phase 4 — MiDaS metric calibration và target candidate

Triển khai:

- MiDaS raw depth path tách khỏi visualization normalization.
- Robust inverse-depth scale/shift calibration từ M52 ground anchors.
- Calibration stability, drift, residual và inlier/coverage checks.
- Robust target ROI sampling, foreground/background handling và uncertainty.
- MiDaS candidate có timestamp, position, covariance, validity và reason codes.
- MiDaS-only logging/replay.

Gate PASS:

- Calibration invalid làm candidate invalid, không silent fallback.
- Candidate metric được validation trên held-out replay.

## 10. Phase 5 — Rule-based fusion baseline

Triển khai:

- Gate tracker, pose sync, age, M52 geometry, MiDaS calibration/ROI,
  source disagreement và EKF innovation.
- Dùng một source tốt với covariance conservative khi source còn lại xấu.
- Predict-only có timeout ngắn khi cả hai source xấu, sau đó invalid.
- Log source choice, weights/reason và covariance.

Gate PASS:

- Không kém current pipeline trên held-out replay về median/p90/p95 position
  error, velocity error, jump rate và stale behavior.
- Không phát mode command thật trong phase này.

## 11. Phase 6 — Target EKF và send-time prediction

Triển khai:

- EKF position/velocity với variable `dt` theo đúng timestamp.
- Position/range/bearing update chỉ khi semantic/covariance hợp lệ.
- Mahalanobis gate, bounded process noise và covariance floor.
- Session-change reset, finite predict-only timeout và target-loss policy.
- Bounded send-time extrapolation với covariance inflation theo age.
- Log state gốc và state sau compensation.

Gate PASS:

- Stationary target velocity bias thấp.
- Moving-target compensation cải thiện held-out error.
- Không tăng tail error khi target dừng hoặc đổi hướng.

## 12. Phase 7 — NED to WGS84 và MAVLink FOLLOW_TARGET

Triển khai:

- Chuyển local absolute NED target sang WGS84/AMSL theo contract Phase 0B/2.
- Chống cộng drone position hai lần, North/East swap và Down/Up/AMSL mix.
- Map đầy đủ `FOLLOW_TARGET` fields và capabilities.
- Configurable send rate theo đúng PX4 build đã xác minh.
- Chống duplicate/out-of-order, stale/invalid beacon và catch-up burst.
- Theo dõi actual send rate và gap distribution.

Gate PASS:

- Static target WGS84 ổn định.
- Known N/E movement đúng dấu và độ lớn.
- Không có invalid/zero beacon; packet inspection khớp contract.

## 13. Phase 8 — Native PX4 Follow handover và loss handling

Triển khai:

- Entry gate: tracker/estimator stable window, beacon warm-up, bounded
  uncertainty/age, no jump/disagreement/OOD, PX4 preconditions và single gimbal
  authority.
- Tính/kiểm tra entry geometry với runtime `FLW_TGT_*` read-only values.
- Explicit user action mới được request native Follow.
- Kiểm tra `COMMAND_ACK = ACCEPTED` và actual PX4 nav state.
- Target loss: bounded predict-only, covariance inflation, verified safe-exit,
  ACK/nav confirmation và không auto re-entry.
- SITL acceptance cho jerk, peak motion, bbox retention và exit time.

Gate PASS:

- First setpoint không vượt threshold đã duyệt.
- Không có velocity/acceleration spike như regression session.
- Loss handling fail-safe và bounded.

## 14. Phase 9 — Dataset chuẩn cho XGBoost

Triển khai:

- Thu nhiều range, motion profile, bbox condition, occlusion, attitude, geometry
  và source-failure regimes.
- Data dictionary, feature schema/version và split manifest.
- Labels cho per-source error/q90 và ground-contact applicability nếu cần.
- Group split theo flight/session/location; không frame leakage.
- Loại ground-truth-derived, future state và absolute identifier khỏi runtime
  features.

Gate PASS:

- Dataset đủ coverage theo failure regime.
- Không leakage; split bị khóa trước model selection.

## 15. Phase 10 — StandardScaler, XGBoost và OOD

Triển khai:

- Fit preprocessing chỉ trên training data.
- StandardScaler dùng cho schema/distribution/OOD; chỉ đưa scaled features vào
  tree model nếu benchmark chứng minh tốt hơn và bundle khóa contract đó.
- Train/tune riêng hai model error/q90 cho M52 và MiDaS.
- Grouped validation/CV, bounded search, seed, early stopping và trial history.
- Đánh giá q90 coverage, p90/p95 error, false accept/reject, jump rate và runtime
  latency theo source/range/attitude/failure regime.
- Bundle model, scaler, schema, preprocessing, OOD thresholds, versions và
  checksums.
- Fail-closed khi feature/schema/checksum/NaN/OOD không hợp lệ.

Gate PASS:

- Hai model được train/tune/calibrate bằng pipeline tái lập.
- Chỉ mở locked test split một lần cho promotion report.
- Nếu chưa cải thiện ổn định, giữ offline/shadow; không tuyên bố heuristic-only
  là hoàn thành dự án.

## 16. Phase 11 — Covariance Intersection và shadow runtime

Triển khai:

- CI chỉ nhận M52/MiDaS candidates hợp lệ, calibrated covariance, bounded age,
  acceptable disagreement và non-OOD input.
- Bound/tối ưu CI weight bằng tiêu chí conservative.
- Inflate covariance theo predicted q90, age, OOD và correlation risk.
- Log weight, objective, source health và reason codes.
- Chạy shadow runtime trước khi có quyền ảnh hưởng beacon.

Gate PASS:

- Tail error và jump rate tốt hơn baseline.
- Covariance consistency đạt yêu cầu và không overconfident.

## 17. Phase 12 — UI, observability và operational safety

Triển khai:

- Hiển thị tracker state/score, range/uncertainty, position/velocity validity,
  estimate age và fusion/source health.
- Hiển thị beacon warm-up/rate/gaps, entry-gate reason, requested mode, ACK và
  actual PX4 nav state.
- Hiển thị rõ lý do Follow enabled/disabled.
- Audit và enforce gimbal authority/arbitration.
- Operational checklist, abort procedure, safe defaults và operator-visible
  warnings.
- End-to-end acceptance suite và runbook.

Gate PASS:

- Operator có thể biết hệ thống đang ở state nào và vì sao bị chặn.
- Không dual-control gimbal.
- Bbox retention và safe exit đạt acceptance thresholds.

## 18. Test matrix xuyên suốt

Mỗi phase phải bổ sung test phù hợp, tối thiểu gồm:

- Unit tests cho contract, validation, transform, geometry và covariance.
- Deterministic replay regression.
- Schema/version/checksum/missing-feature failures.
- Timestamp stale, out-of-order, duplicate, frame/session mismatch.
- NaN/Inf/zero/invalid target rejection.
- M52-only, MiDaS-only, both-good, one-bad và both-bad scenarios.
- Stationary, moving, stop, acceleration và direction-change targets.
- OOD/disagreement/occlusion/edge bbox scenarios.
- MAVLink packet inspection, rate/gap và no-catch-up-burst.
- Mode-entry jerk, bbox retention, target-loss safe-exit và no-auto-reentry.
- Full repository regression sau mỗi patch.

Không được ghi “PASS” cho test chưa chạy. Báo cáo phải có exact command và exact
result.

## 19. Báo cáo bắt buộc sau mỗi phase

```text
Phase:
Outcome:
Compatibility/invariants preserved:
Files changed:
Tests run and exact results:
Metrics before/after:
Safety checks:
Known limitations:
Go/No-Go for next phase:
Next proposed patch:
```

Mọi thay đổi phải là patch nhỏ, có rollback/migration note và không ghi đè thay
đổi ngoài phạm vi.

## 20. Definition of Done của toàn dự án

Chỉ tuyên bố hoàn thành khi tất cả điều kiện sau có evidence:

- Tracker hoạt động và session/frame association đúng.
- Camera calibration, geometry và timestamp sync đã test.
- Structured logging và deterministic replay hoạt động.
- M52 và MiDaS candidates có semantics, validity và covariance rõ.
- Rule-based baseline không kém current pipeline trên held-out data.
- EKF position/velocity và latency compensation được validation.
- NED→WGS84/AMSL conversion đúng và liên tục.
- `FOLLOW_TARGET` mapping/rate/gap behavior được kiểm chứng.
- Native Follow chỉ vào sau warm-up, entry gate, user action, ACK và nav-state
  confirmation.
- Mode-entry jerk không tái diễn trong SITL acceptance set.
- Target loss có bounded safe exit và không auto re-entry.
- Gimbal không dual-control và bbox retention đạt yêu cầu.
- StandardScaler/model bundle có schema/version/checksum và OOD fail-closed.
- Hai XGBoost error/q90 models đã train, tune, calibrate và tái lập được.
- CI chỉ được promote khi cải thiện tail error/covariance consistency.
- UI/runbook cho biết rõ state, blockers và abort path.
- Không sửa PX4, không ghi PX4 parameter và không dùng OFFBOARD thay native
  Follow.
- Mọi limitation còn lại được ghi bằng evidence và được người dùng chấp nhận.

## 21. Việc tiếp theo ngay lúc này

Không bắt đầu P1B. Trình tự gần nhất là:

1. Người vận hành đưa SITL về trạng thái safe/disarmed, stop tracking và loại bỏ
   QGroundControl/browser control contamination.
2. Xác minh trạng thái bằng read-only checks.
3. Thu lại Phase 0B mục 5.1–5.4 trong session sạch, vẫn không bật Follow.
4. Trình protocol riêng trước measured-target shadow stream.
5. Sau khi shadow gates PASS, trình exact Follow-entry/exit command để xin phê
   duyệt cho jerk fixture.
6. Cập nhật Phase 0B gate report và chỉ đề xuất P1B nếu kết luận là GO.

## 22. Tài liệu nguồn

- `M52_MIDAS_SAFE_FOLLOW_PHASE0B_GATE_REPORT_20260803.md`
- `M52_MIDAS_SAFE_FOLLOW_P1A_HANDOFF_20260802.md`
- `CODEX_IMPLEMENTATION_PROMPT_M52_MIDAS_XGBOOST_SAFE_FOLLOW.md`
- `CODEX_AUDIT_IMPLEMENTATION_PLAN_M52_MIDAS_XGBOOST.md`
- `docs/M52_MIDAS_XGB_COMPATIBILITY_REPORT.md`

