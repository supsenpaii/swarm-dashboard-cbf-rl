---
title: "M52 MiDaS Safe Follow - P1A Handoff 2026-08-02"
project: "Swarm Dashboard"
date: "2026-08-02"
resume_date: "2026-08-03"
status: "p1a-static-unit-go-runtime-no-go"
phase: "P1A safety containment"
tags:
  - swarm-dashboard
  - session-handoff
  - M52
  - MiDaS
  - PX4
  - follow-target
  - safety
  - P1A
---

# M52 + MiDaS Safe Follow — P1A handoff

## Kết luận cần nhớ

P1A safety containment đã được triển khai trong `/home/sup/swarm_dashboard`.

- **P1A static/unit gate: GO.**
- **Phase 0B runtime gate: NO-GO / chưa đóng.**
- Không được tiếp tục P1B, bật PX4 Follow mode hoặc tuyên bố runtime pass trước khi Phase 0B evidence được thu và người dùng review.
- Không có PX4 source/firmware nào bị sửa.
- Không cài dependency, không tải model, không chạy flight thật.
- Không chạy replay hoặc SITL trong patch P1A này.

## Tài liệu bắt buộc phải đọc lại khi tiếp tục

```text
docs/M52_MIDAS_XGB_COMPATIBILITY_REPORT.md
CODEX_AUDIT_IMPLEMENTATION_PLAN_M52_MIDAS_XGBOOST.md
CODEX_IMPLEMENTATION_PROMPT_M52_MIDAS_XGBOOST_SAFE_FOLLOW.md
M52_MIDAS_SAFE_FOLLOW_P1A_HANDOFF_20260802.md
```

Repository có `.codegraph/`. Khi cần tìm symbol/call path, phải dùng:

```bash
codegraph explore "<câu hỏi hoặc symbol>"
```

trước `rg`, `grep`, `find` hoặc đọc code lan man.

## P1A đã thay đổi gì

### 1. MAVLink/PX4 containment

File: `mavlink_manual_bridge.py`

- Xóa đường cấu hình PX4 bằng `param_set_send` khỏi production bridge.
- `FLW_TGT_DST`, `HT`, `FA`, `ALT_M`, `MAX_VEL`, `RS` chỉ được đọc bằng `PARAM_REQUEST_READ`, cache và validate.
- Safe native Follow mode request có flag riêng:

  ```text
  SWARM_SAFE_NATIVE_FOLLOW_MODE_REQUEST_ENABLED=false
  ```

- Legacy automation có flag riêng, mặc định off, và không thể cùng bật với safe native Follow mode entry:

  ```text
  SWARM_LEGACY_AUTOMATION_ENABLED=false
  ```

- `VisualFollowTargetState` yêu cầu target contract tối thiểu:

  ```text
  session_id > 0
  source_kind non-empty và không phải selection/provisional/apparent/bootstrap
  target_semantics == measured_target
  measurement_timestamp_s fresh
  lat/lon/alt/velocity/quality/covariance finite và đúng range
  lat/lon không đồng thời bằng 0
  frame cùng session phải có timestamp tăng nghiêm ngặt
  session cũ không được tái sử dụng
  ```

- Invalid payload fail-closed và vô hiệu hóa state đang active; không silent fallback.
- Stale target không còn được giữ làm degraded beacon. Bounded prediction chỉ được thêm ở phase EKF/send-time prediction sau gate riêng.
- Direct `FOLLOW_TARGET` sender cũng reject zero, NaN/Inf, covariance/capability invalid.
- Sau khi từng observe native Follow, mất nav state sẽ latch `reacquire_required`; không auto re-entry.
- Entry ACK và safe-exit ACK/nav confirmation được tách trạng thái.
- Safe native Follow path không còn neutral `MANUAL_CONTROL` heartbeat. Chỉ được xem xét lại khi Phase 7 có SITL evidence và log riêng chứng minh không tạo motion.
- Legacy MQTT automation messages bị reject khi legacy flag off.

### 2. Tracking/workflow containment

File: `tracking_web.py`

- Các default sau chuyển sang off:

  ```text
  body yaw
  legacy tracking follow
  native Follow Target
  apparent-size follow
  legacy automation
  ```

- `SWARM_VISUAL_FOLLOW_AUTO_START` bị cố ý bỏ qua; `visual_follow_auto_start` luôn `False`.
- Explicit authorization chỉ chấp nhận target metric đã converged.
- Selection anchor, provisional estimate, metric handover, apparent-size và bootstrap source không thể đi vào safe beacon path.
- RGB bootstrap automation bị block khi legacy automation không được opt-in.
- Target gửi xuống bridge mang `session_id`, `source_kind`, `target_semantics=measured_target`.
- Tracker, selected bbox và gimbal pointing hiện tại không bị reset bởi containment patch.

### 3. API/control containment

File: `main.py`

- `SWARM_TRACKING_OFFBOARD_ENABLED` mặc định `false`.
- `command_visual_follow_target()` validate session/source/semantics/finite fields/zero coordinate trước khi publish.
- Invalid target publish một disable command thay vì phát beacon giả.

### 4. Safe defaults

File: `.env.example`

```text
SWARM_TRACKING_OFFBOARD_ENABLED=false
SWARM_TRACKING_BODY_YAW_ENABLED=false
SWARM_TRACKING_FOLLOW_ENABLED=false
SWARM_NATIVE_FOLLOW_TARGET_ENABLED=false
SWARM_APPARENT_SIZE_FOLLOW_ENABLED=false
SWARM_VISUAL_FOLLOW_AUTO_START=false
SWARM_SAFE_NATIVE_FOLLOW_MODE_REQUEST_ENABLED=false
SWARM_LEGACY_AUTOMATION_ENABLED=false
```

`SWARM_VISUAL_FOLLOW_TARGET_ENABLED=true` vẫn giữ estimator/tracker feature tồn tại, nhưng native sender và real mode request vẫn bị chặn bởi các flag riêng mặc định off.

## Files đã chỉnh sửa

```text
mavlink_manual_bridge.py
tracking_web.py
main.py
.env.example
test_mavlink_attitude_bridge.py
test_follow_workflow.py
test_tracking_motion_altitude_hold.py
```

Workspace root hiện không phải Git working tree hợp lệ. Không được bịa `git diff`, commit hoặc rollback bằng Git. Không chỉnh các archive/handoff cũ.

Current SHA-256 sau P1A:

```text
906658277980620d9c43787a6bef81c486f7761f86f552bf213ef9de7ed6d960  mavlink_manual_bridge.py
5da7d17e4fe8592b32ea9420d02887e24df6890062c8d05ee341a2e5d8a6ef77  tracking_web.py
1009e161840a974b70d9bdd37751c3030f77568b6b7f1fdf5c32d29ff06a93ce  main.py
662ca924620b87cff47883d736ceaa85e5c3bcdaf7225f1f476a58a1e36a554c  .env.example
110661c721751792ae923d550a3ff683f7cd40b92d7cc2c04634bbbdbe480d24  test_mavlink_attitude_bridge.py
fa31efc78bffa0b6c47024379b8e1691db9f699ffb4b73ba069a185db7baabcd  test_follow_workflow.py
f0bba3ab4b46a8944bae2ea564f0c5954776098b0111fd960a13676459700280  test_tracking_motion_altitude_hold.py
```

## Tests và exact results

Targeted P1A:

```bash
env PYTHONDONTWRITEBYTECODE=1 \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  PYTHONPATH=/home/sup/swarm_dashboard/.venv/lib/python3.12/site-packages:/usr/lib/python3/dist-packages:/home/sup/swarm_dashboard \
  .venv/bin/python -m pytest -p no:cacheprovider -q \
  test_mavlink_attitude_bridge.py \
  test_follow_workflow.py \
  test_visual_follow_target.py \
  test_tracking_motion_altitude_hold.py
```

Kết quả cuối:

```text
68 passed in 0.42s
```

Full repository regression, loại archived handoff snapshot có test basename trùng:

```bash
env PYTHONDONTWRITEBYTECODE=1 \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  PYTHONPATH=/home/sup/swarm_dashboard/.venv/lib/python3.12/site-packages:/usr/lib/python3/dist-packages:/home/sup/swarm_dashboard \
  .venv/bin/python -m pytest -p no:cacheprovider -q \
  --ignore=swarm_dashboard_handoff_20260729
```

Kết quả cuối:

```text
192 passed, 1 warning in 1.42s
```

Warning còn lại là `PytestReturnNotNoneWarning` có sẵn trong `test_bearing_target_estimator.py::test_config`; không phải P1A failure.

Static check cuối:

```bash
rg -n "param_set_send|send_visual_follow_manual_heartbeat|SAFE_NATIVE_FOLLOW_NEUTRAL_HEARTBEAT_ENABLED|send_centered_controls" \
  mavlink_manual_bridge.py tracking_web.py main.py .env.example
```

Kết quả: không có match.

## Acceptance tests đã thêm/cập nhật

- No PX4 parameter write trong production bridge.
- Mode request feature flag off không phát `MAV_CMD_DO_SET_MODE` cho native Follow.
- Auto-start environment override bị bỏ qua.
- Selection/provisional target authorization bị reject.
- Zero beacon và NaN/Inf packet bị reject.
- Stale measurement không được phát lại như beacon hợp lệ.
- Duplicate/out-of-order frame và previous-session target bị reject.
- No automatic re-entry sau khi Follow nav state từng được observed rồi mất.
- Legacy altitude-hold regression chỉ chạy khi test chủ động opt-in OFFBOARD; production default vẫn off.
- Safe native path không có neutral manual heartbeat sender.

## Metrics đã có và chưa có

Đã có:

- Production occurrence của `param_set_send`: `0`.
- Unit/static gate: pass theo kết quả trên.
- Default authority flags: off.

Chưa chạy, không được suy đoán pass:

```text
PX4 runtime version/commit confirmation
read-only FLW_TGT_* values từ đúng SITL instance
sensor capture timestamp vs callback monotonic time
camera/vehicle/gimbal delay và clock offset
runtime resolution/FOV/intrinsics/profile
optical axes/quaternion order/ENU↔NED signs
local/global origin và AMSL convention
FOLLOW_TARGET measured rate/gaps/field inspection
mode-entry jerk and first-setpoint deltas
vehicle peak speed/acceleration/overshoot/settling
bbox retention across handover
safe-exit time
structured replay/determinism
```

## Known limitations / blockers

1. Phase 0B runtime evidence chưa được thu trong session P1A.
2. Structured logging và deterministic replay chưa tồn tại; đây là P1B sau khi Phase 0B được review.
3. Workflow hiện tại chưa được migrate hoàn toàn sang canonical state list mới.
4. M52 chưa có independent target candidate đúng canonical contract.
5. Chưa có StandardScaler/OOD/XGBoost/CI runtime hoặc grouped dataset.
6. Read-only parameter validation mới có unit evidence; chưa có evidence từ đúng PX4 runtime.
7. Các threshold read-only parameter hiện cần được đối chiếu Phase 0B evidence và chuyển thành documented config nếu tiếp tục dùng; không được coi numeric assumptions hiện tại là runtime-validated.
8. Neutral heartbeat đã bị loại. Nếu PX4 runtime yêu cầu joystick link để giữ Follow, đây là blocker phải báo, không được tự bật workaround.

## Việc cần làm ngày 2026-08-03

### Bước 1 — Xác minh workspace không đổi

```bash
pwd
sha256sum \
  mavlink_manual_bridge.py tracking_web.py main.py .env.example \
  test_mavlink_attitude_bridge.py test_follow_workflow.py \
  test_tracking_motion_altitude_hold.py
```

So sánh với SHA-256 ở trên. Nếu mismatch, dùng CodeGraph đọc đúng symbol thay đổi trước khi làm tiếp; không overwrite thay đổi mới của người dùng.

### Bước 2 — Rerun P1A tests

Chạy targeted command trước. Nếu pass, chạy full regression command. Nếu fail, dừng và chẩn đoán; không chuyển phase.

### Bước 3 — Thu Phase 0B evidence bằng read-only checks

Trước khi chạy, báo patch protocol:

```text
Phase/Patch: P0B runtime evidence + jerk fixture
Files planned: report/fixture only; không sửa estimator/control
Evidence: static P1A pass, runtime evidence còn thiếu
Invariants: no PX4 write, no real mode request, tracker/gimbal retained
Safety impact: read-only first; SITL only
Migration/rollback: additive fixture/report, không xóa log
Acceptance gate: clock/frame/altitude conventions resolved; baseline replayable
```

Thu theo thứ tự:

1. Exact PX4 version/commit và đúng SITL instance.
2. Read-only `FLW_TGT_DST`, `HT`, `FA`, `ALT_M`, `MAX_VEL`, `RS`.
3. Sensor capture timestamp và callback monotonic timestamp; đo offset/jitter.
4. Vehicle/camera/gimbal timestamp age, interpolation span và delay.
5. Runtime camera width/height/FOV/intrinsics/profile; fail closed nếu mismatch.
6. Optical axes, quaternion order và Gazebo ENU ↔ PX4 NED signs.
7. Local/global origin, home reset behavior và AMSL convention.
8. `FOLLOW_TARGET` field mapping/rate/gaps ở shadow/no-mode nếu có thể.
9. Audit mọi gimbal writer; xác nhận chỉ một authority.

Không sửa thuật toán trong Phase 0B.

### Bước 4 — Jerk regression fixture

- Trước hết thu baseline/no-mode fixture.
- Không xóa hoặc overwrite log cũ; tạo session ID/path mới.
- Ground truth chỉ đi vào evaluation channel.
- Nếu cần bật native Follow mode để đo discontinuity, dừng và báo trước exact SITL command, vehicle, current nav state, exit command và expected safety impact để xin user approval.
- Không bật mode ngoài SITL.
- Sau fixture phải report first setpoint delta, peak velocity/acceleration, overshoot/settling và bbox retention. Nếu không đo được, ghi `not run` hoặc `unknown`, không suy đoán.

### Bước 5 — Phase 0B gate review

Chỉ đề xuất GO khi:

- Không còn clock/frame/altitude convention chưa xác định.
- Có measured baseline.
- Có replayable jerk fixture.
- Không có gimbal conflict hoặc unsupported mode/command.

Nếu gate fail, dừng với evidence, impact, 2–3 options và recommended option.

### Bước 6 — Chỉ sau khi user duyệt Phase 0B

Patch tiếp theo là P1B:

```text
versioned canonical schemas
structured event log
deterministic replay skeleton
schema mismatch rejection
no-future-data checks
replay modes: current-safe, M52-only, MiDaS-only,
              rule selection, XGBoost gated, CI
```

Không bắt đầu estimator/XGBoost/CI trước schemas/log/replay gate.

## Stop conditions vẫn áp dụng

Dừng và xin ý kiến nếu cần:

- sửa PX4 source/firmware/module/build output;
- gửi `PARAM_SET` hoặc thay parameter bằng cơ chế khác;
- cài dependency hoặc tải model;
- bật mode ngoài SITL;
- dùng neutral heartbeat chưa có evidence;
- clock/intrinsics/frame/altitude không xác minh được;
- replay/data thiếu hoặc acceptance gate fail;
- native Follow command unsupported;
- gimbal conflict;
- jerk/spike/bbox regression;
- read-only `FLW_TGT_*` mâu thuẫn safe entry geometry.

## Safety state khi kết thúc handoff

- Không có runtime/SITL process nào được start bởi P1A patch session.
- Không gửi mode command.
- Không ghi PX4 parameter.
- Không có real-flight authorization.
- Safe native mode request mặc định off.
- OFFBOARD/legacy automation mặc định off.

## Câu mở đầu đề xuất cho phiên ngày mai

```text
Tiếp tục từ M52_MIDAS_SAFE_FOLLOW_P1A_HANDOFF_20260802.md.
Dùng CodeGraph trước khi tìm/đọc code. Xác minh SHA và rerun P1A tests.
Sau đó thu Phase 0B runtime evidence bằng read-only SITL checks; không bật
Follow mode nếu chưa báo exact command và được tôi duyệt. Không triển khai P1B
trước khi trình Phase 0B gate report.
```

