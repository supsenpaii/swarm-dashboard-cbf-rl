# Prompt tiếp tục dự án — M52 + MiDaS + XGBoost + PX4 Follow Target

> Cập nhật cùng ngày: lát cắt range/XGBoost offline-shadow trong Mục 12 đã được
> triển khai thành contract v2. Trước khi làm tiếp, đọc
> `M52_MIDAS_RANGE_APPLICABILITY_V2_IMPLEMENTATION_20260803.md`. Không làm lại
> các bước 3–5 của Mục 12 và không nạp bundle v1/v3 cũ. Hiện có 23 thư mục dữ
> liệu v2 / 1,791 mẫu thô; 1,631 mẫu qua deterministic envelope. Sweep
> front-facing 7.5/8.0/8.5 m đã bổ sung ba group độc lập: 56 mẫu mixed (42/14),
> 42 mẫu mixed (34/8), và 33 mẫu negative. Candidate gần nhất vẫn là v14
> NO-GO: lateral test precision 1.0, recall 0.844, false-accept 0 và corrected
> MAE 0.395 m, nhưng validation recall 0, false-accept 0.10 và residual
> prediction std 1.043 m vượt giới hạn. Candidate v15 validation pass và
> residual prediction std giảm còn 0.122 m, nhưng test vẫn NO-GO vì false-accept
> 0.1087; audit theo group cho thấy front-8 bị bỏ sót 34/34 correctable. Trainer
> đã được containment bằng equal-total group weights và promotion gate per-group.
> Candidate v16 diagnostic chứng minh weighting không đủ: validation precision
> 0.698/false-accept 0.108, test vẫn bỏ sót 34/34 front-8 và nhận nhầm 5 front-10.
> Bundle v10–v16 chỉ là diagnostic, không được kích hoạt. Residual
> correction vẫn phải
> `off`; deterministic applicability gate mặc định `active`. Không tiếp tục
> tích lũy các frame SITL tĩnh gần như giống nhau. Người dùng xác nhận dự án
> hiện chỉ dùng mô phỏng; Gazebo truth vẫn là label source. Runtime collector RTK-fixed
> đã được triển khai trong `range_ground_truth.py`: WGS84/ECEF, đồng bộ ≤100 ms,
> combined uncertainty ≤0.20 m, bắt buộc hai lever arm body-FRD và khóa
> source/quality trong manifest nhưng không được bật. Bước SITL tiếp theo có
> Policy và v16 đã được khóa checksum trước khi thu hai holdout ngoài glob.
> Fresh evaluation vẫn NO-GO: front-8.2 có 24/24 correctable nhưng v16 nhận 0;
> front-10 có 60/60 uncorrectable và OOD gate loại toàn bộ. Combined coverage
> bằng 0/84. Hai holdout này đã mở label và không còn được dùng làm final
> evidence cho model sau. Chúng được đưa vào train cho v17 diagnostic; v17 vẫn
> NO-GO: validation aggregate pass nhưng lateral-left recall 0/28; test
> false-accept 0.087 và front-8 recall 1/34. Không đổi seed/threshold và không
> bật v15/v16/v17.

Hãy dùng toàn bộ nội dung file này làm prompt mở đầu cho phiên/tài khoản Codex
mới.

---

Tiếp tục dự án tại:

```text
/home/sup/swarm_dashboard
```

Ngày bàn giao: `2026-08-03`, múi giờ `Asia/Ho_Chi_Minh`.

## 1. Mục tiêu người dùng đã chốt

Không mở rộng kiến trúc ngoài mục tiêu sau:

```text
RGB tracking
  -> M52 + MiDaS bù trừ cho nhau để ước lượng khoảng cách tương đối
  -> StandardScaler chuẩn hóa feature
  -> XGBoost hiệu chỉnh/tối ưu sai số khoảng cách
  -> dùng tâm bbox + camera/gimbal pose để suy ra tọa độ 3D thật của vật
  -> nếu vật di chuyển, suy ra và lọc vận tốc từ chuỗi tọa độ
  -> cập nhật liên tục tọa độ + vận tốc thật của vật
  -> gửi MAVLink FOLLOW_TARGET cho PX4 Follow Me
  -> PX4 bám vật và giữ khoảng cách an toàn đã khóa lúc bắt đầu tracking
```

Khoảng cách an toàn phải được chụp đúng một lần khi tracking bắt đầu/target đủ
tin cậy:

```text
D_safe = estimated_relative_distance_at_tracking_start
```

Sau khi khóa, `D_safe` không trôi theo khoảng cách tức thời. `FOLLOW_TARGET`
luôn chứa tọa độ và vận tốc **thật** của vật; tuyệt đối không dịch hoặc tạo tọa
độ mục tiêu giả để ép khoảng cách. Khoảng cách đứng ngoài mục tiêu là contract
riêng để PX4 duy trì.

Nếu tracking, depth, frame transform hoặc velocity không còn hợp lệ thì fail
closed: không phát dữ liệu đoán bừa, báo mất/reacquire target.

## 2. Chỉ dẫn trực tiếp từ người dùng

- Giữ giải pháp đơn giản, không phức tạp hóa phần gimbal hoặc sáng tạo thêm
  control architecture không cần thiết.
- Tracking và xoay gimbal hiện đã hoạt động; không viết lại chỉ vì Gazebo liệt
  kê hai publisher.
- Người dùng đã nói `ok triển khai đi` cho pipeline đã chốt ở Mục 1, nhưng turn
  triển khai bị ngắt trước khi có bất kỳ sửa đổi code nào.
- Có thể triển khai offline/shadow và unit test. Không tự động arm, takeoff,
  chuyển mode hay tạo chuyển động.
- Không bật PX4 Follow mode nếu chưa trình bày **exact entry command, vehicle,
  current state, exit command, expected effect** và được người dùng duyệt rõ.
- Không gửi `PARAM_SET`, mode request hoặc valid live `FOLLOW_TARGET` trong quá
  trình test nếu chưa có protocol/approval tương ứng.
- Không dùng sub-agent trừ khi người dùng yêu cầu.

## 3. Quy tắc thao tác repository

Repository có `.codegraph/`. Bắt buộc dùng CodeGraph trước khi `rg`, tìm hoặc
đọc code:

```bash
codegraph explore "câu hỏi hoặc symbol cần truy vết"
```

Ưu tiên mở rộng code hiện có, không tạo pipeline song song. Dùng `apply_patch`
cho mọi sửa file. Workspace root không phải Git working tree; dùng SHA-256 file
để kiểm tra integrity và luôn giữ thay đổi không liên quan của người dùng.

## 4. Trạng thái hiện tại — phần lớn pipeline đã có

Đừng kết luận rằng phải viết toàn bộ từ đầu. CodeGraph và static search đã xác
nhận các điểm nối sau:

### Depth và fusion

- `depth_model_adapter.py`
  - `MidasSmallAdapter`
  - trả `DepthMap.inverse_depth`, source `midas_small`.
- `m52_adapter.py`
  - `M52GroundAnchorAdapter`
  - dùng ray/ground intersection của M52 làm metric anchors.
- `metric_target_fusion.py`
  - `MetricTargetFusion`
  - mô tả hiện tại: async MiDaS + M52 metric calibration + target EKF
    coordinator.
  - runtime đã có source `m52_midas`.
- `bearing_range_filter.py`
  - đã có temporal filtering trong inverse-depth domain của MiDaS.
- `target_depth_extractor.py`
  - có `TargetDepth` cho vùng target/bbox.

### Tọa độ, vận tốc và khoảng cách ban đầu

- `tracking_web.py`
  - `TrackingManager`
  - đã tích hợp `MetricTargetFusion` và `BearingTargetEstimator`.
  - `_save_initial_safe_distance_locked()` đã tồn tại.
  - các field hiện có:
    - `initial_safe_distance_horizontal_m`
    - `initial_safe_distance_slant_m`
    - `initial_safe_distance_lock_timestamp_s`
    - `initial_safe_distance_lock_count`
    - `initial_safe_distance_source`
  - đã có `target_position_ned`, `target_velocity_ned`, covariance và target
    estimate lifecycle.
  - đã gọi `visual_target_command` ở đường native visual target.
- `bearing_target_estimator.py`
  - `BearingTargetEstimator`
  - đã ước lượng target state/velocity từ quan sát theo thời gian.
- `visual_follow_target.py`
  - có camera ray projection và các kiểu target state liên quan.

### FOLLOW_TARGET

- `main.py`
  - nối `TrackingManager(... visual_target_command=command_visual_follow_target)`.
  - `command_visual_follow_target()` là boundary trước bridge.
- `mavlink_manual_bridge.py`
  - sender thực tế gọi `connection.mav.follow_target_send(...)`.
  - P1A đã thêm validation/fail-closed và feature flags.
- `test_mavlink_attitude_bridge.py`, `test_follow_workflow.py`,
  `test_visual_follow_target.py` đã có coverage cho nhiều contract liên quan.

## 5. Phần thực sự còn thiếu

Tại thời điểm bàn giao không có implementation runtime cho:

- `StandardScaler`;
- XGBoost model/trainer/runtime adapter;
- trained model bundle;
- feature schema/version/checksum;
- dataset đủ để train/validate XGBoost;
- `xgboost`, `scikit-learn`, `joblib` trong project environment theo audit gần
  nhất.

Không được bịa model đã train, fit scaler bằng dữ liệu runtime, hoặc tuyên bố
XGBoost hoạt động khi chưa có artifact thật.

## 6. Hướng triển khai tối thiểu mong muốn

Đầu tiên hãy dùng CodeGraph kiểm tra lại call path hiện hành và trình một patch
manifest ngắn. Sau đó triển khai theo lát cắt nhỏ nhất có thể test offline:

### Bước A — Contract feature đơn giản

Tạo một feature schema versioned, chỉ chứa feature cần thiết và có thể đo thật,
ví dụ:

```text
m52_distance_or_anchor_m
m52_confidence_or_quality
midas_target_inverse_depth_median
midas_target_inverse_depth_spread
bbox_width_px
bbox_height_px
bbox_area_fraction
previous_fused_distance_m
delta_time_s
```

Không đưa ảnh, timestamp tuyệt đối, NED/WGS84 target output hoặc control command
vào XGBoost.

### Bước B — StandardScaler + XGBoost residual correction

Giữ physics pipeline làm baseline. XGBoost chỉ hiệu chỉnh khoảng cách hoặc dự
đoán residual/error, không dự đoán tọa độ target trực tiếp:

```text
features_scaled = scaler.transform(features)
distance_final = distance_physics_base + xgb.predict(features_scaled)
```

Nếu benchmark sau này cho thấy raw features tốt hơn cho tree model thì vẫn giữ
StandardScaler trong bundle để schema/OOD monitoring, nhưng phải khóa rõ model
dùng raw hay scaled. Scaler chỉ `fit` trên training split; validation, test và
runtime chỉ `transform`.

Bundle tối thiểu phải chứa:

```text
feature names/order/types
missing-value policy
scaler
XGBoost model
training manifest
versions
SHA-256 checksums
```

Runtime phải tắt XGBoost và quay về physics baseline an toàn khi bundle thiếu,
schema mismatch, NaN/Inf hoặc inference lỗi. Trước khi có dataset/model thật,
chỉ triển khai collector/trainer contract hoặc shadow adapter; không promote
output giả vào target state.

### Bước C — Tích hợp vào pipeline hiện có

Tích hợp correction tại boundary tạo range measurement của
`MetricTargetFusion`; không tạo một target estimator thứ hai. Output đã hiệu
chỉnh phải tiếp tục đi qua uncertainty/temporal filter và estimator hiện có.

Sau đó dùng pipeline hiện hữu:

```text
bbox center + camera intrinsics + gimbal/camera orientation
  -> bearing/ray
fused distance + bearing + UAV pose
  -> physical target position NED
time sequence of valid target positions
  -> filtered target velocity NED
physical position/velocity
  -> FOLLOW_TARGET mapping
```

### Bước D — Khóa khoảng cách ban đầu

Reuse `_save_initial_safe_distance_locked()`. Xác minh bằng test rằng:

- lock chỉ thành công từ estimate hợp lệ;
- lock count bằng 1 trong một tracking session;
- estimate các frame sau không ghi đè `D_safe`;
- restart/reselect target tạo session mới và reset đúng;
- target coordinates gửi đi vẫn là physical coordinates, không bị offset theo
  `D_safe`.

### Bước E — Velocity và sender contract

Reuse estimator hiện có. Xác minh:

- stationary target cho velocity gần 0;
- constant-velocity target hội tụ đúng hướng/độ lớn;
- invalid `dt`, jump, stale observation và reacquire fail closed;
- `FOLLOW_TARGET` nhận position/velocity đồng bộ cùng timestamp/frame;
- zero/NaN coordinate, invalid covariance/capability hoặc stale target bị từ
  chối;
- không có test nào cần bật Follow mode.

## 7. Acceptance cho patch đầu tiên

Patch đầu tiên nên nhỏ và có thể review. Chỉ coi đạt khi:

- không phá đường tracking/gimbal hiện tại;
- không tạo motion, mode request hay live target beacon;
- feature schema deterministic và được unit-test;
- scaler không bao giờ fit ở runtime;
- model bundle fail closed;
- physics baseline vẫn hoạt động khi XGBoost unavailable;
- initial safe distance lock không bị drift;
- target position/velocity vẫn là physical target state;
- targeted tests pass, sau đó full regression pass;
- cập nhật tài liệu/feature flags nhưng mặc định giữ native Follow và mode
  request off.

## 8. Runtime và safety gate đã thu

P1A file integrity đã được xác minh và targeted tests đã pass. Kết quả gần nhất:

```text
P1A targeted: 68 passed
P0B-A tests: 8 passed
Full regression: 200 passed, 1 known PytestReturnNotNoneWarning
```

Phase 0B runtime evidence đã được thu read-only. Phase 0B vẫn `NO-GO` vì còn
clock/frame/altitude, measured-target shadow và Follow jerk gates. Không dùng
việc triển khai offline/shadow để tuyên bố runtime Follow đã pass.

Gimbal P0B-B đã làm rõ:

- dashboard và PX4 đều advertise publisher;
- cả 6 topic có `0` message trong các idle sample ba giây;
- PX4 bridge chỉ phát khi PX4/QGC target thực sự thay đổi;
- không được coi publisher count đơn thuần là active conflict;
- giữ đường dashboard tracking-to-Gazebo hiện tại;
- không dùng QGC gimbal đồng thời với dashboard tracking cho tới khi có test
  overlap riêng.

## 9. Tài liệu nên đọc theo thứ tự

Chỉ đọc những phần cần thiết sau khi dùng CodeGraph:

1. `NEXT_ACCOUNT_PROMPT_M52_MIDAS_XGBOOST_FOLLOW_20260803.md` — file này.
2. `M52_MIDAS_SAFE_FOLLOW_P1A_HANDOFF_20260802.md` — P1A safety contract.
3. `M52_MIDAS_SAFE_FOLLOW_PHASE0B_GATE_REPORT_20260803.md` — gate chính.
4. `M52_MIDAS_SAFE_FOLLOW_PHASE0B_CLEAN_RERUN_ADDENDUM_20260803.md`.
5. `M52_MIDAS_SAFE_FOLLOW_PHASE0B_GIMBAL_AUTHORITY_ADDENDUM_20260803.md`.
6. `XGBOOST_M52_MIDAS_FUSION_FLOW.md` chỉ để tham khảo feature/training; nếu
   mâu thuẫn về độ phức tạp, ưu tiên mục tiêu đơn giản trong file này.

Các audit/roadmap cũ rất chi tiết. Không biến toàn bộ chúng thành yêu cầu mới
nếu không cần cho lát cắt hiện tại.

## 10. SHA-256 baseline trước phiên mới

```text
1009e161840a974b70d9bdd37751c3030f77568b6b7f1fdf5c32d29ff06a93ce  main.py
5da7d17e4fe8592b32ea9420d02887e24df6890062c8d05ee341a2e5d8a6ef77  tracking_web.py
4cb034bed67cfbddbd3e45a066425783b2e9f1e81944d1d50d97b46bc01496fd  m52_adapter.py
516f72fd0bc62376dc0c9a62b3f5be80440d3810e74338ba65bdc561505777d3  metric_target_fusion.py
64a5ad7fb6911b54402c0f22e31243c94cb2810553c350de94c574f9311f4776  bearing_target_estimator.py
906658277980620d9c43787a6bef81c486f7761f86f552bf213ef9de7ed6d960  mavlink_manual_bridge.py
```

Hãy kiểm tra lại các SHA trước khi sửa và báo mọi mismatch. Workspace hiện
không có PX4/Gazebo/backend/XRCE runtime do phiên trước để lại.

## 11. Artifact/log cần biết

- P0B-A final artifact:
  `artifacts/phase0b_runtime_evidence_20260803_104547.json`
- P0B-B session logs:
  `artifacts/run_20260803_110431`
- P0B-B logs khoảng 3.1 GiB, vẫn ở ổ trong và chưa bị xóa/chuyển ở phiên cuối.
- Hai log lớn cũ đã được bảo toàn trên ổ rời dưới:
  `/media/sup/New Volume/swarm_dashboard_artifacts/`.

Không xóa hoặc di chuyển artifact nếu chưa kiểm tra exact target và báo người
dùng.

## 12. Việc phải làm ngay ở phiên mới

1. Dùng CodeGraph truy `MetricTargetFusion`, `_save_initial_safe_distance_locked`,
   `command_visual_follow_target` và `follow_target_send`.
2. Xác minh SHA baseline và trạng thái không có runtime.
3. Trình patch manifest ngắn cho lát cắt StandardScaler/XGBoost offline/shadow.
4. Không hỏi lại toàn bộ yêu cầu; mục tiêu đã được chốt trong Mục 1.
5. Triển khai patch nhỏ, test targeted, rồi full regression.
6. Báo rõ phần nào thật sự chạy, phần nào còn chờ dataset/model; không tuyên bố
   Follow runtime pass.

---

Kết luận ngắn cho agent mới: **đừng viết lại tracking, gimbal, estimator hoặc
FOLLOW_TARGET path đang có. Hãy bổ sung lớp StandardScaler/XGBoost hiệu chỉnh
range vào `MetricTargetFusion`, giữ physics baseline và initial-distance lock,
test hoàn toàn offline/shadow trước.**
