# Báo cáo thay đổi Swarm Dashboard — 2026-07-30

## 1. Mục tiêu và quyết định kiến trúc

Trong phiên làm việc này, pipeline tracking/follow được chuyển khỏi hướng ước lượng
độ sâu bằng UniDepth sang hướng **bearing-only multi-view geometry**:

- Chỉ sử dụng camera RGB hiện có.
- Không giả định biết kích thước, loại hoặc hình dạng vật được tracking.
- Không dùng bbox size/LUT để suy ra khoảng cách tuyệt đối.
- Ước lượng tia ngắm từ tâm bbox, pose camera và intrinsics.
- Chỉ sinh khoảng cách 3D khi drone đã tạo đủ **baseline tịnh tiến** giữa nhiều
  góc quan sát.
- Nếu hình học chưa đủ tốt, hệ thống tiếp tục tracking 2D/gimbal nhưng không cho
  Follow Target tiến/lùi.
- UniDepth bị loại hoàn toàn khỏi runtime để giảm tải và tránh dùng khoảng cách
  monocular không ổn định làm authority điều khiển.

Đây là thay đổi thay thế cho kiến trúc UniDepth được ghi lại trong tài liệu cũ.
Tài liệu cũ vẫn được giữ làm lịch sử, không còn mô tả pipeline production hiện tại.

## 2. Những vấn đề ban đầu và cách xử lý

### 2.1 Drone yaw qua lại làm lắc thân và mất bbox

Thiết kế mới tách hai vòng điều khiển:

1. Gimbal là vòng nhanh, có nhiệm vụ giữ bbox gần tâm ảnh.
2. Body yaw là vòng chậm, chỉ recenter thân khi yaw gimbal lệch đủ lâu.

Body-yaw outer loop có:

- hysteresis: bật ở khoảng 6°, nhả ở khoảng 2,5°;
- thời gian giữ trước khi kích hoạt: 0,25 s;
- low-pass với hằng số thời gian khoảng 0,3 s;
- giới hạn tốc độ quay thân 10°/s;
- giới hạn slew 30°/s²;
- phanh về 0 trước khi cho phép đảo chiều;
- chặn khi feedback gimbal cũ hoặc không hợp lệ.

Kết quả mong đợi là gimbal bám mục tiêu trước, thân drone quay theo từ từ để trả
gimbal về vùng giữa, tránh hai controller cùng giành quyền yaw.

### 2.2 UniDepth không hiện khoảng cách hoặc nhảy sai số lớn

UniDepth và toàn bộ metric-depth runtime đã bị loại bỏ thay vì tiếp tục hiệu chỉnh:

- không preload model;
- không depth inference worker;
- không metric-depth queue;
- không `metric_depth_*` trong status API;
- không LUT kích thước bbox làm khoảng cách tuyệt đối;
- không torch/UniDepth trong dependency runtime của dashboard.

Khoảng cách mới được lấy từ giao hội nhiều tia ngắm. Estimate chỉ được công nhận
khi đạt các gate về baseline, góc giao hội, reprojection error, condition number,
range và covariance.

### 2.3 Không biết thông tin gì về vật được tracking

Không cần biết kích thước vật. Mỗi frame cung cấp:

- tâm bbox;
- timestamp của ảnh;
- intrinsics camera;
- vị trí và orientation camera tại đúng timestamp đó.

Từ dữ liệu này, hệ thống tạo ray trong hệ NED. Nhiều ray từ các vị trí camera khác
nhau được giải bằng weighted least squares, có pair-consensus để loại outlier.

Điểm quan trọng:

- quay camera/drone tại cùng một vị trí không tạo metric scale;
- drone phải tịnh tiến để tạo baseline;
- khi chưa đủ parallax, trạng thái phải ở `BOOTSTRAPPING` hoặc `DEGRADED`;
- Follow Target không được phép tiến/lùi khi estimate chưa `VALID`.

### 2.4 FPS giảm từ khoảng 30 xuống khoảng 10

Nguồn tải lớn nhất là UniDepth đã được loại bỏ. Ngoài ra:

- tracking dùng latest-frame processing;
- scale refinement chỉ chạy mỗi 3 frame;
- số scale candidate được giới hạn ở `0.92`, `1.00`, `1.08`;
- pose được lưu trong buffer và nội suy theo timestamp;
- status có timing cho queue, tracker, controller và encode;
- không chạy mô hình depth song song với tracker.

Smoke test runtime sau restart đạt 27,5 tracking-loop FPS trên nguồn 27,7 FPS ở
trạng thái chờ chọn bbox, không rơi frame.

## 3. Thành phần mới được thêm

### `bearing_target_estimator.py`

- Quản lý các trạng thái:
  `IDLE`, `TRACKING_2D`, `BOOTSTRAPPING`, `VALID`, `DEGRADED`, `LOST`.
- Chuyển bbox center thành bearing ray.
- Giải target 3D bằng robust multi-ray weighted least squares.
- Pair-consensus loại ray/outlier không nhất quán.
- Gate baseline, intersection angle, reprojection residual, condition number,
  range và covariance.
- Bearing-only EKF với state:
  `[pN, pE, pD, vN, vE, vD]`.
- Xuất khoảng cách, target position/velocity, covariance và reason diagnostics.
- Có bootstrap guidance nhưng `command_authorized=false`; hệ thống không tự bay
  maneuver chỉ để tạo baseline.

### `body_yaw_recenter.py`

- Outer loop recenter thân drone theo yaw gimbal.
- Hysteresis, hold timer, low-pass, rate limit, slew limit.
- Brake-before-reverse.
- Fail-safe khi feedback gimbal stale.

### `pose_time_sync.py`

- Buffer pose có timestamp.
- Nội suy position và velocity tuyến tính.
- Nội suy quaternion bằng SLERP.
- Reject sample out-of-order, quá cũ hoặc có khoảng nội suy quá lớn.

### Unit test mới

- `test_bearing_target_estimator.py`
- `test_body_yaw_recenter.py`
- `test_pose_time_sync.py`

## 4. File hiện có đã thay đổi

### `tracking_web.py`

- Loại toàn bộ import, worker và status của UniDepth.
- Tích hợp bearing target estimator.
- Chỉ publish Follow Target khi bearing estimate hợp lệ và còn mới.
- Gắn timestamp phép đo target vào command.
- Target invalid/stale làm dừng native follow.
- Thêm TTC/bbox-motion guard để chặn tiến tới khi có nguy cơ va chạm.
- Tích hợp body-yaw recenter outer loop.
- Bổ sung bearing/yaw/timing diagnostics.
- Native bearing follow là hướng điều khiển chính.

### `main.py`

- Loại preload/provider UniDepth.
- Thêm pose buffer có timestamp cho vehicle và camera.
- Đồng bộ pose camera với timestamp của image frame.
- Hỗ trợ camera lever-arm cấu hình theo body frame.
- Reject target measurement quá cũ hơn 0,5 s hoặc timestamp nằm trong tương lai.

### `mavlink_manual_bridge.py`

- Kiểm tra timestamp của Visual Follow Target.
- Watchdog neutralize command khi target measurement stale.
- Không cho phép command target cũ tiếp tục điều khiển PX4.

### `tracking_hybrid.py`

- Scale refinement chạy mỗi 3 frame.
- Candidate scale được giới hạn còn `0.92`, `1.00`, `1.08`.
- Bổ sung timing diagnostics.

### `body_attitude_recenter.py`

- Điều chỉnh default theo hướng an toàn hơn.

### `visual_follow_target.py`

- Loại LUT metric range khỏi runtime.
- Thêm `BBoxMotionSafetyEstimator` để phát hiện scale growth/TTC nguy hiểm.

### `static/index.html`

- Loại UI/diagnostics UniDepth.
- Hiển thị trạng thái bearing estimator, khoảng cách, chất lượng hình học,
  covariance, bootstrap/degraded reason và body-yaw recenter.

### Cấu hình và dependency

- `.env.example`: loại cấu hình UniDepth, thêm bearing/yaw/timestamp/lever-arm.
- `requirements.txt`: loại dependency/comment UniDepth và torch khỏi runtime.

## 5. File đã xóa khỏi active source

- `metric_depth_estimator.py`
- `metric_depth_accuracy.py`
- `metric_depth_evaluation.py`
- `test_metric_depth_estimator.py`
- `test_metric_depth_ground_truth.py`
- `test_camera_profile.py`
- Các JSON calibration/camera-profile của UniDepth.
- Các bytecode UniDepth mồ côi.

Thư mục handoff/backup cũ vẫn giữ bản lịch sử và không tham gia runtime.

## 6. Kiểm thử tĩnh

Kết quả trước runtime restart:

```text
python -m unittest discover
Ran 99 tests
OK
```

Ngoài ra:

- `py_compile` thành công cho các module active.
- JavaScript inline của frontend vượt qua `node --check`.
- Scan active runtime không còn tham chiếu UniDepth, `metric_depth` hoặc
  `SWARM_METRIC_DEPTH`.
- Project root hiện không phải Git repository hợp lệ, nên không có `git diff`
  hoặc commit để dùng làm nguồn lịch sử.

## 7. Runtime validation sau khi người vận hành disarm

Không restart PX4 hoặc Gazebo. Chỉ thay backend và MAVLink bridge.

Trạng thái an toàn được xác nhận:

```text
UAV-01: armed=false, failsafe=false
UAV-02: armed=false, failsafe=false
Tracking: inactive
Follow: inactive
Visual follow: inactive
Motion: inactive
MQTT: connected
Gazebo bridge: started
```

Runtime mới xác nhận:

- chỉ một Uvicorn backend và một `mavlink_manual_bridge.py`;
- UniDepth worker không tồn tại;
- `metric_depth_enabled` và `metric_depth_worker` không còn trong response;
- camera UAV-01/UAV-02 khoảng 28,7 FPS tại snapshot cuối;
- gimbal feedback hợp lệ;
- tracking lifecycle `start -> selecting -> stop` hoạt động;
- tracking chờ bbox đạt 27,5 FPS trên source 27,7 FPS;
- `source_frames_dropped=0`;
- frame queue p95 khoảng 1,24 ms;
- JPEG encode p95 khoảng 1,39 ms;
- không kích hoạt follow hoặc motion trong smoke test;
- tracking đã được stop và gimbal trở về home sau test.

Backend và bridge cũ đã được dừng đúng PID. Trong lúc chuyển giao có khoảng ngắn
hai MQTT client cùng ID làm bridge reconnect; sau khi tiến trình cũ thoát hoàn toàn,
socket giữ ổn định và không còn disconnect mới.

## 8. Phần chưa được xác minh đầy đủ

1. Hai camera lúc test chỉ nhìn thấy nền trời/đất, không có mục tiêu hợp lệ để chọn
   bbox.
2. Chưa benchmark FPS khi hybrid KCF và scale refinement thực sự chạy trên mục tiêu.
3. Chưa quan sát live quá trình bearing state:
   `TRACKING_2D -> BOOTSTRAPPING -> VALID`.
4. Chưa kiểm chứng khoảng cách/covariance bằng ground-truth Gazebo.
5. Chưa flight-test gimbal-first/body-yaw-recenter với mục tiêu chuyển động.
6. Camera lever-arm mặc định hiện là 0; nếu camera lệch tâm thân drone, cần nhập
   offset forward/right/down đúng trước khi đánh giá accuracy.
7. Cần baseline tịnh tiến tối thiểu khoảng 0,5 m theo cấu hình hiện tại; rotation-only
   sẽ không bao giờ tạo estimate metric hợp lệ.

Không tuyên bố hệ thống flight-ready cho Follow Target cho đến khi hoàn thành các
validation trên.

## 9. Checklist phiên tiếp theo

1. Xác nhận UAV disarmed/failsafe false trước mọi thử nghiệm.
2. Đưa một mục tiêu có texture rõ vào camera UAV tracking.
3. Chọn bbox và ghi lại:
   - source/tracking FPS;
   - tracker score và bbox;
   - gimbal yaw/rate;
   - body-yaw recenter state/rate;
   - bearing state/reason;
   - baseline, intersection angle, reprojection error;
   - range và covariance.
4. Cho drone/camera tạo baseline tịnh tiến có kiểm soát trong SITL.
5. So sánh target/range với relative pose ground-truth của Gazebo.
6. Chỉ bật Follow Target sau khi estimator giữ `VALID` ổn định và stale watchdog
   đã được quan sát hoạt động.
7. Test riêng các tình huống:
   - rotation-only;
   - baseline quá nhỏ;
   - ray gần song song;
   - bbox outlier/mất target;
   - target chuyển động;
   - measurement stale;
   - TTC/scale growth nguy hiểm.

## 10. Tài liệu liên quan

- `BEARING_FOLLOW_IMPLEMENTATION_REPORT_20260729.md`: báo cáo triển khai kỹ thuật.
- `CLAUDE_IMPLEMENTATION_PROMPT.md`: yêu cầu triển khai gốc đã được áp dụng.
- `Session Handoff 2026-07-29` trong Obsidian: lịch sử pipeline UniDepth trước khi
  quyết định thay thế.

