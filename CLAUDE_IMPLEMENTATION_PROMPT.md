# Prompt triển khai Visual Follow Target

## Model khuyến nghị

- Ưu tiên: **Claude Fable 5**, effort **High/Extra**.
- Phương án thay thế: **Claude Opus 4.8**, effort **Extra/xhigh**.
- Tối ưu chi phí: **Claude Sonnet 5**, effort **xhigh**.
- Nên chạy bằng Claude Code trong repository `/home/sup/swarm_dashboard`.

---

## Prompt giao cho Claude

Bạn đang làm việc trong repository:

```text
/home/sup/swarm_dashboard
```

Mục tiêu của nhiệm vụ này là thay đổi toàn bộ pipeline visual tracking/follow target theo kiến trúc mới:

1. Loại bỏ hoàn toàn UniDepth khỏi runtime.
2. Tối ưu tracker để phục hồi FPS.
3. Tách điều khiển gimbal và body yaw để drone không lắc qua lại.
4. Thay việc dùng UniDepth đo khoảng cách bằng bearing-only multi-view triangulation.
5. Thêm estimator position/velocity cho mục tiêu chuyển động.
6. Chỉ gửi dữ liệu PX4 Follow Target khi vị trí mục tiêu đủ tin cậy.

Không dừng ở mức phân tích hoặc viết kế hoạch. Hãy trực tiếp chỉnh sửa code, bổ sung test và chạy kiểm tra phù hợp.

Thực hiện nhiệm vụ theo từng checkpoint. Sau mỗi checkpoint:

- Tóm tắt file đã sửa.
- Chạy test liên quan.
- Kiểm tra `git diff`.
- Không chuyển sang checkpoint tiếp theo nếu test hiện tại thất bại.
- Không restart PX4, Gazebo hoặc backend và không gửi flight command.

## I. Quy tắc làm việc

1. Trước khi đọc hoặc tìm code:

   - Kiểm tra repository có thư mục `.codegraph/`.
   - Nếu có, bắt buộc dùng CodeGraph trước `grep`, `find` hoặc đọc file:

     ```bash
     codegraph explore "<câu hỏi hoặc tên symbol>"
     ```

   - Dùng CodeGraph để tìm:
     - Luồng camera → tracker → UniDepth → target NED.
     - Luồng bbox → gimbal → body yaw.
     - Luồng target NED/WGS84 → PX4 Follow Target.
     - Tất cả nơi tham chiếu metric depth/UniDepth.
     - Scale refinement trong tracker.
     - Cách lấy PX4 pose, attitude, velocity và gimbal feedback.

2. Kiểm tra `git status` trước khi sửa.

   - Repository có thể đang có thay đổi của người dùng.
   - Không reset, checkout hoặc ghi đè thay đổi không liên quan.
   - Không sử dụng `git reset --hard`.
   - Không tự tạo commit nếu chưa được yêu cầu.

3. Dùng `apply_patch` để chỉnh sửa file.

4. Không khởi động lại backend, Gazebo, PX4 hoặc gửi flight command nếu UAV đang armed.

   - Chỉ chạy unit test/offline test.
   - Không thực hiện lateral maneuver thật.
   - Không gửi lệnh arm, takeoff hoặc mode change.
   - Nếu cần runtime/SITL restart để kiểm tra, trước tiên phải kiểm tra trạng thái an toàn và xin xác nhận của người dùng.

5. Không hardcode intrinsics, camera resolution hoặc home position.

6. Giữ backward compatibility cho các API không liên quan nếu có thể.

7. Không che giấu estimator kém chất lượng bằng smoothing quá mạnh. Phải xuất uncertainty, residual và trạng thái observability thật.

## II. Kiến trúc đích

Pipeline đích:

```text
Camera RGB
    → bbox tracker
    → bbox center
    → pixel-to-bearing trong camera frame
    → camera/gimbal/body transform
    → bearing trong NED
    → timestamp-synchronized pose observations
    → multi-view triangulation / bearing-only estimator
    → target position NED
    → target velocity NED
    → covariance + observability + quality gate
    → NED-to-WGS84 nếu cần
    → PX4 FOLLOW_TARGET
```

Điều khiển hướng:

```text
BBox error
    → gimbal controller nhanh

Gimbal yaw feedback
    → body recenter controller chậm
```

UniDepth không còn nằm trong pipeline này.

## III. Loại bỏ UniDepth

Hãy tìm toàn bộ reference trước khi xóa. Dự kiến có các khu vực như:

- `tracking_web.py`
- `metric_depth_estimator.py`
- `static/index.html`
- Test liên quan metric depth
- Biến môi trường `SWARM_METRIC_DEPTH_*`
- Model configuration và diagnostics
- API/status JSON
- Ground-truth recorder dành riêng cho UniDepth

Loại bỏ:

1. Import và khởi tạo UniDepth.
2. Model loading.
3. Depth worker/thread.
4. Depth mailbox hoặc queue.
5. Inference scheduling.
6. `metric_depth_distance_m`.
7. `metric_depth_state`.
8. `metric_depth_diagnostics`.
9. Ground-truth/calibration recorder chỉ phục vụ UniDepth.
10. Logic `bbox ray + UniDepth range → target NED`.
11. UI hiển thị UniDepth:
    - Model status.
    - Raw distance.
    - ROI MAD/IQR.
    - Uncertainty.
    - Model diagnostics.
12. Các environment variables chỉ phục vụ UniDepth.
13. Test chỉ kiểm tra UniDepth.

Sau khi xác nhận không còn consumer:

- Xóa `metric_depth_estimator.py`.
- Gỡ dependency UniDepth khỏi project.
- Chỉ gỡ PyTorch nếu xác nhận không có thành phần nào khác dùng nó.
- Không xóa model cache hoặc weights ngoài repository.
- Không để import mềm hoặc dead code UniDepth tồn tại trong runtime.

Thay các trường UI/API UniDepth bằng diagnostics của bearing estimator:

- `target_range_m`
- `target_position_ned`
- `target_velocity_ned`
- `range_std_m`
- `position_covariance`
- `baseline_m`
- `intersection_angle_deg`
- `reprojection_error_px`
- `observation_count`
- `estimate_age_ms`
- `observability_score`
- `target_estimator_state`
- `target_estimate_valid`
- `invalid_reason`

Nếu frontend/backend hiện có consumer cần field cũ, cập nhật consumer thay vì giữ field UniDepth giả.

## IV. Sửa gimbal và body yaw

Vấn đề hiện tại:

- Body yaw và gimbal đều phản ứng mạnh theo bbox.
- Body yaw có thể nhanh hơn gimbal.
- Ngưỡng recenter quá chặt.
- Lệnh dễ đảo chiều liên tục.
- Drone lắc yaw và làm mất bbox.

Hãy audit toàn bộ nguồn phát body yaw. Đảm bảo tại một thời điểm chỉ có một authority gửi lệnh body yaw.

### 1. Gimbal inner loop

- Gimbal tiếp tục bám tâm bbox.
- Chạy ở tốc độ tracking hiện có, mục tiêu khoảng 25–30 Hz.
- Low-pass filter bbox error.
- Có deadband nhỏ.
- Có anti-windup.
- Khi tracking invalid/lost phải dừng hoặc giữ gimbal theo chính sách an toàn hiện có.
- Không để body controller tranh quyền với gimbal.

### 2. Body recenter outer loop

Body yaw chủ yếu dựa trên gimbal yaw feedback đã lọc, không dựa trực tiếp mạnh vào bbox.

Thông số mặc định ban đầu:

- Recenter start: `abs(gimbal_yaw) > 6 deg`
- Recenter stop: `abs(gimbal_yaw) < 2.5 deg`
- Hold time trước khi kích hoạt: `0.25 s`
- Body yaw rate tối đa: `10 deg/s`
- Body yaw acceleration/slew: `30 deg/s²`
- Bbox feed-forward vào body: mặc định bằng `0`
- Cho phép cấu hình các giá trị này qua config/environment phù hợp.

Yêu cầu:

- Có hysteresis start/stop.
- Có low-pass filter cho gimbal yaw.
- Có slew-rate limiting.
- Khi lệnh muốn đảo dấu, giảm yaw rate về 0 trước rồi mới tăng theo chiều ngược lại.
- Khi feedback gimbal stale hoặc tracking invalid, command yaw phải giảm an toàn về 0.
- Không dùng lại biến cấu hình apparent-size với ý nghĩa yaw nếu code hiện tại đang làm vậy; tạo tên cấu hình rõ ràng.
- Loại bỏ hoặc vô hiệu hóa đường legacy body-yaw nếu nó có thể chạy đồng thời.
- Thêm diagnostics:
  - Filtered gimbal yaw.
  - Recenter active.
  - Requested yaw rate.
  - Limited yaw rate.
  - Body-yaw command source.
  - Recenter hold timer.
  - Invalid/stale reason.

Viết unit test cho:

- Không recenter trong stop deadband.
- Chỉ recenter sau hold time.
- Hysteresis không bật/tắt liên tục.
- Slew limit.
- Brake-to-zero trước đảo chiều.
- Mất tracking làm command về 0.
- Gimbal feedback stale làm command về 0.

## V. Tối ưu tracker và FPS

Audit `tracking_hybrid.py` hoặc module tracker tương ứng.

Scale refinement hiện có dấu hiệu chạy nhiều scale candidate trên mỗi frame và gây latency lớn.

Thay đổi đề xuất:

- Scale refinement mặc định chạy mỗi 3 frame.
- Candidate mặc định: `0.92, 1.0, 1.08`.
- Frame không chạy scale refinement vẫn cập nhật vị trí bình thường.
- Khi confidence cao, có thể chạy mỗi 4–5 frame.
- Khi confidence giảm, được phép tạm tăng tần suất.
- Không tạo backlog frame.
- Luôn xử lý frame mới nhất.
- Giữ redetection và validation hiện có.
- Các tham số phải cấu hình được.
- Tránh resize, grayscale hoặc appearance extraction lặp lại không cần thiết trong cùng frame.

Thêm hoặc giữ timing diagnostics cho:

- Camera capture.
- Position tracking.
- Scale refinement.
- Redetection.
- Total tracker time.
- Gimbal control.
- Bearing estimator.
- JPEG/UI encoding.

Không tối ưu bằng cách làm mất toàn bộ scale adaptation. Giữ chế độ adaptive và cho phép A/B disable để benchmark.

Mục tiêu:

- Camera khoảng 30 FPS nếu nguồn cung cấp được.
- Tracking ít nhất 20–25 FPS.
- Không có queue tăng dần.
- Gimbal không bị chặn bởi inference hoặc UI streaming.

## VI. Thêm bearing observation

Tạo module riêng, ví dụ:

```text
bearing_target_estimator.py
```

Có thể chọn tên khác phù hợp với conventions của repository.

Tạo kiểu dữ liệu rõ ràng, ví dụ:

### `BearingObservation`

- `timestamp_s`
- `camera_position_ned_m`: vector 3
- `bearing_ned_unit`: vector 3
- `bbox_center_px`
- `bbox_size_px`
- `tracking_score`
- `pose_age_s`
- `gimbal_age_s`
- `frame_index`
- Tracking flags

### `TargetEstimate`

- `timestamp_s`
- `position_ned_m`
- `velocity_ned_mps`
- `covariance`
- `range_m`
- `range_std_m`
- `reprojection_error_px`
- `baseline_m`
- `intersection_angle_deg`
- `observation_count`
- `observability_score`
- `valid`
- `state`
- `reason`

Pixel-to-bearing:

```text
d_camera = normalize(K^-1 * [u,v,1]^T)
d_ned = R_camera_to_ned * d_camera
```

Yêu cầu:

- Normalize bearing.
- Kiểm tra finite values.
- Kiểm tra camera intrinsics hợp lệ.
- Không hardcode camera resolution hoặc focal length.
- Xác nhận convention trục camera/body/NED trong code hiện tại.
- Viết test hướng nhìn trung tâm và các góc trái/phải/trên/dưới.
- Viết test cho quaternion/rotation convention.
- Không phỏng đoán sign convention; xác minh từ code hiện có.

## VII. Đồng bộ timestamp

Đây là yêu cầu quan trọng.

Không được ghép bbox frame cũ với pose/gimbal “latest” mà không xét timestamp.

Thêm buffer có timestamp cho:

- Drone local position.
- Drone attitude.
- Drone velocity.
- Gimbal angles/quaternion.

Tại timestamp của image frame:

- Interpolate position và velocity tuyến tính.
- Interpolate orientation bằng phương pháp phù hợp, ưu tiên quaternion slerp.
- Reject observation nếu pose/gimbal quá cũ hoặc khoảng nội suy quá lớn.
- Không extrapolate dài một cách im lặng.
- Xuất `pose_age`, `gimbal_age` và invalid reason.

Nếu repository đã có timestamp synchronization thì tái sử dụng, không tạo hệ thống thứ hai không cần thiết.

Viết test cho:

- Exact timestamp.
- Interpolation giữa hai pose.
- Stale telemetry.
- Out-of-order telemetry.
- Thiếu dữ liệu gimbal.
- Clock mismatch hoặc timestamp không hợp lệ.

## VIII. Static multi-ray triangulation

Triển khai estimator ban đầu cho mục tiêu đứng yên hoặc chuyển động chậm.

Với camera position `p_i` và unit bearing `d_i`, giải:

```text
min_p Σ w_i ||(I - d_i d_i^T)(p - p_i)||²
```

Yêu cầu:

- Sliding window khoảng 10–30 observations.
- Trọng số theo tracking score và observation quality.
- Robust outlier rejection.
- Condition-number/rank check.
- Reject hình học suy biến.
- Range phải dương.
- Target không được nằm sau camera.
- Tính baseline lớn nhất giữa các camera positions.
- Tính góc giao bearing hữu ích.
- Tính residual theo mét và reprojection error theo pixel nếu có thể.
- Tính covariance hoặc approximation từ normal matrix và residual.
- Không trả kết quả valid nếu geometry chưa observable.

Ngưỡng mặc định ban đầu:

- Minimum baseline: `0.5 m`
- Minimum useful intersection angle: `3 deg`
- Reprojection error: `< 3 px`
- Minimum observations: khoảng 5–8 observation tốt
- Estimate age: `< 0.5 s`
- Range phải nằm trong giới hạn cấu hình.
- Range standard deviation: `< 1 m` hoặc `< 15% range`

Các ngưỡng phải cấu hình được.

Viết synthetic geometry tests:

1. Target đứng yên, camera dịch ngang.
2. Target ở nhiều khoảng cách khác nhau.
3. Camera chỉ xoay tại chỗ: phải invalid.
4. Camera bay thẳng về target: phải báo observability yếu.
5. Baseline quá nhỏ: invalid.
6. Bearing có noise nhỏ: estimate vẫn ổn định.
7. Một số bearing outlier: robust estimator phải loại.
8. Bearing song song hoặc gần song song: invalid.
9. Target phía sau camera: reject.
10. Timestamp hoặc pose stale: reject.

## IX. Bearing-only filter cho target chuyển động

Sau khi static triangulation hoạt động, thêm filter position-velocity:

```text
x = [pN, pE, pD, vN, vE, vD]
```

Có thể dùng EKF hoặc UKF, chọn phương pháp phù hợp với dependency hiện tại.

Motion model:

- Constant velocity.
- Process noise cấu hình được.
- `dt` lấy từ timestamp, có kiểm tra giới hạn.

Measurement:

- Azimuth/elevation hoặc normalized bearing.
- Measurement noise phụ thuộc tracking score/bbox stability.
- Innovation gate để loại measurement bất thường.

Yêu cầu:

- Bootstrap bằng kết quả multi-ray triangulation.
- Không bootstrap từ một bearing duy nhất.
- Xuất position/velocity covariance.
- Reset/re-bootstrap khi filter divergence.
- Covariance phải tăng khi không có observation tốt.
- Không giữ `valid=true` vô hạn khi target mất.
- Không xuất velocity đáng tin cậy trước khi có đủ lịch sử.
- Static triangulation có thể tiếp tục chạy như consistency check.

Nếu việc thêm EKF vào cùng một patch làm rủi ro quá cao, vẫn phải hoàn thành static estimator và tạo interface sạch cho moving-target filter. Tuy nhiên, ưu tiên hoàn thành cả filter nếu test đảm bảo được.

## X. Observability và state machine

Thêm state machine rõ ràng:

- `IDLE`
- `TRACKING_2D`
- `BOOTSTRAPPING`
- `ESTIMATING`
- `VALID`
- `DEGRADED`
- `LOST`

Chỉ `VALID` khi:

- Tracking không lost/redetect/ambiguous.
- Tracking score đạt ngưỡng.
- Pose và gimbal fresh.
- Baseline đủ.
- Intersection angle đủ.
- Reprojection residual đạt.
- Range dương và trong giới hạn.
- Covariance/range uncertainty đạt.
- Estimate còn fresh.

Khi `DEGRADED` hoặc `LOST`:

- Không publish target position cũ như một measurement mới.
- Không tiếp tục forward motion.
- Gimbal vẫn được phép tracking nếu bbox còn hợp lệ.
- Yaw body phải giảm an toàn nếu feedback invalid.
- Có thể yêu cầu re-bootstrap.
- Expose invalid reason rõ ràng.

Observability score nên phản ánh tối thiểu:

- Baseline.
- Bearing intersection angle.
- Condition number.
- Observation count.
- Residual.
- Pose freshness.

## XI. Active lateral bootstrap

Thiết kế interface/state cho quy trình:

1. Lock bbox.
2. Gimbal giữ target.
3. Không tiến về target.
4. Drone dịch ngang khoảng `0.5–1 m`.
5. Thu bearing trong `0.5–1.5 s`.
6. Triangulate range ban đầu.
7. Chỉ chuyển sang `VALID` nếu covariance đạt.

Yêu cầu an toàn:

- Không tự thực hiện maneuver thật trong quá trình development.
- Viết logic/state và test bằng dữ liệu mô phỏng.
- Bootstrap velocity và distance phải cấu hình được.
- Abort nếu mất target, pose stale, failsafe hoặc residual tăng.
- Rotation-only không được tính là baseline.
- Lateral direction phải dựa trên frame phù hợp và không làm drone lao về target.

Trong lúc Follow Target, duy trì observability bằng một trong các cách:

- Follow angle khoảng `15–30 deg`.
- Thành phần vận tốc ngang nhỏ.
- Hoặc yêu cầu lateral recheck khi uncertainty tăng.

Không tự kích hoạt chuyển động này trên drone thật trong nhiệm vụ hiện tại.

## XII. PX4 Follow Target integration

Tìm và tái sử dụng luồng hiện tại có thể bao gồm:

- Target callback.
- `visual_follow_target`.
- NED-to-WGS84 conversion.
- PX4 target publish.

Thay nguồn target:

```text
Cũ:
bbox ray + UniDepth distance

Mới:
BearingTargetEstimator TargetEstimate
```

Chỉ publish khi:

- `estimate.valid == true`
- State là `VALID`
- Estimate fresh
- Covariance đạt
- Tracking hợp lệ
- Không có failsafe ngăn cản

Publish:

- Target position.
- Target velocity khi đã hội tụ.
- Timestamp đúng.
- Không phát target stale lặp lại như dữ liệu mới.

Tần số dự kiến:

- Follow target publish: 10–20 Hz.
- Body yaw controller: 10–20 Hz.
- Gimbal/tracker: 25–30 Hz.

Khi estimate invalid:

- Ngừng cập nhật target hoặc dùng hành vi invalid đúng với PX4 integration hiện có.
- Command forward phải về 0.
- Không tự động giữ nguyên tọa độ cũ và tiếp tục đuổi.
- Chuyển hover/hold/fallback theo cơ chế an toàn sẵn có.

## XIII. Apparent size và TTC

Giữ apparent-size và time-to-contact làm lớp safety phụ.

Không dùng chúng để tạo khoảng cách mét.

Dùng để:

- Phát hiện bbox tăng kích thước quá nhanh.
- Hạn chế forward speed.
- Emergency brake.
- Kiểm tra consistency với estimator.
- Ngăn tiếp cận nếu range estimator nhảy bất thường.

Nếu code hiện tại tái sử dụng biến apparent-size cho body yaw, tách thành các cấu hình có tên và trách nhiệm riêng.

## XIV. Frontend và diagnostics

Xóa toàn bộ UI UniDepth.

Thêm panel/fields:

- Estimator state.
- Valid/invalid.
- Invalid reason.
- Range.
- Range standard deviation.
- Target NED position.
- Target NED velocity.
- Observation count.
- Baseline.
- Intersection angle.
- Reprojection error.
- Observability score.
- Estimate age.
- Pose age.
- Gimbal telemetry age.
- Bootstrap progress.
- Recenter active.
- Requested/limited body yaw rate.

Không làm frontend crash khi các field chưa có hoặc là `null`.

Dùng định dạng rõ ràng:

- Không hiển thị `0 m` khi estimate invalid.
- Hiển thị `N/A` và reason.
- Phân biệt `VALID`, `DEGRADED` và `LOST`.

## XV. Test và kiểm tra

Bắt buộc chạy:

1. Unit test hiện có liên quan.
2. Unit test mới cho:
   - Pixel-to-bearing.
   - Frame transformations.
   - Timestamp synchronization.
   - Triangulation.
   - Outlier rejection.
   - Degenerate geometry.
   - Covariance/quality gate.
   - Estimator state machine.
   - Body yaw hysteresis.
   - Slew rate.
   - Reversal braking.
   - Tracker scale scheduling.
3. Syntax/import check.
4. Frontend JavaScript syntax/build check nếu project có tooling.
5. Tìm lại toàn repository để xác nhận không còn runtime reference tới:
   - UniDepth.
   - `metric_depth`.
   - `SWARM_METRIC_DEPTH_*`.

Nếu giữ lại tài liệu lịch sử có chữ UniDepth thì báo rõ đó chỉ là tài liệu, không phải runtime reference.

Không tuyên bố FPS đã đạt nếu chưa benchmark runtime. Nếu chỉ unit test/offline test, ghi rõ FPS cần được xác minh trong SITL sau.

## XVI. Tiêu chí hoàn thành

Nhiệm vụ được coi là hoàn thành khi:

- Backend không import, load hoặc chạy UniDepth.
- Frontend không còn hiển thị UniDepth.
- Không còn UniDepth trong control path.
- Tracker scale refinement không chạy toàn bộ candidate trên mọi frame mặc định.
- Gimbal là vòng tracking nhanh.
- Body yaw là vòng recenter chậm với hysteresis và slew limit.
- Chỉ có một body-yaw authority.
- Bearing observation sử dụng đúng intrinsics/extrinsics.
- Pose và gimbal được ghép theo timestamp.
- Rotation-only hoặc baseline yếu không tạo range valid.
- Static multi-view triangulation có test synthetic.
- Estimator xuất position, velocity/capability, covariance và validity.
- Có observability quality gate.
- Follow Target không nhận estimate stale/invalid.
- Có fallback khi mất target hoặc mất observability.
- Unit test mới và test liên quan đều pass.
- Không có thay đổi phá hỏng tính năng không liên quan.

## XVII. Cách báo cáo kết quả

Sau khi hoàn tất, trả lời theo cấu trúc:

1. Kết quả tổng quan.
2. Danh sách file đã sửa/thêm/xóa.
3. UniDepth đã được loại bỏ ở đâu.
4. Kiến trúc bearing estimator đã triển khai.
5. Thay đổi gimbal/body yaw.
6. Thay đổi tối ưu FPS.
7. Quality gate và fallback.
8. Các test đã chạy và kết quả cụ thể.
9. Phần nào chưa thể kiểm chứng nếu chưa chạy SITL.
10. Các tham số cần tune trong SITL.
11. Rủi ro còn lại.
12. `git diff --stat` và tóm tắt diff.

Nếu phát hiện kiến trúc thực tế khác mô tả trên, hãy dùng CodeGraph để xác minh và điều chỉnh cách triển khai cho phù hợp, nhưng phải giữ nguyên các mục tiêu kỹ thuật và safety constraints.

Không chỉ đưa ra đề xuất. Hãy thực hiện thay đổi đầy đủ trong repository và xác minh bằng test offline an toàn.
