# Visual Follow Target bằng camera RGB

## Kiến trúc hiện tại

```text
RGB frame + bbox
  → pixel-to-bearing
  → camera quaternion đồng bộ theo timestamp
  → bearing NED
  → robust multi-view triangulation
  → bearing-only EKF [position, velocity]
  → covariance/observability gate
  → NED-to-WGS84
  → PX4 Follow Target
```

Khoảng cách mét chỉ được suy ra từ chuyển động tịnh tiến của camera và giao
của nhiều bearing. Hệ thống không giả định kích thước, loại hay hình dạng của
vật. Xoay camera/drone tại chỗ không tạo baseline và không được phép sinh
range hợp lệ.

## Điều kiện để publish Follow Target

Estimator chỉ trả `VALID` khi đồng thời đạt:

- tracker ổn định, không redetect/ambiguous;
- pose và camera quaternion đủ mới, được ghép tại timestamp của frame;
- tối thiểu 6 observation;
- baseline mặc định ít nhất `0.5 m`;
- góc giao bearing mặc định ít nhất `3°`;
- reprojection error dưới `3 px`;
- range dương, nằm trong giới hạn;
- covariance/range standard deviation đạt quality gate.

Khi `BOOTSTRAPPING`, `DEGRADED` hoặc `LOST`, target cũ không được phát lại như
measurement mới và forward motion bị dừng.

## Bootstrap

Dashboard hiển thị progress và `bootstrap_guidance`. Chỉ sau khi người vận
hành nhấn `BẮT ĐẦU FOLLOW`, chọn trái/phải và xác nhận hành lang trống, safety
layer mới phát một setpoint Offboard local-NED hợp nhất: dịch ngang, yaw và
giữ cao độ. Xoay tại chỗ không đủ để bootstrap.

## State machine và motion authority

```text
SELECTING
  → POINTING
  → POINTING_STABLE
  → READY_FOR_FOLLOW
  → RGB_BOOTSTRAP
  → TARGET_3D_READY
  → FOLLOW_PRESTREAM
  → FOLLOW_MODE_REQUESTED
  → FOLLOWING
```

`FOLLOWING` chỉ được xác nhận khi PX4 báo `nav_state == 19`. Quyền chuyển động:

- pointing: Position/Hold hoặc unified Offboard yaw, translation bằng 0;
- RGB bootstrap: một unified Offboard local-NED duy nhất;
- prestream: không có controller cạnh tranh;
- native Follow: PX4 là authority duy nhất;
- lỗi/HOLD: neutral rồi Position/Hold.

Mọi lỗi tracking, pose, gimbal, estimator, target stream, failsafe hoặc mode
handover đều đi tới `HOLD`; bbox/gimbal vẫn được phép tiếp tục nếu measurement
còn hợp lệ. Không tự chọn mục tiêu hoặc tự bắt đầu Follow lại.

## Gimbal và body yaw

- Gimbal là inner loop nhanh, bám tâm bbox.
- Body yaw là outer loop chậm, chủ yếu dùng gimbal yaw feedback.
- Body recenter bật sau `|gimbal yaw| > 3.75°` liên tục `0.12 s`.
- Dừng recenter khi xuống dưới `1.25°`.
- Giới hạn `8°/s`, slew `40°/s²`.
- Bbox feed-forward vào body mặc định bằng `0`.
- Khi đảo hướng, controller phanh về 0 trước.
- Khi mất tracker hoặc gimbal feedback stale, yaw giảm an toàn về 0.

## Apparent size và TTC

Apparent size không tạo khoảng cách mét. Nó chỉ theo dõi scale ratio và
time-to-contact để chặn forward motion khi bbox nở quá nhanh hoặc nhảy bất
thường.

## Xác minh

Unit test/offline test kiểm tra triangulation, geometry suy biến, outlier,
moving-target EKF, timestamp interpolation/SLERP, yaw hysteresis/slew/reversal
và scale-refinement scheduling. FPS và ngưỡng điều khiển vẫn phải được tune
trong SITL. Không polling endpoint trạng thái lớn ở 10–20 Hz trong một process
backend duy nhất; dùng telemetry/trace riêng để tránh tranh lock với frame
pipeline.
