# Báo cáo triển khai RGB Bearing Follow Target

## Kết quả

- Runtime và frontend không còn import, load, queue hoặc hiển thị UniDepth.
- Khoảng cách tuyệt đối đến vật không biết kích thước được suy ra bằng
  multi-view bearing triangulation.
- EKF constant-velocity xuất target position/velocity NED và covariance.
- PX4 Follow Target chỉ nhận target khi estimator `VALID`.
- Apparent size chỉ còn là scale/TTC safety guard, không tạo mét.
- Gimbal là vòng nhanh; body yaw là vòng recenter chậm có hysteresis, lọc,
  slew limit và brake-before-reverse.
- Scale refinement mặc định chạy mỗi 3 frame với ba candidate
  `0.92,1.0,1.08`.

## Timestamp synchronization

Telemetry position/velocity và camera quaternion được buffer trong cùng clock
monotonic với camera frame. Position/velocity nội suy tuyến tính; quaternion
dùng SLERP. Sample stale, out-of-order hoặc khoảng nội suy quá lớn bị reject.

## Estimator diagnostics

API/UI xuất state, invalid reason, range, range standard deviation, position,
velocity, covariance, baseline, intersection angle, reprojection error,
observation count, estimate age, pose/gimbal age, observability score và
bootstrap progress.

## Safety

Estimator không tự thực hiện lateral bootstrap. `bootstrap_guidance` luôn có
`command_authorized=false`; safety layer/người vận hành phải chọn hướng dịch
ngang. Rotation-only và straight-closing geometry không tạo range valid.

## Xác minh offline

- `python -m unittest discover -v`: 99 test pass.
- Python syntax/import checks pass.
- Inline frontend JavaScript passes `node --check`.
- Không restart backend/PX4/Gazebo và không gửi flight command.

FPS thực tế chưa được tuyên bố vì chưa benchmark runtime/SITL sau thay đổi.
