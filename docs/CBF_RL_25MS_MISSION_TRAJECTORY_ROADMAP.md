# Roadmap mission trajectory và CBF-RL tới 25 m/s

**Trạng thái:** tài liệu thiết kế và kế hoạch triển khai, chưa sửa source/config
**Ngày:** 2026-08-15
**Repository:** /home/sup/swarm_dashboard
**Mục tiêu:** hai UAV x500 nhận mission path từ dashboard, tự giảm tốc ở góc cua, né nhau an toàn và quay lại bám sát quỹ đạo; tốc độ cruise mục tiêu tối đa 25 m/s.

## 1. Giả định và quyết định thiết kế

Tài liệu hiểu yêu cầu “25m/2” là **25 m/s**, không phải gia tốc 25 m/s².

25 m/s phải được hiểu là **cruise speed ceiling**:

- drone được phép đạt 25 m/s trên đoạn thẳng đủ dài;
- drone phải tự giảm tốc trước góc cua;
- drone phải giảm tốc sớm hơn nữa khi có xung đột;
- mission không được hứa rằng drone luôn giữ 25 m/s;
- UI chỉ được cho phép tốc độ đã được server/runtime chứng nhận.

Không giao đồng thời việc bám đường, xử lý góc cua và bảo đảm an toàn cho RL. Kiến trúc đích:

~~~text
Mission từ dashboard
  -> validation hình học
  -> fillet/bo góc + speed profile theo arclength
  -> dự báo xung đột mission-với-mission
  -> trajectory tracker
  -> conflict coordinator
  -> RL residual avoidance
  -> acceleration-aware CBF shield
  -> PX4 velocity setpoint
~~~

Phân công trách nhiệm:

| Tầng | Trách nhiệm |
|---|---|
| Mission validator | Kiểm tra waypoint, geofence, self-intersection, độ dài đoạn và tốc độ đã chứng nhận |
| Path/speed planner | Tạo đường liên tục, giảm tốc theo curvature, quãng phanh và jerk |
| Mission conflict planner | Dự báo time-to-conflict, ưu tiên temporal deconfliction |
| Trajectory tracker | Bám path theo measured progress, không chạy theo wall-clock |
| Conflict coordinator | Gán quyền ưu tiên/yield role, phá đối xứng và deadlock |
| RL policy | Tạo residual nhỏ để né và tối ưu progress/cross-track |
| CBF shield | Bảo đảm dynamic safety margin và giới hạn lệnh khả thi |
| PX4 | Thực hiện velocity command với giới hạn plant, tilt, thrust, acceleration và jerk |

Nguyên tắc quan trọng:

1. Safety do CBF/failsafe chịu trách nhiệm, không dựa vào xác suất RL.
2. Corner handling thuộc path planner, không thuộc RL.
3. Nếu hai mission đã biết trước, ưu tiên điều chỉnh thời gian đến giao điểm thay vì bay lệch khỏi đường.
4. Model của một speed contract không được dùng ở speed contract cao hơn.
5. 20 m/s là milestone production-candidate; 25 m/s là milestone experimental cho tới khi PX4 và plant được chứng minh.

## 2. Baseline hiện có

### 2.1 Mission pipeline

Source hiện tại đã có:

- mission_path từ UI/WebSocket;
- validation lại ở server và bridge;
- MQTT MissionInbox;
- runtime trajectory install;
- ClosedPolylineTrajectory;
- nearest-point entry;
- position-progress follower;
- hard mission speed ceiling;
- acceleration limiting;
- corner speed profile;
- CBF deterministic gate;
- authenticated CBF-RL shadow/active mode;
- model SHA-256 và active contract validation.

Các file chính:

- static/index.html
- main.py
- mission_plan.py
- mavlink_manual_bridge.py
- companion_safety.py
- trajectory_controller.py
- conflict_coordinator.py
- cbf_command_gate.py
- cbf_rl_env.py
- cbf_rl_policy.py
- cbf_rl_train.py
- cbf_rl_shadow.py
- cbf_rl_x500_safety_matrix.py

### 2.2 Bằng chứng 15 m/s

Baseline hiện tại đã có model:

- cbf_rl_policy_x500_20m_15ms_v1.json
- maximum velocity 15 m/s;
- minimum physical separation 20 m;
- policy được train bằng seeded cross-entropy optimizer;
- 6 tham số policy được tối ưu;
- model được khóa bằng SHA-256 khi chạy active.

Artifact chính:

- artifacts/cbf_rl_x500_20m_1_to_15ms_safety_matrix.json
- artifacts/cbf_rl_x500_20m_15ms_active_trajectory_offline.json
- artifacts/cbf_rl_x500_20m_15ms_sitl_evaluation.json
- artifacts/two_uav_x500_20m_15ms_head_on_flight.json
- artifacts/two_uav_x500_20m_15ms_corner_circle_flight.json

Kết quả đáng chú ý:

| Bằng chứng | Kết quả |
|---|---:|
| Safety matrix 1–15 m/s | PASS |
| Số case | 630 |
| Failure count | 0 |
| Minimum physical distance offline | 27.554 m |
| Minimum dynamic margin offline | 5.028 m |
| Active SITL trajectory evaluation | PASS |
| Head-on flight | PASS |
| Circle/corner flight | PASS |
| Circle flight max tracking error | 1.469 m |

Giới hạn của bằng chứng circle:

- hai drone cách nhau khoảng 139 m;
- CBF không can thiệp;
- đây là bằng chứng path/corner tracking, không phải bằng chứng “vừa cua vừa né”.

### 2.3 Train hiện tại chưa phải deep-RL

cbf_rl_train.py hiện dùng seeded cross-entropy optimization:

- sampling population quanh mean;
- rollout từng candidate;
- chọn elite;
- cập nhật mean/deviation;
- lặp theo generation;
- policy cuối cùng vẫn là công thức 6 tham số có thể giải thích.

Đây là baseline phù hợp vì:

- ít dependency;
- dễ audit;
- inference rẻ;
- dễ khóa contract;
- đã có bằng chứng đến 15 m/s.

Không thay bằng PPO/SAC/MAPPO trước khi baseline này thực sự không đạt yêu cầu.

## 3. Đặc tả mục tiêu

### 3.1 Speed contract

Các rung chứng nhận:

~~~text
15 m/s baseline
17 m/s
20 m/s production-candidate
22 m/s experimental
25 m/s experimental
~~~

Mỗi rung phải có:

- model riêng;
- config riêng;
- SHA-256 riêng;
- training report;
- offline matrix;
- shadow report;
- SITL flight report;
- ULog/trace;
- tag hoặc commit xác định.

Không cho phép:

- model 15 m/s chạy ở 20 hoặc 25 m/s;
- tăng UI max trước server/runtime;
- tăng CBF max mà không tăng đồng bộ PX4/tracker/evaluator;
- dùng một chuyến bay PASS để suy ra toàn bộ envelope.

### 3.2 PX4 ceiling

PX4 Parameter Reference hiện công bố MPC_XY_VEL_MAX là giới hạn ngang tuyệt đối cho velocity-controlled modes và miền tham số tối đa là 20 m/s.

Tham số phải audit:

- MPC_XY_VEL_MAX;
- MPC_ACC_HOR;
- MPC_ACC_HOR_MAX;
- MPC_JERK_AUTO;
- MPC_JERK_MAX;
- MPC_TILTMAX_AIR;
- MPC_THR_MAX;
- MPC_THR_HOVER;
- MPC_THR_XY_MARG;
- MPC_XY_VEL_P_ACC;
- COM_OF_LOSS_T;
- COM_OBL_RC_ACT;
- COM_RCL_EXCEPT;
- COM_RC_OVERRIDE.

Kết luận:

- tới 20 m/s: có thể đi theo parameter contract chính thức;
- trên 20 m/s: phải xác minh firmware thực tế;
- nếu parameter schema từ chối 25 m/s, cần PX4 fork/build riêng;
- việc mở range tham số không tự chứng minh x500 có thể bay ổn định ở 25 m/s.

Tham khảo:

- https://docs.px4.io/main/en/advanced_config/parameter_reference
- https://docs.px4.io/main/en/flight_modes/offboard

### 3.3 Tracking contract

Mục tiêu đề xuất:

| Điều kiện | Tiêu chí |
|---|---|
| Đoạn thẳng/curve không conflict | cross-track p95 <= 1 m |
| Toàn flight không conflict | cross-track max <= 2 m |
| Góc cua | không vượt configured corner tolerance |
| Trong avoidance | không ra ngoài avoidance corridor đã định |
| Sau conflict | reacquire path hữu hạn, không branch jump |
| Speed | actual không vượt local planned speed quá tolerance |
| Progress | >= 90% no-conflict baseline |
| Deadlock | không yield vô hạn |

Các ngưỡng phải được chốt lại sau single-UAV envelope. Nếu product yêu cầu corner tolerance nhỏ hơn khoảng cách drone đi trong một control frame thì phải tăng control rate hoặc giảm tốc mạnh hơn.

### 3.4 Safety contract

Đề xuất ban đầu:

- normal physical floor: 20 m;
- emergency absolute floor: 10 m;
- dynamic trigger distance: tính theo closing speed, latency, braking, uncertainty và tracking error;
- không coi 10–20 m là một band có upper bound;
- khoảng cách lớn hơn 20 m không phải lỗi.

Chỉ xem xét adaptive base 10–20 m sau khi fixed 20 m đã PASS toàn bộ tới certified speed.

## 4. Khoảng cách an toàn động

### 4.1 Công thức

Với hai drone:

~~~text
d_required =
    d_base
  + d_uncertainty
  + v_close * (t_state_age + t_command)
  + v_close^2 / (2 * a_relative_braking)
  + d_tracking
~~~

Trong đó:

- d_base: physical floor, giai đoạn đầu là 20 m;
- d_uncertainty: covariance reserve;
- v_close: radial closing speed, không phải full relative norm khi đang bay tiếp tuyến;
- t_state_age: tuổi telemetry/peer state;
- t_command: measured worst-case command latency;
- a_relative_braking: tổng deceleration bảo đảm của hai drone;
- d_tracking: tracking/model reserve.

CBF solve target có thể cộng design buffer:

~~~text
d_solve = d_required + d_design_buffer
~~~

Reported margin phải giữ ý nghĩa:

~~~text
dynamic_margin = actual_distance - d_required
~~~

Không cộng design buffer vào reported margin.

### 4.2 Ví dụ 25 m/s

Giữ contract hiện tại:

- d_base = 20 m;
- t_command = 0.65 s;
- worst peer age = 0.15 s;
- a_relative_braking = 6 m/s²;
- d_tracking = 2 m;
- design buffer = 5 m;
- covariance reserve xấp xỉ 0.055 m.

| Encounter | Closing speed | d_required | d_solve |
|---:|---:|---:|---:|
| 15° | 6.53 m/s | 30.8 m | 35.8 m |
| 30° | 12.94 m/s | 46.4 m | 51.4 m |
| 45° | 19.13 m/s | 67.9 m | 72.9 m |
| 90° | 35.36 m/s | 154.5 m | 159.5 m |
| 180° | 50.00 m/s | 270.4 m | 275.4 m |

Hệ quả:

- “né ở 20 m” là quá muộn ở tốc độ cao;
- mission conflict predictor phải nhìn trước hàng trăm mét;
- head-on 25 m/s cần test arena/geofence đủ lớn;
- nếu measured braking thấp hơn 3 m/s² mỗi drone, d_required còn lớn hơn;
- chỉ được giảm d_required khi có bằng chứng plant tốt hơn, không chỉnh bằng cảm giác.

### 4.3 Adaptive base 10–20 m

Chỉ cân nhắc sau khi fixed 20 m PASS:

~~~text
d_base(v_close) =
    10 m                                  khi non-closing/rất chậm
    ramp liên tục từ 10 tới 20 m          khi closing tăng
    20 m                                  ở high closing speed
~~~

Adaptive base không loại bỏ:

- latency reserve;
- braking reserve;
- covariance;
- tracking reserve;
- design buffer.

10 m chỉ là physical base ở điều kiện thấp, không phải dynamic required distance.

## 5. Corner-aware path và speed planning

### 5.1 Vấn đề hình học

Polyline có đổi hướng tức thời tại waypoint. Ở nonzero speed, góc lý tưởng có curvature vô hạn và không thể bay chính xác.

Cần chọn một trong hai semantics:

1. **Fillet semantics:** cho phép bo trong corner tolerance.
2. **Exact waypoint semantics:** giảm gần 0 m/s, đi qua waypoint rồi đổi hướng.

Mặc định nên dùng fillet. Exact mode chỉ dùng khi mission thật sự yêu cầu chạm waypoint.

### 5.2 Curvature speed

Speed limit tại curvature:

~~~text
v_curve = sqrt(a_lateral_allowed / abs(curvature))
~~~

Với cung tròn:

~~~text
v_curve = sqrt(a_lateral_allowed * radius)
~~~

Nếu dùng safety factor:

~~~text
v_curve = safety_factor * sqrt(a_lateral_allowed * radius)
~~~

Controller hiện tại dùng safety factor gần 0.85.

Ví dụ a_lateral = 3 m/s²:

| Bán kính | Speed vật lý sqrt(a*r) | Với factor 0.85 |
|---:|---:|---:|
| 3.4 m | 3.2 m/s | 2.7 m/s |
| 20 m | 7.7 m/s | 6.6 m/s |
| 70 m | 14.5 m/s | 12.3 m/s |
| 125 m | 19.4 m/s | 16.5 m/s |
| 208 m | 25.0 m/s | 21.3 m/s |

Muốn giữ đủ 25 m/s tại 3 m/s² cần bán kính xấp xỉ 208 m trước safety factor. Do đó góc gắt phải giảm tốc mạnh.

### 5.3 Speed profile theo arclength

Khi mission được install:

1. Chuyển waypoint thành path segments.
2. Phát hiện corner angle.
3. Tính fillet radius từ corner tolerance và độ dài hai cạnh.
4. Sampling path theo arclength.
5. Tính curvature limit.
6. Backward pass theo braking.
7. Forward pass theo acceleration.
8. Jerk smoothing.
9. Lưu local speed ceiling theo arclength.

Backward pass:

~~~text
v_i <= sqrt(v_next^2 + 2 * a_brake * delta_s)
~~~

Forward pass:

~~~text
v_next <= sqrt(v_i^2 + 2 * a_accel * delta_s)
~~~

Final local limit:

~~~text
v_profile(s) = min(
    mission_cruise_speed,
    certified_vehicle_speed,
    curvature_speed,
    braking_pass_speed,
    acceleration_pass_speed,
    conflict_schedule_speed
)
~~~

### 5.4 Mission validation

Reject mission khi:

- ít hơn 3 waypoint cho closed mission;
- waypoint ngoài geofence;
- segment zero/too short;
- speed vượt certified max;
- không đủ quãng phanh trước exact corner;
- self-intersection trong phiên bản đầu;
- path đi xuyên vùng cấm;
- path/time profile tạo conflict không thể schedule.

Không cố hỗ trợ self-intersecting freehand path ngay. Nearest-point progress có thể nhảy sang nhánh khác tại giao điểm. Giải pháp ban đầu nhỏ nhất và an toàn nhất là reject.

## 6. Mission-level deconfliction

### 6.1 Tại sao cần

Reactive avoidance ở 25 m/s có thể quá muộn và làm drone lệch path nhiều. Hai mission đã biết trước cho phép giải quyết phần lớn conflict bằng thời gian.

### 6.2 Workflow

Khi mission được gửi:

1. Time-parameterize path theo speed profile.
2. So sánh từng cặp mission.
3. Tìm closest approach và time-to-conflict.
4. Tính dynamic required separation tại thời điểm đó.
5. Nếu conflict, gán deterministic priority.
6. Giảm tốc/hold drone yield ở upstream safe point.
7. Recompute timing.
8. Chỉ accept khi schedule mới không conflict hoặc có explicit runtime avoidance plan.

### 6.3 Ưu tiên chiến lược

Thứ tự:

1. Time separation: giảm tốc/hold, giữ nguyên geometry.
2. Lateral offset: lệch ngang có giới hạn.
3. Vertical offset: chỉ khi altitude/geofence/vertical dynamics cho phép.
4. Emergency stop/hold/land: fail-safe.

Temporal deconfliction giữ path fidelity tốt nhất.

### 6.4 Priority rule

Không để hai policy tự quyết định đối xứng.

Priority có thể dựa trên:

- mission ID;
- drone ID ổn định;
- earliest arrival;
- payload priority đã validate;
- drone đang ở trong critical segment được ưu tiên đi tiếp.

Role phải latch trong một encounter và có hysteresis trước khi release.

## 7. Acceleration-aware CBF

### 7.1 Vấn đề hiện tại

Safety gate chọn velocity, nhưng vehicle không đổi vận tốc tức thời. Environment đã mô phỏng lag/acceleration, trong khi runtime solver vẫn có thể tạo candidate ngoài reachable set của frame kế tiếp.

Không được:

~~~text
safe velocity từ CBF
  -> post-hoc acceleration limiter
~~~

Post-hoc limiter có thể không đạt safe velocity mà CBF đã giả định.

### 7.2 Hướng sửa tối thiểu

Giữ velocity MAVLink interface nhưng thêm reachable constraints ngay trong solver:

~~~text
norm(v_next - v_current) <= a_guaranteed * dt
~~~

Nếu có jerk state:

~~~text
norm(a_next - a_previous) <= j_guaranteed * dt
~~~

Solver phải đồng thời thỏa:

- maximum velocity;
- acceleration-reachable velocity;
- geofence;
- pairwise barrier constraints;
- optional jerk bound.

### 7.3 Khi nào cần high-order CBF

Chuyển sang discrete-time/high-order CBF nếu:

- reachable-velocity CBF vẫn infeasible ở các case hợp lệ;
- dynamic reserve quá bảo thủ làm mất progress;
- plant lag/acceleration thay đổi làm velocity-level guarantee không ổn định;
- command output oscillate gần barrier;
- post-flight ULog cho thấy achieved motion khác mạnh safety model.

Không nâng kiến trúc trước khi measurement chứng minh cần.

## 8. RL policy v2 đề xuất

### 8.1 Giới hạn policy hiện tại

Policy hiện tại sinh normalized ENU velocity mới. Active runtime cap magnitude theo deterministic speed, nhưng hướng vẫn có thể khác trajectory command đáng kể.

Điều này có thể:

- làm cross-track tăng;
- chậm reacquire;
- xử lý corner và collision cùng lúc không nhất quán;
- khiến policy học lại path following;
- tạo hành vi khó phân tích.

### 8.2 Residual policy

Policy v2 xuất residual trong local path/Frenet frame:

~~~text
action = [
    delta_v_along,
    delta_v_lateral,
    delta_v_vertical
]
~~~

Chuyển thành ENU:

~~~text
v_candidate = v_path + rotate_path_to_enu(action)
~~~

Ràng buộc:

- zero residual khi không có predicted conflict;
- bounded residual;
- không được tăng magnitude quá local speed ceiling;
- deterministic coordinator quyết định yield role và passing side;
- CBF shield luôn là lớp cuối.

### 8.3 Observation

Observation tối thiểu:

| Nhóm | Field |
|---|---|
| Path | tangent, normal, cross-track error, along-track progress |
| Speed plan | local speed ceiling, distance-to-corner, local curvature |
| Own state | velocity trong path frame, acceleration nếu có |
| Peer | relative position/velocity trong own path frame |
| Risk | closing speed, dynamic required distance, dynamic margin |
| Quality | own/peer age, covariance, validity |
| Coordination | yield role, encounter phase |

Không cần đưa toàn bộ mission waypoint list vào actor.

### 8.4 Reward

Safety phải là hard gate/termination, sau đó mới tối ưu reward:

~~~text
reward =
    + progress_reward
    - cross_track_penalty
    - local_speed_error_penalty
    - residual_magnitude_penalty
    - cbf_intervention_penalty
    - jerk_penalty
    - role_flip_penalty
    - deadlock_penalty
    + path_reacquisition_bonus
~~~

Hard failure:

- physical floor violation;
- negative dynamic margin;
- invalid/nonfinite action;
- missing required peer/covariance;
- solver infeasible;
- mission timeout/deadlock.

Không reward việc “giữ gần 20 m”; policy không được học chủ động tiếp cận peer.

### 8.5 Deep-RL escalation

Chỉ dùng deep-RL nếu CEM/residual heuristic không đạt đồng thời safety, progress và tracking.

Nếu cần:

- shared actor cho hai drone;
- centralized critic khi train;
- decentralized observation khi inference;
- PPO/MAPPO cho continuous residual action;
- CBF shield trong mọi rollout;
- model selection theo worst-case hoặc CVaR;
- deterministic runtime inference;
- model version/hash/contract mới.

Không đưa deep-RL dependency vào trước khi có failure artifact chứng minh 6-parameter policy không đủ.

## 9. Single-UAV plant envelope

### 9.1 Mục tiêu

Đo khả năng thật trước khi train 20–25 m/s.

### 9.2 Speed ladder

~~~text
15 -> 17 -> 20 m/s
experimental PX4: 22 -> 25 m/s
~~~

Không bỏ qua rung.

### 9.3 Bài test mỗi rung

1. Straight acceleration.
2. Straight braking.
3. Velocity square wave.
4. 30° turn.
5. 60° turn.
6. 90° turn.
7. Circle R=50 m.
8. Circle R=70 m.
9. Circle R=100 m.
10. Circle R=150 m.
11. Circle R=200 m.
12. Horizontal speed kết hợp climb/descent.
13. Emergency POSCTL/hold transition.
14. Setpoint stream interruption dry/fault gate ở tốc độ an toàn hơn.

### 9.4 Measurement

Ghi từ companion trace và ULog:

- commanded/actual velocity;
- acceleration/deceleration;
- jerk;
- turn radius;
- cross-track;
- response tau/gain;
- command latency;
- telemetry age;
- covariance;
- tilt;
- thrust;
- altitude;
- failsafe flags;
- Offboard gaps;
- Gazebo real-time factor.

### 9.5 Chọn tham số bảo thủ

Không dùng mean braking. Dùng guaranteed value từ nhiều run:

- lower percentile của measured braking;
- upper percentile của response lag;
- upper percentile của command latency;
- upper percentile của tracking error;
- upper percentile của state age.

Những giá trị này tạo speed contract.

### 9.6 Gate

PASS rung tốc độ khi:

- không failsafe;
- không thrust saturation kéo dài;
- altitude giữ trong tolerance;
- speed tracking không runaway;
- braking lặp lại được;
- setpoint gaps nằm trong contract;
- RTF đủ ổn định;
- land/disarm hoàn chỉnh.

Nếu 20 m/s fail plant gate thì không mở PX4 25 m/s.

## 10. Training curriculum

### 10.1 Model ladder

Mỗi speed rung có model riêng:

| Model | Speed contract | Điều kiện mở |
|---|---:|---|
| v15 | 15 m/s | Baseline đã có |
| v17 | 17 m/s | Single-UAV v17 PASS |
| v20 | 20 m/s | Single-UAV v20 PASS |
| v22 | 22 m/s | PX4 experimental + plant v22 PASS |
| v25 | 25 m/s | PX4 experimental + plant v25 PASS |

### 10.2 CEM training levels

**Smoke**

- 3 generations;
- population 12;
- elite 3;
- 1–2 seed;
- mục tiêu: phát hiện plumbing/config lỗi;
- không dùng model smoke để bay.

**Pilot**

- 12 generations;
- population 24;
- elite 6;
- 5 seed độc lập;
- tương đương mức baseline hiện tại;
- mục tiêu: xác định feasibility và failure modes.

**Final**

- 24–30 generations;
- population 48;
- elite 12;
- giữ 3–5 seed tốt nhất;
- early stop nếu holdout không cải thiện nhiều generation;
- chọn theo safety trước, worst-case progress/tracking sau.

Không chọn chỉ theo tổng reward.

### 10.3 Geometry curriculum

Thứ tự:

1. Parallel no-conflict.
2. Parallel unequal speed.
3. Overtaking.
4. Stationary intruder.
5. Crossing 90°.
6. Crossing 45°.
7. Crossing 30°.
8. Shallow crossing 15°.
9. Head-on.
10. Vertical swap.
11. Same orbit close phase.
12. Same orbit safe phase.
13. Opposite orbit.
14. Conflict tại corner.
15. Conflict ngay sau corner.
16. One drone braking while peer turns.
17. Non-convex simple path.
18. Multiple repeated encounters.

### 10.4 Speed curriculum

Không train v25 từ đầu:

~~~text
5 -> 10 -> 15 -> 17 -> 20 -> 22 -> 25 m/s
~~~

Ở mỗi level:

- randomize speed từ level trước tới current level;
- giữ một phần replay của các level cũ;
- không cho model quên low-speed behavior;
- full regression lại toàn bộ speed 1..current.

### 10.5 Domain randomization

Chỉ randomize trong measured/justified range:

- response tau;
- acceleration/braking;
- command latency;
- peer age;
- covariance;
- mass/thrust variation;
- wind;
- sensor noise;
- setpoint jitter;
- initial velocity;
- asymmetric vehicles;
- route phase;
- corner position;
- conflict time;
- brief telemetry delay/dropout dưới fail-safe bound.

Fault vượt contract phải fail closed, không bắt policy “học chịu”.

### 10.6 Training seeds

Đề xuất seed cố định để reproducible:

~~~text
7, 17, 29, 43, 61
~~~

Mỗi report phải ghi:

- git commit;
- config fingerprint;
- seed;
- scenarios;
- generation/population/elite;
- workers;
- start/end time;
- best parameters;
- per-scenario result;
- model SHA-256;
- environment/PX4 contract.

## 11. Offline evaluation

### 11.1 Deterministic safety matrix

Ma trận hiện tại:

~~~text
15 speeds * 14 geometries * 3 dynamics variants = 630 cases
~~~

Mục tiêu 25:

~~~text
25 speeds * 14 geometries * 3 dynamics variants = 1050 cases
~~~

Các geometry:

- horizontal angle 0..180 theo bước 15°;
- vertical 180°.

Dynamics variants hiện có:

- tau 0.45 s, age 0 ms;
- tau 0.75 s, age 100 ms;
- tau 0.75 s, age 150 ms.

### 11.2 Holdout matrix

Không train trên toàn bộ test:

- angles 7.5°, 22.5°, 37.5°, 52.5°, 67.5°;
- intermediate speeds;
- unseen tau;
- unseen peer age;
- asymmetric acceleration;
- different covariance profiles;
- crossing at corner;
- short route after conflict;
- re-entry after avoidance.

### 11.3 PASS criteria

Hard:

- physical_safe = true;
- dynamic_safe = true;
- hold_frames = 0;
- reached_goals/progress = true;
- action finite and bounded;
- no solver infeasibility;
- no invalid required state.

Tracking:

- no-conflict residual gần 0;
- cross-track trong contract;
- local speed ceiling được giữ;
- reacquire path;
- no branch jump;
- no deadlock.

Performance:

- progress >= 90% baseline;
- intervention không kéo dài không cần thiết;
- không yield vô hạn;
- inference nằm trong control-loop budget.

### 11.4 Counterfactual checks

Mỗi candidate nên so:

1. deterministic tracker only;
2. deterministic tracker + coordinator;
3. RL nominal/residual không shield;
4. RL + CBF shield;
5. CBF shield không RL.

Mục tiêu là xác định RL thực sự thêm giá trị gì, không chỉ PASS nhờ CBF.

## 12. SITL rollout

### 12.1 Nguyên tắc

- một thay đổi mỗi flight;
- một speed rung mỗi lần;
- head-on cuối cùng;
- shadow trước active;
- luôn land/disarm;
- dừng stack rồi mới phân tích nặng;
- không train/analyze lớn trên cùng máy trong lúc bay;
- giữ trace append-only và ULog.

### 12.2 Thứ tự

**A. Single UAV**

1. RL off, straight.
2. RL off, circle.
3. RL off, sharp corner.
4. RL shadow, no peer conflict.
5. Xác nhận residual gần 0 và path không đổi.

**B. Two UAV, no conflict**

1. parallel paths;
2. large separation circles;
3. different speed;
4. repeated laps.

**C. Shadow conflict**

1. 90° crossing;
2. 45°;
3. 30°;
4. 15°;
5. head-on;
6. corner conflict.

**D. Active conflict**

Lặp theo speed:

~~~text
5 -> 10 -> 15 -> 17 -> 20 -> 22 -> 25 m/s
~~~

Mỗi mức:

- crossing trước;
- shallow crossing;
- overtaking;
- orbit;
- corner conflict;
- head-on cuối.

### 12.3 Flight precheck

Trước arming:

- cả hai UAV disarmed;
- preflight checks pass;
- correct PX4 instance/system ID;
- model path đúng;
- SHA-256 đúng;
- active ACK đúng;
- speed contract khớp;
- CBF contract khớp;
- covariance present/required;
- peer stream fresh;
- Offboard stream warmed up;
- no abort conditions;
- geofence đủ lớn;
- mission đã install trước OFFBOARD;
- conflict schedule review PASS.

### 12.4 Continuous guard

Abort khi:

- PX4 failsafe;
- sender latch;
- state invalid/stale;
- covariance missing;
- negative dynamic margin;
- physical floor violation;
- speed vượt envelope;
- altitude/geofence violation;
- setpoint gap;
- model contract mismatch;
- trajectory branch jump;
- maximum test duration;
- telemetry/trace silence.

### 12.5 Control frequency

PX4 Offboard chỉ yêu cầu stream lớn hơn khoảng 2 Hz để giữ mode, nhưng performance yêu cầu cao hơn.

Ở 25 m/s:

| Rate | Khoảng đường mỗi frame |
|---:|---:|
| 20 Hz | 1.25 m |
| 30 Hz | 0.83 m |
| 50 Hz | 0.50 m |

Nếu corner tolerance gần 1 m:

- benchmark 20 Hz trước;
- nếu discretization chi phối error, thử 50 Hz;
- chỉ tăng khi CPU, setpoint jitter và Gazebo RTF vẫn PASS;
- CBF dt, training env dt và runtime cadence phải thống nhất.

## 13. UI và operator workflow

UI nên tách:

- requested cruise speed;
- certified maximum speed;
- planned local speed;
- corner tolerance;
- predicted conflict;
- dynamic required separation;
- yield role;
- model/runtime mode.

Map nên hiển thị:

- path màu theo speed profile;
- braking zones;
- corner slowdown;
- conflict point;
- predicted arrival time;
- physical floor circle;
- dynamic trigger radius;
- lý do server reject mission.

Operator workflow:

1. Vẽ mission.
2. Chọn requested cruise speed.
3. Server validate/plan.
4. UI hiển thị actual planned speed profile.
5. Review multi-mission conflicts.
6. Gửi/install.
7. Start mission.
8. Theo dõi dynamic margin, tracking và role.
9. Stop/POSCTL/land.

Không để UI hiển thị “25 m/s accepted” nếu server thực tế clamp về 20 m/s.

## 14. Artifact và reproducibility

Mỗi milestone tạo thư mục:

~~~text
artifacts/cbf_rl_x500_20m_<speed>ms_<date>/
  training_report.json
  model.json
  model.sha256
  safety_matrix.json
  holdout_matrix.json
  path_tracking_matrix.json
  shadow_evaluation.json
  active_sitl_evaluation.json
  flight_summary.json
  companion_trace.jsonl
  ulog_manifest.json
  environment_snapshot.env
  git_commit.txt
~~~

Không overwrite artifact của rung trước.

Model format phải chứa:

- format/version;
- observation fields;
- action meaning;
- minimum separation;
- maximum velocity;
- vehicle profile;
- training metadata;
- parameters;
- parameter SHA;
- source commit;
- config fingerprint.

## 15. File impact dự kiến

Chưa sửa ở giai đoạn viết tài liệu. Khi triển khai, vùng ảnh hưởng dự kiến:

| File | Thay đổi dự kiến |
|---|---|
| mission_plan.py | Validation segment/braking/self-intersection, certified speed |
| trajectory_controller.py | Precomputed arclength speed profile, fillet/exact corner mode |
| companion_safety.py | Dùng planned speed, residual policy, richer telemetry |
| conflict_coordinator.py | Mission time scheduling, latched priority |
| cbf_command_gate.py | Reachable velocity/acceleration constraints, adaptive base sau validation |
| cbf_rl_env.py | Path-relative observations, residual action, plant randomization |
| cbf_rl_policy.py | Policy v2 format và action contract |
| cbf_rl_train.py | Curriculum, multi-seed selection, worst-case metrics |
| cbf_rl_shadow.py | Policy v2 runtime validation và fallback |
| cbf_rl_x500_safety_matrix.py | 1–25 m/s + holdout/path matrices |
| main.py | Mission review/verdict/config snapshot |
| static/index.html | Speed preview, corner/conflict visualization |
| flight drivers | Single-UAV envelope và staged conflict gates |
| test_*.py | Contract/regression tests tương ứng |

Giữ diff theo từng phase, không thay toàn bộ các file trong một commit.

## 16. Phase roadmap và gate

### Phase 0 — Repository baseline

Việc:

- giải quyết local/upstream root history;
- chọn canonical tree;
- full regression;
- tag baseline 15 m/s;
- lưu model/config/artifact hashes.

Gate:

- clean commit;
- baseline reproducible;
- no active flight configuration khi rời máy.

### Phase 1 — Single-UAV envelope tới 20 m/s

Việc:

- 17 và 20 m/s straight/brake/circle/corner;
- system identification;
- parameter audit.

Gate:

- plant PASS;
- conservative braking/lag/latency contract được chốt.

### Phase 2 — Corner speed planner

Việc:

- fillet/exact semantics;
- arclength profile;
- forward/backward pass;
- jerk smoothing;
- mission validation.

Gate:

- offline path matrix PASS;
- single-UAV SITL path PASS tới 20 m/s.

### Phase 3 — Mission conflict scheduling

Việc:

- wire live mission review;
- time-to-conflict;
- deterministic priority;
- temporal deconfliction.

Gate:

- known-path conflicts được xử lý trước runtime CBF;
- no deadlock;
- path geometry giữ nguyên khi slowdown đủ.

### Phase 4 — Acceleration-aware CBF

Việc:

- reachable velocity constraint;
- measured dynamic reserve;
- solver/evaluation updates.

Gate:

- deterministic safety matrix PASS tới 20 m/s;
- no post-shield unsafe slew.

### Phase 5 — Train 17/20 m/s

Việc:

- CEM smoke/pilot/final;
- 5 seed;
- offline + holdout;
- shadow + staged active.

Gate:

- safety, progress và tracking cùng PASS;
- model/auth contract versioned.

### Phase 6 — Residual policy v2

Chỉ mở nếu Phase 5 chứng minh policy hiện tại làm path deviation/deadlock quá mức.

Việc:

- path-relative observation;
- residual action;
- zero-residual no-conflict gate;
- active contract v2.

Gate:

- tốt hơn baseline trên holdout;
- không giảm safety;
- không tăng runtime jitter.

### Phase 7 — PX4 experimental 22/25 m/s

Việc:

- verify/fork PX4 parameter ceiling;
- rebuild;
- single-UAV envelope;
- speed planner validation;
- CBF contract recalculation.

Gate:

- 25 m/s plant PASS;
- no silent clamp;
- no thrust/tilt/altitude failure.

### Phase 8 — Train và validate 22/25 m/s

Việc:

- per-speed models;
- 1050-case matrix;
- holdout;
- shadow;
- staged active;
- head-on cuối.

Gate:

- toàn bộ acceptance criteria PASS;
- landed/disarmed;
- artifact audit hoàn chỉnh.

## 17. Acceptance checklist cuối

### Safety

- [ ] Physical floor không bị vi phạm.
- [ ] Dynamic margin không âm.
- [ ] Không CBF infeasible/hold frame.
- [ ] Không missing covariance khi contract yêu cầu.
- [ ] Không stale peer.
- [ ] Không sender latch/failsafe.
- [ ] Model/hash/contract khớp.

### Tracking

- [ ] Cruise speed đạt trên đoạn đủ dài.
- [ ] Corner speed tự giảm theo profile.
- [ ] Cross-track đạt contract.
- [ ] Không branch jump.
- [ ] Reacquire path sau avoidance.
- [ ] RL residual gần 0 khi không conflict.

### Liveness

- [ ] Cả hai mission có progress.
- [ ] Không deadlock.
- [ ] Không yield vô hạn.
- [ ] Không orbit avoidance vô hạn.
- [ ] Mission hoàn tất hoặc lap progress đạt gate.

### Runtime

- [ ] Control cadence ổn định.
- [ ] Setpoint gap nằm trong contract.
- [ ] Gazebo RTF đạt gate.
- [ ] Inference nằm trong time budget.
- [ ] POSCTL/land fallback hoạt động.
- [ ] Mọi flight kết thúc landed/disarmed.

### Reproducibility

- [ ] Git commit/tag lưu lại.
- [ ] Config fingerprint lưu lại.
- [ ] Model SHA lưu lại.
- [ ] Training seeds lưu lại.
- [ ] Safety/holdout matrix lưu lại.
- [ ] Trace và ULog manifest lưu lại.

## 18. Thứ tự thực hiện khuyến nghị

~~~text
1. Chốt Git và baseline 15 m/s
2. Single-UAV envelope 17/20 m/s
3. Corner speed planner
4. Live mission conflict scheduling
5. Acceleration-aware CBF
6. Train/revalidate 17/20 m/s
7. Chỉ khi cần: residual policy v2/deep-RL
8. Verify hoặc fork PX4 cho >20 m/s
9. Single-UAV 22/25 m/s
10. Train 22/25 m/s
11. Offline 1050-case + holdout
12. Shadow
13. Staged active SITL
14. Head-on 25 m/s cuối cùng
~~~

## 19. Việc nên làm đầu tiên ở phiên triển khai

Không bắt đầu bằng train 25 m/s.

Tranche đầu tiên:

1. Resolve Git history và tag baseline.
2. Tạo một báo cáo single-UAV 17/20 m/s.
3. Đo conservative acceleration/braking/lag/latency.
4. Chốt tracking/corner tolerance.
5. Thiết kế arclength speed profile.
6. Nối review_missions vào live install.

Chỉ sau sáu việc trên mới đủ dữ liệu để khóa safety contract cho train 20–25 m/s.

## 20. Tài liệu tham khảo

- PX4 Parameter Reference: https://docs.px4.io/main/en/advanced_config/parameter_reference
- PX4 Offboard Mode: https://docs.px4.io/main/en/flight_modes/offboard
- Safe Reinforcement Learning Filter for Multicopter Collision-Free Tracking under Disturbances: https://arxiv.org/abs/2410.06852
- Safe Multi-Agent Reinforcement Learning through Decentralized Multiple Control Barrier Functions: https://arxiv.org/abs/2103.12553
