---
title: "M52 MiDaS Safe Follow - Phase 0B Gate Report 2026-08-03"
project: "Swarm Dashboard"
date: "2026-08-03"
status: "phase-0b-no-go"
phase: "P0B runtime evidence"
---

# M52 + MiDaS Safe Follow — Phase 0B gate report

## Decision

**Phase 0B: NO-GO.**

P1A static/unit containment vẫn PASS, nhưng runtime gate chưa thể đóng. Không
triển khai P1B và không yêu cầu/bật PX4 Follow mode từ session này.

Các blocker trực tiếp:

1. Camera/Gazebo source chạy 50/100 Hz nhưng read-only subscriber chỉ xử lý
   khoảng 31/62 Hz và trễ tương đối tăng khoảng 2.24 s trong cửa sổ 6 s.
2. Mỗi UAV có đồng thời hai publisher trên cả ba Gazebo gimbal command topics:
   dashboard backend và PX4 SITL.
3. Local/global origin, home reset và AMSL convention chưa được chứng minh;
   snapshot Gazebo/PX4 có offset dọc chưa giải thích được.
4. Không có valid measured-target stream, vì vậy runtime `FOLLOW_TARGET`
   fields/rate/gaps chưa được đo; publish count/rate vẫn bằng 0.
5. Jerk fixture không chạy vì không bật Follow mode; first-setpoint delta,
   peak velocity/acceleration, overshoot/settling và bbox retention chưa có.
6. Runtime session bị external UI/QGroundControl tương tác trong lúc thu
   evidence nên motion/altitude snapshot không phải controlled baseline.

## Safety protocol và phạm vi

```text
Phase/Patch: P0B runtime evidence + jerk fixture
Files planned: report/fixture only; không sửa estimator/control
Evidence: static P1A pass, runtime evidence còn thiếu
Invariants: no PX4 write, no real mode request, tracker/gimbal retained
Safety impact: read-only first; SITL only
Migration/rollback: additive fixture/report, không xóa log
Acceptance gate: clock/frame/altitude conventions resolved; baseline replayable
```

Đã giữ các invariant sau:

- Không gửi `PARAM_SET`.
- Không gửi arm/takeoff/land/control command.
- Không gọi `/api/tracking/follow`.
- Không gửi PX4 Follow mode request.
- Không phát valid `FOLLOW_TARGET` beacon.
- Không sửa PX4 source, firmware hoặc build output.
- Không sửa estimator/control source.
- Chỉ thêm báo cáo này; runtime tự tạo log session mới.

## P1A integrity và regression

Workspace root không phải Git working tree. Vì vậy SHA có thể xác minh ở đây là
SHA-256 của bảy file P1A, không phải repository commit SHA.

| File | SHA-256 | Result |
|---|---|---|
| `mavlink_manual_bridge.py` | `906658277980620d9c43787a6bef81c486f7761f86f552bf213ef9de7ed6d960` | match |
| `tracking_web.py` | `5da7d17e4fe8592b32ea9420d02887e24df6890062c8d05ee341a2e5d8a6ef77` | match |
| `main.py` | `1009e161840a974b70d9bdd37751c3030f77568b6b7f1fdf5c32d29ff06a93ce` | match |
| `.env.example` | `662ca924620b87cff47883d736ceaa85e5c3bcdaf7225f1f476a58a1e36a554c` | match |
| `test_mavlink_attitude_bridge.py` | `110661c721751792ae923d550a3ff683f7cd40b92d7cc2c04634bbbdbe480d24` | match |
| `test_follow_workflow.py` | `fa31efc78bffa0b6c47024379b8e1691db9f699ffb4b73ba069a185db7baabcd` | match |
| `test_tracking_motion_altitude_hold.py` | `f0bba3ab4b46a8944bae2ea564f0c5954776098b0111fd960a13676459700280` | match |

Rerun results:

```text
Targeted P1A: 68 passed in 0.60s
Full regression: 192 passed, 1 warning in 1.52s
```

Warning duy nhất vẫn là `PytestReturnNotNoneWarning` có sẵn tại
`test_bearing_target_estimator.py::test_config`.

Static forbidden-symbol check cho
`param_set_send|send_visual_follow_manual_heartbeat|SAFE_NATIVE_FOLLOW_NEUTRAL_HEARTBEAT_ENABLED|send_centered_controls`
trả về 0 match.

SHA được kiểm tra lại sau runtime collection và vẫn khớp.

## Controlled SITL instance

Successful session log directory:

```text
/home/sup/swarm_dashboard/artifacts/run_20260803_085956
```

`run_all.sh` cần bốn biến setup rỗng/explicit vì ROS Jazzy và colcon setup
không tương thích trực tiếp với caller dùng `set -u`:

```bash
env AMENT_TRACE_SETUP_FILES= \
  AMENT_PYTHON_EXECUTABLE=/usr/bin/python3 \
  COLCON_TRACE= \
  COLCON_PYTHON_EXECUTABLE=/usr/bin/python3 \
  ./run_all.sh
```

Không có file nào được sửa để workaround. Successful stack gồm Gazebo server,
Gazebo GUI, PX4 `-i 0`, PX4 `-i 1`, MicroXRCEAgent, ROS telemetry, MAVLink
bridge và web backend.

Safe defaults được load từ `.env.example`; native Follow, Follow mode request,
OFFBOARD tracking, body yaw, apparent-size follow và legacy automation đều off.

## Exact PX4 build/runtime identity

Cả hai running process dùng cùng binary:

```text
/mnt/px4ssd/PX4-Autopilot/build/px4_sitl_default/bin/px4
```

Build identity:

```text
PX4_GIT_VERSION_STR: 85df8c2281c2466b30a121b22b0bf33dc69bcfe4
PX4_GIT_TAG_STR: v1.15.4-4-g85df8c2281-dirty
PX4_GIT_BRANCH_NAME: release/1.15
PX4 binary SHA-256: c578ccc373dfd6625577d4f58cde6c0b4025958e75e06b50dfe12511e76dca05
```

Source tree `HEAD` bằng đúng build commit nhưng tree là dirty. Không được suy
diễn behavior từ clean upstream tag.

## Read-only `FLW_TGT_*` values

Values được lấy bằng `PARAM_REQUEST_READ` riêng cho system 1 và 2 qua
localhost QGC proxy. Runtime bridge status sau đó chứa cùng values.

| Parameter | UAV-01 / sysid 1 | UAV-02 / sysid 2 |
|---|---:|---:|
| `FLW_TGT_DST` | 10.0 | 10.0 |
| `FLW_TGT_HT` | 8.8800001144 | 13.6176214218 |
| `FLW_TGT_FA` | 180.0 | -3.9745514393 |
| `FLW_TGT_ALT_M` | 0 | 0 |
| `FLW_TGT_MAX_VEL` | 1.0 | 0.4000000060 |
| `FLW_TGT_RS` | 0.25 | 0.75 |

Đây là blocker cấu hình: hai instances không có cùng Follow geometry/dynamics,
đặc biệt `HT`, `FA`, `MAX_VEL` và `RS`. P1A chỉ đọc/validate, không sửa các
values này.

## Runtime camera profile

Gazebo `CameraInfo` của cả hai UAV:

```text
width=480, height=270
fx=299.840726852417
fy=299.84072327613831
cx=240, cy=135
distortion=[0,0,0,0,0]
projection uses the same fx/fy/cx/cy
```

Image message:

```text
width=480, height=270, step=1440, pixelFormatType=RGB_INT8
source period=20 ms (50 Hz)
```

Sensor introspection reports `horizontal_fov=1.35 rad`, matching configured
profile and `CameraInfo` focal length. However the same introspection also
prints lens-intrinsics defaults `Fx=277, Fy=277, Cx=160, Cy=120`, which conflict
with the actual 480x270 `CameraInfo`. The runtime pipeline must explicitly name
which artifact is authoritative and reject the other; gate is not closed by an
implicit assumption.

Camera and camera-IMU sensor pose reported by Gazebo:

```text
XYZ=[-0.0412, 0, -0.162] m
RPY=[0, 0, 3.14] rad
```

This records the installed transform but does not yet prove center/corner ray
signs or the full optical-axis convention against known ground intersections.

## Clock/rate/delay evidence

A separate read-only Gazebo subscriber recorded source header timestamps and
callback `time.monotonic()` for 6 seconds without publishing.

| Stream | Samples | Source rate | Receipt rate | Receipt-source offset span |
|---|---:|---:|---:|---:|
| UAV-02 camera image | 187 | 50.00 Hz | 31.19 Hz | 2243.56 ms |
| UAV-02 camera IMU | 374 | 100.05 Hz | 62.39 Hz | 2250.68 ms |
| `/world/default/pose/info` | 189 | 50.54 Hz | 31.54 Hz | 2240.31 ms |

Camera source periods were exactly 20 ms; camera-IMU source periods were
8–12 ms. Receipt periods had large burst/jitter:

```text
camera receipt median=29.43 ms, p95=52.43 ms, max=119.92 ms
camera IMU receipt median=8.75 ms, p95=40.03 ms, max=123.89 ms
pose receipt median=28.42 ms, p95=52.55 ms, max=122.60 ms
```

The absolute offset contains the host-boot-monotonic versus Gazebo-sim epoch
and is not itself latency. Its 2.24–2.25 s growth over the window is evidence
that this subscriber did not keep up with source time. Dashboard status during
external tracking corroborated approximately 31 Hz source/tracking processing,
despite the sensor producing 50 Hz.

This measurement is sufficient to reject a runtime timing PASS. It is not yet
a causal attribution: full-image Python callback work and concurrent external
tracking/QGC load can both contribute. A clean recorder must quantify queue
depth/drops and latest-frame behavior without itself building backlog.

PX4 fast-pose MQTT evidence showed about 20 Hz per vehicle with explicit
`source_timestamp_us`, `quaternion_xyzw` and local/attitude source ages. Sampled
ages alternated around 0 and 50 ms. This does not resolve the Gazebo sensor
clock to PX4 source clock offset.

## Frame, origin and altitude evidence

Observed horizontal mapping is consistent only at a coarse level with:

```text
PX4 local North ≈ Gazebo Y minus per-instance spawn Y
PX4 local East  ≈ Gazebo X minus per-instance spawn X
PX4 local Down  ≈ negative Gazebo Z plus an unresolved origin offset
```

Examples were not captured from a clean, synchronized snapshot. During the
session, external flight/control activity occurred, and PX4 local Down versus
Gazebo Z differed by more than one metre in snapshots. Global `altitude_msl_m`,
ellipsoid altitude and Gazebo Z also did not provide one documented constant
mapping across both instances.

Therefore the following remain unresolved:

- per-instance local origin epoch;
- home reset behavior;
- ground/home plane offset;
- AMSL versus ellipsoid/Gazebo Z mapping;
- synchronized round-trip NED → WGS84 → NED error;
- known-ray optical-axis and ENU↔NED sign proof.

## Gimbal authority audit

All roll/pitch/yaw command topics have two simultaneous publishers.

```text
Dashboard backend Gazebo Transport address: tcp://192.168.4.164:37957
UAV-01 PX4 address:                       tcp://192.168.4.164:43393
UAV-02 PX4 address:                       tcp://192.168.4.164:40165
```

Socket ownership mapped `37957` to web backend PID 19158, `43393` to PX4
instance PID 18065, and `40165` to PX4 instance PID 18066.

This is a direct Phase 0B gate failure. Native Follow was not required to
observe the conflict; enabling it before arbitration is unsafe.

## Follow-target and mode-entry evidence

Initial controlled snapshot:

```text
both UAVs armed=false
nav_state=2
tracking inactive
follow workflow IDLE
native_visual_follow_enabled=false
mode_request_attempts=0
FOLLOW_TARGET publish_count=0, publish_rate_hz=0
```

Later snapshot still showed:

```text
native_visual_follow_enabled=false
visual_follow_requested=false
visual_follow_active=false
visual_target_stream_active=false
mode_request_attempts=0
FOLLOW_TARGET publish_count=0, publish_rate_hz=0
```

Consequently runtime `FOLLOW_TARGET` field mapping/rate/gap inspection and
mode-entry jerk metrics are **NOT RUN**, not inferred as pass.

## External-interaction contamination

At 09:05 local time, QGroundControl was started outside this evidence process.
Browser/UI traffic also issued:

```text
POST /api/tracking/start
POST /api/tracking/stop
POST /api/tracking/start
POST /api/tracking/bbox
```

Later snapshots showed both vehicles armed, UAV-02 tracking session 3 in
`POINTING`, and UAV-01/UAV-02 in non-identical nav states. These actions were
not issued by the Phase 0B read-only commands.

Containment did remain effective for the legacy path: the MAVLink bridge
rejected 6477 `offboard_follow` messages during the observed window. No
`PARAM_SET`, production `param_set_send`, logged Follow mode request or valid
`FOLLOW_TARGET` send was found. The rejection storm is itself a logging/load
problem to address before a clean timing baseline.

Because external control changed the vehicle/tracking state, motion, altitude,
bbox and handover observations from this session cannot qualify as the required
controlled regression fixture.

## Gate matrix

| Required gate | Result |
|---|---|
| P1A SHA/static/unit integrity | PASS |
| Exact PX4 build/runtime identity | PASS, dirty build recorded |
| Read-only `FLW_TGT_*` values | PASS, but cross-instance mismatch found |
| Runtime camera resolution/FOV/K/distortion | PARTIAL, CameraInfo captured; introspection conflict unresolved |
| Sensor capture vs callback monotonic | FAIL, growing backlog/offset span |
| Vehicle/camera/gimbal delay and clock offset | FAIL |
| Optical axes/quaternion/ENU↔NED proof | PARTIAL/FAIL |
| Local/global origin and AMSL convention | FAIL |
| `FOLLOW_TARGET` fields/rate/gaps | NOT RUN |
| Single gimbal authority | FAIL, two publishers per axis |
| Controlled baseline/replayable jerk fixture | NOT RUN / contaminated |
| Follow mode remained disabled by this session | PASS |

## Recommended next action

Recommended option: **rerun P0B in an isolated session before any Follow-mode
approval**.

Required conditions:

1. Close/disconnect QGroundControl and browser control clients, or explicitly
   reserve them as read-only observers.
2. Start a new session ID/log directory; do not overwrite this evidence.
3. Eliminate or arbitrate the simultaneous PX4/dashboard gimbal publishers.
4. Use a latest-frame/non-blocking timing recorder that records source stamp,
   receipt monotonic, drops and queue age without copying every image payload.
5. Capture synchronized Gazebo pose, PX4 local/global/home and altitude fields
   at known stationary points and after a documented home/reset event.
6. Resolve the CameraInfo versus introspection intrinsics contract.
7. Obtain a measured-target shadow stream and inspect `FOLLOW_TARGET` packets
   without entering Follow mode.
8. Only after those gates pass, present an exact SITL Follow-entry command,
   vehicle/current nav state, safe exit command and expected impact for user
   approval before collecting the jerk fixture.

Alternative options are to keep this session only as a containment/load
diagnostic, or stop at NO-GO and first fix the orchestration/gimbal/timing
instrumentation under a separately reviewed Phase 0B patch. Neither option
authorizes P1B.

## Runtime left running

At report time the controlled stack was intentionally **not stopped**, because
QGroundControl/browser clients were actively interacting with it. Stopping it
without coordination could interrupt the user's SITL activity. No Follow mode
was enabled by this evidence session.

## P0B-A read-only instrumentation update — 2026-08-03 10:45 +07:00

This section supersedes the earlier camera/timing interpretation where the two
conflict. It does not change the overall Phase 0B decision: **NO-GO**.

### Patch scope and integrity

Only the approved additive collector, its tests and this report were changed:

```text
phase0b_runtime_evidence.py
test_phase0b_runtime_evidence.py
M52_MIDAS_SAFE_FOLLOW_PHASE0B_GATE_REPORT_20260803.md
```

No production estimator/control file, `main.py`, `tracking_web.py`,
`mavlink_manual_bridge.py`, PX4 source or firmware was changed. The seven P1A
SHA-256 values were checked again and all still match the handoff.

Final verification:

```text
P1A targeted:        68 passed in 0.62s
P0B-A collector:      8 passed in 0.03s
Full regression:    200 passed, 1 known warning in 1.61s

phase0b_runtime_evidence.py
  fc74278a208fd2d239607335f51ba6f9f0d9140b451caf02fc04f554f354e7b2
test_phase0b_runtime_evidence.py
  2af46a10f29151d9b27a4cd0e4e0a1a5cf5bf3152e92e237845c5977b90de727
```

The collector AST test rejects calls named `advertise`, `publish`,
`publish_raw`, `param_set_send`, `command_long_send`, `follow_target_send` or
`write`. Runtime operations were limited to Gazebo
`subscribe/topic_info/unsubscribe` and HTTP `GET /api/drones`. Camera callbacks
recorded header/dimension metadata and observed payload length but copied zero
image payload bytes.

### Authoritative evidence artifact

Final isolated session:

```text
/home/sup/swarm_dashboard/artifacts/run_20260803_104512
```

Final evidence:

```text
/home/sup/swarm_dashboard/artifacts/phase0b_runtime_evidence_20260803_104547.json
SHA-256 d2e4a050bdb26af6906eec6797aa84b209ce2ad6ba7ad0ad56cfe9e058963418
```

The earlier development artifact `phase0b_runtime_evidence_20260803_104336.json`
is superseded because receipt-rate normalization initially used Gazebo
`real_time`. The final collector correctly uses `delta_sim / delta_host`, since
receipt intervals are measured with host monotonic time; it records
`delta_sim / delta_real` separately.

### Safety evidence

Preflight, all 24 periodic API samples and postflight agreed:

```text
UAV-01 armed=false, failsafe=false
UAV-02 armed=false, failsafe=false
tracking_active=false
visual_follow_requested=false
visual_follow_active=false
visual_target_stream_active=false
follow_mode_request_attempts=0
FOLLOW_TARGET enabled=false, publish_count=0, mode_request_attempts=0
```

No tracking start, target stream, parameter write, mode request, Follow entry
or HTTP control POST occurred. The stack was stopped after the postflight
check; no P0B-A SITL/PX4/Gazebo process was left running.

### RTF-normalized timing result

Gazebo stats established:

```text
delta_sim=9.980000 s
delta_host=11.985901 s
normalization_rtf=0.832644945
clock mapping=affine sim-to-host from stats endpoints
```

| Stream | Callbacks | Source Hz | Receipt Hz | Expected receipt Hz at RTF | Ratio | Missing/duplicate/out-of-order |
|---|---:|---:|---:|---:|---:|---:|
| UAV-01 camera | 499 | 50.000 | 41.556 | 41.632 | 0.9982 | 0 / 0 / 0 |
| UAV-01 camera IMU | 999 | 100.000 | 83.262 | 83.264 | 1.0000 | 0 / 0 / 0 |
| UAV-02 camera | 499 | 50.000 | 41.594 | 41.632 | 0.9991 | 0 / 0 / 0 |
| UAV-02 camera IMU | 999 | 100.000 | 83.261 | 83.264 | 1.0000 | 0 / 0 / 0 |
| World pose | 499 | 50.000 | 41.594 | 41.632 | 0.9991 | 0 / 0 / 0 |

No metadata sample was overwritten. The collector therefore found no evidence
of subscriber backlog or callback loss in this window. Gazebo Python does not
expose transport-internal queue age, so that metric remains explicitly
unavailable rather than being inferred from cross-domain timestamp offsets.

### CameraInfo contract result

Both vehicles passed the collector's explicit authority
`gazebo_camera_info`:

```text
width=480, height=270
fx=299.840726852417, fy=299.8407232761383
cx=240, cy=135
distortion=[0,0,0,0,0]
expected horizontal FOV=1.35 rad
```

This closes the P0B-A evidence check for the runtime message. Production code
does not yet enforce that authority, so adoption of this contract outside the
collector remains a separate reviewed change.

### Gimbal authority and gate decision

Gazebo `topic_info` reported two publishers before and after collection on all
six command topics: roll/pitch/yaw for both `x500_custom_0` and
`x500_custom_1`. No command topic was published by the collector.

Final gate result recorded by the artifact:

```text
P0B-A result: FAIL
Phase 0B decision: NO-GO
P1B authorized: false
Follow mode authorized: false
```

Direct blocker: dual gimbal authority. Vehicle/camera/gimbal clock alignment,
frame/origin/altitude, measured-target shadow traffic and the Follow-mode jerk
fixture also remain open. The last two require separate protocols; Follow mode
still requires presentation and approval of an exact entry/exit command.
