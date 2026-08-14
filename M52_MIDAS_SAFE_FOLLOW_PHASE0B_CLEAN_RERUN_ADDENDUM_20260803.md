---
title: "M52 MiDaS Safe Follow - Phase 0B Clean Rerun Addendum"
project: "Swarm Dashboard"
date: "2026-08-03"
status: "phase-0b-no-go"
phase: "P0B isolated read-only rerun"
parent_report: "M52_MIDAS_SAFE_FOLLOW_PHASE0B_GATE_REPORT_20260803.md"
---

# M52 + MiDaS Safe Follow — Phase 0B clean rerun addendum

## Decision

**Phase 0B remains NO-GO.**

An isolated SITL restart removed QGroundControl/browser contamination. A later
RTF-correlated check corrected the initial interpretation of the timing sample:
the lower wall-clock receipt rate matched Gazebo's slower-than-wall simulation
rate and did not show message loss in that window. The clock mapping is still
incomplete, and the gimbal, camera-intrinsics, frame/origin/altitude and
cross-instance parameter gates remain open.

No P1B work is authorized by this rerun.

## Scope and safety record

The operator confirmed both vehicles were disarmed, closed QGroundControl and
closed the dashboard control tab. Read-only checks then confirmed:

```text
UAV-01 armed=false, failsafe=false
UAV-02 armed=false, failsafe=false
tracking_active=false
visual_follow_requested=false
visual_follow_active=false
visual_target_stream_active=false
follow_mode_request_attempts=0
FOLLOW_TARGET publish_count=0 for both vehicles
```

The old stack was stopped only after explicit operator approval. Exact stop:

```bash
kill -INT 17897
```

`run_all.sh` handled child-process cleanup. The clean stack was started with:

```bash
env AMENT_TRACE_SETUP_FILES= \
  AMENT_PYTHON_EXECUTABLE=/usr/bin/python3 \
  COLCON_TRACE= \
  COLCON_PYTHON_EXECUTABLE=/usr/bin/python3 \
  ./run_all.sh
```

New session logs:

```text
/home/sup/swarm_dashboard/artifacts/run_20260803_094502
```

No source code was edited. No arm, tracking start, target beacon, mode request,
`PARAM_SET` or Follow command was sent.

## Isolation result

Before the new measurement, one stale read-only Gazebo topic subscriber from
the previous audit remained alive:

```text
PID 26631
gz-transport-topic -f -d 5 \
  -t /world/default/model/x500_custom_0/link/camera_link/sensor/camera/image
```

It was terminated by exact PID because it could alter camera subscriber load.
After that cleanup, Gazebo discovery and topic introspection worked normally.

No QGroundControl process or established TCP client to port 8000 was present
during the recorded checks.

## Runtime identity

Both PX4 instances used:

```text
/mnt/px4ssd/PX4-Autopilot/build/px4_sitl_default/bin/px4
```

Identity was unchanged:

```text
HEAD: 85df8c2281c2466b30a121b22b0bf33dc69bcfe4
describe: v1.15.4-4-g85df8c2281-dirty
branch: release/1.15
binary SHA-256: c578ccc373dfd6625577d4f58cde6c0b4025958e75e06b50dfe12511e76dca05
```

The source tree remains dirty, so runtime behavior must not be inferred from a
clean upstream tag.

## Read-only Follow parameter result

Six parameters were requested with MAVLink `PARAM_REQUEST_READ`. There was no
`PARAM_SET`.

| Parameter | UAV-01 / sysid 1 | UAV-02 / sysid 2 |
|---|---:|---:|
| `FLW_TGT_DST` | 10.0 | 10.0 |
| `FLW_TGT_HT` | 8.8800001144 | 13.6176214218 |
| `FLW_TGT_FA` | 180.0 | -3.9745514393 |
| `FLW_TGT_ALT_M` | 0 | 0 |
| `FLW_TGT_MAX_VEL` | 1.0 | 0.4000000060 |
| `FLW_TGT_RS` | 0.25 | 0.75 |

The values are unchanged from the previous report and still differ materially
between instances. The same request did not receive `AUTOPILOT_VERSION`
responses within its eight-second window; PX4 identity was therefore verified
from the exact running binaries and source metadata instead.

## Clean timing measurement

The existing read-only subscriber recorded source header timestamp and callback
`time.monotonic()` for six seconds. It did not publish.

| Stream | Samples | Source rate | Receipt rate | Offset span |
|---|---:|---:|---:|---:|
| UAV-02 camera image | 240 | 50.00 Hz | 39.92 Hz | 1206.66 ms |
| UAV-02 camera IMU | 480 | 100.04 Hz | 79.90 Hz | 1207.78 ms |
| `/world/default/pose/info` | 239 | 50.00 Hz | 39.89 Hz | 1205.67 ms |

Receipt timing remained bursty:

| Stream | Receipt median | Receipt p95 | Receipt max |
|---|---:|---:|---:|
| Camera image | 22.11 ms | 42.32 ms | 209.97 ms |
| Camera IMU | 8.26 ms | 33.19 ms | 201.59 ms |
| World pose | 21.42 ms | 43.61 ms | 118.16 ms |

The raw wall-clock rates alone were initially interpreted as subscriber
backlog. The correction below supersedes that interpretation.

### RTF-correlated timing correction

A second six-second read-only sample captured Gazebo `/stats` immediately
before and after the timing subscriber:

```text
stats before: sim=740.404 s, real=926.480291669 s
stats after:  sim=745.392 s, real=932.675338471 s
delta sim=4.988 s
delta real=6.195046802 s
mean real-time factor=0.805159373
```

The corresponding stream sample was:

| Stream | Count | Source span | Source rate | Receipt rate |
|---|---:|---:|---:|---:|
| Camera image | 242 | 4.820 s | 50.00 Hz | 40.30 Hz |
| Camera IMU | 484 | 4.828 s | 100.04 Hz | 80.64 Hz |
| World pose | 241 | 4.800 s | 50.00 Hz | 40.26 Hz |

At the measured source periods, the expected inclusive counts are exactly 242,
484 and 241. Receipt rates also equal source rates multiplied by the observed
RTF within measurement tolerance. Therefore this window contains no evidence
of dropped callbacks or growing subscriber backlog.

The growing `receipt_monotonic - source_sim_time` span is primarily the
difference between wall-clock and slower simulation-time advance. It is not a
latency measurement unless the two clock domains are first mapped through the
Gazebo stats timeline.

The timing gate remains PARTIAL rather than PASS because the current recorder
does not yet provide a persistent sim↔host clock mapping, production queue age,
callback/drop counters or end-to-end vehicle/camera/gimbal alignment. A
latest-frame/non-blocking, RTF-aware instrumentation patch is still required,
but it must not assume that wall-rate below nominal source-rate means backlog.

## Camera contract result

Both runtime `CameraInfo` messages again reported:

```text
width=480, height=270
fx=299.840726852417
fy=299.84072327613831
cx=240, cy=135
distortion=[0,0,0,0,0]
```

Direct sensor introspection again reported:

```text
horizontal_fov=1.35 rad
image=480x270 RGB_INT8
lens Fx=277, Fy=277, Cx=160, Cy=120
sensor pose XYZ=[-0.0412,0,-0.162] m
sensor pose RPY=[0,0,3.14] rad
```

The `CameraInfo` values match the 1.35-radian FOV and 480-pixel width, while
the lens-intrinsics fields do not. The runtime pipeline has not yet explicitly
selected and validated one authoritative contract, so this gate remains open.

## Gimbal authority result

Every roll/pitch/yaw command topic still had two publishers.

| Vehicle | Backend publisher | PX4 publisher |
|---|---|---|
| UAV-01 / `x500_custom_0` | `tcp://192.168.4.164:39671`, PID 48359 | `tcp://192.168.4.164:45801`, PID 47306 |
| UAV-02 / `x500_custom_1` | `tcp://192.168.4.164:39671`, PID 48359 | `tcp://192.168.4.164:36865`, PID 47307 |

The subscriber was Gazebo server PID 47152 at port 33871. This is a direct
single-authority FAIL even with tracking inactive and no external UI client.

## Frame/origin/altitude snapshot

At 2026-08-03T09:48:54+07:00, Gazebo model poses were:

```text
x500_custom_0 XYZ=[0,0,-0.013] m
x500_custom_1 XYZ=[0,5,-0.013] m
```

The nearest dashboard/PX4 snapshot reported:

```text
UAV-01 local NED=[0.50,0.02,-0.40] m
       global altitude_msl=-0.84 m, ellipsoid=0.22 m
UAV-02 local NED=[0.07,0.00,-1.15] m
       global altitude_msl=0.07 m, ellipsoid=0.12 m
```

Both vehicles were disarmed and nav state 2. Horizontal spawn mapping is still
only coarsely consistent; vertical local/global offsets differ between the two
instances and are not explained by one documented origin/altitude mapping.
Home/reset behavior and synchronized NED→WGS84→NED error remain unmeasured.

## Updated gate matrix

| Required gate | Clean rerun result |
|---|---|
| Isolated baseline, no QGC/browser control | PASS |
| Both vehicles disarmed, tracking/Follow disabled | PASS |
| Exact PX4 runtime identity | PASS |
| Read-only `FLW_TGT_*` values | PASS evidence; configuration mismatch remains |
| Runtime camera profile | PARTIAL; authority conflict unresolved |
| Sensor capture versus callback timing | PARTIAL; RTF-normalized sample shows no loss, clock/queue mapping incomplete |
| Vehicle/camera/gimbal clock alignment | FAIL |
| Optical axes/quaternion/ENU↔NED proof | NOT CLOSED |
| Local/global origin and AMSL convention | FAIL |
| Single gimbal authority | FAIL |
| Measured-target shadow packet inspection | NOT RUN |
| Follow-mode jerk fixture | NOT RUN; no approval requested |
| Follow remained disabled | PASS |

## Required next action

Phase 0B cannot be closed using further repetitions of the same read-only
subscriber alone. The next proposed work is a separately reviewed Phase 0B
instrumentation/authority patch, not P1B:

1. Add latest-frame/non-blocking timing instrumentation with queue/drop/age
   accounting and negligible payload copying.
2. Define and enforce one gimbal authority/arbitration contract.
3. Make runtime `CameraInfo` the explicit camera contract or reject it with an
   evidence-backed alternative.
4. Add synchronized Gazebo/PX4 local/global/home/altitude capture and known-ray
   fixtures.
5. Rerun read-only Phase 0B after those changes.

Measured-target shadow traffic requires a separate exact protocol and approval
because it writes outbound MAVLink. Follow entry remains prohibited until all
preceding gates pass and the operator approves an exact entry/exit command.
