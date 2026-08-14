---
title: "M52 MiDaS Safe Follow - Phase 0B Gimbal Authority Addendum"
project: "Swarm Dashboard"
date: "2026-08-03"
status: "p0b-b-partial"
phase: "P0B-B read-only gimbal authority attribution"
parent_report: "M52_MIDAS_SAFE_FOLLOW_PHASE0B_GATE_REPORT_20260803.md"
---

# Phase 0B P0B-B — gimbal authority attribution

## Decision

**P0B-B is PARTIAL. Phase 0B remains NO-GO.**

The existence of two Gazebo Transport publishers on each gimbal command topic
does **not**, by itself, prove an active command conflict. In the controlled
idle sample below, all six topics received zero messages. The earlier Phase 0B
interpretation that publisher count alone is a direct runtime conflict is
therefore superseded by this addendum.

The remaining authority question is narrower: PX4 intentionally publishes a
new gimbal target when its PX4/QGroundControl target changes, while dashboard
tracking publishes direct Gazebo targets at its tracking command rate. Their
overlap during active tracking was not exercised in this read-only session.
Consequently active handover/override behavior remains open.

No P1B work is authorized by this result. Follow mode was not requested or
entered.

## Scope and safety record

No production source was edited. The only repository change from this session
is this evidence document.

The stack was started with the same controlled command used by the clean P0B
rerun:

```bash
env AMENT_TRACE_SETUP_FILES= \
  AMENT_PYTHON_EXECUTABLE=/usr/bin/python3 \
  COLCON_TRACE= \
  COLCON_PYTHON_EXECUTABLE=/usr/bin/python3 \
  ./run_all.sh
```

Session logs:

```text
/home/sup/swarm_dashboard/artifacts/run_20260803_110431
```

The session performed only Gazebo topic discovery/subscribe, HTTP GET, process
and socket inspection, source inspection, hashes, and filesystem diagnostics.
It did not call an API POST, publish a Gazebo message, send a MAVLink command,
arm a vehicle, start tracking, select a bounding box, or enable Follow.

Final runtime safety snapshot before shutdown:

```text
UAV-01 armed=false, failsafe=false, nav_state=2
UAV-02 armed=false, failsafe=false, nav_state=2
tracking active=false, state=inactive
gimbal_active=false, gimbal_command_fps=0.0
follow_workflow_state=IDLE
native_visual_follow_enabled=false
visual_follow_requested=false
visual_follow_active=false
follow_mode_request_attempts=0
follow_target_publish_rate_hz=0.0
```

The complete stack was stopped by sending `Ctrl+C` to the same `run_all.sh`
terminal. Postflight process and port inspection found no PX4, Gazebo,
MicroXRCEAgent, ROS launch, MAVLink bridge, web backend, port 8000 listener or
port 8888 listener left running by this session.

## CodeGraph and static attribution

CodeGraph was used before searching or reading code.

Dashboard `GazeboDashboardBridge.start()` advertises roll, pitch and yaw
publishers for both vehicles at backend startup. It does not send a value at
advertise time. `GazeboDashboardBridge.publish_gimbal()` sends a
`gz.msgs.Double` only when called by the active tracking gimbal controller.
The tracking path requires a tracking result with a bounding box; inactive,
no-bbox and reset paths do not call this publisher.

PX4's modified `GZMixingInterfaceServo` also advertises all three topics at
initialization. `publishGimbalOutput()` publishes only when the corresponding
PX4 gimbal output changes. It caches the last value and suppresses repeats
within `1e-12`. Its source comment and the accompanying local integration guide
explicitly state the intent: a direct Gazebo command persists until a genuinely
new PX4/QGroundControl target appears.

The custom airframe maps:

```text
SIM_GZ_SV_FUNC1=420  # Gimbal Roll
SIM_GZ_SV_FUNC2=421  # Gimbal Pitch
SIM_GZ_SV_FUNC3=422  # Gimbal Yaw
MNT_MODE_IN=4        # MAVLink Gimbal Protocol v2 input
MNT_MODE_OUT=0       # AUX/gimbal_controls output path
```

This is a deliberate last-new-target-wins coexistence design, not continuous
dual-stream arbitration.

## Runtime endpoint attribution

All three axes of a vehicle share the same publisher endpoints.

| Vehicle | PX4 publisher | Dashboard publisher | Gazebo subscriber |
|---|---|---|---|
| UAV-01 / `x500_custom_0` | `tcp://192.168.4.164:33639`, PID 91986 | `tcp://192.168.4.164:45489`, PID 93146 | `tcp://192.168.4.164:38291`, PID 91875 |
| UAV-02 / `x500_custom_1` | `tcp://192.168.4.164:36981`, PID 91987 | `tcp://192.168.4.164:45489`, PID 93146 | `tcp://192.168.4.164:38291`, PID 91875 |

`ss -ltnp` independently mapped the endpoints to the exact `px4`, Python web
backend and Gazebo server processes.

## Idle publication measurement

Each topic was subscribed for three seconds while both vehicles were disarmed,
tracking was inactive, gimbal control was inactive, and Follow was disabled.
The observer did not publish.

| Topic | Messages observed |
|---|---:|
| `/model/x500_custom_0/command/gimbal_roll` | 0 |
| `/model/x500_custom_0/command/gimbal_pitch` | 0 |
| `/model/x500_custom_0/command/gimbal_yaw` | 0 |
| `/model/x500_custom_1/command/gimbal_roll` | 0 |
| `/model/x500_custom_1/command/gimbal_pitch` | 0 |
| `/model/x500_custom_1/command/gimbal_yaw` | 0 |

This passes the idle criterion: no unintended gimbal command stream and no
evidence of PX4 continuously overwriting the dashboard path.

## Integrity evidence

```text
1009e161840a974b70d9bdd37751c3030f77568b6b7f1fdf5c32d29ff06a93ce  main.py
5da7d17e4fe8592b32ea9420d02887e24df6890062c8d05ee341a2e5d8a6ef77  tracking_web.py
ecd19f196d8e2b40144ed48725552779f34ed87ea6a7048901c397dafd0248ad  GZMixingInterfaceServo.cpp
bb7a9e4504cd2d885fc3e798d6e6ce85881ef021dab35fdc8cbef85340f2f7df  GZMixingInterfaceServo.hpp
c578ccc373dfd6625577d4f58cde6c0b4025958e75e06b50dfe12511e76dca05  px4
```

The PX4 source files are dirty relative to the checked-out commit. Their
modification timestamps precede the running PX4 binary by approximately ten
seconds, and the observed idle suppression matches the modified source. This
supports, but does not replace, a clean reproducible build requirement.

## Gate update and next action

| Criterion | Result |
|---|---|
| Publisher identities resolved | PASS |
| Idle command stream absent | PASS |
| Continuous PX4 overwrite absent while idle | PASS |
| Dashboard active-tracking stream measured | NOT RUN |
| PX4/QGC target change during dashboard tracking measured | NOT RUN |
| Explicit ownership/interlock policy | OPEN |
| No Follow entry during this evidence session | PASS |

Recommended current action:

1. Keep the existing dashboard-to-Gazebo tracking path; do not rewrite it only
   because discovery reports two publishers.
2. Treat PX4/QGroundControl gimbal input as an external override that must not
   be used concurrently with dashboard tracking until an ownership policy is
   accepted.
3. In a separate, explicitly authorized disarmed SITL fixture, start ordinary
   tracking (not Follow), select a known target, record all six command topics,
   and verify the dashboard rate and target-loss behavior.
4. Only if overlap testing is required, present the exact PX4/QGC gimbal test
   command and safe exit procedure before sending it.
5. Keep Phase 0B NO-GO because active authority overlap and the other parent
   report gates remain unresolved. Do not begin P1B.

