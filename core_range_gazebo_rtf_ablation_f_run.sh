#!/usr/bin/env bash
# CORE_RANGE_GAZEBO_RTF_STUTTER_AND_GATE_VALIDATION ablation F: keep
# Gazebo/PX4/ROS2 running continuously (never restarted) but restart the
# dashboard backend (uvicorn main:app) between every rep. This isolates
# backend-process accumulation (Python memory growth, GC pressure, thread/
# socket accumulation inside the long-lived FastAPI/uvicorn process) from
# Gazebo-side accumulation, which the long-run baseline (config A/G) and
# the prior 5Hz-headless task's per-session-full-restart evidence (config
# E) already cover.
#
# Always headless: SWARM_START_GAZEBO_GUI=false unconditional.

set -Eeuo pipefail

if [[ $# -ne 3 ]]; then
  echo "usage: $0 MIN_DURATION_S REP_MOVEMENT_DURATION_S OUTPUT_ROOT" >&2
  exit 2
fi

min_duration_s="$1"; rep_duration_s="$2"; output_root="$3"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
backend_pid=""
monitor_pid=""
declare -a teardown_extra_pids=()

if [[ -e "${output_root}" ]]; then
  echo "refusing to overwrite existing root: ${output_root}" >&2
  exit 3
fi
mkdir -p "${output_root}/reps"

stop_backend() {
  if [[ -n "${backend_pid}" ]] && kill -0 "${backend_pid}" 2>/dev/null; then
    kill -TERM "${backend_pid}" 2>/dev/null || true
    local deadline=$((SECONDS + 10))
    while kill -0 "${backend_pid}" 2>/dev/null && ((SECONDS < deadline)); do sleep 0.25; done
    if kill -0 "${backend_pid}" 2>/dev/null; then kill -KILL "${backend_pid}" 2>/dev/null || true; fi
    wait "${backend_pid}" 2>/dev/null || true
  fi
  backend_pid=""
}

start_backend() {
  local rep_index="$1"
  env -u PYTHONPATH SWARM_START_GAZEBO_GUI=false SWARM_CAMERA_TRACE_JSONL="${output_root}/camera_source_trace.jsonl" \
    "${script_dir}/.venv/bin/python" -m uvicorn main:app --app-dir "${script_dir}" --host 0.0.0.0 --port 8000 \
    >"${output_root}/backend_rep_${rep_index}.log" 2>&1 &
  backend_pid=$!
  local deadline=$((SECONDS + 30))
  while ((SECONDS < deadline)); do
    if curl --fail --silent --max-time 1 http://127.0.0.1:8000/health >/dev/null 2>&1; then return 0; fi
    if ! kill -0 "${backend_pid}" 2>/dev/null; then echo "backend exited before health gate" >&2; return 1; fi
    sleep 0.5
  done
  echo "backend health gate timed out" >&2
  return 1
}

stop_all() {
  if [[ -n "${monitor_pid}" ]] && kill -0 "${monitor_pid}" 2>/dev/null; then
    kill -TERM "${monitor_pid}" 2>/dev/null || true
    wait "${monitor_pid}" 2>/dev/null || true
  fi
  monitor_pid=""
  stop_backend
  for pid in "${teardown_extra_pids[@]:-}"; do
    [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null && kill -TERM "${pid}" 2>/dev/null || true
  done
  sleep 2
  for pid in "${teardown_extra_pids[@]:-}"; do
    [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null && kill -KILL "${pid}" 2>/dev/null || true
  done
}
cleanup() { local code=$?; trap - EXIT INT TERM; stop_all; exit "${code}"; }
trap cleanup EXIT INT TERM

if pgrep -x mosquitto >/dev/null 2>&1; then :; else mosquitto -v >"${output_root}/mosquitto.log" 2>&1 & disown; sleep 1; fi
MicroXRCEAgent udp4 -p 8888 >"${output_root}/xrce.log" 2>&1 &
xrce_pid=$!
sleep 1

gz_env="/mnt/px4ssd/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh"
gz_world="/mnt/px4ssd/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf"
px4_bin="/mnt/px4ssd/PX4-Autopilot/build/px4_sitl_default/bin/px4"

bash -c '
  set -euo pipefail
  source "$1"
  exec gz sim --verbose=1 -r -s "$2"
' _ "${gz_env}" "${gz_world}" >"${output_root}/gazebo_server.log" 2>&1 &
gazebo_pid=$!
sleep 2

bash -c '
  set -euo pipefail
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then set +u; source /opt/ros/jazzy/setup.bash; set -u; fi
  source "$1"; cd "$2"
  export PX4_SYS_AUTOSTART=22000 PX4_SIM_MODEL=gz_x500_custom PX4_GZ_STANDALONE=1
  export PX4_GZ_MODEL_POSE="0,0,1,0,0,0"
  exec "$3" -i 0
' _ "${gz_env}" "/mnt/px4ssd/PX4-Autopilot" "${px4_bin}" >"${output_root}/px4_uav_01.log" 2>&1 &
px4_01_pid=$!
bash -c '
  set -euo pipefail
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then set +u; source /opt/ros/jazzy/setup.bash; set -u; fi
  source "$1"; cd "$2"
  export PX4_SYS_AUTOSTART=22000 PX4_SIM_MODEL=gz_x500_custom PX4_GZ_STANDALONE=1
  export PX4_GZ_MODEL_POSE="11.5,0,1,0,0,0"
  exec "$3" -i 1
' _ "${gz_env}" "/mnt/px4ssd/PX4-Autopilot" "${px4_bin}" >"${output_root}/px4_uav_02.log" 2>&1 &
px4_02_pid=$!
sleep 2

bash -c '
  set -euo pipefail; set +u; source "$1"; set -u
  export ROS_LOCALHOST_ONLY=1
  exec ros2 launch swarm_telemetry two_uav_nodes.launch.py
' _ "/home/sup/ws_px4/install/setup.bash" >"${output_root}/ros_telemetry.log" 2>&1 &
ros_pid=$!
sleep 2

"${script_dir}/.venv/bin/python" "${script_dir}/mavlink_manual_bridge.py" >"${output_root}/mavlink_bridge.log" 2>&1 &
mavlink_pid=$!
sleep 1

teardown_extra_pids=("${xrce_pid}" "${gazebo_pid}" "${px4_01_pid}" "${px4_02_pid}" "${ros_pid}" "${mavlink_pid}")

if ! start_backend 0; then echo "initial backend start failed" >&2; exit 5; fi

if pgrep -f "gz sim -g" >/dev/null 2>&1; then
  echo "gazebo GUI client detected despite SWARM_START_GAZEBO_GUI=false" >&2
  exit 8
fi

env PYTHONPATH="${script_dir}/.venv/lib/python3.12/site-packages" \
  "${script_dir}/.venv/bin/python" -c \
  'from range_v2_r3_6a_capture import wait_for_observation_only_ready; wait_for_observation_only_ready()'

setup_log="${output_root}/static_environment_services.log"
bash -c '
  set -euo pipefail; source "$1"
  gz service -s /world/default/set_physics --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout 5000 --req "gravity {x: 0 y: 0 z: 0}"
  gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 --req "name: \"x500_custom_0\" position {x: 0 y: 0 z: 1} orientation {w: 1}"
  gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 --req "name: \"x500_custom_1\" position {x: 11.5 y: 0 z: 1} orientation {w: 1}"
' _ "${gz_env}" >"${setup_log}"
if [[ "$(grep -c 'data: true' "${setup_log}")" -ne 3 ]]; then echo "static setup failed" >&2; exit 6; fi
sleep 2

env -u PYTHONPATH "${script_dir}/.venv/bin/python" "${script_dir}/core_range_gazebo_rtf_long_run_monitor.py" \
  --output "${output_root}" --duration-s "$((min_duration_s + 180))" \
  >"${output_root}/monitor_stdout.log" 2>&1 &
monitor_pid=$!

shared_root="${output_root}/shared"
run_started=$SECONDS
rep=0
direction="approach"
while (( SECONDS - run_started < min_duration_s )); do
  rep=$((rep + 1))
  if (( rep > 1 )); then
    echo "restarting backend before rep ${rep}" | tee -a "${output_root}/rep_log.txt"
    stop_backend
    if ! start_backend "${rep}"; then
      echo "backend restart failed before rep ${rep}" | tee -a "${output_root}/rep_log.txt"
      break
    fi
  fi
  rep_label="${output_root}/reps/rep_$(printf '%02d' "${rep}")_${direction}"
  if [[ "${direction}" == "approach" ]]; then
    start=11.5; end=3.5; bbox_x=0.465; bbox_y=0.25; bbox_w=0.07; bbox_h=0.095
  else
    start=3.5; end=11.5; bbox_x=0.398; bbox_y=0.155; bbox_w=0.204; bbox_h=0.279
  fi
  echo "REP ${rep} direction=${direction} start=${start} end=${end} elapsed_s=$((SECONDS - run_started))" | tee -a "${output_root}/rep_log.txt"

  set +e
  env \
    SWARM_RANGE_DATASET_DIR="${shared_root}" \
    SWARM_RANGE_PHYSICAL_DIAGNOSTICS_DIR="${shared_root}" \
    SWARM_RANGE_DATASET_RUN_ID="core_rtf_ablation_f_rep_${rep}" \
    SWARM_RANGE_DATASET_TARGET_ID=UAV-02 \
    SWARM_RANGE_DATASET_DEPTH_RATE_HZ=5.0 \
    SWARM_RANGE_RESIDUAL_MODE=off \
    "${script_dir}/.venv/bin/python" "${script_dir}/core_range_dynamic_capture.py" \
    --root "${shared_root}" --scenario-id "rtf_ablation_f_rep_${rep}_${direction}" \
    --dataset-role core_range_dynamic_5hz_headless_post_fixes \
    --start-range-m "${start}" --end-range-m "${end}" \
    --lateral-m 0.0 --target-z-m 1.0 \
    --yaw-start-deg 0.0 --yaw-end-deg 0.0 \
    --movement-duration-s "${rep_duration_s}" --hold-duration-s 0.0 \
    --pose-update-rate-hz 5.0 --pitch-deg -10.0 \
    --bbox-x "${bbox_x}" --bbox-y "${bbox_y}" --bbox-width "${bbox_w}" --bbox-height "${bbox_h}" \
    --minimum-raw-frames 15 --capture-timeout-s 300 \
    >"${rep_label}.capture_result.json" 2>"${rep_label}.capture_stderr.log"
  rep_status=$?
  set -e
  echo "REP ${rep} status=${rep_status}" | tee -a "${output_root}/rep_log.txt"

  if [[ "${direction}" == "approach" ]]; then direction="recede"; else direction="approach"; fi
done

echo "ablation F capture loop complete: ${rep} reps, elapsed $((SECONDS - run_started))s" | tee -a "${output_root}/rep_log.txt"

stop_all
