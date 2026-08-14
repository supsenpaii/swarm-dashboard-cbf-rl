#!/usr/bin/env bash
# CORE_RANGE_RTF_STUTTER_ROOT_CAUSE_AND_MITIGATION Phase 2 staged-ablation
# launcher. Read-only observation: never arms/takes off/OFFBOARD/Follow
# Target. Always headless (SWARM_START_GAZEBO_GUI=false where applicable).
set -Eeuo pipefail

if [[ $# -ne 4 ]]; then
  echo "usage: $0 CONFIG_ID(A|B|C|X_ON|X_OFF|PX4_LOG_ON|PX4_LOG_OFF|L0|L1|L2|L3|L4) DURATION_S OUTPUT_ROOT WORLD_SDF" >&2
  exit 2
fi

config_id="$1"; duration_s="$2"; output_root="$3"; world_sdf="$4"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
px4_root="${PX4_AUTOPILOT_ROOT:-/mnt/px4ssd/PX4-Autopilot}"
gz_env="${px4_root}/build/px4_sitl_default/rootfs/gz_env.sh"

if [[ -e "${output_root}" ]]; then
  echo "refusing to overwrite existing root: ${output_root}" >&2
  exit 3
fi
mkdir -p "${output_root}"

declare -a pids=()
stack_pid=""
run_all_pid=""

cleanup() {
  local code=$?
  trap - EXIT INT TERM
  if [[ -n "${run_all_pid}" ]] && kill -0 "${run_all_pid}" 2>/dev/null; then
    kill -TERM "${run_all_pid}" 2>/dev/null || true
    local deadline=$((SECONDS + 20))
    while kill -0 "${run_all_pid}" 2>/dev/null && ((SECONDS < deadline)); do sleep 0.25; done
    if kill -0 "${run_all_pid}" 2>/dev/null; then kill -KILL "${run_all_pid}" 2>/dev/null || true; fi
  fi
  for p in "${pids[@]:-}"; do
    if [[ -n "${p}" ]] && kill -0 "${p}" 2>/dev/null; then
      kill -TERM "${p}" 2>/dev/null || true
      sleep 0.5
      kill -KILL "${p}" 2>/dev/null || true
    fi
  done
  exit "${code}"
}
trap cleanup EXIT INT TERM

case "${config_id}" in
  A)
    bash -c '
      set -euo pipefail
      source "$1"
      exec gz sim --verbose=1 -r -s "$2"
    ' _ "${gz_env}" "${world_sdf}" >"${output_root}/gazebo_server.log" 2>&1 &
    pids+=($!)
    sleep 8
    ;;
  B)
    bash -c '
      set -euo pipefail
      source "$1"
      exec gz sim --verbose=1 -r -s "$2"
    ' _ "${gz_env}" "${world_sdf}" >"${output_root}/gazebo_server.log" 2>&1 &
    pids+=($!)
    sleep 5
    bash -c '
      set -euo pipefail
      if [[ -f /opt/ros/jazzy/setup.bash ]]; then set +u; source /opt/ros/jazzy/setup.bash; set -u; fi
      source "$1"; cd "$2"
      export PX4_SYS_AUTOSTART=22000 PX4_SIM_MODEL=gz_x500_custom PX4_GZ_STANDALONE=1
      export PX4_GZ_MODEL_POSE="0,0,1,0,0,0"
      exec "$3" -i 0
    ' _ "${gz_env}" "${px4_root}" "${px4_root}/build/px4_sitl_default/bin/px4" >"${output_root}/px4_uav_01.log" 2>&1 &
    pids+=($!)
    bash -c '
      set -euo pipefail
      if [[ -f /opt/ros/jazzy/setup.bash ]]; then set +u; source /opt/ros/jazzy/setup.bash; set -u; fi
      source "$1"; cd "$2"
      export PX4_SYS_AUTOSTART=22000 PX4_SIM_MODEL=gz_x500_custom PX4_GZ_STANDALONE=1
      export PX4_GZ_MODEL_POSE="11.5,0,1,0,0,0"
      exec "$3" -i 1
    ' _ "${gz_env}" "${px4_root}" "${px4_root}/build/px4_sitl_default/bin/px4" >"${output_root}/px4_uav_02.log" 2>&1 &
    pids+=($!)
    sleep 12
    ;;
  C|X_ON|X_OFF|PX4_LOG_ON|PX4_LOG_OFF|L0|L1|L2|L3|L4)
    fsync_on_rotate=true
    if [[ "${config_id}" == "X_OFF" ]]; then
      fsync_on_rotate=false
    fi
    discard_process_logs=""
    if [[ "${config_id}" == "PX4_LOG_OFF" ]]; then
      discard_process_logs="px4_uav_01,px4_uav_02"
    fi
    start_xrce=true
    start_ros_telemetry=true
    start_mavlink_bridge=true
    start_web_backend=true
    if [[ "${config_id}" =~ ^L[0-4]$ ]]; then
      discard_process_logs="px4_uav_01,px4_uav_02"
      start_xrce=false
      start_ros_telemetry=false
      start_mavlink_bridge=false
      start_web_backend=false
      if [[ "${config_id}" != "L0" ]]; then start_xrce=true; fi
      if [[ "${config_id}" =~ ^L[234]$ ]]; then start_ros_telemetry=true; fi
      if [[ "${config_id}" =~ ^L[34]$ ]]; then start_mavlink_bridge=true; fi
      if [[ "${config_id}" == "L4" ]]; then start_web_backend=true; fi
    fi
    env SWARM_START_GAZEBO_GUI=false \
      SWARM_LOG_DIR="${output_root}/runtime_logs" \
      SWARM_RUNTIME_LOG_MAX_BYTES=4194304 \
      SWARM_RUNTIME_LOG_FSYNC_ON_ROTATE="${fsync_on_rotate}" \
      SWARM_RUNTIME_LOG_FSYNC_TRACE=true \
      SWARM_RUNTIME_DISCARD_PROCESS_LOGS="${discard_process_logs}" \
      SWARM_START_XRCE="${start_xrce}" \
      SWARM_START_ROS_TELEMETRY="${start_ros_telemetry}" \
      SWARM_START_MAVLINK_BRIDGE="${start_mavlink_bridge}" \
      SWARM_START_WEB_BACKEND="${start_web_backend}" \
      "${script_dir}/run_all.sh" >"${output_root}/stack_supervisor.log" 2>&1 &
    run_all_pid=$!
    deadline=$((SECONDS + 60))
    while [[ "${start_web_backend}" == "true" ]] && ((SECONDS < deadline)); do
      if curl --fail --silent --max-time 1 http://127.0.0.1:8000/health >/dev/null 2>&1; then break; fi
      if ! kill -0 "${run_all_pid}" 2>/dev/null; then echo "stack exited before health gate" >&2; exit 4; fi
      sleep 0.5
    done
    if [[ "${start_web_backend}" == "true" ]]; then
      curl --fail --silent --max-time 1 http://127.0.0.1:8000/health >/dev/null
    else
      sleep 12
      kill -0 "${run_all_pid}"
    fi
    if pgrep -f "gz sim -g" >/dev/null 2>&1; then
      echo "gazebo GUI client detected despite SWARM_START_GAZEBO_GUI=false" >&2
      exit 8
    fi
    ;;
  *)
    echo "invalid config_id: ${config_id}" >&2
    exit 2
    ;;
esac

# --- observation-only trace for the full config duration ---
"${script_dir}/.venv/bin/python" "${script_dir}/core_range_rtf_stutter_event_tracer.py" \
  --output "${output_root}/per_second_metrics.csv" --duration-s "${duration_s}" \
  --config-label "${config_id}"

echo "config ${config_id} trace complete"
