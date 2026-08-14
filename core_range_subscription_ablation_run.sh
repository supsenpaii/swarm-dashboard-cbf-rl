#!/usr/bin/env bash
# One clean, observation-only Gazebo subscription ablation repeat.
set -Eeuo pipefail

if [[ $# -ne 4 ]]; then
  echo "usage: $0 CONFIG_ID REPEAT DURATION_S OUTPUT_ROOT" >&2
  exit 2
fi

config_id="$1"
repeat="$2"
duration_s="$3"
output_root="$4"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
run_dir="${output_root}/runs/${config_id}_run${repeat}"

if [[ ! "${duration_s}" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
   ! awk -v value="${duration_s}" 'BEGIN { exit !(value >= 600) }'; then
  echo "duration must be numeric and at least 600 seconds" >&2
  exit 2
fi
if [[ ! "${repeat}" =~ ^[123]$ ]]; then
  echo "repeat must be 1, 2, or 3" >&2
  exit 2
fi
if [[ -e "${run_dir}" ]]; then
  echo "refusing to overwrite ${run_dir}" >&2
  exit 3
fi
mkdir -p "${run_dir}/runtime_logs"

profile=""
topics=""
start_backend=true
case "${config_id}" in
  S0) start_backend=false; profile=zero ;;
  S1) profile=zero ;;
  S2_camera_uav01) profile=single; topics=camera_uav01 ;;
  S2_camera_uav02) profile=single; topics=camera_uav02 ;;
  S2_body_imu_uav01) profile=single; topics=body_imu_uav01 ;;
  S2_camera_imu_uav01) profile=single; topics=camera_imu_uav01 ;;
  S2_body_imu_uav02) profile=single; topics=body_imu_uav02 ;;
  S2_camera_imu_uav02) profile=single; topics=camera_imu_uav02 ;;
  S2_front_lidar) profile=single; topics=front_lidar ;;
  S3) profile=count ;;
  S4) profile=copy ;;
  # Preserve the pre-fix eager-production control after production itself
  # moves to on-demand camera subscriptions.
  S5) profile=eager ;;
  S6_on_demand) profile=production ;;
  *) echo "unknown configuration: ${config_id}" >&2; exit 2 ;;
esac

declare -a owned_pids=()
stack_pid=""
cleaning_up=0

stack_patterns=(
  'MicroXRCEAgent.*udp4.*8888'
  '^gz sim'
  'px4 -i [01]'
  'mavlink_manual_bridge.py'
  'uvicorn main:app'
  'ros2 launch.*(two_uav_nodes|isolated_swarm)'
)

assert_clean() {
  local found=0 pattern
  for pattern in "${stack_patterns[@]}"; do
    if pgrep -af "${pattern}" >"${run_dir}/unexpected_processes.txt" 2>/dev/null; then
      found=1
    fi
  done
  if ss -ltnp 2>/dev/null | awk '$4 ~ /:(8000|8001|8888)$/ {print}' \
      >"${run_dir}/unexpected_listeners.txt" && \
     [[ -s "${run_dir}/unexpected_listeners.txt" ]]; then
    found=1
  fi
  if ss -lunp 2>/dev/null | awk '$5 ~ /:8888$/ {print}' \
      >"${run_dir}/unexpected_udp_listeners.txt" && \
     [[ -s "${run_dir}/unexpected_udp_listeners.txt" ]]; then
    found=1
  fi
  if ((found)); then
    echo "stack/listener cleanliness check failed" >&2
    return 1
  fi
}

cleanup() {
  local code=$?
  if ((cleaning_up)); then return; fi
  cleaning_up=1
  trap - EXIT INT TERM
  if [[ -n "${stack_pid}" ]] && kill -0 "${stack_pid}" 2>/dev/null; then
    kill -INT "${stack_pid}" 2>/dev/null || true
    local deadline=$((SECONDS + 20))
    while kill -0 "${stack_pid}" 2>/dev/null && ((SECONDS < deadline)); do sleep 0.25; done
    if kill -0 "${stack_pid}" 2>/dev/null; then
      kill -TERM "${stack_pid}" 2>/dev/null || true
    fi
  fi
  local pid
  for pid in "${owned_pids[@]:-}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
  sleep 2
  if ! assert_clean; then code=9; fi
  printf '%s\n' "${code}" >"${run_dir}/exit_code.txt"
  exit "${code}"
}
trap cleanup EXIT INT TERM

assert_clean
start_wall_s="$(date +%s.%N)"
start_monotonic_ns="$(cut -d' ' -f1 /proc/uptime | awk '{printf "%.0f", $1 * 1000000000}')"
printf '%s,%s,%s,%s,%s\n' \
  config_id repeat requested_duration_s start_wall_s start_monotonic_ns \
  >"${run_dir}/run_identity.csv"
printf '%s,%s,%s,%s,%s\n' \
  "${config_id}" "${repeat}" "${duration_s}" "${start_wall_s}" "${start_monotonic_ns}" \
  >>"${run_dir}/run_identity.csv"

env \
  SWARM_START_GAZEBO_GUI=false \
  SWARM_START_WEB_BACKEND="${start_backend}" \
  SWARM_GAZEBO_SUBSCRIPTION_PROFILE="${profile}" \
  SWARM_GAZEBO_SUBSCRIPTION_TOPICS="${topics}" \
  SWARM_GAZEBO_SUBSCRIPTION_TRACE_CSV="${run_dir}/callback_phase_raw.csv" \
  SWARM_RANGE_DATASET_DIR= \
  SWARM_LOG_DIR="${run_dir}/runtime_logs" \
  SWARM_RUNTIME_LOG_MAX_BYTES=4194304 \
  SWARM_RUNTIME_LOG_FSYNC_ON_ROTATE=false \
  SWARM_RUNTIME_LOG_FSYNC_TRACE=false \
  SWARM_RUNTIME_DISCARD_PROCESS_LOGS=px4_uav_01,px4_uav_02 \
  "${script_dir}/run_all.sh" >"${run_dir}/stack_supervisor.log" 2>&1 &
stack_pid=$!

deadline=$((SECONDS + 75))
while ((SECONDS < deadline)); do
  if ! kill -0 "${stack_pid}" 2>/dev/null; then
    echo "stack exited during startup" >&2
    exit 4
  fi
  if [[ "${start_backend}" == true ]]; then
    if curl --fail --silent --max-time 1 http://127.0.0.1:8000/health >/dev/null 2>&1; then break; fi
  elif pgrep -f 'px4_sitl_default/bin/px4 -i 0' >/dev/null 2>&1 && \
       pgrep -f 'px4_sitl_default/bin/px4 -i 1' >/dev/null 2>&1; then
    sleep 8
    break
  fi
  sleep 0.5
done
if [[ "${start_backend}" == true ]]; then
  curl --fail --silent --max-time 2 http://127.0.0.1:8000/health >/dev/null
fi

"${script_dir}/.venv/bin/python" \
  "${script_dir}/core_range_rtf_stutter_event_tracer.py" \
  --output "${run_dir}/per_second_metrics.csv" \
  --duration-s "${duration_s}" --config-label "${config_id}" &
owned_pids+=("$!")
wait "${owned_pids[-1]}"

end_wall_s="$(date +%s.%N)"
printf '%s,%s\n' end_wall_s "${end_wall_s}" >"${run_dir}/completion.csv"
echo "completed ${config_id} repeat ${repeat}"
