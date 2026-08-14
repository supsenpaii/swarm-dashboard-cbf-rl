#!/usr/bin/env bash
# shellcheck disable=SC2016,SC2317

# Start the complete Swarm Dashboard stack from one terminal.
# Usage:
#   ./run_all.sh          Start everything and keep supervising it.
#   ./run_all.sh --check  Validate the local installation without starting it.

set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mode="${1:-start}"

if [[ "${mode}" != "start" && "${mode}" != "--check" ]]; then
  echo "Usage: $0 [--check]" >&2
  exit 2
fi

# A real .env takes precedence.  When it does not exist, load the project's
# runtime defaults, then use the paths of the current development machine.
set -a
if [[ -f "${script_dir}/.env" ]]; then
  # shellcheck disable=SC1091
  source "${script_dir}/.env"
else
  # shellcheck disable=SC1091
  source "${script_dir}/.env.example"
  SWARM_TRACKING_PACKAGE_ROOT="/home/sup/ws_px4/src/lfc_gimbal_gazebo"
  SWARM_ROS_SETUP="/home/sup/ws_px4/install/setup.bash"
  PX4_AUTOPILOT_ROOT="/mnt/px4ssd/PX4-Autopilot"
fi
set +a

PX4_AUTOPILOT_ROOT="${PX4_AUTOPILOT_ROOT:-/mnt/px4ssd/PX4-Autopilot}"
SWARM_ROS_SETUP="${SWARM_ROS_SETUP:-/home/sup/ws_px4/install/setup.bash}"
SWARM_TRACKING_PACKAGE_ROOT="${SWARM_TRACKING_PACKAGE_ROOT:-/home/sup/ws_px4/src/lfc_gimbal_gazebo}"
python_bin="${SWARM_PYTHON_BIN:-${script_dir}/.venv/bin/python}"
runtime_log_max_bytes="${SWARM_RUNTIME_LOG_MAX_BYTES:-8388608}"
runtime_logging_enabled="${SWARM_RUNTIME_LOGGING_ENABLED:-true}"
start_mavlink_bridge="${SWARM_START_MAVLINK_BRIDGE:-true}"
start_gazebo_gui="${SWARM_START_GAZEBO_GUI:-true}"
px4_bin="${PX4_AUTOPILOT_ROOT}/build/px4_sitl_default/bin/px4"
gz_env="${PX4_AUTOPILOT_ROOT}/build/px4_sitl_default/rootfs/gz_env.sh"
gz_world="${PX4_AUTOPILOT_ROOT}/Tools/simulation/gz/worlds/default.sdf"
gz_gui_config="${script_dir}/gazebo_gui_light.config"
uav_01_model_pose="${SWARM_UAV_01_MODEL_POSE:-0,0,0,0,0,0}"
uav_02_model_pose="${SWARM_UAV_02_MODEL_POSE:-0,5,0,0,0,0}"

export PX4_AUTOPILOT_ROOT SWARM_ROS_SETUP SWARM_TRACKING_PACKAGE_ROOT

errors=0

require_command() {
  local command_name="$1"
  if command -v "${command_name}" >/dev/null 2>&1; then
    printf '  [OK] command: %s\n' "${command_name}"
  else
    printf '  [MISSING] command: %s\n' "${command_name}" >&2
    errors=$((errors + 1))
  fi
}

require_file() {
  local path="$1"
  if [[ -f "${path}" ]]; then
    printf '  [OK] file: %s\n' "${path}"
  else
    printf '  [MISSING] file: %s\n' "${path}" >&2
    errors=$((errors + 1))
  fi
}

require_executable() {
  local path="$1"
  if [[ -x "${path}" ]]; then
    printf '  [OK] executable: %s\n' "${path}"
  else
    printf '  [MISSING] executable: %s\n' "${path}" >&2
    errors=$((errors + 1))
  fi
}

echo "Checking Swarm Dashboard runtime..."
require_command pgrep
require_command setsid
require_command mosquitto
require_command MicroXRCEAgent
require_command gz
require_command ros2
require_command curl
require_file "${script_dir}/main.py"
require_file "${script_dir}/mavlink_manual_bridge.py"
require_file "${script_dir}/static/index.html"
require_file "${script_dir}/isolated_swarm.launch.py"
require_file "${script_dir}/bounded_log_writer.py"
require_file "${gz_env}"
require_file "${gz_world}"
require_file "${gz_gui_config}"
require_file "${SWARM_ROS_SETUP}"
require_executable "${python_bin}"
require_executable "${px4_bin}"

if [[ ${errors} -ne 0 ]]; then
  echo "Runtime check failed with ${errors} missing item(s)." >&2
  exit 1
fi

if [[ "${mode}" == "--check" ]]; then
  echo "Runtime check passed."
  exit 0
fi

# Refuse to create duplicate flight-control/backend processes.  A system MQTT
# broker is safe to reuse, but the rest of this stack must have one owner.
declare -a duplicate_patterns=(
  'MicroXRCEAgent.*udp4.*8888'
  '^gz sim'
  'px4 -i [01]'
  'mavlink_manual_bridge.py'
  'uvicorn main:app'
  'ros2 launch.*(two_uav_nodes|isolated_swarm)'
)

for pattern in "${duplicate_patterns[@]}"; do
  if pgrep -af "${pattern}" >/dev/null 2>&1; then
    echo "A stack process is already running (pattern: ${pattern}):" >&2
    pgrep -af "${pattern}" >&2 || true
    echo "Stop the old stack before running $0." >&2
    exit 1
  fi
done

run_stamp="$(date +%Y%m%d_%H%M%S)"
log_dir="${SWARM_LOG_DIR:-${script_dir}/artifacts/run_${run_stamp}}"
mkdir -p "${log_dir}"

declare -a child_pids=()
declare -A child_names=()
cleaning_up=0

start_process() {
  local name="$1"
  shift
  local log_file="${log_dir}/${name}.log"

  if [[ "${runtime_logging_enabled,,}" =~ ^(0|false|no|off)$ ]]; then
    setsid bash -c '
      exec "$@" >/dev/null 2>&1
    ' _ "$@" &
  else
    setsid bash -c '
    set -o pipefail
    python_bin="$1"
    writer="$2"
    log_file="$3"
    max_bytes="$4"
    shift 4
    "$@" 2>&1 | "$python_bin" "$writer" \
      --path "$log_file" --max-bytes "$max_bytes"
    ' _ "${python_bin}" "${script_dir}/bounded_log_writer.py" \
      "${log_file}" "${runtime_log_max_bytes}" "$@" &
  fi
  local pid=$!
  child_pids+=("${pid}")
  child_names["${pid}"]="${name}"
  printf '  %-18s PID %-8s log: %s\n' "${name}" "${pid}" "${log_file}"
}

process_alive() {
  kill -0 "$1" 2>/dev/null
}

process_group_alive() {
  kill -0 -- "-$1" 2>/dev/null
}

show_log_tail() {
  local name="$1"
  local log_file="${log_dir}/${name}.log"
  if [[ -s "${log_file}" ]]; then
    echo "----- last lines from ${log_file} -----" >&2
    tail -n 30 "${log_file}" >&2 || true
  fi
}

assert_alive() {
  local pid="$1"
  local name="${child_names[${pid}]}"
  if ! process_alive "${pid}"; then
    echo "${name} stopped during startup." >&2
    show_log_tail "${name}"
    exit 1
  fi
}

wait_for_tcp() {
  local host="$1"
  local port="$2"
  local timeout_seconds="$3"
  local label="$4"
  local deadline=$((SECONDS + timeout_seconds))

  while ((SECONDS < deadline)); do
    if (exec 3<>"/dev/tcp/${host}/${port}") 2>/dev/null; then
      exec 3>&- 3<&-
      return 0
    fi
    sleep 0.25
  done

  echo "Timed out waiting for ${label} on ${host}:${port}." >&2
  return 1
}

wait_for_http() {
  local url="$1"
  local timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))

  while ((SECONDS < deadline)); do
    if curl --fail --silent --max-time 1 "${url}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done

  echo "Timed out waiting for ${url}." >&2
  return 1
}

cleanup() {
  local exit_code=$?
  if [[ ${cleaning_up} -eq 1 ]]; then
    return
  fi
  cleaning_up=1
  trap - EXIT INT TERM

  if ((${#child_pids[@]} > 0)); then
    echo
    echo "Stopping processes started by run_all.sh..."
  fi

  local index pid name
  for ((index = ${#child_pids[@]} - 1; index >= 0; index--)); do
    pid="${child_pids[${index}]}"
    name="${child_names[${pid}]}"
    if process_alive "${pid}"; then
      printf '  stopping %-18s PID %s\n' "${name}" "${pid}"
      kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
    fi
  done

  local deadline=$((SECONDS + 8))
  while ((SECONDS < deadline)); do
    local any_alive=0
    for pid in "${child_pids[@]}"; do
      if process_alive "${pid}"; then
        any_alive=1
        break
      fi
    done
    [[ ${any_alive} -eq 0 ]] && break
    sleep 0.25
  done

  for pid in "${child_pids[@]}"; do
    if process_group_alive "${pid}"; then
      name="${child_names[${pid}]}"
      printf '  terminating %-15s PGID %s\n' "${name}" "${pid}"
      kill -TERM -- "-${pid}" 2>/dev/null || true
    fi
  done

  deadline=$((SECONDS + 3))
  while ((SECONDS < deadline)); do
    local any_group_alive=0
    for pid in "${child_pids[@]}"; do
      if process_group_alive "${pid}"; then
        any_group_alive=1
        break
      fi
    done
    [[ ${any_group_alive} -eq 0 ]] && break
    sleep 0.25
  done

  for pid in "${child_pids[@]}"; do
    if process_group_alive "${pid}"; then
      name="${child_names[${pid}]}"
      printf '  killing %-19s PGID %s\n' "${name}" "${pid}"
      kill -KILL -- "-${pid}" 2>/dev/null || true
    fi
  done

  wait 2>/dev/null || true
  echo "Logs: ${log_dir}"
  exit "${exit_code}"
}

trap cleanup EXIT INT TERM

echo "Starting Swarm Dashboard stack..."
echo "Logs: ${log_dir}"

if pgrep -x mosquitto >/dev/null 2>&1; then
  echo "  mqtt               reusing the running Mosquitto broker"
else
  start_process mqtt mosquitto -v
  mqtt_pid="${child_pids[-1]}"
  sleep 0.5
  assert_alive "${mqtt_pid}"
fi
wait_for_tcp 127.0.0.1 1883 10 "MQTT broker"

start_process xrce MicroXRCEAgent udp4 -p 8888
xrce_pid="${child_pids[-1]}"
sleep 0.5
assert_alive "${xrce_pid}"

echo "DEBUG-PRE-GAZEBO:"; printenv | grep -i GZ_SIM >&2 || echo "DEBUG: none found" >&2
start_process gazebo_server bash -c '
  set -euo pipefail
  echo "INNER-BEFORE-SOURCE:" >&2
  printenv | grep -i GZ_SIM >&2 || echo "INNER: none" >&2
  source "$1"
  echo "INNER-AFTER-SOURCE:" >&2
  printenv | grep -i GZ_SIM >&2 || echo "INNER: none" >&2
  exec gz sim --verbose=1 -r -s "$2"
' _ "${gz_env}" "${gz_world}"
gazebo_server_pid="${child_pids[-1]}"
sleep 2
assert_alive "${gazebo_server_pid}"

if [[ ! "${start_gazebo_gui,,}" =~ ^(0|false|no|off)$ ]]; then
  start_process gazebo_gui bash -c '
    set -euo pipefail
    source "$1"
    exec gz sim -g --gui-config "$2"
  ' _ "${gz_env}" "${gz_gui_config}"
  gazebo_gui_pid="${child_pids[-1]}"
  sleep 1
  assert_alive "${gazebo_gui_pid}"
else
  echo "  gazebo_gui         disabled by SWARM_START_GAZEBO_GUI"
fi

start_process px4_uav_01 bash -c '
  set -euo pipefail
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
  fi
  source "$1"
  cd "$2"
  export PX4_SYS_AUTOSTART=22000
  export PX4_SIM_MODEL=gz_x500_custom
  export PX4_GZ_STANDALONE=1
  export PX4_GZ_MODEL_POSE="$4"
  exec "$3" -i 0
' _ "${gz_env}" "${PX4_AUTOPILOT_ROOT}" "${px4_bin}" \
  "${uav_01_model_pose}"
px4_uav_01_pid="${child_pids[-1]}"

start_process px4_uav_02 bash -c '
  set -euo pipefail
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
  fi
  source "$1"
  cd "$2"
  export PX4_SYS_AUTOSTART=22000
  export PX4_SIM_MODEL=gz_x500_custom
  export PX4_GZ_STANDALONE=1
  export PX4_GZ_MODEL_POSE="$4"
  exec "$3" -i 1
' _ "${gz_env}" "${PX4_AUTOPILOT_ROOT}" "${px4_bin}" \
  "${uav_02_model_pose}"
px4_uav_02_pid="${child_pids[-1]}"
sleep 2
assert_alive "${px4_uav_01_pid}"
assert_alive "${px4_uav_02_pid}"

start_process ros_telemetry bash -c '
  set -euo pipefail
  set +u
  source "$1"
  set -u
  export ROS_LOCALHOST_ONLY=1
  exec ros2 launch swarm_telemetry two_uav_nodes.launch.py
' _ "${SWARM_ROS_SETUP}"
ros_telemetry_pid="${child_pids[-1]}"
sleep 2
assert_alive "${ros_telemetry_pid}"

if [[ ! "${start_mavlink_bridge,,}" =~ ^(0|false|no|off)$ ]]; then
  start_process mavlink_bridge "${python_bin}" "${script_dir}/mavlink_manual_bridge.py"
  mavlink_bridge_pid="${child_pids[-1]}"
  sleep 1
  assert_alive "${mavlink_bridge_pid}"
else
  echo "  mavlink_bridge     disabled by SWARM_START_MAVLINK_BRIDGE"
fi

start_process web_backend "${python_bin}" -m uvicorn main:app \
  --app-dir "${script_dir}" --host 0.0.0.0 --port 8000
web_backend_pid="${child_pids[-1]}"
sleep 0.5
assert_alive "${web_backend_pid}"

if ! wait_for_http "http://127.0.0.1:8000/health" 30; then
  show_log_tail web_backend
  exit 1
fi

echo
echo "Swarm Dashboard is ready: http://127.0.0.1:8000"
echo "Press Ctrl+C to stop the complete stack."

# End the stack if any owned component exits.  This avoids leaving a partial
# flight-control stack running after a failure.
failed_pid=""
wait -n -p failed_pid "${child_pids[@]}" || component_status=$?
component_status="${component_status:-0}"
failed_name="${child_names[${failed_pid}]:-unknown}"
echo "Component ${failed_name} (PID ${failed_pid:-unknown}) exited with status ${component_status}." >&2
show_log_tail "${failed_name}"
exit "${component_status}"
