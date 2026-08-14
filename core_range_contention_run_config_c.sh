#!/usr/bin/env bash
# CORE_RANGE_LIVE_STACK_CONTENTION_ISOLATION -- ablation C:
# Gazebo server (headless, no GUI) + a single PX4 SITL instance (needed
# only to spawn the camera-bearing vehicle model) + a minimal standalone
# gz-transport camera subscriber driving the real LatestDepthWorker /
# MidasSmallAdapter directly (core_range_contention_config_c_camera_worker.py).
#
# Deliberately does NOT start: mosquitto, MicroXRCEAgent, the Gazebo GUI,
# the second UAV, the ROS2 telemetry launch, mavlink_manual_bridge.py, or
# main.py/uvicorn. This isolates "does Gazebo's own rendering/simulation
# loop plus PX4 SITL physics contend for the GPU/CPU with MiDaS" from
# everything else the full stack (config D) also adds.

set -Eeuo pipefail

if [[ $# -ne 2 ]]; then
  echo "usage: $0 DURATION_S OUTPUT_ROOT" >&2
  exit 2
fi

duration_s="$1"
output_root="$2"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
gazebo_px4_root="${PX4_AUTOPILOT_ROOT:-/mnt/px4ssd/PX4-Autopilot}"
gazebo_world="${gazebo_px4_root}/Tools/simulation/gz/worlds/default.sdf"
gazebo_environment="${gazebo_px4_root}/build/px4_sitl_default/rootfs/gz_env.sh"
px4_bin="${gazebo_px4_root}/build/px4_sitl_default/bin/px4"

for pattern in '^gz sim' 'px4 -i 0' 'px4 -i 1'; do
  if pgrep -af "${pattern}" >/dev/null 2>&1; then
    echo "a stack process is already running (pattern: ${pattern}); stop it first" >&2
    pgrep -af "${pattern}" >&2 || true
    exit 1
  fi
done

mkdir -p "${output_root}"

declare -a child_pids=()
cleaning_up=0
cleanup() {
  local exit_code=$?
  if [[ ${cleaning_up} -eq 1 ]]; then return; fi
  cleaning_up=1
  trap - EXIT INT TERM
  echo "Stopping config-C minimal stack..."
  local pid
  for ((i = ${#child_pids[@]} - 1; i >= 0; i--)); do
    pid="${child_pids[i]}"
    if kill -0 "${pid}" 2>/dev/null; then
      kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
    fi
  done
  sleep 2
  for pid in "${child_pids[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
  echo "config-C stack stopped. Residual processes (should be empty):"
  pgrep -af 'gz sim|px4 -i|MicroXRCEAgent|mavlink_manual_bridge|uvicorn main:app' || echo "  (none)"
  exit "${exit_code}"
}
trap cleanup EXIT INT TERM

setsid bash -c '
  set -euo pipefail
  source "$1"
  exec gz sim --verbose=1 -r -s "$2"
' _ "${gazebo_environment}" "${gazebo_world}" >"${output_root}/gazebo_server.log" 2>&1 &
child_pids+=("$!")
sleep 3

setsid bash -c '
  set -euo pipefail
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    set +u; source /opt/ros/jazzy/setup.bash; set -u
  fi
  source "$1"
  cd "$2"
  export PX4_SYS_AUTOSTART=22000
  export PX4_SIM_MODEL=gz_x500_custom
  export PX4_GZ_STANDALONE=1
  export PX4_GZ_MODEL_POSE="0,0,0,0,0,0"
  exec "$3" -i 0
' _ "${gazebo_environment}" "${gazebo_px4_root}" "${px4_bin}" >"${output_root}/px4_uav_01.log" 2>&1 &
child_pids+=("$!")

echo "Waiting for the camera-bearing vehicle model to spawn and start rendering..."
sleep 10

echo "Running the config-C camera+MiDaS harness for ${duration_s}s..."
"${script_dir}/.venv/bin/python" "${script_dir}/core_range_contention_config_c_camera_worker.py" \
  --output "${output_root}" --duration-s "${duration_s}"
harness_status=$?

exit "${harness_status}"
