#!/usr/bin/env bash

set -Eeuo pipefail

if [[ $# -lt 10 || $# -gt 12 ]]; then
  echo "usage: $0 SCENARIO X Y DISTANCE PITCH BBOX_X BBOX_Y BBOX_W BBOX_H OUTPUT_ROOT [PREWARM_TIMEOUT_S] [CAPTURE_TIMEOUT_S]" >&2
  exit 2
fi

scenario_id="$1"
target_x="$2"
target_y="$3"
distance_m="$4"
pitch_deg="$5"
bbox_x="$6"
bbox_y="$7"
bbox_w="$8"
bbox_h="$9"
output_root="${10}"
prewarm_timeout_s="${11:-90}"
capture_timeout_s="${12:-90}"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
dataset_role="${CORE_RANGE_DATASET_ROLE:-core_range_development}"
run_id="${CORE_RANGE_RUN_ID:-core_priority_${scenario_id}_20260804}"
stack_pid=""

if [[ -e "${output_root}" ]]; then
  echo "refusing to overwrite existing scenario root: ${output_root}" >&2
  exit 3
fi
mkdir -p "${output_root}"

stop_stack() {
  if [[ -n "${stack_pid}" ]] && kill -0 "${stack_pid}" 2>/dev/null; then
    kill -TERM "${stack_pid}" 2>/dev/null || true
    local deadline=$((SECONDS + 15))
    while kill -0 "${stack_pid}" 2>/dev/null && ((SECONDS < deadline)); do
      sleep 0.25
    done
    if kill -0 "${stack_pid}" 2>/dev/null; then
      kill -KILL "${stack_pid}" 2>/dev/null || true
    fi
    wait "${stack_pid}" 2>/dev/null || true
  fi
  stack_pid=""
}

cleanup() {
  local code=$?
  trap - EXIT INT TERM
  stop_stack
  exit "${code}"
}
trap cleanup EXIT INT TERM

env \
  SWARM_RANGE_DATASET_DIR="${output_root}" \
  SWARM_RANGE_PHYSICAL_DIAGNOSTICS_DIR="${output_root}" \
  SWARM_RANGE_DATASET_RUN_ID="${run_id}" \
  SWARM_RANGE_DATASET_TARGET_ID=UAV-02 \
  SWARM_RANGE_RESIDUAL_MODE=off \
  SWARM_UAV_01_MODEL_POSE="0,0,1,0,0,0" \
  SWARM_UAV_02_MODEL_POSE="${target_x},${target_y},1,0,0,0" \
  SWARM_LOG_DIR="${output_root}/runtime_logs" \
  SWARM_RUNTIME_LOG_MAX_BYTES=4194304 \
  "${script_dir}/run_all.sh" >"${output_root}/stack_supervisor.log" 2>&1 &
stack_pid=$!

deadline=$((SECONDS + 60))
while ((SECONDS < deadline)); do
  if curl --fail --silent --max-time 1 http://127.0.0.1:8000/health >/dev/null 2>&1; then
    break
  fi
  if ! kill -0 "${stack_pid}" 2>/dev/null; then
    echo "stack exited before health gate" >&2
    exit 4
  fi
  sleep 0.5
done
if ! curl --fail --silent --max-time 1 http://127.0.0.1:8000/health >/dev/null; then
  echo "stack health timeout" >&2
  exit 5
fi

env PYTHONPATH="${script_dir}/.venv/lib/python3.12/site-packages" \
  "${script_dir}/.venv/bin/python" -c \
  'from range_v2_r3_6a_capture import wait_for_observation_only_ready; wait_for_observation_only_ready()'

static_setup_log="${output_root}/static_environment_services.log"
bash -c '
  set -euo pipefail
  source "$1"
  gz service -s /world/default/set_physics \
    --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout 5000 \
    --req "gravity {x: 0 y: 0 z: 0}"
  gz service -s /world/default/set_pose \
    --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 \
    --req "name: \"x500_custom_0\" position {x: 0 y: 0 z: 1} orientation {w: 1}"
  gz service -s /world/default/set_pose \
    --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 \
    --req "name: \"x500_custom_1\" position {x: $2 y: $3 z: 1} orientation {w: 1}"
' _ \
  /mnt/px4ssd/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh \
  "${target_x}" "${target_y}" >"${static_setup_log}"
if [[ "$(grep -c 'data: true' "${static_setup_log}")" -ne 3 ]]; then
  echo "static environment service setup failed" >&2
  exit 6
fi
sleep 2

env \
  SWARM_RANGE_DATASET_DIR="${output_root}" \
  SWARM_RANGE_DATASET_RUN_ID="${run_id}" \
  SWARM_RANGE_DATASET_TARGET_ID=UAV-02 \
  SWARM_RANGE_RESIDUAL_MODE=off \
  "${script_dir}/.venv/bin/python" \
  "${script_dir}/range_v2_r3_6a_capture.py" \
  --root "${output_root}" \
  --scenario-id "${scenario_id}" \
  --dataset-role "${dataset_role}" \
  --distance "${distance_m}" \
  --pitch "${pitch_deg}" \
  --bbox-x "${bbox_x}" \
  --bbox-y "${bbox_y}" \
  --bbox-width "${bbox_w}" \
  --bbox-height "${bbox_h}" \
  --prewarm \
  --prewarm-timeout "${prewarm_timeout_s}" \
  --minimum-raw-rows 30 \
  --before-perturbation-rows 8 \
  --capture-timeout "${capture_timeout_s}" \
  --perturbation none \
  >"${output_root}/capture_result.json"

stop_stack
