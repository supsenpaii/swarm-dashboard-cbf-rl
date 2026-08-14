#!/usr/bin/env bash
# Observation-only two-UAV capture at the frozen 5 Hz dataset scheduler rate.
set -Eeuo pipefail

if [[ $# -ne 17 ]]; then
  echo "usage: $0 ROI_POLICY ID START END LATERAL Z YAW_START YAW_END DURATION HOLD RATE PITCH BBOX_X BBOX_Y BBOX_W BBOX_H OUTPUT" >&2
  exit 2
fi

roi_policy="$1"; scenario_id="$2"; start_range="$3"; end_range="$4"; lateral="$5"; target_z="$6"
yaw_start="$7"; yaw_end="$8"; duration="$9"; hold="${10}"; rate="${11}"; pitch="${12}"
bbox_x="${13}"; bbox_y="${14}"; bbox_w="${15}"; bbox_h="${16}"; output_root="${17}"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
stack_pid=""

if [[ -e "${output_root}" ]]; then
  echo "refusing to overwrite existing dynamic root: ${output_root}" >&2
  exit 3
fi
mkdir -p "${output_root}"

stop_stack() {
  if [[ -n "${stack_pid}" ]] && kill -0 "${stack_pid}" 2>/dev/null; then
    kill -TERM "${stack_pid}" 2>/dev/null || true
    local deadline=$((SECONDS + 15))
    while kill -0 "${stack_pid}" 2>/dev/null && ((SECONDS < deadline)); do sleep 0.25; done
    if kill -0 "${stack_pid}" 2>/dev/null; then kill -KILL "${stack_pid}" 2>/dev/null || true; fi
    wait "${stack_pid}" 2>/dev/null || true
  fi
  stack_pid=""
}
cleanup() { local code=$?; trap - EXIT INT TERM; stop_stack; exit "${code}"; }
trap cleanup EXIT INT TERM

initial_x="$(python3 -c 'import math,sys; r=float(sys.argv[1]); y=float(sys.argv[2]); print(math.sqrt(r*r-y*y))' "${start_range}" "${lateral}")"
declare -a recovery_env=()
if [[ "${roi_policy}" == "central_quantile_region" ]]; then
  recovery_env=(SWARM_TARGET_ROI_RECOVERY_POLICY=central_quantile_region)
elif [[ "${roi_policy}" != "default" ]]; then
  echo "invalid ROI policy: ${roi_policy}" >&2
  exit 7
fi

env \
  SWARM_RANGE_DATASET_DIR="${output_root}" \
  SWARM_RANGE_PHYSICAL_DIAGNOSTICS_DIR="${output_root}" \
  SWARM_RANGE_DATASET_RUN_ID="core_dynamic_5hz_${scenario_id}" \
  SWARM_RANGE_DATASET_TARGET_ID=UAV-02 \
  SWARM_RANGE_DATASET_DEPTH_RATE_HZ=5.0 \
  SWARM_RANGE_RESIDUAL_MODE=off \
  "${recovery_env[@]}" \
  SWARM_UAV_01_MODEL_POSE="0,0,1,0,0,0" \
  SWARM_UAV_02_MODEL_POSE="${initial_x},${lateral},${target_z},0,0,0" \
  SWARM_LOG_DIR="${output_root}/runtime_logs" \
  SWARM_RUNTIME_LOG_MAX_BYTES=4194304 \
  "${script_dir}/run_all.sh" >"${output_root}/stack_supervisor.log" 2>&1 &
stack_pid=$!

deadline=$((SECONDS + 60))
while ((SECONDS < deadline)); do
  if curl --fail --silent --max-time 1 http://127.0.0.1:8000/health >/dev/null 2>&1; then break; fi
  if ! kill -0 "${stack_pid}" 2>/dev/null; then echo "stack exited before health gate" >&2; exit 4; fi
  sleep 0.5
done
curl --fail --silent --max-time 1 http://127.0.0.1:8000/health >/dev/null

env PYTHONPATH="${script_dir}/.venv/lib/python3.12/site-packages" \
  "${script_dir}/.venv/bin/python" -c \
  'from range_v2_r3_6a_capture import wait_for_observation_only_ready; wait_for_observation_only_ready()'

setup_log="${output_root}/static_environment_services.log"
bash -c '
  set -euo pipefail; source "$1"
  gz service -s /world/default/set_physics --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout 5000 --req "gravity {x: 0 y: 0 z: 0}"
  gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 --req "name: \"x500_custom_0\" position {x: 0 y: 0 z: 1} orientation {w: 1}"
  gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 --req "name: \"x500_custom_1\" position {x: $2 y: $3 z: $4} orientation {w: 1}"
' _ /mnt/px4ssd/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh "${initial_x}" "${lateral}" "${target_z}" >"${setup_log}"
if [[ "$(grep -c 'data: true' "${setup_log}")" -ne 3 ]]; then echo "dynamic static setup failed" >&2; exit 6; fi
sleep 2

env \
  SWARM_RANGE_DATASET_DIR="${output_root}" \
  SWARM_RANGE_DATASET_RUN_ID="core_dynamic_5hz_${scenario_id}" \
  SWARM_RANGE_DATASET_TARGET_ID=UAV-02 \
  SWARM_RANGE_DATASET_DEPTH_RATE_HZ=5.0 \
  SWARM_RANGE_RESIDUAL_MODE=off \
  "${recovery_env[@]}" \
  "${script_dir}/.venv/bin/python" "${script_dir}/core_range_dynamic_capture.py" \
  --root "${output_root}" --scenario-id "${scenario_id}" \
  --dataset-role core_range_dynamic_5hz_post_contention_fix \
  --start-range-m "${start_range}" --end-range-m "${end_range}" \
  --lateral-m "${lateral}" --target-z-m "${target_z}" \
  --yaw-start-deg "${yaw_start}" --yaw-end-deg "${yaw_end}" \
  --movement-duration-s "${duration}" --hold-duration-s "${hold}" \
  --pose-update-rate-hz "${rate}" --pitch-deg "${pitch}" \
  --bbox-x "${bbox_x}" --bbox-y "${bbox_y}" --bbox-width "${bbox_w}" --bbox-height "${bbox_h}" \
  --minimum-raw-frames 40 --capture-timeout-s 90 \
  >"${output_root}/capture_result.json"

stop_stack
