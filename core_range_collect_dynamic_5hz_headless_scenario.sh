#!/usr/bin/env bash
# Headless variant of core_range_collect_dynamic_5hz_scenario.sh (left
# unmodified -- it belongs to the quarantined 2026-08-05 5hz corpus and its
# own manifest references it by name) for
# CORE_RANGE_5HZ_HEADLESS_DYNAMIC_RECOLLECTION_AND_RETRAIN.
#
# Differences from the original, both additive/opt-in and precommitted by
# docs/CORE_RANGE_CAMERA_SOURCE_FPS_REPORT.md's root-cause finding:
#   - SWARM_START_GAZEBO_GUI=false is set unconditionally inside this
#     script (not left to caller convention) so headless is guaranteed
#     regardless of how this script is invoked;
#   - SWARM_CAMERA_TRACE_JSONL is set so camera-source FPS is directly
#     verifiable per session (not just inferred), reusing
#     camera_source_trace.py from the camera-source-FPS task;
#   - a background `gz topic -e -t /world/default/stats` RTF sampler runs
#     for the duration of the capture, reusing the same approach as
#     core_range_camera_source_fps_audit.py's monitor subcommand;
#   - --dataset-role is core_range_dynamic_5hz_headless_post_fixes.
set -Eeuo pipefail

if [[ $# -ne 17 && $# -ne 18 ]]; then
  echo "usage: $0 ROI_POLICY ID START END LATERAL Z YAW_START YAW_END DURATION HOLD RATE PITCH BBOX_X BBOX_Y BBOX_W BBOX_H OUTPUT [legacy_wall_clock|sim_time_plugin]" >&2
  exit 2
fi

roi_policy="$1"; scenario_id="$2"; start_range="$3"; end_range="$4"; lateral="$5"; target_z="$6"
yaw_start="$7"; yaw_end="$8"; duration="$9"; hold="${10}"; rate="${11}"; pitch="${12}"
bbox_x="${13}"; bbox_y="${14}"; bbox_w="${15}"; bbox_h="${16}"; output_root="${17}"
trajectory_driver="${18:-legacy_wall_clock}"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
stack_pid=""
rtf_sampler_pid=""

if [[ -e "${output_root}" ]]; then
  echo "refusing to overwrite existing dynamic root: ${output_root}" >&2
  exit 3
fi
if [[ "${trajectory_driver}" != "legacy_wall_clock" && "${trajectory_driver}" != "sim_time_plugin" ]]; then
  echo "invalid trajectory driver: ${trajectory_driver}" >&2
  exit 9
fi
mkdir -p "${output_root}"

trajectory_version="$("${script_dir}/.venv/bin/python" -c 'from sim_time_trajectory import TRAJECTORY_DRIVER_VERSION; print(TRAJECTORY_DRIVER_VERSION)')"
trajectory_checksum="$("${script_dir}/.venv/bin/python" -c 'from sim_time_trajectory import TRAJECTORY_CONTRACT_SHA256; print(TRAJECTORY_CONTRACT_SHA256)')"
plugin_build_dir="${script_dir}/build/sim_time_trajectory_plugin"
if [[ ! -f "${plugin_build_dir}/libswarm_sim_time_trajectory.so" ]]; then
  echo "sim-time trajectory plugin is not built" >&2
  exit 10
fi

declare -a gz_world_env=()
if [[ "${trajectory_driver}" == "sim_time_plugin" ]]; then
  # GZ_SIM_SERVER_CONFIG_PATH is only consulted by gz-sim as a fallback for
  # worlds with no system plugins of their own; PX4's default.sdf already
  # declares its own systems, so a custom server.config is silently never
  # loaded. The plugin has to be declared directly in the world SDF, so
  # build a one-off overlay copy of the source world here instead.
  px4_root="${PX4_AUTOPILOT_ROOT:-/mnt/px4ssd/PX4-Autopilot}"
  source_world="${px4_root}/Tools/simulation/gz/worlds/default.sdf"
  overlay_world="${output_root}/gz_world_sim_time_trajectory.sdf"
  "${script_dir}/.venv/bin/python" "${script_dir}/sim_time_trajectory_plugin/build_overlay_world.py" \
    "${source_world}" "${overlay_world}"
  gz_world_env=(SWARM_GZ_WORLD_SDF="${overlay_world}")
fi

stop_stack() {
  if [[ -n "${rtf_sampler_pid}" ]] && kill -0 "${rtf_sampler_pid}" 2>/dev/null; then
    kill -TERM "${rtf_sampler_pid}" 2>/dev/null || true
    wait "${rtf_sampler_pid}" 2>/dev/null || true
  fi
  rtf_sampler_pid=""
  if [[ -n "${stack_pid}" ]] && kill -0 "${stack_pid}" 2>/dev/null; then
    kill -TERM "${stack_pid}" 2>/dev/null || true
    local deadline=$((SECONDS + 15))
    while kill -0 "${stack_pid}" 2>/dev/null && ((SECONDS < deadline)); do sleep 0.25; done
    if kill -0 "${stack_pid}" 2>/dev/null; then kill -KILL "${stack_pid}" 2>/dev/null || true; fi
    wait "${stack_pid}" 2>/dev/null || true
  fi
  stack_pid=""
}
start_rtf_sampler() {
  local destination="$1"
  bash -c '
    topic="/world/default/stats"
    while true; do
      echo "--- $(date +%s.%N) ---"
      timeout 2 gz topic -e -t "$topic" -n 1 2>&1
      sleep 1
    done
  ' >"${destination}" 2>&1 &
  rtf_sampler_pid=$!
}
stop_rtf_sampler() {
  if [[ -n "${rtf_sampler_pid}" ]] && kill -0 "${rtf_sampler_pid}" 2>/dev/null; then
    kill -TERM "${rtf_sampler_pid}" 2>/dev/null || true
    wait "${rtf_sampler_pid}" 2>/dev/null || true
  fi
  rtf_sampler_pid=""
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
  GZ_SIM_SYSTEM_PLUGIN_PATH="${plugin_build_dir}" \
  SWARM_SIM_TIME_TRAJECTORY_LOG="${output_root}/trajectory_events.jsonl" \
  SWARM_SIM_TIME_TRAJECTORY_DRIVER_VERSION="${trajectory_version}" \
  SWARM_SIM_TIME_TRAJECTORY_CONTRACT_SHA256="${trajectory_checksum}" \
  SWARM_START_GAZEBO_GUI=false \
  SWARM_CAMERA_TRACE_JSONL="${output_root}/camera_source_trace.jsonl" \
  SWARM_RANGE_DATASET_DIR="${output_root}" \
  SWARM_RANGE_PHYSICAL_DIAGNOSTICS_DIR="${output_root}" \
  SWARM_RANGE_DATASET_RUN_ID="core_dynamic_5hz_headless_${scenario_id}" \
  SWARM_RANGE_DATASET_TARGET_ID=UAV-02 \
  SWARM_RANGE_DATASET_DEPTH_RATE_HZ=5.0 \
  SWARM_RANGE_RESIDUAL_MODE=off \
  "${gz_world_env[@]}" \
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

if pgrep -f "gz sim -g" >/dev/null 2>&1; then
  echo "gazebo GUI client detected despite SWARM_START_GAZEBO_GUI=false" >&2
  exit 8
fi

rtf_precheck_log="${output_root}/gz_rtf_precheck.log"
start_rtf_sampler "${rtf_precheck_log}"

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

# Capture is permitted to start only when the immediately preceding RTF
# window passes the same hard threshold as the final session audit.  Reset
# the sampler afterwards so gz_world_stats.log covers the collection window,
# not stack startup/calibration.
"${script_dir}/.venv/bin/python" "${script_dir}/core_range_collect_dynamic_5hz_headless_batch.py" \
  rtf-check --log "${rtf_precheck_log}" \
  --output-json "${output_root}/rtf_precheck.json" --tail-samples 5
stop_rtf_sampler
rtf_log="${output_root}/gz_world_stats.log"
start_rtf_sampler "${rtf_log}"

env \
  SWARM_START_GAZEBO_GUI=false \
  SWARM_CAMERA_TRACE_JSONL="${output_root}/camera_source_trace.jsonl" \
  SWARM_RANGE_DATASET_DIR="${output_root}" \
  SWARM_RANGE_DATASET_RUN_ID="core_dynamic_5hz_headless_${scenario_id}" \
  SWARM_RANGE_DATASET_TARGET_ID=UAV-02 \
  SWARM_RANGE_DATASET_DEPTH_RATE_HZ=5.0 \
  SWARM_RANGE_RESIDUAL_MODE=off \
  "${recovery_env[@]}" \
  "${script_dir}/.venv/bin/python" "${script_dir}/core_range_dynamic_capture.py" \
  --root "${output_root}" --scenario-id "${scenario_id}" \
  --dataset-role core_range_dynamic_5hz_headless_post_fixes \
  --start-range-m "${start_range}" --end-range-m "${end_range}" \
  --lateral-m "${lateral}" --target-z-m "${target_z}" \
  --yaw-start-deg "${yaw_start}" --yaw-end-deg "${yaw_end}" \
  --movement-duration-s "${duration}" --hold-duration-s "${hold}" \
  --pose-update-rate-hz "${rate}" --pitch-deg "${pitch}" \
  --trajectory-driver "${trajectory_driver}" \
  --bbox-x "${bbox_x}" --bbox-y "${bbox_y}" --bbox-width "${bbox_w}" --bbox-height "${bbox_h}" \
  --minimum-raw-frames 40 --capture-timeout-s 90 \
  >"${output_root}/capture_result.json"

stop_stack
