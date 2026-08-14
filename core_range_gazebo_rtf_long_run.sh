#!/usr/bin/env bash
# CORE_RANGE_GAZEBO_RTF_STUTTER_AND_GATE_VALIDATION section 3: a single,
# PERSISTENT headless Gazebo/PX4/ROS2/backend stack (launched once, NOT
# relaunched between reps -- unlike every prior CORE_RANGE_* collection
# script, which tears the whole stack down and relaunches it fresh for
# every session) kept alive for a minimum of MIN_DURATION_S while repeated
# bounded approaching/receding capture reps run back-to-back against it,
# oscillating the target 11.5<->3.5m so each rep's start range matches the
# previous rep's end range (no reset needed between reps). This is the one
# run in this task designed to distinguish "stutter accumulates within one
# long-lived Gazebo process" from "stutter is a per-launch, machine-wide
# effect" (the prior 5Hz headless task's 13 attempts were ALL fresh-stack
# launches and still showed stutter concentrated late in the batch, so this
# run is the first direct test of the single-process hypothesis).
#
# Always headless: SWARM_START_GAZEBO_GUI=false set unconditionally.

set -Eeuo pipefail

if [[ $# -ne 3 ]]; then
  echo "usage: $0 MIN_DURATION_S REP_MOVEMENT_DURATION_S OUTPUT_ROOT" >&2
  exit 2
fi

min_duration_s="$1"; rep_duration_s="$2"; output_root="$3"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
stack_pid=""
monitor_pid=""

if [[ -e "${output_root}" ]]; then
  echo "refusing to overwrite existing root: ${output_root}" >&2
  exit 3
fi
mkdir -p "${output_root}/reps"

stop_all() {
  if [[ -n "${monitor_pid}" ]] && kill -0 "${monitor_pid}" 2>/dev/null; then
    kill -TERM "${monitor_pid}" 2>/dev/null || true
    wait "${monitor_pid}" 2>/dev/null || true
  fi
  monitor_pid=""
  if [[ -n "${stack_pid}" ]] && kill -0 "${stack_pid}" 2>/dev/null; then
    kill -TERM "${stack_pid}" 2>/dev/null || true
    local deadline=$((SECONDS + 20))
    while kill -0 "${stack_pid}" 2>/dev/null && ((SECONDS < deadline)); do sleep 0.25; done
    if kill -0 "${stack_pid}" 2>/dev/null; then kill -KILL "${stack_pid}" 2>/dev/null || true; fi
    wait "${stack_pid}" 2>/dev/null || true
  fi
  stack_pid=""
}
cleanup() { local code=$?; trap - EXIT INT TERM; stop_all; exit "${code}"; }
trap cleanup EXIT INT TERM

export SWARM_START_GAZEBO_GUI=false
export SWARM_CAMERA_TRACE_JSONL="${output_root}/camera_source_trace.jsonl"

env \
  SWARM_RANGE_DATASET_DIR="${output_root}/shared" \
  SWARM_RANGE_PHYSICAL_DIAGNOSTICS_DIR="${output_root}/shared" \
  SWARM_RANGE_DATASET_RUN_ID="core_rtf_long_run" \
  SWARM_RANGE_DATASET_TARGET_ID=UAV-02 \
  SWARM_RANGE_DATASET_DEPTH_RATE_HZ=5.0 \
  SWARM_RANGE_RESIDUAL_MODE=off \
  SWARM_UAV_01_MODEL_POSE="0,0,1,0,0,0" \
  SWARM_UAV_02_MODEL_POSE="11.5,0,1,0,0,0" \
  SWARM_LOG_DIR="${output_root}/runtime_logs" \
  SWARM_RUNTIME_LOG_MAX_BYTES=8388608 \
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

env PYTHONPATH="${script_dir}/.venv/lib/python3.12/site-packages" \
  "${script_dir}/.venv/bin/python" -c \
  'from range_v2_r3_6a_capture import wait_for_observation_only_ready; wait_for_observation_only_ready()'

setup_log="${output_root}/static_environment_services.log"
bash -c '
  set -euo pipefail; source "$1"
  gz service -s /world/default/set_physics --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean --timeout 5000 --req "gravity {x: 0 y: 0 z: 0}"
  gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 --req "name: \"x500_custom_0\" position {x: 0 y: 0 z: 1} orientation {w: 1}"
  gz service -s /world/default/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 --req "name: \"x500_custom_1\" position {x: 11.5 y: 0 z: 1} orientation {w: 1}"
' _ /mnt/px4ssd/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh >"${setup_log}"
if [[ "$(grep -c 'data: true' "${setup_log}")" -ne 3 ]]; then echo "static setup failed" >&2; exit 6; fi
sleep 2

env -u PYTHONPATH "${script_dir}/.venv/bin/python" "${script_dir}/core_range_gazebo_rtf_long_run_monitor.py" \
  --output "${output_root}" --duration-s "$((min_duration_s + 300))" \
  >"${output_root}/monitor_stdout.log" 2>&1 &
monitor_pid=$!

# IMPORTANT: the backend (main.py/uvicorn) was launched once above with its
# diagnostics/dataset directory fixed at ${output_root}/shared -- that env
# var cannot be changed per-rep on an already-running process, so every rep
# below MUST target the SAME shared directory (not a fresh per-rep one) or
# core_range_dynamic_capture.py's wait_for_new_sidecar_session() polls a
# directory the backend never writes to and times out
# (new_sidecar_session_not_observed). Reps are distinguished by session_id
# (assigned fresh by the backend each capture) and scenario_id, both
# recorded in the single shared physical_diagnostics.jsonl/capture_events.jsonl.
shared_root="${output_root}/shared"
run_started=$SECONDS
rep=0
direction="approach"
while (( SECONDS - run_started < min_duration_s )); do
  rep=$((rep + 1))
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
    SWARM_RANGE_DATASET_RUN_ID="core_rtf_long_run_rep_${rep}" \
    SWARM_RANGE_DATASET_TARGET_ID=UAV-02 \
    SWARM_RANGE_DATASET_DEPTH_RATE_HZ=5.0 \
    SWARM_RANGE_RESIDUAL_MODE=off \
    "${script_dir}/.venv/bin/python" "${script_dir}/core_range_dynamic_capture.py" \
    --root "${shared_root}" --scenario-id "rtf_long_run_rep_${rep}_${direction}" \
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

echo "long-run capture loop complete: ${rep} reps, elapsed $((SECONDS - run_started))s" | tee -a "${output_root}/rep_log.txt"

stop_all
