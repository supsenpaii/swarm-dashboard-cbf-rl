#!/usr/bin/env bash
# Interleaved post-fix validation: lazy production against the old eager mode.
set -Eeuo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "usage: $0 OUTPUT_ROOT [DURATION_S]" >&2
  exit 2
fi

output_root="$1"
duration_s="${2:-600}"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

mkdir -p "${output_root}"
for repeat in 1 2 3; do
  for config_id in S6_on_demand S5; do
    run_dir="${output_root}/runs/${config_id}_run${repeat}"
    if [[ -f "${run_dir}/exit_code.txt" ]] && [[ "$(<"${run_dir}/exit_code.txt")" == 0 ]]; then
      echo "already complete: ${config_id} repeat ${repeat}"
      continue
    fi
    "${script_dir}/core_range_subscription_ablation_run.sh" \
      "${config_id}" "${repeat}" "${duration_s}" "${output_root}"
  done
done

"${script_dir}/.venv/bin/python" \
  "${script_dir}/core_range_subscription_fix_validation_analyze.py" \
  --root "${output_root}"
