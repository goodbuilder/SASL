#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
sim_root=$(git -C "$script_dir" rev-parse --show-toplevel)/apps/Simulation
campaign_root="$sim_root/recordings/stereo_v1_1/paper1_v1_2_001"
output_dir="$campaign_root/outputs/20261021-postfreeze-summary-review"

required_files=(
  "$output_dir/stereo_v1_2_summary_report_ready_v2.xlsx"
  "$campaign_root/analysis_latest.json"
  "$campaign_root/analysis_versions/analysis_9cfd7c6cd10b/analysis_freeze.json"
  "$campaign_root/analysis_versions/analysis_9cfd7c6cd10b/path_selection.json"
  "$campaign_root/analysis_versions/analysis_9cfd7c6cd10b/evaluation_metrics.csv"
  "$campaign_root/analysis_versions/analysis_9cfd7c6cd10b/attitude_variant_comparisons.csv"
)
missing=()
for file in "${required_files[@]}"; do
  [[ -f "$file" ]] || missing+=("${file#"$sim_root/"}")
done
if ((${#missing[@]})); then
  printf 'Cannot build report v3; frozen report inputs are missing:\n' >&2
  printf '  %s\n' "${missing[@]}" >&2
  printf 'Restore the archived paper1_v1_2_001 analysis/report inputs first.\n' >&2
  exit 2
fi

primary_node_modules=${CODEX_PRIMARY_NODE_MODULES:-${HOME}/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules}
if [[ ! -d "$primary_node_modules/@oai/artifact-tool" ]]; then
  printf 'Codex spreadsheet runtime not found at %s\n' "$primary_node_modules" >&2
  printf 'Set CODEX_PRIMARY_NODE_MODULES to that runtime\x27s node_modules directory.\n' >&2
  exit 2
fi
if ! command -v node >/dev/null 2>&1; then
  printf 'Node.js is required to build the report workbook.\n' >&2
  exit 2
fi

tmp_dir=$(mktemp -d "${TMPDIR:-/tmp}/sasl-report-v3.XXXXXX")
cleanup() {
  rm -rf -- "$tmp_dir"
}
trap cleanup EXIT
ln -s -- "$primary_node_modules" "$tmp_dir/node_modules"
cp -- "$script_dir/build_report_workbook_v3.mjs" "$tmp_dir/build_report_workbook_v3.mjs"
cd -- "$sim_root"
node "$tmp_dir/build_report_workbook_v3.mjs"
