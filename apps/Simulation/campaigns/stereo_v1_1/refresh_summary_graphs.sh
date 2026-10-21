#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(git -C "$script_dir" rev-parse --show-toplevel)
default_workbook="$repo_root/apps/Simulation/recordings/stereo_v1_1/paper1_v1_2_001/stereo_v1_2_summary.xlsx"

if [[ $# -gt 1 ]]; then
  printf 'Usage: %s [summary-workbook.xlsx]\n' "$0" >&2
  exit 2
fi
workbook=${1:-$default_workbook}
if [[ ! -f "$workbook" ]]; then
  printf 'Summary workbook not found: %s\n' "$workbook" >&2
  printf 'Pass the regenerated .xlsx path as the first argument.\n' >&2
  exit 2
fi
workbook=$(realpath -e -- "$workbook")

primary_node_modules=${CODEX_PRIMARY_NODE_MODULES:-${HOME}/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules}
if [[ ! -d "$primary_node_modules/@oai/artifact-tool" ]]; then
  printf 'Codex spreadsheet runtime not found at %s\n' "$primary_node_modules" >&2
  printf 'Set CODEX_PRIMARY_NODE_MODULES to its node_modules directory.\n' >&2
  exit 2
fi

tmp_dir=$(mktemp -d "${TMPDIR:-/tmp}/sasl-summary-graphs.XXXXXX")
candidate="${workbook}.graph-refresh-${BASHPID}.xlsx"
cleanup() {
  rm -f -- "$candidate"
  rm -f -- "$candidate.inspect.ndjson"
  rm -rf -- "$tmp_dir"
}
trap cleanup EXIT

ln -s -- "$primary_node_modules" "$tmp_dir/node_modules"
cp -- "$script_dir/build_summary_graphs.mjs" "$tmp_dir/build_summary_graphs.mjs"
node "$tmp_dir/build_summary_graphs.mjs" "$workbook" "$candidate" "$tmp_dir/previews"
test -s "$candidate"
mv -- "$candidate" "$workbook"
printf 'Updated graphs: %s\n' "$workbook"
