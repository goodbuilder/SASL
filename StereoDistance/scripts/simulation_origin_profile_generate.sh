#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
find_sim_dir() {
  if [[ -n "${SASL_ROOT:-}" ]]; then
    printf '%s\n' "$SASL_ROOT/Simulation"
    return 0
  fi
  local dir="$ROOT_DIR"
  while [[ "$dir" != "/" ]]; do
    if [[ -d "$dir/Simulation" ]]; then
      printf '%s\n' "$dir/Simulation"
      return 0
    fi
    dir="$(dirname "$dir")"
  done
  printf '%s\n' "$ROOT_DIR/Simulation"
}
SIM_DIR="${SIM_DIR:-$(find_sim_dir)}"
PYTHON="${PYTHON:-python3}"
OUTPUT_DIR=""
TELEMETRY_CSV=""
VALIDATION_TELEMETRY_CSV=""
MIN_SAMPLES="${ORIGIN_PROFILE_MIN_SAMPLES:-30}"
PROFILE_ID="${ORIGIN_PROFILE_ID:-}"
SPAWN_FRONT_M="${ORIGIN_PROFILE_SPAWN_FRONT_M:-}"
PROFILE_KIND="${ORIGIN_PROFILE_KIND:-origin-bbox}"
MIN_CELL_SAMPLES="${ORIGIN_PROFILE_MIN_CELL_SAMPLES:-50}"
RESIDUAL_GAIN="${ORIGIN_PROFILE_RESIDUAL_GAIN:-0.2}"
MIN_CORRECTION_NORM_M="${ORIGIN_PROFILE_MIN_CORRECTION_NORM_M:-0.25}"
MIN_RANGE_CORRECTION_M="${ORIGIN_PROFILE_MIN_RANGE_CORRECTION_M:-0.25}"
AXIS_MODEL_KINDS="${ORIGIN_PROFILE_AXIS_MODEL_KINDS:-none}"
MIN_AXIS_IMPROVEMENT_M="${ORIGIN_PROFILE_MIN_AXIS_IMPROVEMENT_M:-0.02}"
MIN_AXIS_IMPROVEMENT_RATIO="${ORIGIN_PROFILE_MIN_AXIS_IMPROVEMENT_RATIO:-0.001}"
DISABLED_AXES=()
if [[ "${ORIGIN_PROFILE_DISABLE_AXES+x}" ]]; then
  IFS=',' read -r -a DISABLED_AXES <<< "$ORIGIN_PROFILE_DISABLE_AXES"
else
  DISABLED_AXES=("forward")
fi
DRY_RUN=false

usage() {
  cat <<'USAGE'
Usage:
  scripts/simulation_origin_profile_generate.sh [options]

Flies the Origin Profile path, compares bbox/YOLO Forward/Right/Down values
against Simulation origin Forward/Right/Down truth, and writes an Origin Profile.

Options:
  --output-dir DIR       Directory for telemetry, profile, comparison CSV, summary.
  --telemetry CSV        Use an existing telemetry.csv instead of flying the path.
  --validation-telemetry CSV
                         Use validation telemetry for per-cell/per-axis gain selection.
  --profile-id ID
  --min-samples N
  --profile-kind KIND    alignment or origin-bbox. Default: origin-bbox
  --min-cell-samples N   Minimum samples per Origin-BBOX residual cell. Default: 50
  --residual-gain N      Scale applied to Origin-BBOX residuals. Default: 0.2
  --min-correction-norm-m N
                          Disable vector correction below this post-gain norm. Default: 0.25
  --min-range-correction-m N
                          Disable scalar range correction below this post-gain magnitude. Default: 0.25
  --axis-model-kinds LIST Constant-gain validation by default. Use comma-separated
                         axis_range,axis_poly,axis_screen_poly to enable local regressions.
                         Default: none
  --min-axis-improvement-m N
                         Minimum validation RMSE improvement before applying an axis. Default: 0.02
  --min-axis-improvement-ratio N
                         Minimum validation improvement ratio before applying an axis. Default: 0.001
  --disable-axis AXIS    Disable an Origin-BBOX axis correction. Repeatable.
                         Default disabled axis: forward
  --clear-disabled-axes  Clear the default disabled-axis list before adding --disable-axis.
  --spawn-front-m N      Override DRONE_SPAWN_Y_M for range-relative waypoints.
  --dry-run              Generate the path command only.
USAGE
}

while (($#)); do
  case "$1" in
    --output-dir) OUTPUT_DIR="${2:?missing value for --output-dir}"; shift 2 ;;
    --telemetry) TELEMETRY_CSV="${2:?missing value for --telemetry}"; shift 2 ;;
    --validation-telemetry) VALIDATION_TELEMETRY_CSV="${2:?missing value for --validation-telemetry}"; shift 2 ;;
    --profile-id) PROFILE_ID="${2:?missing value for --profile-id}"; shift 2 ;;
    --min-samples) MIN_SAMPLES="${2:?missing value for --min-samples}"; shift 2 ;;
    --profile-kind) PROFILE_KIND="${2:?missing value for --profile-kind}"; shift 2 ;;
    --min-cell-samples) MIN_CELL_SAMPLES="${2:?missing value for --min-cell-samples}"; shift 2 ;;
    --residual-gain) RESIDUAL_GAIN="${2:?missing value for --residual-gain}"; shift 2 ;;
    --min-correction-norm-m) MIN_CORRECTION_NORM_M="${2:?missing value for --min-correction-norm-m}"; shift 2 ;;
    --min-range-correction-m) MIN_RANGE_CORRECTION_M="${2:?missing value for --min-range-correction-m}"; shift 2 ;;
    --axis-model-kinds) AXIS_MODEL_KINDS="${2:?missing value for --axis-model-kinds}"; shift 2 ;;
    --min-axis-improvement-m) MIN_AXIS_IMPROVEMENT_M="${2:?missing value for --min-axis-improvement-m}"; shift 2 ;;
    --min-axis-improvement-ratio) MIN_AXIS_IMPROVEMENT_RATIO="${2:?missing value for --min-axis-improvement-ratio}"; shift 2 ;;
    --disable-axis) DISABLED_AXES+=("${2:?missing value for --disable-axis}"); shift 2 ;;
    --clear-disabled-axes) DISABLED_AXES=(); shift ;;
    --spawn-front-m) SPAWN_FRONT_M="${2:?missing value for --spawn-front-m}"; shift 2 ;;
    --dry-run) DRY_RUN=true; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$OUTPUT_DIR" ]]; then
  OUTPUT_DIR="$SIM_DIR/bags/origin_profile_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$OUTPUT_DIR"

if [[ -z "$TELEMETRY_CSV" ]]; then
  path_args=()
  if [[ -n "$SPAWN_FRONT_M" ]]; then
    path_args+=(--spawn-front-m "$SPAWN_FRONT_M")
  fi
  if [[ "$DRY_RUN" == true ]]; then
    path_args+=(--dry-run)
  fi
  bash "$ROOT_DIR/scripts/simulation_origin_profile_path.sh" --output-dir "$OUTPUT_DIR/path" "${path_args[@]}"
  if [[ "$DRY_RUN" == true ]]; then
    exit 0
  fi
  TELEMETRY_CSV="$OUTPUT_DIR/path/telemetry/telemetry.csv"
fi

PROFILE_JSON="$OUTPUT_DIR/origin_profile.json"
COMPARISON_CSV="$OUTPUT_DIR/origin_profile_comparison.csv"
SUMMARY_JSON="$OUTPUT_DIR/origin_profile_summary.json"
profile_args=()
if [[ -n "$PROFILE_ID" ]]; then
  profile_args+=(--profile-id "$PROFILE_ID")
fi
if [[ -n "$VALIDATION_TELEMETRY_CSV" ]]; then
  profile_args+=(--validation-telemetry "$VALIDATION_TELEMETRY_CSV")
fi
profile_args+=(--axis-model-kinds "$AXIS_MODEL_KINDS")
profile_args+=(--min-axis-improvement-m "$MIN_AXIS_IMPROVEMENT_M")
profile_args+=(--min-axis-improvement-ratio "$MIN_AXIS_IMPROVEMENT_RATIO")
for axis in "${DISABLED_AXES[@]}"; do
  if [[ -n "$axis" ]]; then
    profile_args+=(--disable-axis "$axis")
  fi
done

"$PYTHON" "$ROOT_DIR/scripts/simulation_origin_tools.py" build-profile \
  --telemetry "$TELEMETRY_CSV" \
  --profile "$PROFILE_JSON" \
  --comparison-csv "$COMPARISON_CSV" \
  --summary-json "$SUMMARY_JSON" \
  --min-samples "$MIN_SAMPLES" \
  --profile-kind "$PROFILE_KIND" \
  --min-cell-samples "$MIN_CELL_SAMPLES" \
  --residual-gain "$RESIDUAL_GAIN" \
  --min-correction-norm-m "$MIN_CORRECTION_NORM_M" \
  --min-range-correction-m "$MIN_RANGE_CORRECTION_M" \
  "${profile_args[@]}"

printf 'Origin profile: %s\n' "$PROFILE_JSON"
printf 'Comparison CSV: %s\n' "$COMPARISON_CSV"
printf 'Summary JSON: %s\n' "$SUMMARY_JSON"
