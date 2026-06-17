#!/usr/bin/env bash
set -euo pipefail

CALLER_CWD="$(pwd)"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

usage() {
  cat <<'USAGE'
Usage:
  ./run_estimated_to_truth_from_csv.sh [options] [calibration.csv]

Runs the recorded calibration CSV promotion flow:
  preflight -> candidate search -> runtime profile replay -> promotion check -> PASS gate

Options:
  --input-csv FILE       Calibration CSV. If omitted, use the newest CSV under results/calibration_flight.
  --output-dir DIR       Output directory. Default: results/estimated_to_truth_calibration_<timestamp>
  --target-range-m M     Target range for PASS statistics. Default: 1852.
  --target-window-m M    Half-width around target range. Default: 30.
  --min-target-count N   Minimum validation target-window rows. Default: 3.
  --strict-preflight     Stop before fitting if preflight target coverage is insufficient.
  --help, -h             Show this help.

Examples:
  ./run_operator_view.sh --record-calibration-csv results/calibration_flight/telemetry.csv --calibration-sim-truth
  ./run_estimated_to_truth_from_csv.sh results/calibration_flight/telemetry.csv
  ./run_estimated_to_truth_from_csv.sh --strict-preflight
USAGE
}

input_csv=""
output_dir=""
target_range_m="1852"
target_window_m="30"
min_target_count="3"
strict_preflight=false

while (($#)); do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --input-csv)
      input_csv="${2:?missing value for --input-csv}"
      shift 2
      ;;
    --output-dir)
      output_dir="${2:?missing value for --output-dir}"
      shift 2
      ;;
    --target-range-m)
      target_range_m="${2:?missing value for --target-range-m}"
      shift 2
      ;;
    --target-window-m)
      target_window_m="${2:?missing value for --target-window-m}"
      shift 2
      ;;
    --min-target-count)
      min_target_count="${2:?missing value for --min-target-count}"
      shift 2
      ;;
    --strict-preflight)
      strict_preflight=true
      shift
      ;;
    --*)
      echo "unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      if [[ -n "$input_csv" ]]; then
        echo "error: input CSV was provided more than once" >&2
        exit 2
      fi
      input_csv="$1"
      shift
      ;;
  esac
done

if [[ -z "$input_csv" ]]; then
  input_csv="$(
    find "$ROOT_DIR/results/calibration_flight" -type f -name '*.csv' -printf '%T@ %p\n' 2>/dev/null \
      | sort -nr \
      | awk 'NR == 1 { $1=""; sub(/^ /, ""); print; exit }'
  )"
fi

if [[ -n "$input_csv" && "$input_csv" != /* && ! -f "$input_csv" && -f "$CALLER_CWD/$input_csv" ]]; then
  input_csv="$CALLER_CWD/$input_csv"
fi

if [[ -z "$input_csv" ]]; then
  echo "error: no calibration CSV found. Pass --input-csv or record one with --record-calibration-csv." >&2
  exit 2
fi

if [[ ! -f "$input_csv" ]]; then
  echo "error: calibration CSV not found: $input_csv" >&2
  exit 2
fi

if [[ -z "$output_dir" ]]; then
  output_dir="$ROOT_DIR/results/estimated_to_truth_calibration_$(date +%Y%m%d_%H%M%S)"
elif [[ "$output_dir" != /* ]]; then
  output_dir="$CALLER_CWD/$output_dir"
fi

python_bin="$ROOT_DIR/.venv/bin/python"
if [[ ! -x "$python_bin" ]]; then
  python_bin="$(command -v python3)"
fi

export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/matplotlib-estimated-to-truth}"

cmd=(
  "$python_bin"
  "$ROOT_DIR/scripts/run_estimated_to_truth_calibration_exp.py"
  --mode csv
  --input-csv "$input_csv"
  --output-dir "$output_dir"
  --target-range-m "$target_range_m"
  --target-window-m "$target_window_m"
  --min-target-count "$min_target_count"
)
if [[ "$strict_preflight" == true ]]; then
  cmd+=(--strict-preflight)
fi

printf '%s\n' "Calibration CSV: $input_csv"
printf '%s\n' "Output directory: $output_dir"
printf '%s\n' "Target range/window: ${target_range_m} m +/- ${target_window_m} m"
printf '%s\n' "Minimum target-window count: $min_target_count"
printf '%s\n' "Command:"
printf '  %q' "${cmd[@]}"
printf '\n'

"${cmd[@]}"

printf '%s\n' "Done."
printf '%s\n' "Summary: $output_dir/promotion_summary.txt"
printf '%s\n' "Metrics: $output_dir/metrics.json"
