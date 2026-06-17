#!/usr/bin/env bash
set -euo pipefail

CALLER_CWD="$(pwd)"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

usage() {
  cat <<'USAGE'
Usage:
  ./run_operator_view_with_calibration_profile.sh [options] [-- run_operator_view args...]

Starts operator view with an exported estimated-to-truth StereoDistance runtime profile.

Options:
  --profile-json FILE    Runtime profile JSON. Default: newest sim3_runtime_profile.json under results/.
  --results-root DIR     Search root for latest profile. Default: results
  --print-only           Print the resolved command without running it.
  --help, -h             Show this help.

Examples:
  ./run_operator_view_with_calibration_profile.sh
  ./run_operator_view_with_calibration_profile.sh --profile-json results/estimated_to_truth_calibration_exp/sim3_runtime_profile.json
  ./run_operator_view_with_calibration_profile.sh -- --raw-view --no-px4
USAGE
}

profile_json=""
results_root="$ROOT_DIR/results"
print_only=false
operator_args=()

while (($#)); do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --profile-json)
      profile_json="${2:?missing value for --profile-json}"
      shift 2
      ;;
    --results-root)
      results_root="${2:?missing value for --results-root}"
      shift 2
      ;;
    --print-only)
      print_only=true
      shift
      ;;
    --)
      shift
      operator_args+=("$@")
      break
      ;;
    *)
      operator_args+=("$1")
      shift
      ;;
  esac
done

if [[ "$results_root" != /* ]]; then
  if [[ -d "$results_root" ]]; then
    results_root="$ROOT_DIR/$results_root"
  else
    results_root="$CALLER_CWD/$results_root"
  fi
fi

if [[ -n "$profile_json" && "$profile_json" != /* ]]; then
  if [[ -f "$profile_json" ]]; then
    profile_json="$ROOT_DIR/$profile_json"
  elif [[ -f "$CALLER_CWD/$profile_json" ]]; then
    profile_json="$CALLER_CWD/$profile_json"
  fi
fi

if [[ -z "$profile_json" ]]; then
  profile_json="$(
    find "$results_root" -type f -name 'sim3_runtime_profile.json' -printf '%T@ %p\n' 2>/dev/null \
      | sort -nr \
      | awk 'NR == 1 { $1=""; sub(/^ /, ""); print; exit }'
  )"
fi

if [[ -z "$profile_json" ]]; then
  echo "error: no sim3_runtime_profile.json found under $results_root" >&2
  echo "Run ./run_estimated_to_truth_from_csv.sh first or pass --profile-json." >&2
  exit 2
fi

if [[ ! -f "$profile_json" ]]; then
  echo "error: runtime profile not found: $profile_json" >&2
  exit 2
fi

cmd=(
  "$ROOT_DIR/run_operator_view.sh"
  --bbox-adjustment
  "$profile_json"
)
cmd+=("${operator_args[@]}")

printf '%s\n' "Runtime profile: $profile_json"
printf '%s\n' "Command:"
printf '  %q' "${cmd[@]}"
printf '\n'

if [[ "$print_only" == true ]]; then
  exit 0
fi

exec "${cmd[@]}"
