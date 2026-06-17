#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_DIR="${SIM_DIR:-$(cd "$ROOT_DIR/../.." && pwd)/Simulation}"
PYTHON="${PYTHON:-python3}"
OUTPUT_DIR=""
RANGES_M="${ORIGIN_PROFILE_RANGES_M:-1100,900,700,500,380,320}"
SPEED_MPS="${ORIGIN_PROFILE_SPEED_MPS:-10}"
HOLD_S="${ORIGIN_PROFILE_HOLD_S:-3}"
FINAL_HOVER_S="${ORIGIN_PROFILE_FINAL_HOVER_S:-3}"
ACCEPTANCE_RADIUS_M="${ORIGIN_PROFILE_ACCEPTANCE_RADIUS_M:-4}"
WAYPOINT_TIMEOUT_S="${ORIGIN_PROFILE_WAYPOINT_TIMEOUT_S:-180}"
SPAWN_FRONT_M="${ORIGIN_PROFILE_SPAWN_FRONT_M:-}"
RANDOMIZE=false
RANDOM_SEED="${ORIGIN_PROFILE_RANDOM_SEED:-0}"
RANDOM_WAYPOINT_COUNT="${ORIGIN_PROFILE_RANDOM_WAYPOINT_COUNT:-6}"
RANDOM_RANGE_MIN_M="${ORIGIN_PROFILE_RANDOM_RANGE_MIN_M:-}"
RANDOM_RANGE_MAX_M="${ORIGIN_PROFILE_RANDOM_RANGE_MAX_M:-}"
RANDOM_LATERAL_FRACTION_LIMIT="${ORIGIN_PROFILE_RANDOM_LATERAL_FRACTION_LIMIT:-0.85}"
RANDOM_ALTITUDE_JITTER_M="${ORIGIN_PROFILE_RANDOM_ALTITUDE_JITTER_M:-2.5}"
LAND_AFTER=true
DRY_RUN=false

usage() {
  cat <<'USAGE'
Usage:
  scripts/simulation_origin_profile_path.sh [options]

Creates a StereoDistance-named Forward/Right/Down Origin Profile waypoint path,
then flies it through Simulation/run_px4_waypoint_mission.sh.

Requires Simulation/run_operator_view.sh to already be running unless the caller
starts it separately.

Options:
  --output-dir DIR       Directory for waypoints and telemetry recording.
  --ranges-m CSV         Camera-forward ranges. Default: 1100,900,700,500,380,320
  --randomize            Generate a random overlap-safe waypoint path.
  --random-seed N        Seed for --randomize.
  --random-waypoint-count N
  --random-range-min-m N
  --random-range-max-m N
  --random-lateral-fraction-limit N
  --random-altitude-jitter-m N
  --speed-mps N
  --hold-s N
  --final-hover-s N
  --acceptance-radius-m N
  --waypoint-timeout-s N
  --spawn-front-m N      Override DRONE_SPAWN_Y_M for range-relative waypoints.
  --no-land
  --dry-run              Generate path and print the mission command only.
USAGE
}

while (($#)); do
  case "$1" in
    --output-dir) OUTPUT_DIR="${2:?missing value for --output-dir}"; shift 2 ;;
    --ranges-m) RANGES_M="${2:?missing value for --ranges-m}"; shift 2 ;;
    --randomize) RANDOMIZE=true; shift ;;
    --random-seed) RANDOM_SEED="${2:?missing value for --random-seed}"; shift 2 ;;
    --random-waypoint-count) RANDOM_WAYPOINT_COUNT="${2:?missing value for --random-waypoint-count}"; shift 2 ;;
    --random-range-min-m) RANDOM_RANGE_MIN_M="${2:?missing value for --random-range-min-m}"; shift 2 ;;
    --random-range-max-m) RANDOM_RANGE_MAX_M="${2:?missing value for --random-range-max-m}"; shift 2 ;;
    --random-lateral-fraction-limit) RANDOM_LATERAL_FRACTION_LIMIT="${2:?missing value for --random-lateral-fraction-limit}"; shift 2 ;;
    --random-altitude-jitter-m) RANDOM_ALTITUDE_JITTER_M="${2:?missing value for --random-altitude-jitter-m}"; shift 2 ;;
    --speed-mps) SPEED_MPS="${2:?missing value for --speed-mps}"; shift 2 ;;
    --hold-s) HOLD_S="${2:?missing value for --hold-s}"; shift 2 ;;
    --final-hover-s) FINAL_HOVER_S="${2:?missing value for --final-hover-s}"; shift 2 ;;
    --acceptance-radius-m) ACCEPTANCE_RADIUS_M="${2:?missing value for --acceptance-radius-m}"; shift 2 ;;
    --waypoint-timeout-s) WAYPOINT_TIMEOUT_S="${2:?missing value for --waypoint-timeout-s}"; shift 2 ;;
    --spawn-front-m) SPAWN_FRONT_M="${2:?missing value for --spawn-front-m}"; shift 2 ;;
    --no-land) LAND_AFTER=false; shift ;;
    --dry-run) DRY_RUN=true; shift ;;
    --help|-h) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$OUTPUT_DIR" ]]; then
  OUTPUT_DIR="$SIM_DIR/bags/origin_profile_path_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$OUTPUT_DIR"
WAYPOINT_CSV="$OUTPUT_DIR/origin_profile_waypoints.csv"
RECORD_DIR="$OUTPUT_DIR/telemetry"

write_path_args=(
  "$PYTHON" "$ROOT_DIR/scripts/simulation_origin_tools.py" write-path
  --simulation-dir "$SIM_DIR"
  --ranges-m "$RANGES_M"
  --output "$WAYPOINT_CSV"
)
if [[ -n "$SPAWN_FRONT_M" ]]; then
  write_path_args+=(--spawn-front-m "$SPAWN_FRONT_M")
fi
if [[ "$RANDOMIZE" == true ]]; then
  write_path_args+=(
    --randomize
    --random-seed "$RANDOM_SEED"
    --random-waypoint-count "$RANDOM_WAYPOINT_COUNT"
    --random-lateral-fraction-limit "$RANDOM_LATERAL_FRACTION_LIMIT"
    --random-altitude-jitter-m "$RANDOM_ALTITUDE_JITTER_M"
  )
  if [[ -n "$RANDOM_RANGE_MIN_M" ]]; then
    write_path_args+=(--random-range-min-m "$RANDOM_RANGE_MIN_M")
  fi
  if [[ -n "$RANDOM_RANGE_MAX_M" ]]; then
    write_path_args+=(--random-range-max-m "$RANDOM_RANGE_MAX_M")
  fi
fi
"${write_path_args[@]}"

mission=(
  "$SIM_DIR/run_px4_waypoint_mission.sh"
  --path-file "$WAYPOINT_CSV"
  --record-dir "$RECORD_DIR"
  --speed-mps "$SPEED_MPS"
  --hold-s "$HOLD_S"
  --final-hover-s "$FINAL_HOVER_S"
  --acceptance-radius-m "$ACCEPTANCE_RADIUS_M"
  --waypoint-timeout-s "$WAYPOINT_TIMEOUT_S"
)
if [[ "$LAND_AFTER" == false ]]; then
  mission+=(--no-land)
fi

printf 'Origin profile path: %s\n' "$WAYPOINT_CSV"
printf 'Telemetry output: %s\n' "$RECORD_DIR"
printf 'Command:'
printf ' %q' "${mission[@]}"
printf '\n'

if [[ "$DRY_RUN" == true ]]; then
  exit 0
fi

"${mission[@]}"
