#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

SIM_CONFIG="${SIM_CONFIG:-$ROOT_DIR/config/simulation.env}"
SIM_SCENARIO_CONFIG="${SIM_SCENARIO_CONFIG:-}"
SIM_EXPLICIT_NAMES=(
  AUTO_FOV_MARGIN_RATIO AUTO_FOV_DETECTION_TIMEOUT_S
  CAM_L_OPTICAL_ROLL_DEG CAM_L_OPTICAL_PITCH_DEG CAM_L_OPTICAL_YAW_DEG
  CAM_R_OPTICAL_ROLL_DEG CAM_R_OPTICAL_PITCH_DEG CAM_R_OPTICAL_YAW_DEG
  CAMERA_BASELINE_M CAMERA_HEIGHT_M CAMERA_HORIZONTAL_FOV_RAD
  CAMERA_IMAGE_WIDTH_PX CAMERA_IMAGE_HEIGHT_PX
  AUTO_WAYPOINT_COUNT AUTO_WAYPOINT_MIN_RANGE_M AUTO_WAYPOINT_MAX_RANGE_M
  AUTO_WAYPOINT_SEED
)
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/load_sim_config.sh"
load_sim_config_preserving_env "${SIM_EXPLICIT_NAMES[@]}"

usage() {
  cat <<'USAGE'
Usage:
  ./run_auto.sh [options]

Starts a FOV-guarded mission. Recording is opt-in from the rqt operator
panel: press Save CSV or Save MP4 while Auto is running.

By default this first generates a waypoint approach path with
scripts/generate_fov_constrained_waypoints.py, verified point-by-point
against the actual configured camera optical attitude (CAM_L/R_OPTICAL_*_DEG)
-- not a fixed pitch assumption -- so it works for any camera pose. The
mission then flies that path and stops to hover the instant both cameras'
bboxes fill the safe margin on every side (see px4_auto_mission.py
--waypoints-xy-alt), instead of the older straight-line approach.

Options:
  --bbox-margin-ratio N          Safe image-edge margin. Default: 0.05.
  --fov-detection-timeout-s N    Maximum age of paired detections. Default: 0.5.
  --waypoint-count N             Points to generate for the approach path. Default: 8.
  --waypoint-min-range-m N       Nearest generated point's range, m. Default: 500.
  --waypoint-max-range-m N       Farthest generated point's range, m. Default: 2000.
  --waypoint-seed N              RNG seed for waypoint generation. Default: 20260729.
  --waypoints-dir DIR            Where to write the generated waypoint set.
                                 Default: results/auto_waypoints/<timestamp>.
  --no-generate-waypoints        Skip generation; fall back to run_mission.sh's
                                 own straight-line --altitude-m/--distance-m approach.
  --help, -h                     Show this help.

Passing --path-file, --waypoints-xy-alt, or --waypoint yourself skips
generation and uses exactly what you gave run_mission.sh instead.

All other options are passed to run_mission.sh, including --quick and mission limits.

For offline attitude-error calibration, use ./run_auto_attitude_sweep.sh. It
restarts the simulation for each physical optical-RDF attitude case and uses
the explicit research recorder; normal Auto recording remains rqt-controlled.
USAGE
}

fov_margin_ratio="${AUTO_FOV_MARGIN_RATIO:-0.05}"
fov_detection_timeout_s="${AUTO_FOV_DETECTION_TIMEOUT_S:-0.5}"
waypoint_count="${AUTO_WAYPOINT_COUNT:-8}"
waypoint_min_range_m="${AUTO_WAYPOINT_MIN_RANGE_M:-500}"
waypoint_max_range_m="${AUTO_WAYPOINT_MAX_RANGE_M:-2000}"
waypoint_seed="${AUTO_WAYPOINT_SEED:-20260729}"
waypoints_dir=""
generate_waypoints=true
mission_args=()
mission_supplies_path=false

while (($#)); do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --bbox-margin-ratio)
      fov_margin_ratio="${2:?missing value for $1}"
      shift 2
      ;;
    --fov-detection-timeout-s)
      fov_detection_timeout_s="${2:?missing value for $1}"
      shift 2
      ;;
    --waypoint-count)
      waypoint_count="${2:?missing value for $1}"
      shift 2
      ;;
    --waypoint-min-range-m)
      waypoint_min_range_m="${2:?missing value for $1}"
      shift 2
      ;;
    --waypoint-max-range-m)
      waypoint_max_range_m="${2:?missing value for $1}"
      shift 2
      ;;
    --waypoint-seed)
      waypoint_seed="${2:?missing value for $1}"
      shift 2
      ;;
    --waypoints-dir)
      waypoints_dir="${2:?missing value for $1}"
      shift 2
      ;;
    --no-generate-waypoints)
      generate_waypoints=false
      shift
      ;;
    --path-file|--waypoints-xy-alt|--waypoint)
      mission_supplies_path=true
      mission_args+=("$1" "${2:?missing value for $1}")
      shift 2
      ;;
    *)
      mission_args+=("$1")
      shift
      ;;
  esac
done

path_file=""
if [[ "$generate_waypoints" == true && "$mission_supplies_path" == false ]]; then
  if [[ -z "$waypoints_dir" ]]; then
    waypoints_dir="$ROOT_DIR/results/auto_waypoints/$(date +%Y%m%d_%H%M%S)"
  fi
  pixel_margin_px="$(awk -v w="${CAMERA_IMAGE_WIDTH_PX:-1280}" -v r="$fov_margin_ratio" 'BEGIN { printf "%.6f", w * r }')"
  echo "Generating $waypoint_count FOV-verified waypoint(s) (range ${waypoint_min_range_m}-${waypoint_max_range_m} m)" \
    "for optical attitude L=(${CAM_L_OPTICAL_ROLL_DEG:-0},${CAM_L_OPTICAL_PITCH_DEG:-0},${CAM_L_OPTICAL_YAW_DEG:-0})" \
    "R=(${CAM_R_OPTICAL_ROLL_DEG:-0},${CAM_R_OPTICAL_PITCH_DEG:-0},${CAM_R_OPTICAL_YAW_DEG:-0}) deg -> $waypoints_dir"
  python3 "$ROOT_DIR/scripts/generate_fov_constrained_waypoints.py" \
    --output-dir "$waypoints_dir" \
    --seed "$waypoint_seed" \
    --calibration-count 0 \
    --validation-count "$waypoint_count" \
    --min-range-m "$waypoint_min_range_m" \
    --max-range-m "$waypoint_max_range_m" \
    --camera-baseline-m "${CAMERA_BASELINE_M:-25.0}" \
    --camera-height-m "${CAMERA_HEIGHT_M:-0.1}" \
    --cam-l-optical-roll-deg "${CAM_L_OPTICAL_ROLL_DEG:-0.0}" \
    --cam-l-optical-pitch-deg "${CAM_L_OPTICAL_PITCH_DEG:-0.0}" \
    --cam-l-optical-yaw-deg "${CAM_L_OPTICAL_YAW_DEG:-0.0}" \
    --cam-r-optical-roll-deg "${CAM_R_OPTICAL_ROLL_DEG:-0.0}" \
    --cam-r-optical-pitch-deg "${CAM_R_OPTICAL_PITCH_DEG:-0.0}" \
    --cam-r-optical-yaw-deg "${CAM_R_OPTICAL_YAW_DEG:-0.0}" \
    --horizontal-fov-rad "${CAMERA_HORIZONTAL_FOV_RAD:-0.154997710698}" \
    --image-width-px "${CAMERA_IMAGE_WIDTH_PX:-1280}" \
    --image-height-px "${CAMERA_IMAGE_HEIGHT_PX:-720}" \
    --pixel-margin-px "$pixel_margin_px"
  path_file="$waypoints_dir/validation_mission.csv"
fi

set +e
if [[ -n "$path_file" ]]; then
  "$ROOT_DIR/run_mission.sh" \
    --fov-guard \
    --bbox-margin-ratio "$fov_margin_ratio" \
    --fov-detection-timeout-s "$fov_detection_timeout_s" \
    --path-file "$path_file" \
    "${mission_args[@]}"
else
  "$ROOT_DIR/run_mission.sh" \
    --fov-guard \
    --bbox-margin-ratio "$fov_margin_ratio" \
    --fov-detection-timeout-s "$fov_detection_timeout_s" \
    "${mission_args[@]}"
fi
mission_status=$?
set -e
exit "$mission_status"
