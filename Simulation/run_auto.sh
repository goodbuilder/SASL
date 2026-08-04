#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

SIM_CONFIG="${SIM_CONFIG:-$ROOT_DIR/config/simulation.env}"
SIM_SCENARIO_CONFIG="${SIM_SCENARIO_CONFIG:-}"
SIM_EXPLICIT_NAMES=(AUTO_FOV_MARGIN_RATIO AUTO_FOV_DETECTION_TIMEOUT_S AUTO_FOV_DETECTIONS_TOPIC)
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/load_sim_config.sh"
load_sim_config_preserving_env "${SIM_EXPLICIT_NAMES[@]}"

usage() {
  cat <<'USAGE'
Usage:
  ./run_auto.sh [options]

Starts a forward-only FOV-guarded mission. Recording is opt-in from the rqt
operator panel: press Save CSV or Save MP4 while Auto is running.

Options:
  --bbox-margin-ratio N         Safe image-edge margin. Default: 0.05.
  --fov-detection-timeout-s N    Maximum age of paired detections. Default: 0.5.
  --help, -h                    Show this help.

All other options are passed to run_mission.sh, including --quick and mission limits.

For offline attitude-error calibration, use ./run_auto_attitude_sweep.sh. It
restarts the simulation for each physical optical-RDF attitude case and uses
the explicit research recorder; normal Auto recording remains rqt-controlled.
USAGE
}

fov_margin_ratio="${AUTO_FOV_MARGIN_RATIO:-0.05}"
fov_detection_timeout_s="${AUTO_FOV_DETECTION_TIMEOUT_S:-0.5}"
mission_args=()

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
    *)
      mission_args+=("$1")
      shift
      ;;
  esac
done

set +e
"$ROOT_DIR/run_mission.sh" \
  --fov-guard \
  --bbox-margin-ratio "$fov_margin_ratio" \
  --fov-detection-timeout-s "$fov_detection_timeout_s" \
  "${mission_args[@]}"
mission_status=$?
set -e
exit "$mission_status"
