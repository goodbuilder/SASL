#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

resolve_executable() {
  local candidate resolved
  for candidate in "$@"; do
    [[ -n "$candidate" ]] || continue
    if [[ "$candidate" == */* ]]; then
      if [[ -x "$candidate" ]]; then
        printf '%s\n' "$candidate"
        return 0
      fi
      continue
    fi
    resolved="$(command -v "$candidate" 2>/dev/null || true)"
    if [[ -n "$resolved" && -x "$resolved" ]]; then
      printf '%s\n' "$resolved"
      return 0
    fi
  done
  return 1
}

resolve_pymavlink_python() {
  local candidate executable
  for candidate in "$@"; do
    executable="$(resolve_executable "$candidate" || true)"
    [[ -n "$executable" ]] || continue
    if "$executable" -c 'import pymavlink' >/dev/null 2>&1; then
      printf '%s\n' "$executable"
      return 0
    fi
  done
  return 1
}

MISSION_PYTHON="${MISSION_PYTHON:-}"

SIM_CONFIG="${SIM_CONFIG:-$ROOT_DIR/config/simulation.env}"
SIM_SCENARIO_CONFIG="${SIM_SCENARIO_CONFIG:-}"
SIM_EXPLICIT_NAMES=(
  DRONE_TAKEOFF_ALTITUDE_M DRONE_FORWARD_SPEED_MPS DRONE_MOVE_DISTANCE_M
  DRONE_TARGET_DISTANCE_M DRONE_APPROACH_TIMEOUT_S DRONE_TAKEOFF_TIMEOUT_S
  DRONE_FIRST_HOVER_S DRONE_FINAL_HOVER_S DRONE_LAND_TIMEOUT_S
  DRONE_SETPOINT_HZ AUTO_FOV_MARGIN_RATIO AUTO_FOV_DETECTION_TIMEOUT_S
  AUTO_FOV_DETECTIONS_TOPIC DRONE_LATERAL_BIAS_M
  DRONE_LATERAL_SWEEP_AMPLITUDE_M DRONE_LATERAL_SWEEP_PERIOD_S
  DRONE_MPC_Z_VEL_MAX_UP_MPS DRONE_MPC_Z_VEL_MAX_DN_MPS
  DRONE_WAYPOINT_ACCEPTANCE_RADIUS_M
)
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/load_sim_config.sh"
load_sim_config_preserving_env "${SIM_EXPLICIT_NAMES[@]}"

usage() {
  cat <<'USAGE'
Usage:
  ./run_mission.sh [options]

Starts the drone auto mission.

Default:
  Send the auto takeoff/approach/land mission to an already running PX4 SITL.
  Start Gazebo/rqt/PX4 first with ./run_operator_view.sh.

Options:
  --quick                Use shorter smoke-test mission settings.
  --target-distance-m N  Stop approach when /ground_truth is at or below N. Default: 200.
  --altitude-m N         Takeoff altitude.
  --speed-mps N          Forward approach speed.
  --distance-m N         Fallback max movement if /ground_truth is unavailable.
  --approach-timeout-s N Approach timeout in seconds.
  --mpc-z-vel-max-up-mps N
                         Override PX4 MPC_Z_VEL_MAX_UP (climb limit) before takeoff.
  --mpc-z-vel-max-dn-mps N
                         Override PX4 MPC_Z_VEL_MAX_DN (descent limit) before takeoff.
  --lateral-bias-m N     Constant lateral offset (m) the approach holds the target
                         away from image-center. Default: 0 (centered).
  --lateral-sweep-amplitude-m N  Triangle-wave lateral sweep amplitude (m) added on
                         top of --lateral-bias-m during the approach. Default: 0 (off).
  --lateral-sweep-period-s N     Sweep period in seconds. Default: 24.
  --connect STRING       pymavlink connection string. Default: udpin:0.0.0.0:14540
  --fov-guard            Require both full detection bboxes inside the safe FOV area.
  --bbox-margin-ratio N  Inset each image edge by N before forward motion. Default: 0.05.
  --fov-detection-timeout-s N  Maximum age of the paired detections. Default: 0.5.
  --waypoints-xy-alt LIST  Semicolon-separated north,east,altitude_agl_m path. With
                         --fov-guard, flies this path (instead of the straight
                         --altitude-m/--distance-m approach) and stops to hover in
                         place the instant both cameras' bboxes span the full safe
                         box on every side. Must be passed explicitly (not read
                         from WAYPOINTS_XY_ALT_M, which run_px4_waypoint_mission.sh
                         also uses for its own, unrelated mission).
  --path-file FILE       Read waypoints from a north,east,altitude_agl_m CSV/text
                         file instead of --waypoints-xy-alt.
  --waypoint-acceptance-radius-m N  Distance (m) at which a waypoint counts as
                         reached. Default: 5.
  --check-connection     Check PX4 connection without moving the drone.
  --help, -h             Show this help.

Examples:
  ./run_operator_view.sh --record-lite
  ./run_mission.sh
  ./run_mission.sh --quick
  ./run_mission.sh --target-distance-m 200 --altitude-m 10 --speed-mps 15
USAGE
}

mission_args=()
provided_args=()

has_provided_arg() {
  local name="$1"
  local arg
  for arg in "${provided_args[@]}"; do
    [[ "$arg" == "$name" ]] && return 0
  done
  return 1
}

add_default_arg() {
  local name="$1"
  local value="$2"
  if [[ -n "${value:-}" ]] && ! has_provided_arg "$name"; then
    mission_args+=("$name" "$value")
  fi
}

while (($#)); do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --quick)
      provided_args+=(
        "--target-distance-m"
        "--speed-mps"
        "--first-hover-s"
        "--final-hover-s"
        "--approach-timeout-s"
      )
      mission_args+=(
        "--target-distance-m"
        "200"
        "--speed-mps"
        "30"
        "--first-hover-s"
        "1"
        "--final-hover-s"
        "1"
        "--approach-timeout-s"
        "600"
      )
      shift
      ;;
    --check-connection)
      mission_args+=(--check-connection)
      shift
      ;;
    --connect)
      provided_args+=("$1")
      mission_args+=("$1" "${2:?missing value for $1}")
      shift 2
      ;;
    --fov-guard)
      mission_args+=("$1")
      shift
      ;;
    --bbox-margin-ratio|--fov-detection-timeout-s)
      provided_args+=("$1")
      mission_args+=("$1" "${2:?missing value for $1}")
      shift 2
      ;;
    --target-distance-m|--altitude-m|--speed-mps|--approach-timeout-s)
      provided_args+=("$1")
      mission_args+=("$1" "${2:?missing value for $1}")
      shift 2
      ;;
    --lateral-bias-m|--lateral-sweep-amplitude-m|--lateral-sweep-period-s)
      provided_args+=("$1")
      mission_args+=("$1" "${2:?missing value for $1}")
      shift 2
      ;;
    --mpc-z-vel-max-up-mps|--mpc-z-vel-max-dn-mps)
      provided_args+=("$1")
      mission_args+=("$1" "${2:?missing value for $1}")
      shift 2
      ;;
    --waypoints-xy-alt)
      provided_args+=("$1")
      # A single "=" joined token, since a leading "-100,..." waypoint would
      # otherwise look like an unrecognized option to Python's argparse.
      mission_args+=("--waypoints-xy-alt=${2:?missing value for $1}")
      shift 2
      ;;
    --waypoint)
      provided_args+=("$1")
      mission_args+=("--waypoint=${2:?missing value for $1}")
      shift 2
      ;;
    --path-file)
      provided_args+=("$1")
      mission_args+=("$1" "${2:?missing value for $1}")
      shift 2
      ;;
    --waypoint-acceptance-radius-m)
      provided_args+=("$1")
      mission_args+=("$1" "${2:?missing value for $1}")
      shift 2
      ;;
    --distance-m|--takeoff-timeout-s|--first-hover-s|--final-hover-s|--land-timeout-s|--setpoint-hz)
      provided_args+=("$1")
      mission_args+=("$1" "${2:?missing value for $1}")
      shift 2
      ;;
    *)
      echo "unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

add_default_arg --altitude-m "${DRONE_TAKEOFF_ALTITUDE_M:-}"
add_default_arg --speed-mps "${DRONE_FORWARD_SPEED_MPS:-}"
add_default_arg --distance-m "${DRONE_MOVE_DISTANCE_M:-}"
add_default_arg --target-distance-m "${DRONE_TARGET_DISTANCE_M:-}"
add_default_arg --approach-timeout-s "${DRONE_APPROACH_TIMEOUT_S:-}"
add_default_arg --takeoff-timeout-s "${DRONE_TAKEOFF_TIMEOUT_S:-}"
add_default_arg --first-hover-s "${DRONE_FIRST_HOVER_S:-}"
add_default_arg --final-hover-s "${DRONE_FINAL_HOVER_S:-}"
add_default_arg --land-timeout-s "${DRONE_LAND_TIMEOUT_S:-}"
add_default_arg --setpoint-hz "${DRONE_SETPOINT_HZ:-}"
add_default_arg --lateral-bias-m "${DRONE_LATERAL_BIAS_M:-}"
add_default_arg --lateral-sweep-amplitude-m "${DRONE_LATERAL_SWEEP_AMPLITUDE_M:-}"
add_default_arg --lateral-sweep-period-s "${DRONE_LATERAL_SWEEP_PERIOD_S:-}"
add_default_arg --mpc-z-vel-max-up-mps "${DRONE_MPC_Z_VEL_MAX_UP_MPS:-}"
add_default_arg --mpc-z-vel-max-dn-mps "${DRONE_MPC_Z_VEL_MAX_DN_MPS:-}"
# WAYPOINTS_XY_ALT_M is deliberately NOT wired as an automatic default here:
# it's also read by run_px4_waypoint_mission.sh, and this script's plain
# (non-waypoint) callers -- e.g. run_operator_view.sh's --auto-mission,
# which never passes --fov-guard -- must keep working. The waypoint+FOV-hover
# flow only activates when --waypoints-xy-alt/--waypoint is explicitly passed.
add_default_arg --waypoint-acceptance-radius-m "${DRONE_WAYPOINT_ACCEPTANCE_RADIUS_M:-}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT_DIR/install/setup.bash"
fi
set -u

MISSION_PYTHON="$(resolve_pymavlink_python \
  "$MISSION_PYTHON" \
  "${PYTHON_EXECUTABLE:-}" \
  "$ROOT_DIR/.venv/bin/python" \
  "$ROOT_DIR/.venv-px4-local/bin/python" \
  "$ROOT_DIR/.venv-px4-v1.17.0/bin/python" \
  python3 || true
)"
if [[ -z "$MISSION_PYTHON" ]]; then
  echo "error: px4_auto_mission.py requires a Python interpreter with pymavlink." >&2
  echo "Set MISSION_PYTHON or install pymavlink in apps/Simulation/.venv." >&2
  exit 2
fi

exec "$MISSION_PYTHON" "$ROOT_DIR/scripts/px4_auto_mission.py" "${mission_args[@]}"
