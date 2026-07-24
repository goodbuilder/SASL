#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

SIM_CONFIG="${SIM_CONFIG:-$ROOT_DIR/config/simulation.env}"
OPERATOR_RECORDER_PYTHON="${OPERATOR_RECORDER_PYTHON:-}"

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

if ! OPERATOR_RECORDER_PYTHON="$(resolve_executable \
  "$OPERATOR_RECORDER_PYTHON" \
  "$ROOT_DIR/.venv/bin/python" \
  "$ROOT_DIR/.venv/bin/python3" \
  python3
)"; then
  OPERATOR_RECORDER_PYTHON="python3"
fi

DRONE_TAKEOFF_ALTITUDE_M_FROM_ENV="${DRONE_TAKEOFF_ALTITUDE_M+x}"; DRONE_TAKEOFF_ALTITUDE_M_ENV_VALUE="${DRONE_TAKEOFF_ALTITUDE_M:-}"
DRONE_FORWARD_SPEED_MPS_FROM_ENV="${DRONE_FORWARD_SPEED_MPS+x}"; DRONE_FORWARD_SPEED_MPS_ENV_VALUE="${DRONE_FORWARD_SPEED_MPS:-}"
DRONE_MOVE_DISTANCE_M_FROM_ENV="${DRONE_MOVE_DISTANCE_M+x}"; DRONE_MOVE_DISTANCE_M_ENV_VALUE="${DRONE_MOVE_DISTANCE_M:-}"
DRONE_TARGET_DISTANCE_M_FROM_ENV="${DRONE_TARGET_DISTANCE_M+x}"; DRONE_TARGET_DISTANCE_M_ENV_VALUE="${DRONE_TARGET_DISTANCE_M:-}"
DRONE_APPROACH_TIMEOUT_S_FROM_ENV="${DRONE_APPROACH_TIMEOUT_S+x}"; DRONE_APPROACH_TIMEOUT_S_ENV_VALUE="${DRONE_APPROACH_TIMEOUT_S:-}"
DRONE_TAKEOFF_TIMEOUT_S_FROM_ENV="${DRONE_TAKEOFF_TIMEOUT_S+x}"; DRONE_TAKEOFF_TIMEOUT_S_ENV_VALUE="${DRONE_TAKEOFF_TIMEOUT_S:-}"
DRONE_FIRST_HOVER_S_FROM_ENV="${DRONE_FIRST_HOVER_S+x}"; DRONE_FIRST_HOVER_S_ENV_VALUE="${DRONE_FIRST_HOVER_S:-}"
DRONE_FINAL_HOVER_S_FROM_ENV="${DRONE_FINAL_HOVER_S+x}"; DRONE_FINAL_HOVER_S_ENV_VALUE="${DRONE_FINAL_HOVER_S:-}"
DRONE_LAND_TIMEOUT_S_FROM_ENV="${DRONE_LAND_TIMEOUT_S+x}"; DRONE_LAND_TIMEOUT_S_ENV_VALUE="${DRONE_LAND_TIMEOUT_S:-}"
DRONE_SETPOINT_HZ_FROM_ENV="${DRONE_SETPOINT_HZ+x}"; DRONE_SETPOINT_HZ_ENV_VALUE="${DRONE_SETPOINT_HZ:-}"

if [[ ! -f "$SIM_CONFIG" ]]; then
  echo "error: SIM_CONFIG not found: $SIM_CONFIG" >&2
  exit 2
fi
if [[ -f "$SIM_CONFIG" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "$SIM_CONFIG"
  set +a
fi
[[ -n "$DRONE_TAKEOFF_ALTITUDE_M_FROM_ENV" ]] && DRONE_TAKEOFF_ALTITUDE_M="$DRONE_TAKEOFF_ALTITUDE_M_ENV_VALUE"
[[ -n "$DRONE_FORWARD_SPEED_MPS_FROM_ENV" ]] && DRONE_FORWARD_SPEED_MPS="$DRONE_FORWARD_SPEED_MPS_ENV_VALUE"
[[ -n "$DRONE_MOVE_DISTANCE_M_FROM_ENV" ]] && DRONE_MOVE_DISTANCE_M="$DRONE_MOVE_DISTANCE_M_ENV_VALUE"
[[ -n "$DRONE_TARGET_DISTANCE_M_FROM_ENV" ]] && DRONE_TARGET_DISTANCE_M="$DRONE_TARGET_DISTANCE_M_ENV_VALUE"
[[ -n "$DRONE_APPROACH_TIMEOUT_S_FROM_ENV" ]] && DRONE_APPROACH_TIMEOUT_S="$DRONE_APPROACH_TIMEOUT_S_ENV_VALUE"
[[ -n "$DRONE_TAKEOFF_TIMEOUT_S_FROM_ENV" ]] && DRONE_TAKEOFF_TIMEOUT_S="$DRONE_TAKEOFF_TIMEOUT_S_ENV_VALUE"
[[ -n "$DRONE_FIRST_HOVER_S_FROM_ENV" ]] && DRONE_FIRST_HOVER_S="$DRONE_FIRST_HOVER_S_ENV_VALUE"
[[ -n "$DRONE_FINAL_HOVER_S_FROM_ENV" ]] && DRONE_FINAL_HOVER_S="$DRONE_FINAL_HOVER_S_ENV_VALUE"
[[ -n "$DRONE_LAND_TIMEOUT_S_FROM_ENV" ]] && DRONE_LAND_TIMEOUT_S="$DRONE_LAND_TIMEOUT_S_ENV_VALUE"
[[ -n "$DRONE_SETPOINT_HZ_FROM_ENV" ]] && DRONE_SETPOINT_HZ="$DRONE_SETPOINT_HZ_ENV_VALUE"

usage() {
  cat <<'USAGE'
Usage:
  ./run_px4_waypoint_mission.sh [options]

Sends a PX4 offboard waypoint mission to an already running PX4 SITL.
Start Gazebo/rqt/PX4 first with ./run_operator_view.sh.

Defaults are read from config/simulation.env.
Command-line options override the config file for one run.

Waypoint format:
  north,east,altitude_agl_m

Options:
  --waypoint X,Y,ALT     Add one waypoint. Repeatable.
  --path-file FILE       Read waypoints from CSV/text file.
  --speed-mps N          Horizontal speed limit.
  --acceptance-radius-m N
                         Waypoint reached radius.
  --waypoint-timeout-s N Timeout per waypoint.
  --takeoff-timeout-s N Takeoff timeout.
  --hold-s N             Hold time at each waypoint.
  --final-hover-s N      Hover time before optional landing.
  --land-timeout-s N     Landing timeout.
  --setpoint-hz N        PX4 offboard setpoint rate.
  --land-after           Land after the final waypoint. Default from config.
  --no-land              Do not land after the final waypoint.
  --record-dir DIR       Save lite CSV telemetry to DIR. Default: bags/waypoint_TIMESTAMP_lite.
  --no-record            Disable automatic lite CSV recording.
  --connect STRING       pymavlink connection string.
  --check-connection     Check PX4 connection without moving.
  --help, -h             Show this help.

Examples:
  ./run_px4_waypoint_mission.sh
  ./run_px4_waypoint_mission.sh --waypoint -300,0,10 --waypoint -600,0,10
  SIM_CONFIG=config/my_simulation.env ./run_px4_waypoint_mission.sh
USAGE
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

args=()
mission_cli=()
cli_sets_waypoints=false
check_connection=false
record_lite="${WAYPOINT_RECORD_LITE:-true}"
record_dir="${WAYPOINT_RECORD_DIR:-}"

while (($#)); do
  case "$1" in
    --record-dir)
      record_lite=true
      record_dir="${2:?missing value for --record-dir}"
      shift 2
      ;;
    --no-record)
      record_lite=false
      shift
      ;;
    --waypoint|--path-file|--waypoints-xy-alt|--speed-mps|--acceptance-radius-m|--waypoint-timeout-s|--takeoff-timeout-s|--hold-s|--final-hover-s|--land-timeout-s|--setpoint-hz|--connect)
      mission_cli+=("$1=${2:?missing value for $1}")
      shift 2
      ;;
    --check-connection)
      check_connection=true
      mission_cli+=("$1")
      shift
      ;;
    *)
      mission_cli+=("$1")
      shift
      ;;
  esac
done

for arg in "${mission_cli[@]}"; do
  case "$arg" in
    --waypoint|--path-file|--waypoints-xy-alt|--waypoint=*|--path-file=*|--waypoints-xy-alt=*)
      cli_sets_waypoints=true
      ;;
  esac
done

if [[ "$cli_sets_waypoints" == false && -n "${WAYPOINTS_XY_ALT_M:-}" ]]; then
  args+=(--waypoints-xy-alt="$WAYPOINTS_XY_ALT_M")
fi
if [[ -n "${WAYPOINT_SPEED_MPS:-${DRONE_FORWARD_SPEED_MPS:-}}" ]]; then
  args+=(--speed-mps "${WAYPOINT_SPEED_MPS:-${DRONE_FORWARD_SPEED_MPS:-}}")
fi
if [[ -n "${WAYPOINT_ACCEPTANCE_RADIUS_M:-}" ]]; then
  args+=(--acceptance-radius-m "$WAYPOINT_ACCEPTANCE_RADIUS_M")
fi
if [[ -n "${WAYPOINT_TIMEOUT_S:-${DRONE_APPROACH_TIMEOUT_S:-}}" ]]; then
  args+=(--waypoint-timeout-s "${WAYPOINT_TIMEOUT_S:-${DRONE_APPROACH_TIMEOUT_S:-}}")
fi
if [[ -n "${WAYPOINT_TAKEOFF_TIMEOUT_S:-${DRONE_TAKEOFF_TIMEOUT_S:-}}" ]]; then
  args+=(--takeoff-timeout-s "${WAYPOINT_TAKEOFF_TIMEOUT_S:-${DRONE_TAKEOFF_TIMEOUT_S:-}}")
fi
if [[ -n "${WAYPOINT_HOLD_S:-${DRONE_FIRST_HOVER_S:-}}" ]]; then
  args+=(--hold-s "${WAYPOINT_HOLD_S:-${DRONE_FIRST_HOVER_S:-}}")
fi
if [[ -n "${WAYPOINT_FINAL_HOVER_S:-${DRONE_FINAL_HOVER_S:-}}" ]]; then
  args+=(--final-hover-s "${WAYPOINT_FINAL_HOVER_S:-${DRONE_FINAL_HOVER_S:-}}")
fi
if [[ -n "${WAYPOINT_LAND_TIMEOUT_S:-${DRONE_LAND_TIMEOUT_S:-}}" ]]; then
  args+=(--land-timeout-s "${WAYPOINT_LAND_TIMEOUT_S:-${DRONE_LAND_TIMEOUT_S:-}}")
fi
if [[ -n "${WAYPOINT_SETPOINT_HZ:-${DRONE_SETPOINT_HZ:-}}" ]]; then
  args+=(--setpoint-hz "${WAYPOINT_SETPOINT_HZ:-${DRONE_SETPOINT_HZ:-}}")
fi
case "${WAYPOINT_LAND_AFTER:-true}" in
  true|TRUE|1|yes|YES) args+=(--land-after) ;;
  false|FALSE|0|no|NO) args+=(--no-land) ;;
esac

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT_DIR/install/setup.bash"
fi
set -u

printf '%s\n' "Simulation config source: $SIM_CONFIG"

record_pid=""
cleanup_recording() {
  if [[ -n "$record_pid" ]] && kill -0 "$record_pid" >/dev/null 2>&1; then
    kill -TERM "-$record_pid" >/dev/null 2>&1 || kill "$record_pid" >/dev/null 2>&1 || true
    wait "$record_pid" >/dev/null 2>&1 || true
  fi
}
trap cleanup_recording EXIT INT TERM

case "$record_lite" in
  true|TRUE|1|yes|YES)
    if [[ "$check_connection" == false ]]; then
      if [[ -z "$record_dir" ]]; then
        record_dir="$ROOT_DIR/bags/waypoint_$(date +%Y%m%d_%H%M%S)_lite"
      fi
      mkdir -p "$record_dir"
      setsid "$OPERATOR_RECORDER_PYTHON" -m stereo_ground_sim.operator_recorder \
        --mode lite \
        --output-dir "$record_dir" &
      record_pid="$!"
      printf '%s\n' "Recording waypoint lite session to: $record_dir"
      sleep 1
    fi
    ;;
esac

python3 "$ROOT_DIR/scripts/px4_waypoint_mission.py" "${args[@]}" "${mission_cli[@]}"
