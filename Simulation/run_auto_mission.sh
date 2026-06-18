#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

DRONE_MISSION_CONFIG="${DRONE_MISSION_CONFIG:-$ROOT_DIR/config/drone_mission.env}"
SIM_SCENARIO_CONFIG="${SIM_SCENARIO_CONFIG:-}"

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

if [[ -f "$DRONE_MISSION_CONFIG" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "$DRONE_MISSION_CONFIG"
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

if [[ -n "$SIM_SCENARIO_CONFIG" ]]; then
  if [[ "$SIM_SCENARIO_CONFIG" != /* && -f "$ROOT_DIR/config/scenarios/$SIM_SCENARIO_CONFIG" ]]; then
    SIM_SCENARIO_CONFIG="$ROOT_DIR/config/scenarios/$SIM_SCENARIO_CONFIG"
  fi
  if [[ ! -f "$SIM_SCENARIO_CONFIG" ]]; then
    echo "error: SIM_SCENARIO_CONFIG not found: $SIM_SCENARIO_CONFIG" >&2
    exit 2
  fi
  set -a
  # shellcheck disable=SC1090
  source "$SIM_SCENARIO_CONFIG"
  set +a
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
fi

usage() {
  cat <<'USAGE'
Usage:
  ./run_auto_mission.sh [options]

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
  --connect STRING       pymavlink connection string. Default: udpin:0.0.0.0:14540
  --check-connection     Check PX4 connection without moving the drone.
  --help, -h             Show this help.

Examples:
  ./run_operator_view.sh --record-lite
  ./run_auto_mission.sh
  ./run_auto_mission.sh --quick
  ./run_auto_mission.sh --target-distance-m 200 --altitude-m 10 --speed-mps 15
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
    --target-distance-m|--altitude-m|--speed-mps|--approach-timeout-s)
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

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT_DIR/install/setup.bash"
fi
set -u

exec python3 "$ROOT_DIR/scripts/px4_auto_mission.py" "${mission_args[@]}"
