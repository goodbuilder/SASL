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

SIM_EXPLICIT_NAMES=(
  DRONE_TAKEOFF_ALTITUDE_M DRONE_FORWARD_SPEED_MPS DRONE_MOVE_DISTANCE_M
  DRONE_TARGET_DISTANCE_M DRONE_APPROACH_TIMEOUT_S DRONE_TAKEOFF_TIMEOUT_S
  DRONE_FIRST_HOVER_S DRONE_FINAL_HOVER_S DRONE_LAND_TIMEOUT_S
  DRONE_SETPOINT_HZ
  WAYPOINTS_XY_ALT_M WAYPOINT_SPEED_MPS WAYPOINT_ACCEPTANCE_RADIUS_M
  WAYPOINT_TIMEOUT_S WAYPOINT_HOLD_S WAYPOINT_RAMP_S WAYPOINT_VERTICAL_RAMP_S
  WAYPOINT_FINAL_HOVER_S WAYPOINT_LAND_TIMEOUT_S WAYPOINT_SETPOINT_HZ
  WAYPOINT_LAND_AFTER WAYPOINT_STOP_AFTER_HOVER WAYPOINT_RECORD_FULL WAYPOINT_RECORD_DIR
  WAYPOINT_MPC_Z_VEL_MAX_UP_MPS WAYPOINT_MPC_Z_VEL_MAX_DN_MPS
  WAYPOINT_MPC_TILTMAX_AIR_DEG
  WAYPOINT_MC_ROLLRATE_P WAYPOINT_MC_PITCHRATE_P
  WAYPOINT_MC_ROLL_P WAYPOINT_MC_PITCH_P WAYPOINT_MPC_XY_VEL_P_ACC
  WAYPOINT_MPC_ACC_HOR_MAX
)
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/load_sim_config.sh"
load_sim_config_preserving_env "${SIM_EXPLICIT_NAMES[@]}"

usage() {
  cat <<'USAGE'
Usage:
  ./run_px4_waypoint_mission.sh [options]

Sends a PX4 offboard waypoint mission to an already running PX4 SITL.
Start Gazebo/rqt/PX4 first with ./run_operator_view.sh.

Defaults are read from config/simulation.env.
Command-line options override the config file for one run.

Waypoint format:
  north,east,altitude_agl_m[,hold_s]
  hold_s overrides --hold-s for that one waypoint (e.g. to hover only at
  some waypoints in a dense path); omit it to use --hold-s.

Options:
  --waypoint X,Y,ALT     Add one waypoint. Repeatable.
  --path-file FILE       Read waypoints from CSV/text file.
  --speed-mps N          Horizontal speed limit.
  --mpc-z-vel-max-up-mps N
                         Override PX4 MPC_Z_VEL_MAX_UP (climb limit) before the mission.
  --mpc-z-vel-max-dn-mps N
                         Override PX4 MPC_Z_VEL_MAX_DN (descent limit) before the mission.
  --mpc-tiltmax-air-deg N
                         Override PX4 MPC_TILTMAX_AIR (max roll/pitch lean angle) before the mission.
  --mc-rollrate-p N      Override PX4 MC_ROLLRATE_P (roll rate loop gain) before the mission.
  --mc-pitchrate-p N     Override PX4 MC_PITCHRATE_P (pitch rate loop gain) before the mission.
  --mc-roll-p N          Override PX4 MC_ROLL_P (roll attitude loop gain) before the mission.
  --mc-pitch-p N         Override PX4 MC_PITCH_P (pitch attitude loop gain) before the mission.
  --mpc-xy-vel-p-acc N   Override PX4 MPC_XY_VEL_P_ACC (horizontal velocity loop gain) before the mission.
  --mpc-acc-hor-max N    Override PX4 MPC_ACC_HOR_MAX (horizontal acceleration cap) before the mission.
  --acceptance-radius-m N
                         Waypoint reached radius.
  --waypoint-timeout-s N Timeout per waypoint.
  --hold-s N             Hover time at each waypoint. 0 disables the hover.
  --ramp-s N              Time to accelerate/decelerate horizontal speed at the start/end of a leg. 0 disables ramping.
  --vertical-ramp-s N     Time to accelerate/decelerate vertical speed at the start/end of a leg. 0 disables ramping.
  --final-hover-s N      Hover time before optional landing. 0 disables the hover.
  --land-timeout-s N     Landing timeout.
  --setpoint-hz N        PX4 offboard setpoint rate.
  --land-after           Land after the final waypoint. Default from config.
  --no-land              Do not land after the final waypoint.
  --stop-after-hover     With --no-land: exit once --final-hover-s elapses instead of
                         hovering indefinitely until interrupted (Ctrl+C).
  --no-stop-after-hover  With --no-land: hover indefinitely until interrupted. Default.
  --record-dir DIR       Save full (CSV + overlay MP4) telemetry to DIR. Default: Recordings/waypoint_TIMESTAMP_full.
  --no-record            Disable automatic full telemetry recording.
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
record_full="${WAYPOINT_RECORD_FULL:-true}"
record_dir="${WAYPOINT_RECORD_DIR:-}"

while (($#)); do
  case "$1" in
    --record-dir)
      record_full=true
      record_dir="${2:?missing value for --record-dir}"
      shift 2
      ;;
    --no-record)
      record_full=false
      shift
      ;;
    --waypoint|--path-file|--waypoints-xy-alt|--speed-mps|--mpc-z-vel-max-up-mps|--mpc-z-vel-max-dn-mps|--mpc-tiltmax-air-deg|--mc-rollrate-p|--mc-pitchrate-p|--mc-roll-p|--mc-pitch-p|--mpc-xy-vel-p-acc|--mpc-acc-hor-max|--acceptance-radius-m|--waypoint-timeout-s|--hold-s|--ramp-s|--vertical-ramp-s|--final-hover-s|--land-timeout-s|--setpoint-hz|--connect)
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
if [[ -n "${WAYPOINT_MPC_Z_VEL_MAX_UP_MPS:-}" ]]; then
  args+=(--mpc-z-vel-max-up-mps "$WAYPOINT_MPC_Z_VEL_MAX_UP_MPS")
fi
if [[ -n "${WAYPOINT_MPC_Z_VEL_MAX_DN_MPS:-}" ]]; then
  args+=(--mpc-z-vel-max-dn-mps "$WAYPOINT_MPC_Z_VEL_MAX_DN_MPS")
fi
if [[ -n "${WAYPOINT_MPC_TILTMAX_AIR_DEG:-}" ]]; then
  args+=(--mpc-tiltmax-air-deg "$WAYPOINT_MPC_TILTMAX_AIR_DEG")
fi
if [[ -n "${WAYPOINT_MC_ROLLRATE_P:-}" ]]; then
  args+=(--mc-rollrate-p "$WAYPOINT_MC_ROLLRATE_P")
fi
if [[ -n "${WAYPOINT_MC_PITCHRATE_P:-}" ]]; then
  args+=(--mc-pitchrate-p "$WAYPOINT_MC_PITCHRATE_P")
fi
if [[ -n "${WAYPOINT_MC_ROLL_P:-}" ]]; then
  args+=(--mc-roll-p "$WAYPOINT_MC_ROLL_P")
fi
if [[ -n "${WAYPOINT_MC_PITCH_P:-}" ]]; then
  args+=(--mc-pitch-p "$WAYPOINT_MC_PITCH_P")
fi
if [[ -n "${WAYPOINT_MPC_XY_VEL_P_ACC:-}" ]]; then
  args+=(--mpc-xy-vel-p-acc "$WAYPOINT_MPC_XY_VEL_P_ACC")
fi
if [[ -n "${WAYPOINT_MPC_ACC_HOR_MAX:-}" ]]; then
  args+=(--mpc-acc-hor-max "$WAYPOINT_MPC_ACC_HOR_MAX")
fi
if [[ -n "${WAYPOINT_ACCEPTANCE_RADIUS_M:-}" ]]; then
  args+=(--acceptance-radius-m "$WAYPOINT_ACCEPTANCE_RADIUS_M")
fi
if [[ -n "${WAYPOINT_TIMEOUT_S:-${DRONE_APPROACH_TIMEOUT_S:-}}" ]]; then
  args+=(--waypoint-timeout-s "${WAYPOINT_TIMEOUT_S:-${DRONE_APPROACH_TIMEOUT_S:-}}")
fi
if [[ -n "${WAYPOINT_HOLD_S:-${DRONE_FIRST_HOVER_S:-}}" ]]; then
  args+=(--hold-s "${WAYPOINT_HOLD_S:-${DRONE_FIRST_HOVER_S:-}}")
fi
if [[ -n "${WAYPOINT_RAMP_S:-}" ]]; then
  args+=(--ramp-s "$WAYPOINT_RAMP_S")
fi
if [[ -n "${WAYPOINT_VERTICAL_RAMP_S:-}" ]]; then
  args+=(--vertical-ramp-s "$WAYPOINT_VERTICAL_RAMP_S")
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
case "${WAYPOINT_STOP_AFTER_HOVER:-false}" in
  true|TRUE|1|yes|YES) args+=(--stop-after-hover) ;;
  false|FALSE|0|no|NO) args+=(--no-stop-after-hover) ;;
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

case "$record_full" in
  true|TRUE|1|yes|YES)
    if [[ "$check_connection" == false ]]; then
      if [[ -z "$record_dir" ]]; then
        record_dir="$ROOT_DIR/Recordings/waypoint_$(date +%Y%m%d_%H%M%S)_full"
      fi
      mkdir -p "$record_dir"
      setsid "$OPERATOR_RECORDER_PYTHON" -m stereo_ground_sim.operator_recorder \
        --mode full \
        --output-dir "$record_dir" &
      record_pid="$!"
      printf '%s\n' "Recording waypoint full session to: $record_dir"
      sleep 1
    fi
    ;;
esac

python3 "$ROOT_DIR/scripts/px4_waypoint_mission.py" "${args[@]}" "${mission_cli[@]}"
