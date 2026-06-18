#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

DEFAULT_BBOX_AFFINE_PROFILE="$ROOT_DIR/config/bbox_adjustments/reference_bbox_center_affine.csv"
BBOX_AFFINE_PROFILE_PATH="${BBOX_AFFINE_PROFILE_PATH:-$DEFAULT_BBOX_AFFINE_PROFILE}"
SCENARIO_CONFIG="${SIM_SCENARIO_CONFIG:-}"
RUN_AUTO_MISSION=false
OPERATOR_STARTUP_WAIT_S="${OPERATOR_STARTUP_WAIT_S:-20}"
operator_args=()

usage() {
  cat <<'USAGE'
Usage:
  ./run_bbox_affine_adjustment_sim.sh [options] [-- run_operator_view args...]

Starts the simulation with a 2D bbox-center affine correction CSV.

Default profile:
  Simulation/config/bbox_adjustments/reference_bbox_center_affine.csv

Coordinate contract:
  - input correction fields are left/right image bbox center u/v pixels
  - the CSV is a 2D pixel affine correction for bbox centers
  - corrected bbox centers are passed to the existing simulation triangulation path
  - downstream position topics keep the current StereoDistance simulation contract

Options:
  --scenario FILE_OR_NAME   Set SIM_SCENARIO_CONFIG. A bare name is resolved
                            under Simulation/config/scenarios/.
  --small-drone-close       Alias for --scenario small_drone_close_2m_4x.env
  --small-drone-mid         Alias for --scenario small_drone_mid_5m_8x.env
  --profile FILE            Override the bbox affine CSV path.
  --auto-mission            Start operator view in the background, wait, then
                            run ./run_auto_mission.sh in the same scenario.
  --wait-s SECONDS          Wait before auto mission. Default: 20 or
                            OPERATOR_STARTUP_WAIT_S.
  --help, -h                Show this help.

Examples:
  ./run_bbox_affine_adjustment_sim.sh

  ./run_bbox_affine_adjustment_sim.sh --small-drone-close

  ./run_bbox_affine_adjustment_sim.sh --small-drone-mid --auto-mission

  ./run_bbox_affine_adjustment_sim.sh --small-drone-close -- --record-lite bags/bbox_affine_close_lite
USAGE
}

resolve_scenario_config() {
  local value="$1"
  if [[ -z "$value" ]]; then
    printf '%s' ""
    return 0
  fi
  if [[ "$value" != /* && -f "$ROOT_DIR/config/scenarios/$value" ]]; then
    printf '%s' "$ROOT_DIR/config/scenarios/$value"
    return 0
  fi
  if [[ "$value" != /* && -f "$ROOT_DIR/$value" ]]; then
    printf '%s' "$ROOT_DIR/$value"
    return 0
  fi
  printf '%s' "$value"
}

while (($#)); do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --scenario)
      SCENARIO_CONFIG="${2:?missing value for --scenario}"
      shift 2
      ;;
    --small-drone-close)
      SCENARIO_CONFIG="small_drone_close_2m_4x.env"
      shift
      ;;
    --small-drone-mid)
      SCENARIO_CONFIG="small_drone_mid_5m_8x.env"
      shift
      ;;
    --profile)
      BBOX_AFFINE_PROFILE_PATH="${2:?missing value for --profile}"
      shift 2
      ;;
    --auto-mission)
      RUN_AUTO_MISSION=true
      shift
      ;;
    --wait-s)
      OPERATOR_STARTUP_WAIT_S="${2:?missing value for --wait-s}"
      shift 2
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

if [[ "$BBOX_AFFINE_PROFILE_PATH" != /* ]]; then
  BBOX_AFFINE_PROFILE_PATH="$ROOT_DIR/$BBOX_AFFINE_PROFILE_PATH"
fi
if [[ ! -f "$BBOX_AFFINE_PROFILE_PATH" ]]; then
  echo "error: bbox affine CSV not found: $BBOX_AFFINE_PROFILE_PATH" >&2
  exit 2
fi

SCENARIO_CONFIG="$(resolve_scenario_config "$SCENARIO_CONFIG")"
if [[ -n "$SCENARIO_CONFIG" && ! -f "$SCENARIO_CONFIG" ]]; then
  echo "error: scenario config not found: $SCENARIO_CONFIG" >&2
  exit 2
fi

export SIM_SCENARIO_CONFIG="$SCENARIO_CONFIG"

echo "bbox affine profile: $BBOX_AFFINE_PROFILE_PATH"
if [[ -n "$SIM_SCENARIO_CONFIG" ]]; then
  echo "SIM_SCENARIO_CONFIG: $SIM_SCENARIO_CONFIG"
else
  echo "SIM_SCENARIO_CONFIG: <default>"
fi

if [[ "$RUN_AUTO_MISSION" != true ]]; then
  exec "$ROOT_DIR/run_operator_view.sh" \
    --bbox-adjustment "$BBOX_AFFINE_PROFILE_PATH" \
    "${operator_args[@]}"
fi

"$ROOT_DIR/run_operator_view.sh" \
  --bbox-adjustment "$BBOX_AFFINE_PROFILE_PATH" \
  "${operator_args[@]}" &
operator_pid=$!

cleanup() {
  if kill -0 "$operator_pid" 2>/dev/null; then
    kill "$operator_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Waiting ${OPERATOR_STARTUP_WAIT_S}s before auto mission..."
sleep "$OPERATOR_STARTUP_WAIT_S"
"$ROOT_DIR/run_auto_mission.sh"
wait "$operator_pid"
