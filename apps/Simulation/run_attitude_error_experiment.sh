#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INTEGRATION_ROOT="$(cd "$ROOT_DIR/../.." && pwd)"
cd "$ROOT_DIR"

SHOW_HELP=false
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  SHOW_HELP=true
fi

SIM_CONFIG="${SIM_CONFIG:-$ROOT_DIR/config/simulation.env}"
SIM_EXPLICIT_NAMES=(
  CAMERA_BASELINE_M CAMERA_HEIGHT_M CAMERA_HORIZONTAL_FOV_RAD
  CAMERA_IMAGE_WIDTH_PX CAMERA_IMAGE_HEIGHT_PX CAMERA_UPDATE_RATE_HZ
  CAMERA_CLIP_NEAR_M CAMERA_CLIP_FAR_M
  YOLO_MODEL_PATH YOLO_CLASS_NAME YOLO_CONFIDENCE YOLO_DEVICE YOLO_TARGET_HZ
  YOLO_IMGSZ YOLO_PYTHON
  DRONE_SPAWN_X_M DRONE_SPAWN_Y_M DRONE_SPAWN_Z_M DRONE_SPAWN_YAW_DEG
  DRONE_TAKEOFF_ALTITUDE_M DRONE_FORWARD_SPEED_MPS DRONE_TARGET_DISTANCE_M
  DRONE_FIRST_HOVER_S DRONE_FINAL_HOVER_S DRONE_LAND_DURATION_S
  DRONE_LAND_TIMEOUT_S
  ATTITUDE_EXPERIMENT_ANGLE_STEP_DEG ATTITUDE_EXPERIMENT_ANGLE_MIN_DEG
  ATTITUDE_EXPERIMENT_ANGLE_MAX_DEG ATTITUDE_EXPERIMENT_AXIS_RANGES
  ATTITUDE_EXPERIMENT_AXES ATTITUDE_EXPERIMENT_INCLUDE_NEGATIVE
  ATTITUDE_EXPERIMENT_QUICK ATTITUDE_OUTPUT_DIR
)
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/load_sim_config.sh"
load_sim_config_preserving_env "${SIM_EXPLICIT_NAMES[@]}"

resolve_model_spec() {
  local candidate
  for candidate in "$@"; do
    [[ -n "$candidate" ]] || continue
    if [[ -f "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
    if [[ "$candidate" != */* ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  return 1
}

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

if ! YOLO_MODEL_PATH="$(resolve_model_spec "${YOLO_MODEL_PATH:-yolov8n.pt}" yolov8n.pt)"; then
  YOLO_MODEL_PATH=""
fi
YOLO_BOOTSTRAP_PREFERRED_PYTHON="${YOLO_PYTHON:-python3}"
if [[ -n "${YOLO_PYTHON:-}" && "${YOLO_PYTHON}" == */* ]]; then
  YOLO_PYTHON="$(resolve_executable "${YOLO_PYTHON}" || true)"
fi
if [[ "$SHOW_HELP" != true && -n "$YOLO_MODEL_PATH" ]]; then
  if YOLO_PYTHON="$(bash "$ROOT_DIR/scripts/ensure_yolo_runtime.sh" \
    --venv-dir "$ROOT_DIR/.venv" \
    --preferred-python "$YOLO_BOOTSTRAP_PREFERRED_PYTHON"
  )"; then
    :
  else
    YOLO_PYTHON="$(resolve_executable "${YOLO_PYTHON:-}" python3 || true)"
  fi
elif ! YOLO_PYTHON="$(resolve_executable "${YOLO_PYTHON:-}" python3)"; then
  YOLO_PYTHON=""
fi

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros2-attitude-experiment-log}"
export GZ_CONFIG_HOME="${GZ_CONFIG_HOME:-/tmp/gz-attitude-experiment-config}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/matplotlib-attitude-error}"
export GZ_SIM_RESOURCE_PATH="$ROOT_DIR/install/stereo_ground_sim/share/stereo_ground_sim/models:${GZ_SIM_RESOURCE_PATH:-}"

if [[ "${1:-}" == "--gazebo-legacy" ]]; then
  shift
fi

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
  exec python3 "$ROOT_DIR/scripts/attitude_error_experiment.py" "$@"
fi

experiment_args=()

append_if_unset() {
  local option="$1"
  local value="$2"
  shift 2
  local existing
  for existing in "$@"; do
    [[ "$existing" == "$option" ]] && return 0
  done
  [[ -n "$value" ]] || return 0
  experiment_args+=("$option" "$value")
}

append_flag_if_true() {
  local option="$1"
  local value="$2"
  shift 2
  local existing
  for existing in "$@"; do
    [[ "$existing" == "$option" ]] && return 0
  done
  case "$value" in
    true|TRUE|1|yes|YES|on|ON) 
      experiment_args+=("$option")
      ;;
  esac
}

append_if_unset --angle-step-deg "${ATTITUDE_EXPERIMENT_ANGLE_STEP_DEG:-}" "$@"
append_if_unset --angle-min-deg "${ATTITUDE_EXPERIMENT_ANGLE_MIN_DEG:-}" "$@"
append_if_unset --angle-max-deg "${ATTITUDE_EXPERIMENT_ANGLE_MAX_DEG:-}" "$@"
if [[ -n "${ATTITUDE_EXPERIMENT_AXIS_RANGES:-}" ]]; then
  axis_range_already_set=false
  for existing in "$@"; do
    case "$existing" in
      --axis-range|--axis-range=*) axis_range_already_set=true ;;
    esac
  done
  if [[ "$axis_range_already_set" == false ]]; then
    # shellcheck disable=SC2206
    configured_axis_ranges=(${ATTITUDE_EXPERIMENT_AXIS_RANGES})
    for configured_axis_range in "${configured_axis_ranges[@]}"; do
      experiment_args+=(--axis-range "$configured_axis_range")
    done
  fi
fi
if [[ -n "${ATTITUDE_EXPERIMENT_AXES:-}" ]]; then
  axes_already_set=false
  for existing in "$@"; do
    [[ "$existing" == "--axes" ]] && axes_already_set=true
  done
  if [[ "$axes_already_set" == false ]]; then
    experiment_args+=(--axes)
    # shellcheck disable=SC2206
    configured_axes=(${ATTITUDE_EXPERIMENT_AXES})
    experiment_args+=("${configured_axes[@]}")
  fi
fi
append_flag_if_true --include-negative "${ATTITUDE_EXPERIMENT_INCLUDE_NEGATIVE:-false}" "$@"
append_flag_if_true --quick "${ATTITUDE_EXPERIMENT_QUICK:-false}" "$@"
append_if_unset --baseline-m "${CAMERA_BASELINE_M:-}" "$@"
append_if_unset --camera-height-m "${CAMERA_HEIGHT_M:-}" "$@"
append_if_unset --horizontal-fov-rad "${CAMERA_HORIZONTAL_FOV_RAD:-}" "$@"
append_if_unset --image-width-px "${CAMERA_IMAGE_WIDTH_PX:-}" "$@"
append_if_unset --image-height-px "${CAMERA_IMAGE_HEIGHT_PX:-}" "$@"
append_if_unset --camera-update-rate-hz "${CAMERA_UPDATE_RATE_HZ:-}" "$@"
append_if_unset --camera-clip-near-m "${CAMERA_CLIP_NEAR_M:-}" "$@"
append_if_unset --camera-clip-far-m "${CAMERA_CLIP_FAR_M:-}" "$@"
append_if_unset --approach-start-distance-m "${DRONE_SPAWN_Y_M:-}" "$@"
append_if_unset --approach-end-distance-m "${DRONE_TARGET_DISTANCE_M:-}" "$@"
append_if_unset --takeoff-altitude-m "${DRONE_TAKEOFF_ALTITUDE_M:-}" "$@"
append_if_unset --target-spawn-x-m "${DRONE_SPAWN_X_M:-}" "$@"
append_if_unset --target-spawn-y-m "${DRONE_SPAWN_Y_M:-}" "$@"
append_if_unset --target-spawn-z-m "${DRONE_SPAWN_Z_M:-}" "$@"
append_if_unset --target-spawn-yaw-deg "${DRONE_SPAWN_YAW_DEG:-}" "$@"
if [[ -n "${DRONE_SPAWN_Y_M:-}" && -n "${DRONE_TARGET_DISTANCE_M:-}" && -n "${DRONE_FORWARD_SPEED_MPS:-}" ]]; then
  computed_approach_duration_s="$(
    awk -v start="$DRONE_SPAWN_Y_M" -v end="$DRONE_TARGET_DISTANCE_M" -v speed="$DRONE_FORWARD_SPEED_MPS" \
      'BEGIN { if (speed > 0) printf "%.6f", (start > end ? start - end : end - start) / speed }'
  )"
  append_if_unset --approach-duration-s "$computed_approach_duration_s" "$@"
fi
append_if_unset --first-hover-duration-s "${DRONE_FIRST_HOVER_S:-}" "$@"
append_if_unset --final-hover-duration-s "${DRONE_FINAL_HOVER_S:-}" "$@"
append_if_unset --land-duration-s "${DRONE_LAND_DURATION_S:-}" "$@"
append_if_unset --yolo-model "${YOLO_MODEL_PATH:-}" "$@"
append_if_unset --yolo-class "${YOLO_CLASS_NAME:-}" "$@"
append_if_unset --yolo-confidence "${YOLO_CONFIDENCE:-}" "$@"
append_if_unset --yolo-device "${YOLO_DEVICE:-}" "$@"
append_if_unset --yolo-target-hz "${YOLO_TARGET_HZ:-}" "$@"
append_if_unset --yolo-imgsz "${YOLO_IMGSZ:-}" "$@"
append_if_unset --yolo-python "${YOLO_PYTHON:-}" "$@"
append_if_unset --output-dir "${ATTITUDE_OUTPUT_DIR:-}" "$@"

colcon build \
  --base-paths "$ROOT_DIR/stereo_ground_sim" "$INTEGRATION_ROOT/common/ros/simulation_interfaces" \
  --packages-select simulation_interfaces stereo_ground_sim

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT_DIR/install/setup.bash"
set -u

export PYTHONPATH="$INTEGRATION_ROOT/common/python:$ROOT_DIR/stereo_ground_sim${PYTHONPATH:+:$PYTHONPATH}"

exec python3 "$ROOT_DIR/scripts/attitude_error_experiment.py" "${experiment_args[@]}" "$@"
