#!/usr/bin/env bash
set -euo pipefail

CALLER_CWD="$(pwd)"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

SHOW_HELP=false
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  SHOW_HELP=true
fi

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros2-launch-log}"
export GZ_CONFIG_HOME="${GZ_CONFIG_HOME:-/tmp/gz-config}"
PX4_CONSOLE_LOG="${PX4_CONSOLE_LOG:-/dev/null}"
export GZ_SIM_RESOURCE_PATH="$ROOT_DIR/install/stereo_ground_sim/share/stereo_ground_sim/models:${GZ_SIM_RESOURCE_PATH:-}"
RQT_PYTHONPATH="$ROOT_DIR/python_patches${PYTHONPATH:+:$PYTHONPATH}"
DEFAULT_OVERLAY_HZ="${DEFAULT_OVERLAY_HZ:-20.0}"
DEFAULT_PLOT_HZ="${DEFAULT_PLOT_HZ:-30.0}"
DEFAULT_ACTUAL_DISTANCE_HZ="${DEFAULT_ACTUAL_DISTANCE_HZ:-15.0}"
DEFAULT_PLOT_WINDOW_S="${DEFAULT_PLOT_WINDOW_S:-30.0}"
PX4_DIR="${PX4_DIR:-}"
PX4_GZ_MODEL_NAME="${PX4_GZ_MODEL_NAME:-x500_15m}"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-stereo_ground_drone}"
GAZEBO_MAJOR_VERSION="${GAZEBO_MAJOR_VERSION:-8}"
CAMERA_MODEL_CONFIG="${CAMERA_MODEL_CONFIG:-$ROOT_DIR/config/camera_model.env}"
YOLO_CONFIG="${YOLO_CONFIG:-$ROOT_DIR/config/yolo.env}"
ATTITUDE_CONFIG="${ATTITUDE_CONFIG:-$ROOT_DIR/config/camera_attitude.env}"
DRONE_MISSION_CONFIG="${DRONE_MISSION_CONFIG:-$ROOT_DIR/config/drone_mission.env}"
SIM_SCENARIO_CONFIG="${SIM_SCENARIO_CONFIG:-}"

YOLO_MODEL_PATH_FROM_ENV="${YOLO_MODEL_PATH+x}"; YOLO_MODEL_PATH_ENV_VALUE="${YOLO_MODEL_PATH:-}"
YOLO_CLASS_NAME_FROM_ENV="${YOLO_CLASS_NAME+x}"; YOLO_CLASS_NAME_ENV_VALUE="${YOLO_CLASS_NAME:-}"
YOLO_CONFIDENCE_FROM_ENV="${YOLO_CONFIDENCE+x}"; YOLO_CONFIDENCE_ENV_VALUE="${YOLO_CONFIDENCE:-}"
YOLO_DEVICE_FROM_ENV="${YOLO_DEVICE+x}"; YOLO_DEVICE_ENV_VALUE="${YOLO_DEVICE:-}"
YOLO_TARGET_HZ_FROM_ENV="${YOLO_TARGET_HZ+x}"; YOLO_TARGET_HZ_ENV_VALUE="${YOLO_TARGET_HZ:-}"
YOLO_IMGSZ_FROM_ENV="${YOLO_IMGSZ+x}"; YOLO_IMGSZ_ENV_VALUE="${YOLO_IMGSZ:-}"
YOLO_PYTHON_FROM_ENV="${YOLO_PYTHON+x}"; YOLO_PYTHON_ENV_VALUE="${YOLO_PYTHON:-}"
YOLO_PYTHONPATH_FROM_ENV="${YOLO_PYTHONPATH+x}"; YOLO_PYTHONPATH_ENV_VALUE="${YOLO_PYTHONPATH:-}"

CAMERA_BASELINE_M_FROM_ENV="${CAMERA_BASELINE_M+x}"; CAMERA_BASELINE_M_ENV_VALUE="${CAMERA_BASELINE_M:-}"
CAMERA_HEIGHT_M_FROM_ENV="${CAMERA_HEIGHT_M+x}"; CAMERA_HEIGHT_M_ENV_VALUE="${CAMERA_HEIGHT_M:-}"
CAMERA_HORIZONTAL_FOV_RAD_FROM_ENV="${CAMERA_HORIZONTAL_FOV_RAD+x}"; CAMERA_HORIZONTAL_FOV_RAD_ENV_VALUE="${CAMERA_HORIZONTAL_FOV_RAD:-}"
CAMERA_IMAGE_WIDTH_PX_FROM_ENV="${CAMERA_IMAGE_WIDTH_PX+x}"; CAMERA_IMAGE_WIDTH_PX_ENV_VALUE="${CAMERA_IMAGE_WIDTH_PX:-}"
CAMERA_IMAGE_HEIGHT_PX_FROM_ENV="${CAMERA_IMAGE_HEIGHT_PX+x}"; CAMERA_IMAGE_HEIGHT_PX_ENV_VALUE="${CAMERA_IMAGE_HEIGHT_PX:-}"
CAMERA_UPDATE_RATE_HZ_FROM_ENV="${CAMERA_UPDATE_RATE_HZ+x}"; CAMERA_UPDATE_RATE_HZ_ENV_VALUE="${CAMERA_UPDATE_RATE_HZ:-}"
CAMERA_CLIP_NEAR_M_FROM_ENV="${CAMERA_CLIP_NEAR_M+x}"; CAMERA_CLIP_NEAR_M_ENV_VALUE="${CAMERA_CLIP_NEAR_M:-}"
CAMERA_CLIP_FAR_M_FROM_ENV="${CAMERA_CLIP_FAR_M+x}"; CAMERA_CLIP_FAR_M_ENV_VALUE="${CAMERA_CLIP_FAR_M:-}"

DRONE_SPAWN_X_M_FROM_ENV="${DRONE_SPAWN_X_M+x}"; DRONE_SPAWN_X_M_ENV_VALUE="${DRONE_SPAWN_X_M:-}"
DRONE_SPAWN_Y_M_FROM_ENV="${DRONE_SPAWN_Y_M+x}"; DRONE_SPAWN_Y_M_ENV_VALUE="${DRONE_SPAWN_Y_M:-}"
DRONE_SPAWN_Z_M_FROM_ENV="${DRONE_SPAWN_Z_M+x}"; DRONE_SPAWN_Z_M_ENV_VALUE="${DRONE_SPAWN_Z_M:-}"
DRONE_ORIGIN_Z_OFFSET_M_FROM_ENV="${DRONE_ORIGIN_Z_OFFSET_M+x}"; DRONE_ORIGIN_Z_OFFSET_M_ENV_VALUE="${DRONE_ORIGIN_Z_OFFSET_M:-}"
DRONE_SPAWN_YAW_DEG_FROM_ENV="${DRONE_SPAWN_YAW_DEG+x}"; DRONE_SPAWN_YAW_DEG_ENV_VALUE="${DRONE_SPAWN_YAW_DEG:-}"
DRONE_TAKEOFF_ALTITUDE_M_FROM_ENV="${DRONE_TAKEOFF_ALTITUDE_M+x}"; DRONE_TAKEOFF_ALTITUDE_M_ENV_VALUE="${DRONE_TAKEOFF_ALTITUDE_M:-}"
DRONE_FORWARD_SPEED_MPS_FROM_ENV="${DRONE_FORWARD_SPEED_MPS+x}"; DRONE_FORWARD_SPEED_MPS_ENV_VALUE="${DRONE_FORWARD_SPEED_MPS:-}"
DRONE_TARGET_DISTANCE_M_FROM_ENV="${DRONE_TARGET_DISTANCE_M+x}"; DRONE_TARGET_DISTANCE_M_ENV_VALUE="${DRONE_TARGET_DISTANCE_M:-}"
DRONE_FIRST_HOVER_S_FROM_ENV="${DRONE_FIRST_HOVER_S+x}"; DRONE_FIRST_HOVER_S_ENV_VALUE="${DRONE_FIRST_HOVER_S:-}"
DRONE_FINAL_HOVER_S_FROM_ENV="${DRONE_FINAL_HOVER_S+x}"; DRONE_FINAL_HOVER_S_ENV_VALUE="${DRONE_FINAL_HOVER_S:-}"
DRONE_LAND_DURATION_S_FROM_ENV="${DRONE_LAND_DURATION_S+x}"; DRONE_LAND_DURATION_S_ENV_VALUE="${DRONE_LAND_DURATION_S:-}"
DRONE_LAND_TIMEOUT_S_FROM_ENV="${DRONE_LAND_TIMEOUT_S+x}"; DRONE_LAND_TIMEOUT_S_ENV_VALUE="${DRONE_LAND_TIMEOUT_S:-}"

if [[ -f "$DRONE_MISSION_CONFIG" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "$DRONE_MISSION_CONFIG"
  set +a
fi
[[ -n "$DRONE_SPAWN_X_M_FROM_ENV" ]] && DRONE_SPAWN_X_M="$DRONE_SPAWN_X_M_ENV_VALUE"
[[ -n "$DRONE_SPAWN_Y_M_FROM_ENV" ]] && DRONE_SPAWN_Y_M="$DRONE_SPAWN_Y_M_ENV_VALUE"
[[ -n "$DRONE_SPAWN_Z_M_FROM_ENV" ]] && DRONE_SPAWN_Z_M="$DRONE_SPAWN_Z_M_ENV_VALUE"
[[ -n "$DRONE_ORIGIN_Z_OFFSET_M_FROM_ENV" ]] && DRONE_ORIGIN_Z_OFFSET_M="$DRONE_ORIGIN_Z_OFFSET_M_ENV_VALUE"
[[ -n "$DRONE_SPAWN_YAW_DEG_FROM_ENV" ]] && DRONE_SPAWN_YAW_DEG="$DRONE_SPAWN_YAW_DEG_ENV_VALUE"
[[ -n "$DRONE_TAKEOFF_ALTITUDE_M_FROM_ENV" ]] && DRONE_TAKEOFF_ALTITUDE_M="$DRONE_TAKEOFF_ALTITUDE_M_ENV_VALUE"
[[ -n "$DRONE_FORWARD_SPEED_MPS_FROM_ENV" ]] && DRONE_FORWARD_SPEED_MPS="$DRONE_FORWARD_SPEED_MPS_ENV_VALUE"
[[ -n "$DRONE_TARGET_DISTANCE_M_FROM_ENV" ]] && DRONE_TARGET_DISTANCE_M="$DRONE_TARGET_DISTANCE_M_ENV_VALUE"
[[ -n "$DRONE_FIRST_HOVER_S_FROM_ENV" ]] && DRONE_FIRST_HOVER_S="$DRONE_FIRST_HOVER_S_ENV_VALUE"
[[ -n "$DRONE_FINAL_HOVER_S_FROM_ENV" ]] && DRONE_FINAL_HOVER_S="$DRONE_FINAL_HOVER_S_ENV_VALUE"
[[ -n "$DRONE_LAND_DURATION_S_FROM_ENV" ]] && DRONE_LAND_DURATION_S="$DRONE_LAND_DURATION_S_ENV_VALUE"
[[ -n "$DRONE_LAND_TIMEOUT_S_FROM_ENV" ]] && DRONE_LAND_TIMEOUT_S="$DRONE_LAND_TIMEOUT_S_ENV_VALUE"

if [[ -f "$CAMERA_MODEL_CONFIG" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "$CAMERA_MODEL_CONFIG"
  set +a
fi
[[ -n "$CAMERA_BASELINE_M_FROM_ENV" ]] && CAMERA_BASELINE_M="$CAMERA_BASELINE_M_ENV_VALUE"
[[ -n "$CAMERA_HEIGHT_M_FROM_ENV" ]] && CAMERA_HEIGHT_M="$CAMERA_HEIGHT_M_ENV_VALUE"
[[ -n "$CAMERA_HORIZONTAL_FOV_RAD_FROM_ENV" ]] && CAMERA_HORIZONTAL_FOV_RAD="$CAMERA_HORIZONTAL_FOV_RAD_ENV_VALUE"
[[ -n "$CAMERA_IMAGE_WIDTH_PX_FROM_ENV" ]] && CAMERA_IMAGE_WIDTH_PX="$CAMERA_IMAGE_WIDTH_PX_ENV_VALUE"
[[ -n "$CAMERA_IMAGE_HEIGHT_PX_FROM_ENV" ]] && CAMERA_IMAGE_HEIGHT_PX="$CAMERA_IMAGE_HEIGHT_PX_ENV_VALUE"
[[ -n "$CAMERA_UPDATE_RATE_HZ_FROM_ENV" ]] && CAMERA_UPDATE_RATE_HZ="$CAMERA_UPDATE_RATE_HZ_ENV_VALUE"
[[ -n "$CAMERA_CLIP_NEAR_M_FROM_ENV" ]] && CAMERA_CLIP_NEAR_M="$CAMERA_CLIP_NEAR_M_ENV_VALUE"
[[ -n "$CAMERA_CLIP_FAR_M_FROM_ENV" ]] && CAMERA_CLIP_FAR_M="$CAMERA_CLIP_FAR_M_ENV_VALUE"

if [[ -f "$YOLO_CONFIG" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "$YOLO_CONFIG"
  set +a
fi
[[ -n "$YOLO_MODEL_PATH_FROM_ENV" ]] && YOLO_MODEL_PATH="$YOLO_MODEL_PATH_ENV_VALUE"
[[ -n "$YOLO_CLASS_NAME_FROM_ENV" ]] && YOLO_CLASS_NAME="$YOLO_CLASS_NAME_ENV_VALUE"
[[ -n "$YOLO_CONFIDENCE_FROM_ENV" ]] && YOLO_CONFIDENCE="$YOLO_CONFIDENCE_ENV_VALUE"
[[ -n "$YOLO_DEVICE_FROM_ENV" ]] && YOLO_DEVICE="$YOLO_DEVICE_ENV_VALUE"
[[ -n "$YOLO_TARGET_HZ_FROM_ENV" ]] && YOLO_TARGET_HZ="$YOLO_TARGET_HZ_ENV_VALUE"
[[ -n "$YOLO_IMGSZ_FROM_ENV" ]] && YOLO_IMGSZ="$YOLO_IMGSZ_ENV_VALUE"
[[ -n "$YOLO_PYTHON_FROM_ENV" ]] && YOLO_PYTHON="$YOLO_PYTHON_ENV_VALUE"
[[ -n "$YOLO_PYTHONPATH_FROM_ENV" ]] && YOLO_PYTHONPATH="$YOLO_PYTHONPATH_ENV_VALUE"

if [[ -n "$SIM_SCENARIO_CONFIG" ]]; then
  if [[ "$SIM_SCENARIO_CONFIG" != /* && -f "$CALLER_CWD/$SIM_SCENARIO_CONFIG" ]]; then
    SIM_SCENARIO_CONFIG="$CALLER_CWD/$SIM_SCENARIO_CONFIG"
  elif [[ "$SIM_SCENARIO_CONFIG" != /* && -f "$ROOT_DIR/config/scenarios/$SIM_SCENARIO_CONFIG" ]]; then
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
  [[ -n "$CAMERA_BASELINE_M_FROM_ENV" ]] && CAMERA_BASELINE_M="$CAMERA_BASELINE_M_ENV_VALUE"
  [[ -n "$CAMERA_HEIGHT_M_FROM_ENV" ]] && CAMERA_HEIGHT_M="$CAMERA_HEIGHT_M_ENV_VALUE"
  [[ -n "$CAMERA_HORIZONTAL_FOV_RAD_FROM_ENV" ]] && CAMERA_HORIZONTAL_FOV_RAD="$CAMERA_HORIZONTAL_FOV_RAD_ENV_VALUE"
  [[ -n "$CAMERA_IMAGE_WIDTH_PX_FROM_ENV" ]] && CAMERA_IMAGE_WIDTH_PX="$CAMERA_IMAGE_WIDTH_PX_ENV_VALUE"
  [[ -n "$CAMERA_IMAGE_HEIGHT_PX_FROM_ENV" ]] && CAMERA_IMAGE_HEIGHT_PX="$CAMERA_IMAGE_HEIGHT_PX_ENV_VALUE"
  [[ -n "$CAMERA_UPDATE_RATE_HZ_FROM_ENV" ]] && CAMERA_UPDATE_RATE_HZ="$CAMERA_UPDATE_RATE_HZ_ENV_VALUE"
  [[ -n "$CAMERA_CLIP_NEAR_M_FROM_ENV" ]] && CAMERA_CLIP_NEAR_M="$CAMERA_CLIP_NEAR_M_ENV_VALUE"
  [[ -n "$CAMERA_CLIP_FAR_M_FROM_ENV" ]] && CAMERA_CLIP_FAR_M="$CAMERA_CLIP_FAR_M_ENV_VALUE"
  [[ -n "$DRONE_SPAWN_X_M_FROM_ENV" ]] && DRONE_SPAWN_X_M="$DRONE_SPAWN_X_M_ENV_VALUE"
  [[ -n "$DRONE_SPAWN_Y_M_FROM_ENV" ]] && DRONE_SPAWN_Y_M="$DRONE_SPAWN_Y_M_ENV_VALUE"
  [[ -n "$DRONE_SPAWN_Z_M_FROM_ENV" ]] && DRONE_SPAWN_Z_M="$DRONE_SPAWN_Z_M_ENV_VALUE"
  [[ -n "$DRONE_ORIGIN_Z_OFFSET_M_FROM_ENV" ]] && DRONE_ORIGIN_Z_OFFSET_M="$DRONE_ORIGIN_Z_OFFSET_M_ENV_VALUE"
  [[ -n "$DRONE_SPAWN_YAW_DEG_FROM_ENV" ]] && DRONE_SPAWN_YAW_DEG="$DRONE_SPAWN_YAW_DEG_ENV_VALUE"
  [[ -n "$DRONE_TAKEOFF_ALTITUDE_M_FROM_ENV" ]] && DRONE_TAKEOFF_ALTITUDE_M="$DRONE_TAKEOFF_ALTITUDE_M_ENV_VALUE"
  [[ -n "$DRONE_FORWARD_SPEED_MPS_FROM_ENV" ]] && DRONE_FORWARD_SPEED_MPS="$DRONE_FORWARD_SPEED_MPS_ENV_VALUE"
  [[ -n "$DRONE_TARGET_DISTANCE_M_FROM_ENV" ]] && DRONE_TARGET_DISTANCE_M="$DRONE_TARGET_DISTANCE_M_ENV_VALUE"
  [[ -n "$DRONE_FIRST_HOVER_S_FROM_ENV" ]] && DRONE_FIRST_HOVER_S="$DRONE_FIRST_HOVER_S_ENV_VALUE"
  [[ -n "$DRONE_FINAL_HOVER_S_FROM_ENV" ]] && DRONE_FINAL_HOVER_S="$DRONE_FINAL_HOVER_S_ENV_VALUE"
  [[ -n "$DRONE_LAND_DURATION_S_FROM_ENV" ]] && DRONE_LAND_DURATION_S="$DRONE_LAND_DURATION_S_ENV_VALUE"
  [[ -n "$DRONE_LAND_TIMEOUT_S_FROM_ENV" ]] && DRONE_LAND_TIMEOUT_S="$DRONE_LAND_TIMEOUT_S_ENV_VALUE"
  [[ -n "$YOLO_MODEL_PATH_FROM_ENV" ]] && YOLO_MODEL_PATH="$YOLO_MODEL_PATH_ENV_VALUE"
  [[ -n "$YOLO_CLASS_NAME_FROM_ENV" ]] && YOLO_CLASS_NAME="$YOLO_CLASS_NAME_ENV_VALUE"
  [[ -n "$YOLO_CONFIDENCE_FROM_ENV" ]] && YOLO_CONFIDENCE="$YOLO_CONFIDENCE_ENV_VALUE"
  [[ -n "$YOLO_DEVICE_FROM_ENV" ]] && YOLO_DEVICE="$YOLO_DEVICE_ENV_VALUE"
  [[ -n "$YOLO_TARGET_HZ_FROM_ENV" ]] && YOLO_TARGET_HZ="$YOLO_TARGET_HZ_ENV_VALUE"
  [[ -n "$YOLO_IMGSZ_FROM_ENV" ]] && YOLO_IMGSZ="$YOLO_IMGSZ_ENV_VALUE"
  [[ -n "$YOLO_PYTHON_FROM_ENV" ]] && YOLO_PYTHON="$YOLO_PYTHON_ENV_VALUE"
  [[ -n "$YOLO_PYTHONPATH_FROM_ENV" ]] && YOLO_PYTHONPATH="$YOLO_PYTHONPATH_ENV_VALUE"
fi

resolve_existing_dir() {
  local candidate
  for candidate in "$@"; do
    [[ -n "$candidate" ]] || continue
    if [[ -d "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  return 1
}

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

if [[ -z "$PX4_DIR" ]]; then
  PX4_DIR="$(resolve_existing_dir \
    "$ROOT_DIR/PX4-Autopilot" \
    "$ROOT_DIR/../PX4-Autopilot" \
    "$HOME/PX4-Autopilot" \
    "$HOME/source/PX4-Autopilot" || true
  )"
fi
PX4_PYTHON_EXECUTABLE="${PX4_PYTHON_EXECUTABLE:-${PYTHON_EXECUTABLE:-$ROOT_DIR/.venv/bin/python}}"

DEFAULT_YOLO_MODEL="${DEFAULT_YOLO_MODEL:-${YOLO_MODEL_PATH:-yolov8n_uav.pt}}"
if ! DEFAULT_YOLO_MODEL="$(resolve_model_spec \
  "$DEFAULT_YOLO_MODEL" \
  yolov8n_uav.pt \
  uav1.pt \
  yolov8n.pt
)"; then
  DEFAULT_YOLO_MODEL="${DEFAULT_YOLO_MODEL:-}"
fi
DEFAULT_YOLO_CLASS="${DEFAULT_YOLO_CLASS:-${YOLO_CLASS_NAME:-UAV}}"
DEFAULT_YOLO_CONFIDENCE="${DEFAULT_YOLO_CONFIDENCE:-${YOLO_CONFIDENCE:-0.10}}"
DEFAULT_YOLO_DEVICE="${DEFAULT_YOLO_DEVICE:-${YOLO_DEVICE:-cuda:0}}"
DEFAULT_YOLO_HZ="${DEFAULT_YOLO_HZ:-${YOLO_TARGET_HZ:-40.0}}"
DEFAULT_YOLO_IMGSZ="${DEFAULT_YOLO_IMGSZ:-${YOLO_IMGSZ:-1280}}"
DEFAULT_YOLO_PYTHON="${DEFAULT_YOLO_PYTHON:-${YOLO_PYTHON:-}}"
YOLO_BOOTSTRAP_PREFERRED_PYTHON="${DEFAULT_YOLO_PYTHON:-python3}"
if [[ -n "$DEFAULT_YOLO_PYTHON" && "$DEFAULT_YOLO_PYTHON" == */* ]]; then
  DEFAULT_YOLO_PYTHON="$(resolve_executable "$DEFAULT_YOLO_PYTHON" || true)"
fi
if [[ -n "${YOLO_PYTHONPATH:-}" && -d "$YOLO_PYTHONPATH" ]]; then
  export PYTHONPATH="$YOLO_PYTHONPATH${PYTHONPATH:+:$PYTHONPATH}"
fi
if [[ "$SHOW_HELP" != true && -z "$DEFAULT_YOLO_MODEL" ]]; then
  printf '%s\n' "warning: YOLO model is not configured; stereo YOLO distance estimation will stay disabled." >&2
fi
if [[ "$SHOW_HELP" != true && -n "$DEFAULT_YOLO_MODEL" ]]; then
  if DEFAULT_YOLO_PYTHON="$(bash "$ROOT_DIR/scripts/ensure_yolo_runtime.sh" \
    --venv-dir "$ROOT_DIR/.venv" \
    --preferred-python "$YOLO_BOOTSTRAP_PREFERRED_PYTHON"
  )"; then
    local_site_packages="$(find "$ROOT_DIR/.venv" -maxdepth 3 -type d -name site-packages 2>/dev/null | head -n 1 || true)"
    if [[ -n "$local_site_packages" && -d "$local_site_packages" ]]; then
      export PYTHONPATH="$local_site_packages${PYTHONPATH:+:$PYTHONPATH}"
    fi
  else
    printf '%s\n' "warning: failed to prepare YOLO Python runtime; stereo YOLO distance estimation may stay disabled." >&2
    DEFAULT_YOLO_PYTHON="$(resolve_executable "$DEFAULT_YOLO_PYTHON" python3 || true)"
  fi
fi
if [[ "$SHOW_HELP" != true && -z "$DEFAULT_YOLO_PYTHON" ]]; then
  printf '%s\n' "warning: no executable Python found for YOLO node; using ROS launch defaults." >&2
fi
OPERATOR_RECORDER_PYTHON="${OPERATOR_RECORDER_PYTHON:-${DEFAULT_YOLO_PYTHON:-$(command -v python3 2>/dev/null || true)}}"

if [[ -f "$ATTITUDE_CONFIG" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "$ATTITUDE_CONFIG"
  set +a
fi

usage() {
  cat <<'USAGE'
Usage:
  ./run_operator_view.sh [options] [ros2 launch args...]

Starts the simulation plus standard ROS UI tools for observation:
  - one rqt_gui window
  - two Image View plugins for left/right cameras
  - three Plot plugins comparing actual vs estimated Forward/Right/Down
  - optional Dynamic Reconfigure plugin for changing YOLO parameters

This operator view exposes truth and is for calibration/validation observation;
it is not a sealed evaluation or Test surface.

Use --bag-view when the single rqt_gui window should contain Image View plus
the Bag plugin instead of the distance Plot plugin.

Default launch choices for operator observation:
  - gz_gui:=true
  - launch_rviz:=false

The stereo experiment target is moved by QGroundControl after the PX4 SITL
bridge connects. By default this script starts PX4 SITL in standalone Gazebo
mode and attaches it to the experiment target model so QGC can connect over
MAVLink.

Camera attitude defaults are read from config/camera_attitude.env.
Drone spawn and auto mission defaults are read from config/drone_mission.env.
Set SIM_SCENARIO_CONFIG to a file under config/scenarios/ to override baseline,
camera height, zoom/FOV, target size/range, mission, and YOLO settings as one
named digital-twin scenario. Explicit environment variables and launch args
still override scenario values.

Options:
  --save [DIR]           Save a lite CSV session while observing. Alias for --record-lite.
  --save-full [DIR]      Save CSV plus left/right overlay MP4. Alias for --record-full.
  --record-lite [DIR]    Record telemetry CSV while observing.
  --record-full [DIR]    Record telemetry CSV plus left/right overlay MP4.
  --record-debug [DIR]   Record CSV, overlay MP4, and debug MCAP rosbag.
  --record-calibration-csv [FILE]
                         Record future-calibration raw CSV from PX4 NED truth,
                         raw ray estimates, detections, and ray quality.
  --calibration-sim-truth
                         Use /ground_truth_position as fallback calibration truth.
  --calibration-truth-topic TOPIC
                         PX4 truth topic for calibration CSV. Default: /fmu/out/vehicle_local_position.
  --calibration-local-truth-topic TOPIC
                         PX4 local NED topic recorded into local_truth_* fields.
  --calibration-global-truth-topic TOPIC
                         PX4 global position topic recorded into global_truth_* fields.
  --calibration-gps-truth-topic TOPIC
                         PX4 GPS position topic recorded into gps_truth_* fields.
  --calibration-truth-source SOURCE
                         PX4 truth source contract: local_ned, global_position, or gps_position.
                         Default: local_ned.
  --calibration-sim-truth-role ROLE
                         Role for /ground_truth_position: oracle or training. Default: oracle.
  --calibration-hz VALUE Calibration CSV write rate. Default: 10.0.
  --yolo-model FILE      Use this YOLO .pt file for drone detection.
  --select-yolo-model    Open a file picker for selecting a YOLO .pt file.
  --yolo-class NAME      Target class name in the .pt model. Default: UAV.
  --yolo-conf VALUE      YOLO confidence threshold. Default: 0.10.
  --yolo-device DEVICE   YOLO device. Default: cuda:0, with CPU fallback if unavailable.
  --yolo-hz VALUE        YOLO scheduler target in Hz. Default: 40.0 to keep measured output above 20 Hz.
  --yolo-imgsz PIXELS    YOLO inference image size. Default from config/yolo.env.
  --yolo-python FILE     Python used for the YOLO distance node. Default: local .venv or python3.
  --plot-hz VALUE        Distance graph topic publish rate in Hz. Default: 30.0.
  --actual-hz VALUE      Gazebo actual-distance pose read rate in Hz. Default: 15.0.
  --plot-window S        Visible rqt_plot X-axis window in seconds. Default: 30.0.
  --detection-view       Show YOLO overlay image topics. This is the default.
  --raw-view             Show raw camera topics instead of YOLO overlay topics.
  YOLO_PYTHONPATH can point at a Python site-packages directory containing
                         cv2, torch, and ultralytics.
  --params               Also open Dynamic Reconfigure in rqt_gui.
  --no-px4               Do not start PX4 SITL / MAVLink for QGC.
  --px4-dir DIR          PX4-Autopilot directory. Default: $PX4_DIR
  --px4-model-name NAME  Existing Gazebo model to attach PX4 to. Default: x500_15m.
  --bag-view             Show Image View plus Bag plugin in one rqt_gui window.
  --gazebo-gui           Explicitly show Gazebo GUI. This is the default unless
                         --headless or gz_gui:=false is used.
  --no-gazebo-gui        Do not show Gazebo GUI; rqt_gui still starts unless
                         --headless is also used.
  --headless             Do not start Gazebo GUI or rqt_gui. Useful for automated
                         record-lite smoke runs.
  --auto-mission         Start PX4 offboard takeoff/approach/land after the app starts.
  --mission              Alias for --auto-mission.
  --quick-mission        Faster PX4 offboard approach settings for smoke testing.
  --mission-delay S      Delay before auto mission starts. Default: 25.
  --mission-arg ARG      Pass ARG to ./run_auto_mission.sh. Repeatable.
USAGE
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

record_mode=""
record_dir=""
calibration_csv_path=""
run_mission=false
mission_delay_s=25
bag_view=false
open_params=false
start_px4=true
show_detection_images=true
headless=false
launch_input=()
mission_args=()
selected_yolo_model=""

while (($#)); do
  case "$1" in
    --save|--record-lite)
      record_mode="lite"
      shift
      if [[ $# -gt 0 && "${1:-}" != --* && "${1:-}" != *:=* ]]; then
        record_dir="$1"
        shift
      fi
      ;;
    --save-full|--record-full)
      record_mode="full"
      shift
      if [[ $# -gt 0 && "${1:-}" != --* && "${1:-}" != *:=* ]]; then
        record_dir="$1"
        shift
      fi
      ;;
    --record-debug)
      record_mode="debug"
      shift
      if [[ $# -gt 0 && "${1:-}" != --* && "${1:-}" != *:=* ]]; then
        record_dir="$1"
        shift
      fi
      ;;
    --record-calibration-csv)
      shift
      if [[ $# -gt 0 && "${1:-}" != --* && "${1:-}" != *:=* ]]; then
        calibration_csv_path="$1"
        shift
      else
        calibration_csv_path="$ROOT_DIR/results/calibration_flight/telemetry_$(date +%Y%m%d_%H%M%S).csv"
      fi
      if [[ "$calibration_csv_path" != /* ]]; then
        calibration_csv_path="$CALLER_CWD/$calibration_csv_path"
      fi
      launch_input+=("record_calibration_csv:=true")
      launch_input+=("calibration_csv_path:=$calibration_csv_path")
      ;;
    --calibration-sim-truth)
      launch_input+=("calibration_sim_truth_topic:=/ground_truth_position")
      shift
      ;;
    --calibration-truth-topic)
      launch_input+=("calibration_truth_topic:=${2:?missing value for --calibration-truth-topic}")
      shift 2
      ;;
    --calibration-local-truth-topic)
      launch_input+=("calibration_local_truth_topic:=${2:?missing value for --calibration-local-truth-topic}")
      shift 2
      ;;
    --calibration-global-truth-topic)
      launch_input+=("calibration_global_truth_topic:=${2:?missing value for --calibration-global-truth-topic}")
      shift 2
      ;;
    --calibration-gps-truth-topic)
      launch_input+=("calibration_gps_truth_topic:=${2:?missing value for --calibration-gps-truth-topic}")
      shift 2
      ;;
    --calibration-truth-source)
      launch_input+=("calibration_truth_source:=${2:?missing value for --calibration-truth-source}")
      shift 2
      ;;
    --calibration-sim-truth-role)
      launch_input+=("calibration_sim_truth_role:=${2:?missing value for --calibration-sim-truth-role}")
      shift 2
      ;;
    --calibration-hz)
      launch_input+=("calibration_record_hz:=${2:?missing value for --calibration-hz}")
      shift 2
      ;;
    --yolo-model)
      launch_input+=("yolo_model_path:=${2:?missing value for --yolo-model}")
      selected_yolo_model="$2"
      shift 2
      ;;
    --select-yolo-model)
      if command -v zenity >/dev/null 2>&1; then
        selection_dir="$ROOT_DIR/"
        if [[ "$DEFAULT_YOLO_MODEL" == */* ]]; then
          selection_dir="${DEFAULT_YOLO_MODEL%/*}/"
        fi
        selected_yolo_model="$(zenity --file-selection --title='Select YOLO .pt model' --file-filter='YOLO model (*.pt) | *.pt' --filename="$selection_dir" 2>/dev/null || true)"
        if [[ -n "$selected_yolo_model" ]]; then
          launch_input+=("yolo_model_path:=$selected_yolo_model")
        fi
      else
        echo "warning: zenity not found; use --yolo-model /path/to/model.pt" >&2
      fi
      shift
      ;;
    --yolo-class)
      launch_input+=("yolo_class_name:=${2:?missing value for --yolo-class}")
      shift 2
      ;;
    --yolo-conf)
      launch_input+=("yolo_confidence:=${2:?missing value for --yolo-conf}")
      shift 2
      ;;
    --yolo-device)
      launch_input+=("yolo_device:=${2:?missing value for --yolo-device}")
      shift 2
      ;;
    --yolo-hz)
      launch_input+=("yolo_target_hz:=${2:?missing value for --yolo-hz}")
      shift 2
      ;;
    --yolo-imgsz)
      launch_input+=("yolo_imgsz:=${2:?missing value for --yolo-imgsz}")
      shift 2
      ;;
    --yolo-python)
      launch_input+=("yolo_python:=${2:?missing value for --yolo-python}")
      shift 2
      ;;
    --plot-hz)
      launch_input+=("plot_publish_hz:=${2:?missing value for --plot-hz}")
      shift 2
      ;;
    --actual-hz)
      launch_input+=("actual_distance_read_hz:=${2:?missing value for --actual-hz}")
      shift 2
      ;;
    --plot-window)
      DEFAULT_PLOT_WINDOW_S="${2:?missing value for --plot-window}"
      shift 2
      ;;
    --detection-view)
      show_detection_images=true
      shift
      ;;
    --raw-view)
      show_detection_images=false
      shift
      ;;
    --bag-view)
      bag_view=true
      shift
      ;;
    --gazebo-gui)
      launch_input+=("gz_gui:=true")
      shift
      ;;
    --no-gazebo-gui)
      launch_input+=("gz_gui:=false")
      shift
      ;;
    --headless)
      headless=true
      show_detection_images=false
      shift
      ;;
    --params)
      open_params=true
      shift
      ;;
    --no-px4)
      start_px4=false
      shift
      ;;
    --px4-dir)
      PX4_DIR="${2:?missing value for --px4-dir}"
      shift 2
      ;;
    --px4-model-name)
      PX4_GZ_MODEL_NAME="${2:?missing value for --px4-model-name}"
      shift 2
      ;;
    --auto-mission|--mission)
      run_mission=true
      shift
      ;;
    --quick-mission)
      run_mission=true
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
        "180"
      )
      shift
      ;;
    --mission-delay)
      mission_delay_s="${2:?missing value for --mission-delay}"
      shift 2
      ;;
    --mission-arg)
      mission_args+=("${2:?missing value for --mission-arg}")
      shift 2
      ;;
    *)
      launch_input+=("$1")
      shift
      ;;
  esac
done

if [[ "$start_px4" == true && ! -d "${PX4_DIR:-}" ]]; then
  printf '%s\n' \
    "error: PX4 mode was requested, but PX4-Autopilot was not found." \
    "Set PX4_DIR to your PX4-Autopilot checkout or run with --no-px4." >&2
  exit 2
fi

if [[ "$GAZEBO_MAJOR_VERSION" -ge 7 ]] && ! command -v gz >/dev/null 2>&1; then
  printf '%s\n' \
    "error: GAZEBO_MAJOR_VERSION=$GAZEBO_MAJOR_VERSION requires Gazebo Sim Harmonic (gz sim), but gz was not found." \
    "Install gz-harmonic, or run with GAZEBO_MAJOR_VERSION=6 for the legacy Fortress installation." >&2
  exit 2
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

children=()
terminate_matching() {
  local signal="$1"
  local pattern="$2"
  local pids
  pids="$(pgrep -f "$pattern" || true)"
  if [[ -z "$pids" ]]; then
    return 0
  fi
  kill "-$signal" $pids >/dev/null 2>&1 || true
}

terminate_gz_sim_processes() {
  local signal="$1"
  local pid args
  while read -r pid args; do
    [[ -n "${pid:-}" && -n "${args:-}" ]] || continue
    case "$args" in
      "gz sim server"|"gz sim gui"|*"stereo_ground_drone_attitude_error.sdf"*)
        kill "-$signal" "$pid" >/dev/null 2>&1 || true
        ;;
    esac
  done < <(ps -eo pid=,args= | awk '{ pid=$1; $1=""; sub(/^ +/, ""); if ($0 ~ /^gz sim( |$)/) print pid, $0 }')
}

cleanup_previous_run() {
  printf '%s\n' "Stopping stale operator-view processes..."

  terminate_matching TERM "ros2 launch stereo_ground_sim stereo_ground_sim.launch.py"
  terminate_matching TERM "stereo_ground_sim.stereo_distance_node"
  terminate_matching TERM "stereo_ground_sim.detection_overlay_node"
  terminate_matching TERM "stereo_ground_sim_overlay/detection_overlay_node"
  terminate_matching TERM "detection_overlay_node --ros-args"
  terminate_matching TERM "ros_gz_bridge/parameter_bridge"
  terminate_matching TERM "parameter_bridge /left_camera/image"
  terminate_matching TERM "parameter_bridge /right_camera/image"
  terminate_matching TERM "gz sim .*stereo_ground_drone_attitude_error.sdf"
  terminate_matching TERM "^gz sim server$"
  terminate_matching TERM "^gz sim gui$"
  terminate_gz_sim_processes TERM
  terminate_matching TERM "rqt_gui --clear-config --perspective-file $ROOT_DIR/config/operator_rqt"
  terminate_matching TERM "rqt_gui --clear-config .*stereo_ground_sim.operator_panel.OperatorPanel"
  terminate_matching TERM "operator_recorder --mode"
  terminate_matching TERM "calibration_csv_recorder"
  terminate_matching TERM "ros2 bag record .* /stereo/distance"
  terminate_matching TERM "run_auto_mission.sh"
  terminate_matching TERM "px4_auto_mission.py"
  terminate_matching TERM "make px4_sitl gz_x500"
  terminate_matching TERM "$PX4_DIR/build/px4_sitl_default/bin/px4"

  sleep 1

  terminate_matching KILL "ros2 launch stereo_ground_sim stereo_ground_sim.launch.py"
  terminate_matching KILL "stereo_ground_sim.stereo_distance_node"
  terminate_matching KILL "stereo_ground_sim.detection_overlay_node"
  terminate_matching KILL "stereo_ground_sim_overlay/detection_overlay_node"
  terminate_matching KILL "detection_overlay_node --ros-args"
  terminate_matching KILL "ros_gz_bridge/parameter_bridge"
  terminate_matching KILL "parameter_bridge /left_camera/image"
  terminate_matching KILL "parameter_bridge /right_camera/image"
  terminate_matching KILL "gz sim .*stereo_ground_drone_attitude_error.sdf"
  terminate_matching KILL "^gz sim server$"
  terminate_matching KILL "^gz sim gui$"
  terminate_gz_sim_processes KILL
  terminate_matching KILL "rqt_gui --clear-config --perspective-file $ROOT_DIR/config/operator_rqt"
  terminate_matching KILL "rqt_gui --clear-config .*stereo_ground_sim.operator_panel.OperatorPanel"
  terminate_matching KILL "operator_recorder --mode"
  terminate_matching KILL "calibration_csv_recorder"
  terminate_matching KILL "ros2 bag record .* /stereo/distance"
  terminate_matching KILL "run_auto_mission.sh"
  terminate_matching KILL "px4_auto_mission.py"
  terminate_matching KILL "make px4_sitl gz_x500"
  terminate_matching KILL "$PX4_DIR/build/px4_sitl_default/bin/px4"

  rm -f /tmp/stereo_ground_drone_attitude_error.sdf /tmp/stereo_distance_node_params.yaml /tmp/detection_overlay_node_params.yaml
}

cleanup() {
  local pid
  for pid in "${children[@]}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -TERM "-$pid" >/dev/null 2>&1 || kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  sleep 1
  for pid in "${children[@]}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -KILL "-$pid" >/dev/null 2>&1 || kill -KILL "$pid" >/dev/null 2>&1 || true
    fi
  done
  terminate_matching KILL "rqt_gui --clear-config --perspective-file $ROOT_DIR/config/operator_rqt"
  terminate_matching KILL "rqt_gui --clear-config .*stereo_ground_sim.operator_panel.OperatorPanel"
  terminate_matching KILL "operator_recorder --mode"
  terminate_matching KILL "calibration_csv_recorder"
  terminate_matching KILL "stereo_ground_sim.detection_overlay_node"
  terminate_matching KILL "stereo_ground_sim_overlay/detection_overlay_node"
  terminate_matching KILL "detection_overlay_node --ros-args"
  terminate_matching KILL "^gz sim server$"
  terminate_matching KILL "^gz sim gui$"
  terminate_gz_sim_processes KILL
}
trap cleanup EXIT INT TERM
cleanup_previous_run

has_launch_arg() {
  local name="$1"
  shift
  local arg
  for arg in "$@"; do
    [[ "$arg" == "$name":=* ]] && return 0
  done
  return 1
}

get_launch_arg_value() {
  local name="$1"
  shift
  local arg
  for arg in "$@"; do
    if [[ "$arg" == "$name":=* ]]; then
      printf '%s\n' "${arg#*:=}"
      return 0
    fi
  done
  return 1
}

launch_args=()
add_default_launch_arg() {
  local name="$1"
  local value="$2"
  shift 2
  if ! has_launch_arg "$name" "$@"; then
    launch_args+=("$name:=$value")
  fi
}

add_optional_default_launch_arg() {
  local name="$1"
  local value="$2"
  shift 2
  if [[ -n "$value" ]] && ! has_launch_arg "$name" "$@"; then
    launch_args+=("$name:=$value")
  fi
}

add_default_launch_arg left_roll_error_deg "${LEFT_ROLL_ERROR_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg left_pitch_error_deg "${LEFT_PITCH_ERROR_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg left_yaw_error_deg "${LEFT_YAW_ERROR_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg right_roll_error_deg "${RIGHT_ROLL_ERROR_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg right_pitch_error_deg "${RIGHT_PITCH_ERROR_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg right_yaw_error_deg "${RIGHT_YAW_ERROR_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg scenario_name "${SIM_SCENARIO_NAME:-unspecified}" "${launch_input[@]}"
add_default_launch_arg target_class "${SIM_TARGET_CLASS:-unspecified}" "${launch_input[@]}"
add_default_launch_arg target_size_m "${SIM_TARGET_SIZE_M:-nan}" "${launch_input[@]}"
add_default_launch_arg scenario_range_min_m "${SIM_RANGE_MIN_M:-nan}" "${launch_input[@]}"
add_default_launch_arg scenario_range_max_m "${SIM_RANGE_MAX_M:-nan}" "${launch_input[@]}"
add_default_launch_arg zoom_index "${SIM_ZOOM_INDEX:-nan}" "${launch_input[@]}"
add_default_launch_arg baseline_m "${CAMERA_BASELINE_M:-25.0}" "${launch_input[@]}"
add_default_launch_arg camera_height_m "${CAMERA_HEIGHT_M:-5.0}" "${launch_input[@]}"
add_default_launch_arg horizontal_fov_rad "${CAMERA_HORIZONTAL_FOV_RAD:-0.154997710698}" "${launch_input[@]}"
add_default_launch_arg camera_image_width_px "${CAMERA_IMAGE_WIDTH_PX:-1280}" "${launch_input[@]}"
add_default_launch_arg camera_image_height_px "${CAMERA_IMAGE_HEIGHT_PX:-720}" "${launch_input[@]}"
add_default_launch_arg camera_update_rate_hz "${CAMERA_UPDATE_RATE_HZ:-60.0}" "${launch_input[@]}"
add_default_launch_arg camera_clip_near_m "${CAMERA_CLIP_NEAR_M:-0.1}" "${launch_input[@]}"
add_default_launch_arg camera_clip_far_m "${CAMERA_CLIP_FAR_M:-2500.0}" "${launch_input[@]}"

drone_spawn_y_default="${DRONE_SPAWN_Y_M:-2000.0}"
approach_start_default="$drone_spawn_y_default"
approach_end_default="${DRONE_TARGET_DISTANCE_M:-500.0}"
if has_launch_arg approach_start_distance_m "${launch_input[@]}" && ! has_launch_arg target_spawn_y_m "${launch_input[@]}"; then
  drone_spawn_y_default="$(get_launch_arg_value approach_start_distance_m "${launch_input[@]}")"
elif has_launch_arg target_spawn_y_m "${launch_input[@]}" && ! has_launch_arg approach_start_distance_m "${launch_input[@]}"; then
  approach_start_default="$(get_launch_arg_value target_spawn_y_m "${launch_input[@]}")"
fi
if has_launch_arg approach_end_distance_m "${launch_input[@]}"; then
  approach_end_default="$(get_launch_arg_value approach_end_distance_m "${launch_input[@]}")"
fi

add_default_launch_arg approach_start_distance_m "$approach_start_default" "${launch_input[@]}"
add_default_launch_arg approach_end_distance_m "$approach_end_default" "${launch_input[@]}"
if [[ -n "$approach_start_default" && -n "$approach_end_default" && -n "${DRONE_FORWARD_SPEED_MPS:-}" ]]; then
  launch_approach_duration_s="$(
    awk -v start="$approach_start_default" -v end="$approach_end_default" -v speed="$DRONE_FORWARD_SPEED_MPS" \
      'BEGIN { if (speed > 0) printf "%.6f", (start > end ? start - end : end - start) / speed }'
  )"
  add_default_launch_arg approach_duration_s "$launch_approach_duration_s" "${launch_input[@]}"
fi
add_default_launch_arg takeoff_altitude_m "${DRONE_TAKEOFF_ALTITUDE_M:-10.0}" "${launch_input[@]}"
add_default_launch_arg first_hover_duration_s "${DRONE_FIRST_HOVER_S:-5.0}" "${launch_input[@]}"
add_default_launch_arg final_hover_duration_s "${DRONE_FINAL_HOVER_S:-5.0}" "${launch_input[@]}"
add_default_launch_arg land_duration_s "${DRONE_LAND_DURATION_S:-10.0}" "${launch_input[@]}"
add_default_launch_arg target_spawn_x_m "${DRONE_SPAWN_X_M:-0.0}" "${launch_input[@]}"
add_default_launch_arg target_spawn_y_m "$drone_spawn_y_default" "${launch_input[@]}"
add_default_launch_arg target_spawn_z_m "${DRONE_SPAWN_Z_M:-0.75}" "${launch_input[@]}"
add_default_launch_arg target_origin_z_offset_m "${DRONE_ORIGIN_Z_OFFSET_M:-0.0}" "${launch_input[@]}"
add_default_launch_arg target_spawn_yaw_deg "${DRONE_SPAWN_YAW_DEG:--90.0}" "${launch_input[@]}"
add_default_launch_arg target_model_name "$PX4_GZ_MODEL_NAME" "${launch_input[@]}"
add_default_launch_arg gz_version "$GAZEBO_MAJOR_VERSION" "${launch_input[@]}"
if [[ -n "$DEFAULT_YOLO_MODEL" ]]; then
  add_default_launch_arg yolo_model_path "$DEFAULT_YOLO_MODEL" "${launch_input[@]}"
  add_default_launch_arg yolo_class_name "$DEFAULT_YOLO_CLASS" "${launch_input[@]}"
  add_default_launch_arg yolo_confidence "$DEFAULT_YOLO_CONFIDENCE" "${launch_input[@]}"
  add_default_launch_arg yolo_device "$DEFAULT_YOLO_DEVICE" "${launch_input[@]}"
  add_default_launch_arg yolo_target_hz "$DEFAULT_YOLO_HZ" "${launch_input[@]}"
  add_default_launch_arg yolo_imgsz "$DEFAULT_YOLO_IMGSZ" "${launch_input[@]}"
  if [[ "$show_detection_images" == true ]]; then
    add_default_launch_arg overlay_publish_hz "$DEFAULT_OVERLAY_HZ" "${launch_input[@]}"
  else
    add_default_launch_arg overlay_publish_hz "0.0" "${launch_input[@]}"
  fi
fi
if [[ -x "$DEFAULT_YOLO_PYTHON" ]]; then
  add_default_launch_arg yolo_python "$DEFAULT_YOLO_PYTHON" "${launch_input[@]}"
fi
add_default_launch_arg plot_publish_hz "$DEFAULT_PLOT_HZ" "${launch_input[@]}"
add_default_launch_arg actual_distance_read_hz "$DEFAULT_ACTUAL_DISTANCE_HZ" "${launch_input[@]}"
	if ! has_launch_arg gz_gui "${launch_input[@]}"; then
  if [[ "$headless" == true ]]; then
    launch_args+=("gz_gui:=false")
  else
    launch_args+=("gz_gui:=true")
  fi
fi
if ! has_launch_arg launch_rviz "${launch_input[@]}"; then
  launch_args+=("launch_rviz:=false")
fi
launch_args+=("${launch_input[@]}")
rqt_yolo_model_path="${DEFAULT_YOLO_MODEL:-}"
explicit_yolo_model_path="$(get_launch_arg_value yolo_model_path "${launch_args[@]}" || true)"
if [[ -n "$explicit_yolo_model_path" ]]; then
  rqt_yolo_model_path="$explicit_yolo_model_path"
fi
rqt_approach_delta="$(
  awk -v start="$approach_start_default" -v end="$approach_end_default" \
    'BEGIN { if (start == start && end == end) printf "%.1f", (start > end ? start - end : end - start); else printf "--" }'
)"
rqt_mission_label="Mission: ${approach_start_default} -> ${approach_end_default} m (approach ${rqt_approach_delta} m)"
rqt_coord_contract="Frame: public FRD [Forward, Right, Down, Range]; Down origin offset ${DRONE_ORIGIN_Z_OFFSET_M:-0.0} m"

printf '%s\n' "Camera attitude source: $ATTITUDE_CONFIG"
printf '%s\n' "Camera model source: $CAMERA_MODEL_CONFIG"
printf '%s\n' "Drone mission source: $DRONE_MISSION_CONFIG"
printf '%s\n' "Applied launch args:"
printf '  %s\n' "${launch_args[@]}"
if [[ -n "$calibration_csv_path" ]]; then
  printf '%s\n' "Calibration CSV will be recorded to: $calibration_csv_path"
fi

colcon build --packages-select stereo_ground_sim stereo_ground_sim_overlay

set +u
# shellcheck disable=SC1091
source "$ROOT_DIR/install/setup.bash"
set -u
RQT_PYTHONPATH="$ROOT_DIR/python_patches${PYTHONPATH:+:$PYTHONPATH}"

setsid ros2 launch stereo_ground_sim stereo_ground_sim.launch.py "${launch_args[@]}" &
children+=("$!")

sleep 8

if [[ "$start_px4" == true ]]; then
  if [[ "$PX4_CONSOLE_LOG" != "/dev/null" ]]; then
    mkdir -p "$(dirname "$PX4_CONSOLE_LOG")"
  fi
  setsid bash -c '
    px4_dir="$1"
    world="$2"
    model_name="$3"
    display_value="$4"
    px4_python="$5"
    cd "$px4_dir"
    export DISPLAY="$display_value"
    if [[ -x "$px4_python" ]]; then
      export PYTHON_EXECUTABLE="$px4_python"
    fi
    export PX4_GZ_STANDALONE=1
    export PX4_GZ_WORLD="$world"
    export PX4_GZ_MODEL_NAME="$model_name"
    export PX4_GZ_NO_FOLLOW=1
    exec make px4_sitl gz_x500
  ' _ "$PX4_DIR" "$PX4_GZ_WORLD" "$PX4_GZ_MODEL_NAME" "${DISPLAY:-:0}" "$PX4_PYTHON_EXECUTABLE" >"$PX4_CONSOLE_LOG" 2>&1 &
  children+=("$!")
  printf '%s\n' "PX4 SITL will attach to Gazebo model: $PX4_GZ_MODEL_NAME"
  printf '%s\n' "QGC should connect through PX4 MAVLink UDP 14550."
  if [[ "$PX4_CONSOLE_LOG" != "/dev/null" ]]; then
    printf '%s\n' "PX4 console log: $PX4_CONSOLE_LOG"
  fi
fi

perspective_file="$ROOT_DIR/config/operator_rqt.perspective"
if [[ "$bag_view" == true ]]; then
  perspective_file="$ROOT_DIR/config/operator_rqt_bag.perspective"
fi

if [[ "$show_detection_images" == true ]]; then
  rqt_image_topic_mode="detection"
else
  rqt_image_topic_mode="raw"
fi

if [[ "$headless" != true ]]; then
  if [[ "$bag_view" != true ]]; then
    setsid env PYTHONPATH="$RQT_PYTHONPATH" RQT_IMAGE_TOPIC_MODE="$rqt_image_topic_mode" RQT_PLOT_WINDOW_S="$DEFAULT_PLOT_WINDOW_S" RQT_YOLO_MODEL_PATH="$rqt_yolo_model_path" RQT_MISSION_LABEL="$rqt_mission_label" RQT_COORD_CONTRACT="$rqt_coord_contract" OPERATOR_RECORDER_PYTHON="$OPERATOR_RECORDER_PYTHON" ros2 run rqt_gui rqt_gui \
      --clear-config \
      --force-discover \
      --standalone stereo_ground_sim.operator_panel.OperatorPanel &
  else
    setsid env PYTHONPATH="$RQT_PYTHONPATH" RQT_IMAGE_TOPIC_MODE="$rqt_image_topic_mode" RQT_PLOT_WINDOW_S="$DEFAULT_PLOT_WINDOW_S" RQT_YOLO_MODEL_PATH="$rqt_yolo_model_path" RQT_MISSION_LABEL="$rqt_mission_label" RQT_COORD_CONTRACT="$rqt_coord_contract" OPERATOR_RECORDER_PYTHON="$OPERATOR_RECORDER_PYTHON" ros2 run rqt_gui rqt_gui \
      --clear-config \
      --force-discover \
      --perspective-file "$perspective_file" &
  fi
  children+=("$!")

  sleep 3
  rqt_pid="$(pgrep -n -f 'rqt_gui --clear-config' || true)"
  if [[ -n "$rqt_pid" ]]; then
    if [[ "$open_params" == true && "$bag_view" != true ]]; then
      for _ in {1..5}; do
        rqt_pid="$(pgrep -n -f '/lib/rqt_gui/rqt_gui --clear-config' || pgrep -n -f 'rqt_gui --clear-config' || true)"
        [[ -n "$rqt_pid" ]] || break
        env PYTHONPATH="$RQT_PYTHONPATH" RQT_IMAGE_TOPIC_MODE="$rqt_image_topic_mode" RQT_PLOT_WINDOW_S="$DEFAULT_PLOT_WINDOW_S" RQT_YOLO_MODEL_PATH="$rqt_yolo_model_path" RQT_MISSION_LABEL="$rqt_mission_label" RQT_COORD_CONTRACT="$rqt_coord_contract" OPERATOR_RECORDER_PYTHON="$OPERATOR_RECORDER_PYTHON" ros2 run rqt_gui rqt_gui \
          --command-pid "$rqt_pid" \
          --command-start-plugin rqt_reconfigure/Param \
          --args /stereo_distance_node >/dev/null 2>&1 && break
        sleep 1
      done
    fi
    if [[ "$bag_view" == true ]]; then
      for _ in {1..8}; do
        rqt_pid="$(pgrep -n -f '/lib/rqt_gui/rqt_gui --clear-config --perspective-file' || pgrep -n -f 'rqt_gui --clear-config --perspective-file' || true)"
        [[ -n "$rqt_pid" ]] || break
        env PYTHONPATH="$RQT_PYTHONPATH" RQT_IMAGE_TOPIC_MODE="$rqt_image_topic_mode" RQT_PLOT_WINDOW_S="$DEFAULT_PLOT_WINDOW_S" RQT_YOLO_MODEL_PATH="$rqt_yolo_model_path" RQT_MISSION_LABEL="$rqt_mission_label" RQT_COORD_CONTRACT="$rqt_coord_contract" OPERATOR_RECORDER_PYTHON="$OPERATOR_RECORDER_PYTHON" ros2 run rqt_gui rqt_gui \
          --command-pid "$rqt_pid" \
          --command-start-plugin stereo_ground_sim/RecorderPanel >/dev/null 2>&1 && break
        sleep 1
      done
    fi
  fi
else
  printf '%s\n' "Headless mode: rqt_gui is disabled."
fi

if [[ -n "$record_mode" ]]; then
  if [[ -z "$record_dir" ]]; then
    record_dir="$ROOT_DIR/bags/operator_$(date +%Y%m%d_%H%M%S)_$record_mode"
  fi
  mkdir -p "$record_dir"
  setsid "$OPERATOR_RECORDER_PYTHON" -m stereo_ground_sim.operator_recorder \
    --mode "$record_mode" \
    --output-dir "$record_dir" &
  children+=("$!")
  printf '%s\n' "Recording $record_mode session to: $record_dir"
fi

if [[ "$run_mission" == true ]]; then
  if [[ "$start_px4" != true ]]; then
    printf '%s\n' "warning: --auto-mission ignored because --no-px4 was set." >&2
  else
    setsid bash -c 'delay="$1"; root="$2"; shift 2; sleep "$delay"; exec "$root/run_auto_mission.sh" "$@"' _ "$mission_delay_s" "$ROOT_DIR" "${mission_args[@]}" &
    children+=("$!")
    printf '%s\n' "PX4 auto mission will start after ${mission_delay_s}s."
  fi
fi

printf '%s\n' "Operator view started."
if [[ "$start_px4" == true ]]; then
  printf '%s\n' "PX4/QGC mode: QGC controls the existing Gazebo model $PX4_GZ_MODEL_NAME."
fi
if [[ "$bag_view" == true ]]; then
  printf '%s\n' "The single rqt_gui window contains left/right camera views and rqt_bag."
elif [[ "$headless" == true ]]; then
  printf '%s\n' "Headless mode: no rqt_gui window was started."
else
  printf '%s\n' "The single rqt_gui window contains left/right camera views and Forward/Right/Down plots."
fi
printf '%s\n' "Use QGC for normal control of the existing x500_15m target."
if [[ "$run_mission" == true && "$start_px4" == true ]]; then
  printf '%s\n' "Auto mission command: ./run_auto_mission.sh ${mission_args[*]}"
fi

wait "${children[0]}"
