#!/usr/bin/env bash
set -euo pipefail

CALLER_CWD="$(pwd)"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INTEGRATION_ROOT="$(cd "$ROOT_DIR/.." && pwd)"
SASL_ROS_WORKSPACE="${SASL_ROS_WORKSPACE:-$INTEGRATION_ROOT/workspace/main}"
PX4_MSGS_SOURCE_DIR="$SASL_ROS_WORKSPACE/src/px4_msgs"
PX4_MSGS_OVERLAY_SETUP="$SASL_ROS_WORKSPACE/install/setup.bash"
PX4_MSGS_OVERLAY_MANIFEST="$SASL_ROS_WORKSPACE/install/share/sasl_ros_workspace/px4_msgs-overlay.env"
PX4_MSGS_PACKAGE_SETUP="$SASL_ROS_WORKSPACE/install/px4_msgs/share/px4_msgs/local_setup.bash"
PX4_MSGS_REF="${PX4_MSGS_REF:-86d8239e962f6939e05c3737784f60c02fa884db}"
cd "$ROOT_DIR"

for arg in "$@"; do
  if [[ "$arg" == "--no-px4" ]]; then
    printf '%s\n' \
      "error: --no-px4 has been removed; PX4 SITL is required for operator view." \
      "Configure a valid PX4-Autopilot checkout with --px4-dir DIR or PX4_DIR." >&2
    exit 2
  fi
done

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros2-launch-log}"
export GZ_CONFIG_HOME="${GZ_CONFIG_HOME:-/tmp/gz-config}"
PX4_CONSOLE_LOG="${PX4_CONSOLE_LOG:-/dev/null}"
PX4_UXRCE_DDS_AGENT="${PX4_UXRCE_DDS_AGENT:-MicroXRCEAgent}"
PX4_UXRCE_DDS_PORT="${PX4_UXRCE_DDS_PORT:-8888}"
PX4_UXRCE_DDS_AGENT_LOG="${PX4_UXRCE_DDS_AGENT_LOG:-/dev/null}"
PX4_UXRCE_DDS_LIBRARY_PATH="${PX4_UXRCE_DDS_LIBRARY_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$ROOT_DIR/install/stereo_ground_sim/share/stereo_ground_sim/models:${GZ_SIM_RESOURCE_PATH:-}"
RQT_PYTHONPATH="$ROOT_DIR/python_patches:$ROOT_DIR/stereo_ground_sim${PYTHONPATH:+:$PYTHONPATH}"
DEFAULT_OVERLAY_HZ="${DEFAULT_OVERLAY_HZ:-5.0}"
DEFAULT_PLOT_HZ="${DEFAULT_PLOT_HZ:-30.0}"
DEFAULT_PLOT_WINDOW_S="${DEFAULT_PLOT_WINDOW_S:-30.0}"
DEFAULT_BBOX_CENTER_ADJUSTMENT_PATH="${DEFAULT_BBOX_CENTER_ADJUSTMENT_PATH:-$ROOT_DIR/../ref/Stereo_Camera_3D_Position_DKL/Cent_Bbox_Adj_Mtrx.csv}"
PX4_DIR="${PX4_DIR:-}"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-stereo_ground_drone}"
GAZEBO_MAJOR_VERSION="${GAZEBO_MAJOR_VERSION:-8}"
SIM_CONFIG="${SIM_CONFIG:-$ROOT_DIR/config/simulation.env}"
SIM_SCENARIO_CONFIG="${SIM_SCENARIO_CONFIG:-}"

SIM_EXPLICIT_NAMES=(
  CAMERA_BASELINE_M CAMERA_HEIGHT_M CAMERA_HORIZONTAL_FOV_RAD
  CAMERA_IMAGE_WIDTH_PX CAMERA_IMAGE_HEIGHT_PX CAMERA_UPDATE_RATE_HZ
  CAMERA_CLIP_NEAR_M CAMERA_CLIP_FAR_M
  CAM_L_OPTICAL_ROLL_DEG CAM_L_OPTICAL_PITCH_DEG CAM_L_OPTICAL_YAW_DEG
  CAM_R_OPTICAL_ROLL_DEG CAM_R_OPTICAL_PITCH_DEG CAM_R_OPTICAL_YAW_DEG
  CAM_L_PX4_ATTITUDE_ROLL_DEG CAM_L_PX4_ATTITUDE_PITCH_DEG CAM_L_PX4_ATTITUDE_YAW_DEG
  CAM_R_PX4_ATTITUDE_ROLL_DEG CAM_R_PX4_ATTITUDE_PITCH_DEG CAM_R_PX4_ATTITUDE_YAW_DEG
  CAM_L_PX4_MODEL_NAME CAM_R_PX4_MODEL_NAME CAM_L_PX4_INSTANCE CAM_R_PX4_INSTANCE
  CAM_L_PX4_DDS_NAMESPACE CAM_R_PX4_DDS_NAMESPACE
  PX4_REQUIRED_VERSION
  COMMON_OPTICAL_ROLL_ERROR_DEG RELATIVE_OPTICAL_ROLL_ERROR_DEG
  COMMON_OPTICAL_PITCH_ERROR_DEG RELATIVE_OPTICAL_PITCH_ERROR_DEG
  COMMON_OPTICAL_YAW_ERROR_DEG RELATIVE_OPTICAL_YAW_ERROR_DEG
  PX4_GZ_MODEL_NAME
  DRONE_SPAWN_X_M DRONE_SPAWN_Y_M DRONE_SPAWN_Z_M DRONE_SPAWN_YAW_DEG
  DRONE_TAKEOFF_ALTITUDE_M DRONE_FORWARD_SPEED_MPS DRONE_MOVE_DISTANCE_M
  DRONE_TARGET_DISTANCE_M DRONE_APPROACH_TIMEOUT_S DRONE_TAKEOFF_TIMEOUT_S
  DRONE_FIRST_HOVER_S DRONE_FINAL_HOVER_S DRONE_LAND_DURATION_S
  DRONE_LAND_TIMEOUT_S DRONE_SETPOINT_HZ
  YOLO_MODEL_PATH YOLO_CLASS_NAME YOLO_CONFIDENCE YOLO_DEVICE YOLO_TARGET_HZ
  YOLO_IMGSZ YOLO_BACKEND YOLO_TENSORRT_ENGINE_PATH MEASUREMENT_QUEUE_CAPACITY
  YOLO_PYTHON YOLO_PYTHONPATH
  BBOX_CENTER_ADJUSTMENT_ENABLE BBOX_CENTER_ADJUSTMENT_PATH
)
# shellcheck disable=SC1091
source "$ROOT_DIR/scripts/load_sim_config.sh"
load_sim_config_preserving_env "${SIM_EXPLICIT_NAMES[@]}"

# Fallback for SIM_CONFIG files that predate this variable.
PX4_GZ_MODEL_NAME="${PX4_GZ_MODEL_NAME:-x500_15m}"

for legacy_px4_attitude_var in \
  CAM_L_PX4_ROLL_DEG CAM_L_PX4_PITCH_DEG CAM_L_PX4_YAW_DEG \
  CAM_R_PX4_ROLL_DEG CAM_R_PX4_PITCH_DEG CAM_R_PX4_YAW_DEG
do
  if [[ -v "$legacy_px4_attitude_var" ]]; then
    printf '%s\n' \
      "error: $legacy_px4_attitude_var is obsolete; it used to set an independent absolute PX4 body attitude." \
      "Use CAM_L_PX4_ATTITUDE_ROLL_DEG / CAM_L_PX4_ATTITUDE_PITCH_DEG / CAM_L_PX4_ATTITUDE_YAW_DEG" \
      "and the matching CAM_R_PX4_ATTITUDE_* variables for the independent PX4 body attitude." >&2
    exit 2
  fi
done

is_px4_checkout() {
  local candidate="${1:-}"
  [[ -d "$candidate" && -f "$candidate/Makefile" && -d "$candidate/Tools" && -d "$candidate/src" ]]
}

resolve_px4_checkout() {
  local candidate
  for candidate in "$@"; do
    [[ -n "$candidate" ]] || continue
    if is_px4_checkout "$candidate"; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  return 1
}

resolve_model_spec() {
  local candidate local_path
  for candidate in "$@"; do
    [[ -n "$candidate" ]] || continue
    if [[ -f "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
    if [[ "$candidate" != */* ]]; then
      # Keep the checked-out workspace self-contained first, while accepting
      # the standard per-user download folders for machine-local YOLO weights.
      # A bare model name is still returned as a final fallback so Ultralytics
      # can resolve its own built-in public weights.
      for local_path in \
        "$ROOT_DIR/$candidate" \
        "$ROOT_DIR/../$candidate" \
        "$HOME/Downloads/$candidate" \
        "$HOME/다운로드/$candidate"; do
        if [[ -f "$local_path" ]]; then
          printf '%s\n' "$local_path"
          return 0
        fi
      done
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
  PX4_DIR="$(resolve_px4_checkout \
    "$ROOT_DIR/PX4-Autopilot" \
    "$ROOT_DIR/../PX4-Autopilot" \
    "$HOME/PX4-Autopilot" \
    "$HOME/source/PX4-Autopilot" || true
  )"
fi
PX4_PYTHON_EXECUTABLE="${PX4_PYTHON_EXECUTABLE:-}"
if [[ -z "$PX4_PYTHON_EXECUTABLE" ]]; then
  for px4_python_candidate in \
    "$ROOT_DIR/.venv-px4-local/bin/python" \
    "$ROOT_DIR/.venv-px4-v1.17.0/bin/python"; do
    if [[ -x "$px4_python_candidate" ]] \
      && "$px4_python_candidate" -c 'import kconfiglib' >/dev/null 2>&1; then
      PX4_PYTHON_EXECUTABLE="$px4_python_candidate"
      break
    fi
  done
fi

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
DEFAULT_YOLO_BACKEND="${DEFAULT_YOLO_BACKEND:-${YOLO_BACKEND:-tensorrt}}"
DEFAULT_YOLO_TENSORRT_ENGINE_PATH="${DEFAULT_YOLO_TENSORRT_ENGINE_PATH:-${YOLO_TENSORRT_ENGINE_PATH:-}}"

usage() {
  cat <<'USAGE'
Usage:
  ./run_operator_view.sh [options] [ros2 launch args...]

Starts the simulation plus standard ROS UI tools for observation:
  - one rqt_gui window
  - two Image View plugins for left/right cameras
  - three Plot plugins comparing actual vs estimated Forward/Right/Down
  - optional Dynamic Reconfigure plugin for changing YOLO parameters

Use --bag-view when the single rqt_gui window should contain Image View plus
the Bag plugin instead of the distance Plot plugin.

Default launch choices for operator observation:
  - gz_gui:=true
  - launch_rviz:=true

The stereo experiment target is moved by QGroundControl after the PX4 SITL
bridge connects. By default this script starts PX4 SITL in standalone Gazebo
mode and attaches it to the experiment target model so QGC can connect over
MAVLink.

Simulation defaults are read from config/simulation.env.
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
                         Record pipeline-ready calibration CSV from PX4 NED truth,
                         StereoCAM estimates, detections, and disparity.
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
  --no-yolo              Disable YOLO without bootstrapping its Python runtime.
  --select-yolo-model    Open a file picker for selecting a YOLO .pt file.
  --yolo-class NAME      Target class name in the .pt model. Default: UAV.
  --yolo-conf VALUE      YOLO confidence threshold. Default: 0.10.
  --yolo-device DEVICE   YOLO device. Default: cuda:0, with CPU fallback if unavailable.
  --yolo-hz VALUE        YOLO scheduler target in Hz. Default: 40.0 to keep measured output above 20 Hz.
  --yolo-imgsz PIXELS    YOLO inference image size. Default from config/simulation.env.
  --yolo-backend NAME    tensorrt (default) or pytorch. TensorRT uses a local FP16 engine.
  --yolo-python FILE     Python used for the YOLO distance node. Default: local .venv or python3.
  --bbox-adjustment FILE Use a 6x4 bbox-center CSV or JSON runtime profile and publish corrected_position.
  --no-bbox-adjustment   Disable bbox-center adjustment even if env defaults are set.
  --observation-fusion-manifest FILE
                         Fuse runtime image observations before raw RDF/profile apply.
  --observation-fusion-tier TIER
                         Select a tier from the observation-fusion manifest.
  --no-observation-fusion
                         Disable observation fusion even if env defaults are set.
  --disparity-residual-profile FILE
                         Apply a runtime residual disparity profile after bbox/profile correction.
  --disparity-residual-feature-selector FILE
                         Let the StereoCAM node select a residual disparity profile from runtime
                         image/disparity/RDF summary features.
  --disparity-residual-feature-catalog FILE
                         Residual profile catalog JSON for --disparity-residual-feature-selector.
  --disparity-residual-feature-window-size N
                         Rolling runtime feature window size for node-side residual selection.
  --disparity-residual-feature-min-samples N
                         Minimum samples before node-side residual selection is allowed.
  --no-disparity-residual
                         Disable residual disparity correction even if env defaults are set.
  --no-disparity-residual-feature-selector
                         Disable node-side residual feature selection.
  --plot-hz VALUE        Distance graph topic publish rate in Hz. Default: 30.0.
  --plot-window S        Visible rqt_plot X-axis window in seconds. Default: 30.0.
  --detection-view       Show YOLO overlay image topics. This is the default.
  --raw-view             Show raw camera topics instead of YOLO overlay topics.
  --preflight-view        Open the preflight perspective (camera and topic readiness only).
  YOLO_PYTHONPATH can point at a Python site-packages directory containing
                         cv2, torch, and ultralytics.
  --params               Also open Dynamic Reconfigure in rqt_gui.
  --px4-dir DIR          PX4-Autopilot directory. Default: $PX4_DIR
  --px4-model-name NAME  Existing Gazebo model to attach PX4 to. Default: x500_15m.
  --bag-view             Show Image View plus Bag plugin in one rqt_gui window.
  --gazebo-gui           Show Gazebo GUI. Default keeps Gazebo server-only while
                         RViz and rqt_gui remain open.
  --no-gazebo-gui        Keep Gazebo server-only; this is the default.
  --headless             Do not start Gazebo GUI or rqt_gui. YOLO and detection
                         overlay topics remain enabled unless --no-yolo or
                         --raw-view is also selected.
  --skip-colcon-build     Reuse the existing ROS install and repo-local Python sources.
  --skip-px4-build        Reuse the existing PX4 SITL binary and build output.
  --auto-mission         Start PX4 offboard takeoff/approach/land after the app starts.
  --mission              Alias for --auto-mission.
  --quick-mission        Faster PX4 offboard approach settings for smoke testing.
  --mission-delay S      Delay before auto mission starts. Default: 25.
  --mission-arg ARG      Pass ARG to ./run_mission.sh. Repeatable.
USAGE
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

require_canonical_px4_msgs_overlay() {
  local expected_commit actual_commit overlay_commit
  if [[ ! -d "$PX4_MSGS_SOURCE_DIR/.git" || ! -f "$PX4_MSGS_SOURCE_DIR/package.xml" ]]; then
    printf '%s\n' \
      "error: canonical px4_msgs source is missing: $PX4_MSGS_SOURCE_DIR" \
      "Run: cd $INTEGRATION_ROOT && scripts/bootstrap_ros_workspace.sh" >&2
    return 2
  fi
  expected_commit="$(git -C "$PX4_MSGS_SOURCE_DIR" rev-parse "${PX4_MSGS_REF}^{commit}" 2>/dev/null || true)"
  actual_commit="$(git -C "$PX4_MSGS_SOURCE_DIR" rev-parse HEAD 2>/dev/null || true)"
  if [[ -z "$expected_commit" || "$actual_commit" != "$expected_commit" ]]; then
    printf '%s\n' \
      "error: canonical px4_msgs revision mismatch." \
      "required: $PX4_MSGS_REF" \
      "actual: ${actual_commit:-unavailable}" \
      "Repair with: cd $INTEGRATION_ROOT && scripts/bootstrap_ros_workspace.sh" >&2
    return 2
  fi
  if [[ ! -f "$PX4_MSGS_OVERLAY_SETUP" ]]; then
    printf '%s\n' \
      "error: canonical px4_msgs overlay is not built: $PX4_MSGS_OVERLAY_SETUP" \
      "Run: cd $INTEGRATION_ROOT && scripts/bootstrap_ros_workspace.sh" >&2
    return 2
  fi
  if [[ ! -f "$PX4_MSGS_OVERLAY_MANIFEST" ]]; then
    printf '%s\n' \
      "error: canonical px4_msgs overlay stamp is missing: $PX4_MSGS_OVERLAY_MANIFEST" \
      "Run: cd $INTEGRATION_ROOT && scripts/bootstrap_ros_workspace.sh" >&2
    return 2
  fi
  if [[ ! -f "$PX4_MSGS_PACKAGE_SETUP" ]]; then
    printf '%s\n' \
      "error: px4_msgs Python environment hook is not built: $PX4_MSGS_PACKAGE_SETUP" \
      "Run: cd $INTEGRATION_ROOT && scripts/bootstrap_ros_workspace.sh" >&2
    return 2
  fi
  overlay_commit="$(sed -n 's/^PX4_MSGS_COMMIT=//p' "$PX4_MSGS_OVERLAY_MANIFEST" | head -n 1)"
  if [[ "$overlay_commit" != "$actual_commit" ]]; then
    printf '%s\n' \
      "error: canonical px4_msgs overlay is stale." \
      "source: $actual_commit" \
      "overlay: ${overlay_commit:-unavailable}" \
      "Rebuild with: cd $INTEGRATION_ROOT && scripts/bootstrap_ros_workspace.sh" >&2
    return 2
  fi
}

if ! require_canonical_px4_msgs_overlay; then
  exit 2
fi

record_mode=""
record_dir=""
record_wait_for_bbox=false
recording_pid=""
calibration_csv_path=""
run_mission=false
mission_delay_s=25
bag_view=false
preflight_view=false
open_params=false
show_detection_images=true
headless=false
launch_input=()
mission_args=()
selected_yolo_model=""
yolo_enabled=true
skip_colcon_build=false
skip_px4_build=false

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
    --record-wait-for-bbox)
      record_wait_for_bbox=true
      shift
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
      yolo_enabled=true
      DEFAULT_YOLO_MODEL="${2:?missing value for --yolo-model}"
      launch_input+=("yolo_model_path:=${2:?missing value for --yolo-model}")
      selected_yolo_model="$2"
      shift 2
      ;;
    --no-yolo)
      yolo_enabled=false
      show_detection_images=false
      shift
      ;;
    --select-yolo-model)
      if command -v zenity >/dev/null 2>&1; then
        selection_dir="$ROOT_DIR/"
        if [[ "$DEFAULT_YOLO_MODEL" == */* ]]; then
          selection_dir="${DEFAULT_YOLO_MODEL%/*}/"
        fi
        selected_yolo_model="$(zenity --file-selection --title='Select YOLO .pt model' --file-filter='YOLO model (*.pt) | *.pt' --filename="$selection_dir" 2>/dev/null || true)"
        if [[ -n "$selected_yolo_model" ]]; then
          yolo_enabled=true
          DEFAULT_YOLO_MODEL="$selected_yolo_model"
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
    --yolo-backend)
      DEFAULT_YOLO_BACKEND="${2:?missing value for --yolo-backend}"
      shift 2
      ;;
    --yolo-python)
      launch_input+=("yolo_python:=${2:?missing value for --yolo-python}")
      shift 2
      ;;
    --bbox-adjustment)
      launch_input+=("enable_bbox_center_adjustment:=true")
      launch_input+=("bbox_center_adjustment_path:=${2:?missing value for --bbox-adjustment}")
      shift 2
      ;;
    --no-bbox-adjustment)
      launch_input+=("enable_bbox_center_adjustment:=false")
      shift
      ;;
    --observation-fusion-manifest)
      launch_input+=("enable_observation_fusion:=true")
      launch_input+=("observation_fusion_manifest_path:=${2:?missing value for --observation-fusion-manifest}")
      shift 2
      ;;
    --observation-fusion-tier)
      launch_input+=("observation_fusion_tier:=${2:?missing value for --observation-fusion-tier}")
      shift 2
      ;;
    --no-observation-fusion)
      launch_input+=("enable_observation_fusion:=false")
      shift
      ;;
    --disparity-residual-profile)
      launch_input+=("enable_disparity_residual_profile:=true")
      launch_input+=("disparity_residual_profile_path:=${2:?missing value for --disparity-residual-profile}")
      shift 2
      ;;
    --disparity-residual-feature-selector)
      launch_input+=("enable_disparity_residual_feature_selector:=true")
      launch_input+=("disparity_residual_feature_selector_model_path:=${2:?missing value for --disparity-residual-feature-selector}")
      shift 2
      ;;
    --disparity-residual-feature-catalog)
      launch_input+=("disparity_residual_feature_selector_catalog_path:=${2:?missing value for --disparity-residual-feature-catalog}")
      shift 2
      ;;
    --disparity-residual-feature-window-size)
      launch_input+=("disparity_residual_feature_selector_window_size:=${2:?missing value for --disparity-residual-feature-window-size}")
      shift 2
      ;;
    --disparity-residual-feature-min-samples)
      launch_input+=("disparity_residual_feature_selector_min_samples:=${2:?missing value for --disparity-residual-feature-min-samples}")
      shift 2
      ;;
    --no-disparity-residual)
      launch_input+=("enable_disparity_residual_profile:=false")
      shift
      ;;
    --no-disparity-residual-feature-selector)
      launch_input+=("enable_disparity_residual_feature_selector:=false")
      shift
      ;;
    --plot-hz)
      launch_input+=("plot_publish_hz:=${2:?missing value for --plot-hz}")
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
    --preflight-view)
      preflight_view=true
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
      shift
      ;;
    --skip-colcon-build)
      skip_colcon_build=true
      shift
      ;;
    --skip-px4-build)
      skip_px4_build=true
      shift
      ;;
    --params)
      open_params=true
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

if ! is_px4_checkout "${PX4_DIR:-}"; then
  printf '%s\n' \
    "error: PX4 SITL requires a valid PX4-Autopilot checkout." \
    "Set PX4_DIR or pass --px4-dir DIR; the checkout must contain Makefile, Tools/, and src/." >&2
  exit 2
fi
PX4_DIR="$(cd "$PX4_DIR" && pwd -P)"
PX4_ACTUAL_VERSION="$(git -C "$PX4_DIR" describe --tags --exact-match HEAD 2>/dev/null || true)"
if [[ "$PX4_ACTUAL_VERSION" != "${PX4_REQUIRED_VERSION:-v1.17.0}" ]]; then
  printf '%s\n' \
    "error: PX4 exact version mismatch." \
    "required: ${PX4_REQUIRED_VERSION:-v1.17.0}" \
    "actual: ${PX4_ACTUAL_VERSION:-unversioned}" \
    "checkout: $PX4_DIR" >&2
  exit 2
fi

require_sensor_gps_definition_match() {
  local px4_sensor_gps ros_sensor_gps
  px4_sensor_gps="$PX4_DIR/msg/SensorGps.msg"
  ros_sensor_gps="$PX4_MSGS_SOURCE_DIR/msg/SensorGps.msg"
  if [[ ! -f "$px4_sensor_gps" || ! -f "$ros_sensor_gps" ]]; then
    # Minimal launcher fixtures and older external test harnesses may not
    # carry interface sources; the runtime workspaces do.  Keep those
    # harnesses runnable while enforcing equality whenever both definitions
    # are present.
    return 0
  fi
  if ! cmp -s "$px4_sensor_gps" "$ros_sensor_gps"; then
    printf '%s\n' \
      "error: PX4 and px4_msgs SensorGps.msg definitions differ." \
      "The DDS CDR layout must match before starting operator view." \
      "PX4: $px4_sensor_gps" \
      "ROS: $ros_sensor_gps" >&2
    diff -u "$px4_sensor_gps" "$ros_sensor_gps" | sed -n '1,80p' >&2 || true
    return 2
  fi
}

if ! require_sensor_gps_definition_match; then
  exit 2
fi

resolve_px4_python() {
  local candidate
  for candidate in "$@"; do
    [[ -n "$candidate" && -x "$candidate" ]] || continue
    if "$candidate" -c 'import kconfiglib' >/dev/null 2>&1; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  return 1
}

if [[ -z "$PX4_PYTHON_EXECUTABLE" ]]; then
  PX4_PYTHON_EXECUTABLE="$(resolve_px4_python \
    "$(dirname "$PX4_DIR")/.venv-px4-${PX4_ACTUAL_VERSION}/bin/python" \
    "$(dirname "$PX4_DIR")/.venv-px4/bin/python" \
    "$(command -v python3 2>/dev/null || true)" \
    || true
  )"
elif ! PX4_PYTHON_EXECUTABLE="$(resolve_px4_python "$PX4_PYTHON_EXECUTABLE" || true)"; then
  PX4_PYTHON_EXECUTABLE=""
fi
if [[ -z "$PX4_PYTHON_EXECUTABLE" ]]; then
  printf '%s\n' \
    "error: PX4 SITL requires a Python interpreter with kconfiglib." \
    "Set PX4_PYTHON_EXECUTABLE or install the versioned sibling .venv-px4-${PX4_ACTUAL_VERSION} environment." >&2
  exit 2
fi

ensure_no_px4_build_in_progress() {
  local pid cwd args
  while read -r pid; do
    [[ -n "${pid:-}" ]] || continue
    cwd="$(readlink -f "/proc/$pid/cwd" 2>/dev/null || true)"
    [[ "$cwd" == "$PX4_DIR" ]] || continue
    args="$(ps -p "$pid" -o args= 2>/dev/null || true)"
    if [[ "$args" == *"px4_sitl_default"* ]]; then
      printf '%s\n' \
        "error: PX4 SITL build is already running in this checkout (PID $pid)." \
        "Stop or wait for that build before starting operator view." >&2
      return 1
    fi
  done < <(pgrep -x make || true)
  return 0
}

if ! ensure_no_px4_build_in_progress; then
  exit 2
fi

DEFAULT_YOLO_PYTHON="${DEFAULT_YOLO_PYTHON:-${YOLO_PYTHON:-}}"
YOLO_BOOTSTRAP_PREFERRED_PYTHON="${DEFAULT_YOLO_PYTHON:-python3}"
if [[ "$yolo_enabled" != true ]]; then
  DEFAULT_YOLO_MODEL=""
  DEFAULT_YOLO_PYTHON=""
fi
if [[ -n "$DEFAULT_YOLO_PYTHON" && "$DEFAULT_YOLO_PYTHON" == */* ]]; then
  DEFAULT_YOLO_PYTHON="$(resolve_executable "$DEFAULT_YOLO_PYTHON" || true)"
fi
if [[ -n "${YOLO_PYTHONPATH:-}" && -d "$YOLO_PYTHONPATH" ]]; then
  export PYTHONPATH="$YOLO_PYTHONPATH${PYTHONPATH:+:$PYTHONPATH}"
fi
if [[ "$yolo_enabled" != true ]]; then
  printf '%s\n' "YOLO disabled by --no-yolo."
elif [[ -z "$DEFAULT_YOLO_MODEL" ]]; then
  printf '%s\n' "warning: YOLO model is not configured; stereo YOLO distance estimation will stay disabled." >&2
fi
if [[ "$yolo_enabled" == true && -n "$DEFAULT_YOLO_MODEL" ]]; then
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
if [[ "$yolo_enabled" == true && -z "$DEFAULT_YOLO_PYTHON" ]]; then
  printf '%s\n' "error: YOLO is enabled but no Python runtime is available." >&2
  exit 2
fi
if [[ "$yolo_enabled" == true ]]; then
  case "$DEFAULT_YOLO_BACKEND" in
    tensorrt)
      if [[ -z "$DEFAULT_YOLO_TENSORRT_ENGINE_PATH" ]]; then
        if [[ "$DEFAULT_YOLO_MODEL" != *.pt ]]; then
          printf '%s\n' "error: TensorRT requires a .pt source model or YOLO_TENSORRT_ENGINE_PATH." >&2
          exit 2
        fi
        DEFAULT_YOLO_TENSORRT_ENGINE_PATH="${DEFAULT_YOLO_MODEL%.pt}.engine"
      fi
      if [[ ! -f "$DEFAULT_YOLO_TENSORRT_ENGINE_PATH" ]]; then
        printf '%s\n' "Preparing TensorRT FP16 engine: $DEFAULT_YOLO_TENSORRT_ENGINE_PATH"
        if ! "$DEFAULT_YOLO_PYTHON" "$ROOT_DIR/scripts/prepare_yolo_tensorrt_engine.py" \
          --model "$DEFAULT_YOLO_MODEL" \
          --engine "$DEFAULT_YOLO_TENSORRT_ENGINE_PATH" \
          --imgsz "$DEFAULT_YOLO_IMGSZ" \
          --batch 2 \
          --device "$DEFAULT_YOLO_DEVICE"; then
          printf '%s\n' "error: TensorRT FP16 engine preparation failed; refusing an unverified PyTorch/CPU fallback." >&2
          exit 2
        fi
      fi
      DEFAULT_YOLO_MODEL="$DEFAULT_YOLO_TENSORRT_ENGINE_PATH"
      ;;
    pytorch)
      ;;
    *)
      printf '%s\n' "error: --yolo-backend must be tensorrt or pytorch, got: $DEFAULT_YOLO_BACKEND" >&2
      exit 2
      ;;
  esac
fi
export YOLO_BACKEND="$DEFAULT_YOLO_BACKEND"
export YOLO_MODEL_PATH="$DEFAULT_YOLO_MODEL"
OPERATOR_RECORDER_PYTHON="${OPERATOR_RECORDER_PYTHON:-${DEFAULT_YOLO_PYTHON:-$(command -v python3 2>/dev/null || true)}}"

if [[ "$GAZEBO_MAJOR_VERSION" -ge 7 ]] && ! command -v gz >/dev/null 2>&1; then
  printf '%s\n' \
    "error: GAZEBO_MAJOR_VERSION=$GAZEBO_MAJOR_VERSION requires Gazebo Sim Harmonic (gz sim), but gz was not found." \
    "Install gz-harmonic, or run with GAZEBO_MAJOR_VERSION=6 for the legacy Fortress installation." >&2
  exit 2
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
source "$PX4_MSGS_OVERLAY_SETUP"
source "$PX4_MSGS_PACKAGE_SETUP"
set -u

children=()
start_uxrce_dds_agent() {
  local existing_pid agent_binary agent_prefix candidate_library_path agent_library_path agent_pid
  existing_pid="$(pgrep -f "[M]icroXRCEAgent.*udp4.*-p[ =]${PX4_UXRCE_DDS_PORT}([[:space:]]|$)" | head -n 1 || true)"
  if [[ -n "$existing_pid" ]]; then
    printf '%s\n' "Using existing MicroXRCEAgent PID $existing_pid on UDP port $PX4_UXRCE_DDS_PORT."
    return 0
  fi

  agent_binary="$(resolve_executable "$PX4_UXRCE_DDS_AGENT" MicroXRCEAgent || true)"
  if [[ -z "$agent_binary" ]]; then
    printf '%s\n' \
      "error: MicroXRCEAgent is required before PX4 SITL starts." \
      "Set PX4_UXRCE_DDS_AGENT to its executable path." >&2
    return 2
  fi

  agent_library_path="$PX4_UXRCE_DDS_LIBRARY_PATH"
  if [[ -z "$agent_library_path" ]]; then
    agent_prefix="$(cd "$(dirname "$agent_binary")/.." && pwd -P)"
    candidate_library_path="$agent_prefix/lib"
    if [[ -d "$candidate_library_path" ]]; then
      agent_library_path="$candidate_library_path${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
  fi
  if [[ "$PX4_UXRCE_DDS_AGENT_LOG" != "/dev/null" ]]; then
    mkdir -p "$(dirname "$PX4_UXRCE_DDS_AGENT_LOG")"
  fi
  if [[ -n "$agent_library_path" ]]; then
    setsid env LD_LIBRARY_PATH="$agent_library_path" "$agent_binary" udp4 -p "$PX4_UXRCE_DDS_PORT" \
      >"$PX4_UXRCE_DDS_AGENT_LOG" 2>&1 &
  else
    setsid "$agent_binary" udp4 -p "$PX4_UXRCE_DDS_PORT" \
      >"$PX4_UXRCE_DDS_AGENT_LOG" 2>&1 &
  fi
  agent_pid="$!"
  children+=("$agent_pid")
  sleep 1
  if ! kill -0 "$agent_pid" >/dev/null 2>&1; then
    printf '%s\n' \
      "error: MicroXRCEAgent exited before PX4 startup." \
      "Inspect: $PX4_UXRCE_DDS_AGENT_LOG" >&2
    return 2
  fi
  printf '%s\n' "MicroXRCEAgent started on UDP port $PX4_UXRCE_DDS_PORT."
}

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
  terminate_matching TERM "rqt_gui .*--clear-config .*--perspective-file .*operator_rqt.*\\.perspective"
  terminate_matching TERM "rqt_gui --clear-config .*stereo_ground_sim.operator_panel.OperatorPanel"
  terminate_matching TERM "operator_recorder --mode"
  terminate_matching TERM "calibration_csv_recorder"
  terminate_matching TERM "ros2 bag record .* /stereo/distance"
  terminate_matching TERM "run_mission.sh"
  terminate_matching TERM "run_auto.sh"
  terminate_matching TERM "px4_auto_mission.py"
  terminate_matching TERM "make px4_sitl gz_x500"
  terminate_matching TERM "$PX4_DIR/build/px4_sitl_default/bin/px4"
  terminate_matching TERM "MicroXRCEAgent.*udp4.*-p[ =]${PX4_UXRCE_DDS_PORT}"

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
  terminate_matching KILL "rqt_gui .*--clear-config .*--perspective-file .*operator_rqt.*\\.perspective"
  terminate_matching KILL "rqt_gui --clear-config .*stereo_ground_sim.operator_panel.OperatorPanel"
  terminate_matching KILL "operator_recorder --mode"
  terminate_matching KILL "calibration_csv_recorder"
  terminate_matching KILL "ros2 bag record .* /stereo/distance"
  terminate_matching KILL "run_mission.sh"
  terminate_matching KILL "run_auto.sh"
  terminate_matching KILL "px4_auto_mission.py"
  terminate_matching KILL "make px4_sitl gz_x500"
  terminate_matching KILL "$PX4_DIR/build/px4_sitl_default/bin/px4"
  terminate_matching KILL "MicroXRCEAgent.*udp4.*-p[ =]${PX4_UXRCE_DDS_PORT}"

  rm -f /tmp/stereo_ground_drone_attitude_error.sdf /tmp/stereo_distance_node_params.yaml /tmp/detection_overlay_node_params.yaml
}

cleanup() {
  local pid
  # The research recorder is owned by the Python stereo-distance node. Send
  # SIGINT first: its main loop turns KeyboardInterrupt into node.shutdown(),
  # which drains the S2 queue and writes pair_records.csv/run_summary.json.
  # A process-group SIGTERM first would terminate that Python process before
  # this finalization path can run.
  if [[ -n "${research_output_dir:-}" ]]; then
    terminate_matching INT "stereo_ground_sim.stereo_distance_node"
    terminate_matching INT "stereo_ground_sim/stereo_distance_node"
    for _ in {1..180}; do
      [[ -f "$research_output_dir/RUN_FINALIZED_VALID" ]] && break
      sleep 1
    done
  fi
  for pid in "${children[@]}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -TERM "-$pid" >/dev/null 2>&1 || kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  # The recorder flushes CSV/MP4/XLSX and writes its final validity marker on
  # SIGTERM.  Give it a bounded grace period before the generic child cleanup
  # escalates to SIGKILL; otherwise Stop can leave RUN_INCOMPLETE despite a
  # completed Auto mission.
  if [[ -n "$recording_pid" ]]; then
    for _ in {1..15}; do
      kill -0 "$recording_pid" >/dev/null 2>&1 || break
      sleep 1
    done
  fi
  sleep 1
  for pid in "${children[@]}"; do
    # ``ros2 run rqt_gui`` can exit after its inner Python process forks.
    # Signal the recorded process group even if that original wrapper PID has
    # already exited, otherwise the inner rqt_gui window survives a restart.
    kill -KILL "-$pid" >/dev/null 2>&1 || kill -KILL "$pid" >/dev/null 2>&1 || true
  done
  terminate_matching KILL "rqt_gui .*--clear-config .*--perspective-file .*operator_rqt.*\\.perspective"
  terminate_matching KILL "rqt_gui --clear-config .*stereo_ground_sim.operator_panel.OperatorPanel"
  terminate_matching KILL "operator_recorder --mode"
  terminate_matching KILL "calibration_csv_recorder"
  terminate_matching KILL "stereo_ground_sim.detection_overlay_node"
  terminate_matching KILL "stereo_ground_sim_overlay/detection_overlay_node"
  terminate_matching KILL "detection_overlay_node --ros-args"
  terminate_matching KILL "ros2 launch stereo_ground_sim stereo_ground_sim.launch.py"
  terminate_matching KILL "stereo_ground_sim.stereo_distance_node"
  terminate_matching KILL "$PX4_DIR/build/px4_sitl_default/bin/px4"
  terminate_matching KILL "MicroXRCEAgent.*udp4.*-p[ =]${PX4_UXRCE_DDS_PORT}"
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

add_default_launch_arg cam_l_optical_roll_deg "${CAM_L_OPTICAL_ROLL_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg cam_l_optical_pitch_deg "${CAM_L_OPTICAL_PITCH_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg cam_l_optical_yaw_deg "${CAM_L_OPTICAL_YAW_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg cam_r_optical_roll_deg "${CAM_R_OPTICAL_ROLL_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg cam_r_optical_pitch_deg "${CAM_R_OPTICAL_PITCH_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg cam_r_optical_yaw_deg "${CAM_R_OPTICAL_YAW_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg cam_l_px4_attitude_roll_deg "${CAM_L_PX4_ATTITUDE_ROLL_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg cam_l_px4_attitude_pitch_deg "${CAM_L_PX4_ATTITUDE_PITCH_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg cam_l_px4_attitude_yaw_deg "${CAM_L_PX4_ATTITUDE_YAW_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg cam_r_px4_attitude_roll_deg "${CAM_R_PX4_ATTITUDE_ROLL_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg cam_r_px4_attitude_pitch_deg "${CAM_R_PX4_ATTITUDE_PITCH_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg cam_r_px4_attitude_yaw_deg "${CAM_R_PX4_ATTITUDE_YAW_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg common_optical_roll_error_deg "${COMMON_OPTICAL_ROLL_ERROR_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg relative_optical_roll_error_deg "${RELATIVE_OPTICAL_ROLL_ERROR_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg common_optical_pitch_error_deg "${COMMON_OPTICAL_PITCH_ERROR_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg relative_optical_pitch_error_deg "${RELATIVE_OPTICAL_PITCH_ERROR_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg common_optical_yaw_error_deg "${COMMON_OPTICAL_YAW_ERROR_DEG:-0.0}" "${launch_input[@]}"
add_default_launch_arg relative_optical_yaw_error_deg "${RELATIVE_OPTICAL_YAW_ERROR_DEG:-0.0}" "${launch_input[@]}"
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
add_default_launch_arg camera_update_rate_hz "${CAMERA_UPDATE_RATE_HZ:-20.0}" "${launch_input[@]}"
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
add_default_launch_arg target_spawn_yaw_deg "${DRONE_SPAWN_YAW_DEG:--90.0}" "${launch_input[@]}"
add_default_launch_arg target_model_name "$PX4_GZ_MODEL_NAME" "${launch_input[@]}"
add_default_launch_arg gz_version "$GAZEBO_MAJOR_VERSION" "${launch_input[@]}"
add_default_launch_arg enable_yolo "$yolo_enabled" "${launch_input[@]}"
add_default_launch_arg enable_gps_baseline_triangulation "${ENABLE_GPS_BASELINE_TRIANGULATION:-true}" "${launch_input[@]}"
add_default_launch_arg gps_baseline_source "${GPS_BASELINE_SOURCE:-gps_baseline}" "${launch_input[@]}"
add_default_launch_arg gps_baseline_max_age_s "${GPS_BASELINE_MAX_AGE_S:-2.5}" "${launch_input[@]}"
add_default_launch_arg px4_attitude_left_topic "${PX4_ATTITUDE_LEFT_TOPIC:-/camera_left/fmu/out/vehicle_attitude}" "${launch_input[@]}"
add_default_launch_arg px4_attitude_right_topic "${PX4_ATTITUDE_RIGHT_TOPIC:-/camera_right/fmu/out/vehicle_attitude}" "${launch_input[@]}"
add_default_launch_arg px4_attitude_max_age_s "${PX4_ATTITUDE_MAX_AGE_S:-2.5}" "${launch_input[@]}"
add_default_launch_arg px4_attitude_pair_skew_s "${PX4_ATTITUDE_PAIR_SKEW_S:-2.5}" "${launch_input[@]}"
if [[ -n "$DEFAULT_YOLO_MODEL" ]]; then
  add_default_launch_arg yolo_model_path "$DEFAULT_YOLO_MODEL" "${launch_input[@]}"
  add_default_launch_arg yolo_class_name "$DEFAULT_YOLO_CLASS" "${launch_input[@]}"
  add_default_launch_arg yolo_confidence "$DEFAULT_YOLO_CONFIDENCE" "${launch_input[@]}"
  add_default_launch_arg yolo_device "$DEFAULT_YOLO_DEVICE" "${launch_input[@]}"
  add_default_launch_arg yolo_target_hz "$DEFAULT_YOLO_HZ" "${launch_input[@]}"
  add_default_launch_arg yolo_imgsz "$DEFAULT_YOLO_IMGSZ" "${launch_input[@]}"
  add_default_launch_arg measurement_queue_capacity "${MEASUREMENT_QUEUE_CAPACITY:-4096}" "${launch_input[@]}"
  add_default_launch_arg measurement_latest_only "${MEASUREMENT_LATEST_ONLY:-true}" "${launch_input[@]}"
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
	add_default_launch_arg enable_bbox_center_adjustment "${BBOX_CENTER_ADJUSTMENT_ENABLE:-false}" "${launch_input[@]}"
  add_default_launch_arg bbox_center_adjustment_path "${BBOX_CENTER_ADJUSTMENT_PATH:-$DEFAULT_BBOX_CENTER_ADJUSTMENT_PATH}" "${launch_input[@]}"
	add_default_launch_arg enable_observation_fusion "${OBSERVATION_FUSION_ENABLE:-false}" "${launch_input[@]}"
	add_optional_default_launch_arg observation_fusion_manifest_path "${OBSERVATION_FUSION_MANIFEST_PATH:-}" "${launch_input[@]}"
	add_optional_default_launch_arg observation_fusion_tier "${OBSERVATION_FUSION_TIER:-}" "${launch_input[@]}"
	add_default_launch_arg enable_disparity_residual_feature_selector "${DISPARITY_RESIDUAL_FEATURE_SELECTOR_ENABLE:-false}" "${launch_input[@]}"
	add_optional_default_launch_arg disparity_residual_feature_selector_model_path "${DISPARITY_RESIDUAL_FEATURE_SELECTOR_MODEL_PATH:-}" "${launch_input[@]}"
	add_optional_default_launch_arg disparity_residual_feature_selector_catalog_path "${DISPARITY_RESIDUAL_FEATURE_SELECTOR_CATALOG_PATH:-}" "${launch_input[@]}"
	add_default_launch_arg disparity_residual_feature_selector_window_size "${DISPARITY_RESIDUAL_FEATURE_SELECTOR_WINDOW_SIZE:-1}" "${launch_input[@]}"
	add_default_launch_arg disparity_residual_feature_selector_min_samples "${DISPARITY_RESIDUAL_FEATURE_SELECTOR_MIN_SAMPLES:-1}" "${launch_input[@]}"
	if ! has_launch_arg gz_gui "${launch_input[@]}"; then
  launch_args+=("gz_gui:=false")
fi
if ! has_launch_arg launch_rviz "${launch_input[@]}"; then
  if [[ "$headless" == true ]]; then
    launch_args+=("launch_rviz:=false")
  else
    launch_args+=("launch_rviz:=true")
  fi
fi
if ! has_launch_arg enable_rviz_rays "${launch_input[@]}"; then
  launch_args+=("enable_rviz_rays:=true")
fi
launch_args+=("${launch_input[@]}")
research_output_dir="$(get_launch_arg_value research_output_dir "${launch_args[@]}" || true)"
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
rqt_coord_contract="Frame: baseline_center_frd [Forward, Right, Down, Range]; Ground Truth vehicle_reference_link at base_link origin"

printf '%s\n' "Simulation config source: $SIM_CONFIG"
printf '%s\n' "Applied launch args:"
printf '  %s\n' "${launch_args[@]}"
if [[ -n "$calibration_csv_path" ]]; then
  printf '%s\n' "Calibration CSV will be recorded to: $calibration_csv_path"
fi

if [[ "$skip_colcon_build" == true ]]; then
  printf '%s\n' "Skipping colcon build; reusing the existing ROS install and repo-local Python sources."
else
  # Build the producer-owned message package together with the Python nodes.
  # Without it, the immutable experiment-context publisher starts from an
  # install that has package metadata but no generated Python message module.
  # Keep the CMake build space outside the repository: colcon/rosidl can
  # mis-handle the non-ASCII checkout path when its build cache is under
  # Simulation/build. The install space remains repo-local and portable.
  operator_colcon_build_base="${SASL_OPERATOR_COLCON_BUILD_BASE:-}"
  if [[ -z "$operator_colcon_build_base" ]]; then
    operator_colcon_build_base="$(mktemp -d /tmp/sasl-operator-colcon-build.XXXXXX)"
  fi
  colcon build \
    --base-paths "$ROOT_DIR" \
    --build-base "$operator_colcon_build_base" \
    --packages-select simulation_interfaces stereo_ground_sim stereo_ground_sim_overlay
fi

# Builds (if needed) a local drop-in replacement for gz-sim's installed
# magnetometer plugin whose built-in geomagnetic table disagrees with PX4's
# own WMM table at this world's lat/lon (see vendor dir README/comments).
# The system gz-sim package is never modified; _generate_world() points the
# world SDF's magnetometer plugin at this .so by absolute path when set.
"$ROOT_DIR/stereo_ground_sim/vendor/gz_magnetometer_wmm_patch/build.sh"
export STEREO_GROUND_SIM_MAGNETOMETER_PLUGIN_PATH="$ROOT_DIR/stereo_ground_sim/vendor/gz_magnetometer_wmm_patch/build/libgz-sim-magnetometer-system.so"

set +u
# shellcheck disable=SC1091
source "$ROOT_DIR/install/setup.bash"
set -u
# The install-space entry points use importlib.metadata to resolve the
# package, while the source checkout owns sasl_common.  Keep both available
# when reusing an existing install with --skip-colcon-build.  The build-space
# egg-info is generated by colcon and is intentionally only a local fallback;
# a normal rebuild recreates it.
export PYTHONPATH="$ROOT_DIR/stereo_ground_sim:$INTEGRATION_ROOT:$ROOT_DIR/build/stereo_ground_sim:$ROOT_DIR/stereo_ground_sim_overlay${PYTHONPATH:+:$PYTHONPATH}"
RQT_PYTHONPATH="$ROOT_DIR/python_patches:$ROOT_DIR/stereo_ground_sim:$INTEGRATION_ROOT:$ROOT_DIR/build/stereo_ground_sim${PYTHONPATH:+:$PYTHONPATH}"

setsid ros2 launch stereo_ground_sim stereo_ground_sim.launch.py "${launch_args[@]}" &
ros_launch_pid="$!"
children+=("$ros_launch_pid")

sleep 8

start_uxrce_dds_agent

if [[ "$PX4_CONSOLE_LOG" != "/dev/null" ]]; then
  mkdir -p "$(dirname "$PX4_CONSOLE_LOG")"
fi
setsid bash -c '
    set -euo pipefail
    px4_dir="$1"
    world="$2"
    target_model="$3"
    left_model="$4"
    right_model="$5"
    left_instance="$6"
    right_instance="$7"
    left_namespace="$8"
    right_namespace="$9"
    display_value="${10}"
    px4_python="${11}"
    uxrce_dds_port="${12}"
    console_log="${13}"
    camera_airframe_src="${14}"
    skip_px4_build="${15}"
    cd "$px4_dir"
    export DISPLAY="$display_value"
    if [[ -x "$px4_python" ]]; then
      export PYTHON_EXECUTABLE="$px4_python"
    fi

    # The left/right camera Pixhawks are static mounts that never fly; they
    # boot on a dedicated airframe (see vendor/px4_camera_airframe/README.md)
    # instead of the target drone airframe 4001_gz_x500, so EKF2_MAG_TYPE can
    # be set for them without affecting the target. Copied into
    # PX4-Autopilot ROMFS (untracked by SASL git) before every build so it
    # stays in sync.
    cp "$camera_airframe_src" \
      "$px4_dir/ROMFS/px4fmu_common/init.d-posix/airframes/4090_camera_px4"

    if [[ "$skip_px4_build" == true ]]; then
      printf '%s\n' "Skipping PX4 SITL build; reusing $px4_dir/build/px4_sitl_default."
    else
      make px4_sitl_default
    fi

    px4_binary="$px4_dir/build/px4_sitl_default/bin/px4"
    gz_bridge_dir="$px4_dir/build/px4_sitl_default/src/modules/simulation/gz_bridge"
    if [[ ! -x "$px4_binary" || ! -d "$gz_bridge_dir" ]]; then
      printf "%s\n" "error: PX4 SITL build did not produce the expected px4 binary and gz_bridge directory." >&2
      exit 2
    fi

    # `make px4_sitl_default` only re-syncs ROMFS -> build/px4_sitl_default/etc
    # (which build/px4_sitl_default/rootfs/N/etc symlinks to, and is what a
    # running instance actually reads airframe files from) when ninja
    # considers that step out of date; a newly-added ROMFS file is not
    # reliably picked up by an incremental build ("ninja: no work to do"
    # even though the file was never copied). Copy directly into the build
    # output too so the airframe is guaranteed present regardless.
    mkdir -p "$px4_dir/build/px4_sitl_default/etc/init.d-posix/airframes"
    cp "$camera_airframe_src" \
      "$px4_dir/build/px4_sitl_default/etc/init.d-posix/airframes/4090_camera_px4"
    if [[ "$left_instance" == "0" || "$right_instance" == "0" || "$left_instance" == "$right_instance" ]]; then
      printf "%s\n" "error: Camera PX4 instances must be distinct non-zero integers." >&2
      exit 2
    fi

    pids=()
    stop_instances() {
      local pid
      for pid in "${pids[@]}"; do
        kill -TERM "$pid" >/dev/null 2>&1 || true
      done
      wait "${pids[@]}" >/dev/null 2>&1 || true
    }
    trap stop_instances EXIT INT TERM

    start_px4_instance() {
      local instance="$1"
      local model_name="$2"
      local dds_namespace="$3"
      local label="$4"
      local sys_autostart="$5"
      local instance_log="$console_log"
      if [[ "$console_log" != "/dev/null" ]]; then
        instance_log="${console_log}.${label}"
      fi
      (
        export PX4_GZ_STANDALONE=1
        export PX4_GZ_WORLD="$world"
        export PX4_GZ_MODEL_NAME="$model_name"
        export PX4_GZ_NO_FOLLOW=1
        export PX4_SYS_AUTOSTART="$sys_autostart"
        export GZ_IP=127.0.0.1
        export PX4_UXRCE_DDS_PORT="$uxrce_dds_port"
        if [[ -n "$dds_namespace" ]]; then
          export PX4_UXRCE_DDS_NS="$dds_namespace"
        else
          unset PX4_UXRCE_DDS_NS || true
        fi
        cd "$gz_bridge_dir"
        exec "$px4_binary" -d -i "$instance"
      ) >"$instance_log" 2>&1 &
      pids+=("$!")
    }

    start_px4_instance 0 "$target_model" "" target 4001
    start_px4_instance "$left_instance" "$left_model" "$left_namespace" camera-left 4090
    start_px4_instance "$right_instance" "$right_model" "$right_namespace" camera-right 4090
    wait -n "${pids[@]}"
  ' _ \
    "$PX4_DIR" \
    "$PX4_GZ_WORLD" \
    "$PX4_GZ_MODEL_NAME" \
    "${CAM_L_PX4_MODEL_NAME:-left_stereo_camera}" \
    "${CAM_R_PX4_MODEL_NAME:-right_stereo_camera}" \
    "${CAM_L_PX4_INSTANCE:-1}" \
    "${CAM_R_PX4_INSTANCE:-2}" \
    "${CAM_L_PX4_DDS_NAMESPACE:-camera_left}" \
    "${CAM_R_PX4_DDS_NAMESPACE:-camera_right}" \
    "${DISPLAY:-:0}" \
    "$PX4_PYTHON_EXECUTABLE" \
    "$PX4_UXRCE_DDS_PORT" \
    "$PX4_CONSOLE_LOG" \
    "$ROOT_DIR/stereo_ground_sim/vendor/px4_camera_airframe/4090_camera_px4" \
    "$skip_px4_build" &
px4_supervisor_pid="$!"
children+=("$px4_supervisor_pid")
printf '%s\n' \
  "Target PX4 instance 0: model=$PX4_GZ_MODEL_NAME namespace=/fmu" \
  "Camera_Left PX4 instance ${CAM_L_PX4_INSTANCE:-1}: model=${CAM_L_PX4_MODEL_NAME:-left_stereo_camera} namespace=/${CAM_L_PX4_DDS_NAMESPACE:-camera_left}/fmu" \
  "Camera_Right PX4 instance ${CAM_R_PX4_INSTANCE:-2}: model=${CAM_R_PX4_MODEL_NAME:-right_stereo_camera} namespace=/${CAM_R_PX4_DDS_NAMESPACE:-camera_right}/fmu"
printf '%s\n' "QGC should connect through PX4 MAVLink UDP 14550; operator MAVLink ATTITUDE uses UDP 14551."
if [[ "$PX4_CONSOLE_LOG" != "/dev/null" ]]; then
  printf '%s\n' "PX4 console log: $PX4_CONSOLE_LOG"
fi

perspective_file="$ROOT_DIR/config/operator_rqt.perspective"
if [[ "$bag_view" == true ]]; then
  perspective_file="$ROOT_DIR/config/operator_rqt_bag.perspective"
elif [[ "$preflight_view" == true ]]; then
  perspective_file="$ROOT_DIR/config/operator_rqt_preflight.perspective"
fi

if [[ "$show_detection_images" == true ]]; then
  rqt_image_topic_mode="detection"
else
  rqt_image_topic_mode="raw"
fi

if [[ "$headless" != true ]]; then
  setsid env PYTHONPATH="$RQT_PYTHONPATH" RQT_IMAGE_TOPIC_MODE="$rqt_image_topic_mode" RQT_PLOT_WINDOW_S="$DEFAULT_PLOT_WINDOW_S" RQT_YOLO_MODEL_PATH="$rqt_yolo_model_path" RQT_MISSION_LABEL="$rqt_mission_label" RQT_COORD_CONTRACT="$rqt_coord_contract" OPERATOR_RECORDER_PYTHON="$OPERATOR_RECORDER_PYTHON" ros2 run rqt_gui rqt_gui \
    --clear-config \
    --force-discover \
    --perspective-file "$perspective_file" &
  children+=("$!")

  sleep 3
  rqt_pid="$(pgrep -n -f 'rqt_gui --clear-config' || true)"
  if [[ -n "$rqt_pid" ]]; then
    if [[ "$open_params" == true ]]; then
      for _ in {1..30}; do
        rqt_pid="$(pgrep -n -f '/lib/rqt_gui/rqt_gui --clear-config' || pgrep -n -f 'rqt_gui --clear-config' || true)"
        [[ -n "$rqt_pid" ]] || break
        env PYTHONPATH="$RQT_PYTHONPATH" RQT_IMAGE_TOPIC_MODE="$rqt_image_topic_mode" RQT_PLOT_WINDOW_S="$DEFAULT_PLOT_WINDOW_S" RQT_YOLO_MODEL_PATH="$rqt_yolo_model_path" RQT_MISSION_LABEL="$rqt_mission_label" RQT_COORD_CONTRACT="$rqt_coord_contract" OPERATOR_RECORDER_PYTHON="$OPERATOR_RECORDER_PYTHON" ros2 run rqt_gui rqt_gui \
          --command-pid "$rqt_pid" \
          --command-start-plugin rqt_reconfigure/Param \
          --args /stereo_distance_node >/dev/null 2>&1 && break
        sleep 1
      done
    fi
  fi
else
  printf '%s\n' "Headless mode: rqt_gui is disabled."
fi

# Gazebo GUI exits as part of the ROS launch tree, so closing it used to
# trigger the launcher's EXIT trap.  With gz_gui:=false there is no equivalent
# GUI process to take the tree down: closing RViz and rqt otherwise leaves the
# ROS launch, PX4 instances, bridge, and DDS agent running.  Treat the visible
# operator windows as the lifecycle owner in non-headless mode as well.
start_gui_watchdog() {
  local launcher_pid="$1"
  local expect_rviz="$2"
  local expect_rqt="$3"
  setsid bash -c '
    set +e
    launcher_pid="$1"
    expect_rviz="$2"
    expect_rqt="$3"
    sleep 30
    while kill -0 "$launcher_pid" >/dev/null 2>&1; do
      rviz_alive=0
      rqt_alive=0
      if [[ "$expect_rviz" == true ]] && pgrep -f "(^|/)rviz2( |$)" >/dev/null 2>&1; then
        rviz_alive=1
      fi
      if [[ "$expect_rqt" == true ]] && pgrep -f "(^|/)rqt_gui( |$)" >/dev/null 2>&1; then
        rqt_alive=1
      fi
      if [[ "$rviz_alive" -eq 0 && "$rqt_alive" -eq 0 ]]; then
        printf "%s\n" "Operator GUI windows closed; stopping the simulation stack." >&2
        kill -TERM "$launcher_pid" >/dev/null 2>&1 || true
        exit 0
      fi
      sleep 2
    done
  ' _ "$launcher_pid" "$expect_rviz" "$expect_rqt" &
  children+=("$!")
}

if [[ "$headless" != true ]]; then
  launch_rviz_value="$(get_launch_arg_value launch_rviz "${launch_args[@]}" || true)"
  if [[ -z "$launch_rviz_value" ]]; then
    launch_rviz_value=true
  fi
  start_gui_watchdog "$$" "$launch_rviz_value" true
fi

if [[ -n "$record_mode" ]]; then
  if [[ -z "$record_dir" ]]; then
    record_dir="$ROOT_DIR/bags/operator_$(date +%Y%m%d_%H%M%S)_$record_mode"
  fi
  mkdir -p "$record_dir"
  recorder_cmd=(
    "$OPERATOR_RECORDER_PYTHON" -m stereo_ground_sim.operator_recorder
    --mode "$record_mode"
    --output-dir "$record_dir"
  )
  if [[ "$record_wait_for_bbox" == true ]]; then
    recorder_cmd+=(--wait-for-bbox)
  fi
  setsid "${recorder_cmd[@]}" &
  recording_pid="$!"
  children+=("$recording_pid")
  printf '%s\n' "Recording $record_mode session to: $record_dir"
fi

if [[ "$run_mission" == true ]]; then
  setsid bash -c 'delay="$1"; root="$2"; shift 2; sleep "$delay"; exec "$root/run_mission.sh" "$@"' _ "$mission_delay_s" "$ROOT_DIR" "${mission_args[@]}" &
  children+=("$!")
  printf '%s\n' "PX4 mission will start after ${mission_delay_s}s."
fi

printf '%s\n' "Operator view started."
printf '%s\n' "PX4/QGC mode: QGC controls the existing Gazebo model $PX4_GZ_MODEL_NAME."
if [[ "$bag_view" == true ]]; then
  printf '%s\n' "The single rqt_gui window contains left/right camera views and rqt_bag."
elif [[ "$preflight_view" == true ]]; then
  printf '%s\n' "The single rqt_gui window contains preflight camera and topic-readiness views."
elif [[ "$headless" == true ]]; then
  printf '%s\n' "Headless mode: no rqt_gui window was started."
else
  printf '%s\n' "The single rqt_gui window contains Setup, Flight, Analysis, and Debug tabs."
fi
printf '%s\n' "Use QGC for normal control of the existing $PX4_GZ_MODEL_NAME target."
if [[ "$run_mission" == true ]]; then
  printf '%s\n' "Mission command: ./run_mission.sh ${mission_args[*]}"
fi

if wait -n "$ros_launch_pid" "$px4_supervisor_pid"; then
  child_status=0
else
  child_status=$?
fi

if ! kill -0 "$px4_supervisor_pid" >/dev/null 2>&1; then
  printf '%s\n' \
    "error: PX4 instance supervisor exited unexpectedly; operator view is stopping." >&2
  if [[ "$child_status" -eq 0 ]]; then
    exit 2
  fi
  exit "$child_status"
fi

exit "$child_status"
