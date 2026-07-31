#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
INTEGRATION_ROOT="$(cd -- "$ROOT_DIR/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
POSE_SIMULATION_ROS_WORKSPACE="${POSE_SIMULATION_ROS_WORKSPACE:-$INTEGRATION_ROOT/workspace/main}"
ROS_SETUP="/opt/ros/$ROS_DISTRO_NAME/setup.bash"
ROS_WORKSPACE_SETUP="$POSE_SIMULATION_ROS_WORKSPACE/install/setup.bash"
LOCAL_SETUP="$ROOT_DIR/install/setup.bash"
RVIZ_CONFIG="${POSE_SIMULATION_RVIZ_CONFIG:-$ROOT_DIR/config/pose_simulation.rviz}"
GAZEBO_WORLD="${POSE_SIMULATION_GAZEBO_WORLD:-}"
build_local=true
start_rqt=true
start_rviz=true
start_gazebo_gui=true
launch_args=()
children=()

usage() {
  cat <<'USAGE'
Usage:
  ./run_pose_simulation.sh [options] [ros2 launch args...]

Builds and starts the independent PoseSimulation diagnostic node with the
standard observation GUIs. This entrypoint does not start or modify the
Simulation application.

Options:
  --no-build             Use the existing PoseSimulation install without rebuilding.
  --no-gui               Start only the PoseSimulation node.
  --node-only            Alias for --no-gui.
  --no-rqt                Do not start rqt_gui.
  --no-rviz               Do not start RViz.
  --no-gazebo-gui        Do not start the Gazebo GUI client.
  --rviz-config FILE     RViz config. Default: config/pose_simulation.rviz.
  --gazebo-world FILE    Optional world passed to the GUI-only Gazebo client.
  --help, -h             Show this help.

Environment:
  POSE_SIMULATION_ROS_WORKSPACE
                 ROS 2 overlay containing px4_msgs. Default:
                 ../workspace/main
  POSE_SIMULATION_BUILD
                 Set to 0 or false to disable the local build.
  POSE_SIMULATION_RVIZ_CONFIG
                 Override the RViz config path.
  POSE_SIMULATION_GAZEBO_WORLD
                 Optional Gazebo world path. By default only the GUI client
                 is opened and it connects to an already running server.

Examples:
  ./run_pose_simulation.sh
  ./run_pose_simulation.sh --no-build evaluation_rate_hz:=5.0
  ./run_pose_simulation.sh --no-gazebo-gui
  ./run_pose_simulation.sh --node-only
  POSE_SIMULATION_ROS_WORKSPACE=/path/to/ros_ws ./run_pose_simulation.sh
USAGE
}

while (($# > 0)); do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --no-build)
      build_local=false
      shift
      ;;
    --no-gui|--node-only)
      start_rqt=false
      start_rviz=false
      start_gazebo_gui=false
      shift
      ;;
    --no-rqt)
      start_rqt=false
      shift
      ;;
    --no-rviz)
      start_rviz=false
      shift
      ;;
    --no-gazebo-gui)
      start_gazebo_gui=false
      shift
      ;;
    --rviz-config)
      if (($# < 2)); then
        printf '%s\n' "error: --rviz-config requires a file path." >&2
        exit 2
      fi
      RVIZ_CONFIG="$2"
      shift 2
      ;;
    --gazebo-world)
      if (($# < 2)); then
        printf '%s\n' "error: --gazebo-world requires a file path." >&2
        exit 2
      fi
      GAZEBO_WORLD="$2"
      shift 2
      ;;
    --)
      shift
      launch_args+=("$@")
      break
      ;;
    *)
      launch_args+=("$1")
      shift
      ;;
  esac
done

case "${POSE_SIMULATION_BUILD:-1}" in
  0|false|False|FALSE|no|No|NO)
    build_local=false
    ;;
esac

cd "$ROOT_DIR"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/pose-simulation-ros-log}"

if [[ -f "$ROS_SETUP" ]]; then
  # shellcheck disable=SC1091
  set +u
  source "$ROS_SETUP"
  set -u
elif ! command -v ros2 >/dev/null 2>&1; then
  printf '%s\n' \
    "error: ROS 2 was not found." \
    "Expected setup file: $ROS_SETUP" >&2
  exit 2
fi

if [[ -f "$ROS_WORKSPACE_SETUP" ]]; then
  # shellcheck disable=SC1091
  set +u
  source "$ROS_WORKSPACE_SETUP"
  set -u
fi

if [[ "$build_local" == true ]]; then
  if ! command -v colcon >/dev/null 2>&1; then
    printf '%s\n' \
      "error: colcon is required to build PoseSimulation." \
      "Install ROS 2 colcon tools or run with --no-build after building the package." >&2
    exit 2
  fi
  printf '%s\n' "Building PoseSimulation from $ROOT_DIR"
  colcon build \
    --base-paths "$ROOT_DIR" \
    --packages-select pose_simulation
fi

if [[ ! -f "$LOCAL_SETUP" ]]; then
  printf '%s\n' \
    "error: PoseSimulation is not installed: $LOCAL_SETUP" \
    "Run without --no-build, or build this package with colcon first." >&2
  exit 2
fi

# shellcheck disable=SC1091
set +u
source "$LOCAL_SETUP"
set -u

if ! ros2 pkg prefix pose_simulation >/dev/null 2>&1; then
  printf '%s\n' \
    "error: the pose_simulation ROS 2 package is unavailable after setup." \
    "Check the ROS overlay at: $POSE_SIMULATION_ROS_WORKSPACE" >&2
  exit 2
fi

if [[ "$start_rqt" == true ]] && ! ros2 pkg prefix rqt_gui >/dev/null 2>&1; then
  printf '%s\n' \
    "error: rqt_gui is not installed in the sourced ROS 2 environment." \
    "Install the ROS 2 rqt GUI package or use --no-rqt/--no-gui." >&2
  exit 2
fi

if [[ "$start_rviz" == true ]] && ! command -v rviz2 >/dev/null 2>&1; then
  printf '%s\n' \
    "error: rviz2 was not found in the sourced ROS 2 environment." \
    "Install RViz or use --no-rviz/--no-gui." >&2
  exit 2
fi

if [[ "$start_gazebo_gui" == true ]] && ! command -v gz >/dev/null 2>&1; then
  printf '%s\n' \
    "error: gz was not found in PATH." \
    "Install Gazebo Sim or use --no-gazebo-gui/--no-gui." >&2
  exit 2
fi

if [[ "$start_rviz" == true && ! -f "$RVIZ_CONFIG" ]]; then
  printf '%s\n' \
    "error: RViz config was not found: $RVIZ_CONFIG" \
    "Use --rviz-config FILE or set POSE_SIMULATION_RVIZ_CONFIG." >&2
  exit 2
fi

if [[ -n "$GAZEBO_WORLD" && ! -f "$GAZEBO_WORLD" ]]; then
  printf '%s\n' "error: Gazebo world was not found: $GAZEBO_WORLD" >&2
  exit 2
fi

cleanup() {
  local pid
  trap - EXIT INT TERM
  for pid in "${children[@]}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill -TERM "-$pid" >/dev/null 2>&1 || kill -TERM "$pid" >/dev/null 2>&1 || true
    fi
  done
  sleep 1
  for pid in "${children[@]}"; do
    kill -KILL "-$pid" >/dev/null 2>&1 || kill -KILL "$pid" >/dev/null 2>&1 || true
  done
}

trap cleanup EXIT INT TERM

start_child() {
  local label="$1"
  shift
  printf '%s\n' "Starting $label: $*"
  setsid "$@" &
  children+=("$!")
}

printf '%s\n' "Starting PoseSimulation (diagnostic-only; no pose command path)."
setsid ros2 launch pose_simulation pose_simulation.launch.py "${launch_args[@]}" &
children+=("$!")

if [[ "$start_rqt" == true ]]; then
  start_child "rqt_gui" ros2 run rqt_gui rqt_gui --clear-config --force-discover
fi

if [[ "$start_rviz" == true ]]; then
  start_child "RViz" rviz2 -d "$RVIZ_CONFIG"
fi

if [[ "$start_gazebo_gui" == true ]]; then
  if [[ -n "$GAZEBO_WORLD" ]]; then
    start_child "Gazebo GUI client" gz sim -g "$GAZEBO_WORLD"
  else
    start_child "Gazebo GUI client" gz sim -g
  fi
fi

if [[ "$start_rqt" == true || "$start_rviz" == true || "$start_gazebo_gui" == true ]]; then
  printf '%s\n' "PoseSimulation and requested observation GUIs are running."
else
  printf '%s\n' "PoseSimulation is running without observation GUIs."
fi
if [[ "$start_gazebo_gui" == true ]]; then
  printf '%s\n' "Gazebo GUI is client-only; no Gazebo server or world is started by this app."
fi
wait "${children[0]}"
