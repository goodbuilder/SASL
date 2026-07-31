#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
INTEGRATION_ROOT="$(cd -- "$ROOT_DIR/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
POSE_SIMULATION_ROS_WORKSPACE="${POSE_SIMULATION_ROS_WORKSPACE:-$INTEGRATION_ROOT/workspace/main}"
ROS_SETUP="/opt/ros/$ROS_DISTRO_NAME/setup.bash"
ROS_WORKSPACE_SETUP="$POSE_SIMULATION_ROS_WORKSPACE/install/setup.bash"
LOCAL_SETUP="$ROOT_DIR/install/setup.bash"
build_local=true
launch_args=()

usage() {
  cat <<'USAGE'
Usage:
  ./run_pose_simulation.sh [--no-build] [ros2 launch args...]

Builds and starts the independent PoseSimulation diagnostic node.
This entrypoint does not start or modify the Simulation application.

Options:
  --no-build     Use the existing PoseSimulation install without rebuilding.
  --help, -h     Show this help.

Environment:
  POSE_SIMULATION_ROS_WORKSPACE
                 ROS 2 overlay containing px4_msgs. Default:
                 ../workspace/main
  POSE_SIMULATION_BUILD
                 Set to 0 or false to disable the local build.

Examples:
  ./run_pose_simulation.sh
  ./run_pose_simulation.sh --no-build evaluation_rate_hz:=5.0
  POSE_SIMULATION_ROS_WORKSPACE=/path/to/ros_ws ./run_pose_simulation.sh
USAGE
}

for arg in "$@"; do
  case "$arg" in
    --help|-h)
      usage
      exit 0
      ;;
    --no-build)
      build_local=false
      ;;
    --)
      shift
      launch_args+=("$@")
      break
      ;;
    *)
      launch_args+=("$arg")
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

printf '%s\n' "Starting PoseSimulation (diagnostic-only; no pose command path)."
exec ros2 launch pose_simulation pose_simulation.launch.py "${launch_args[@]}"
