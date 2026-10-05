#!/usr/bin/env bash
set -euo pipefail

# This entrypoint validates the independent configuration and, by default,
# starts the read-only PoseSimulation ROS node together with its visualization,
# rqt operator view, and RViz display.  It starts the independent Simulation
# runner as a headless child; the two applications still keep their config
# environments and ROS interfaces separate.
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SASL_ROOT="$(cd "$ROOT_DIR/../.." && pwd)"
config_path="${POSESIM_CONFIG:-$ROOT_DIR/config/posesimulation.env}"
profile_path="${POSESIM_PROFILE_CONFIG:-}"
# Runner-only control; do not expose it to the POSESIM_* configuration schema.
ros_setup="${POSESIM_ROS_SETUP:-$ROOT_DIR/install/setup.bash}"
unset POSESIM_ROS_SETUP
simulation_runner="$SASL_ROOT/apps/Simulation/run_operator_view.sh"
simulation_log="/tmp/posesimulation-simulation.log"
run_state_file="/tmp/posesimulation-run.pid"
simulation_pid=""
pose_launch_pid=""
mode="run"
timeout_s="5"
cli_overrides=()

usage() {
  cat <<'USAGE'
Usage:
  ./run_pose_simulation.sh [--run | --check-config | --print-config | --check-inputs] [options]

Validates the independent PoseSimulation configuration contract and, by
default, starts PoseSimulation with its read-only visualization, rqt operator
view, and RViz display. The --check-inputs mode subscribes to external ROS 2
source topics without starting any node.

Options:
  --run                   Validate, then launch PoseSimulation (default).
  --check-config          Validate the resolved configuration and exit.
  --print-config          Print the deterministic resolved configuration.
  --check-inputs          Probe GT and left/right camera input contracts.
  --timeout-s SECONDS     Input-probe wait timeout (default: 5).
  --config FILE           Use FILE as the PoseSimulation base env file.
  --profile FILE          Apply an optional profile env file.
  --set KEY=VALUE         Apply a highest-precedence POSESIM_* override.
                          May be repeated.
  --help, -h              Show this help.

Environment:
  POSESIM_CONFIG           Base env path when --config is not supplied.
  POSESIM_PROFILE_CONFIG   Optional profile path when --profile is absent.
  POSESIM_ROS_SETUP        Optional ROS workspace setup.bash for runtime.
  POSESIM_*                Explicit configuration overrides.

The default run starts the repository's Simulation runner as a child with
Gazebo GUI, Simulation RViz, and Simulation rqt disabled. PoseSimulation's
own read-only rqt/RViz UI remains enabled.

Examples:
  ./run_pose_simulation.sh --check-config
  ./run_pose_simulation.sh --print-config
  POSESIM_EVALUATION_HZ=10 ./run_pose_simulation.sh --check-config
  ./run_pose_simulation.sh --check-config --set POSESIM_EVALUATION_HZ=2
USAGE
}

while (($# > 0)); do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --run)
      mode="run"
      shift
      ;;
    --check-config)
      mode="check"
      shift
      ;;
    --print-config)
      mode="print"
      shift
      ;;
    --check-inputs)
      mode="inputs"
      shift
      ;;
    --timeout-s)
      if (($# < 2)); then
        printf '%s\n' "error: --timeout-s requires a number of seconds." >&2
        exit 2
      fi
      timeout_s="$2"
      shift 2
      ;;
    --config)
      if (($# < 2)); then
        printf '%s\n' "error: --config requires a file path." >&2
        exit 2
      fi
      config_path="$2"
      shift 2
      ;;
    --profile)
      if (($# < 2)); then
        printf '%s\n' "error: --profile requires a file path." >&2
        exit 2
      fi
      profile_path="$2"
      shift 2
      ;;
    --set)
      if (($# < 2)); then
        printf '%s\n' "error: --set requires POSESIM_KEY=VALUE." >&2
        exit 2
      fi
      cli_overrides+=("$2")
      shift 2
      ;;
    *)
      printf '%s\n' "error: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

# Keep the caller's working directory unchanged.  The Python loader resolves
# relative profile paths using that directory, then its documented fallbacks.
export PYTHONPATH="$ROOT_DIR:$SASL_ROOT/apps:$SASL_ROOT/common/python${PYTHONPATH:+:$PYTHONPATH}"

command=(python3 -m pose_simulation.config --config "$config_path")
# Always pass the selected profile value so an explicit empty --profile can
# disable an inherited POSESIM_PROFILE_CONFIG without ambiguity.
command+=(--profile "$profile_path")
for override in "${cli_overrides[@]}"; do
  command+=(--set "$override")
done
if [[ "$mode" == "print" ]]; then
  command+=(--print-config)
else
  command+=(--check-config)
fi

"${command[@]}"

if [[ "$mode" == "inputs" ]]; then
  probe=(python3 -m pose_simulation.input_probe_node --config "$config_path")
  probe+=(--profile "$profile_path" --timeout-s "$timeout_s")
  for override in "${cli_overrides[@]}"; do
    probe+=(--set "$override")
  done
  "${probe[@]}"
elif [[ "$mode" == "check" ]]; then
  printf '%s\n' \
    "PoseSimulation input runtime is separate from Simulation;" \
    "the configuration check passed and no source process was started."
elif [[ "$mode" == "print" ]]; then
  :
else
  # Keep ROS setup local to this child process.  The Simulation environment is
  # intentionally never sourced here; its controls are rejected by the
  # PoseSimulation config loader before this branch is reached.
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    set -u
  fi

  if [[ -f "$ros_setup" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "$ros_setup"
    set -u
  fi

  if ! command -v ros2 >/dev/null 2>&1; then
    printf '%s\n' \
      "error: ros2 was not found after configuration validation." \
      "Source /opt/ros/humble/setup.bash or set POSESIM_ROS_SETUP to a ROS workspace setup.bash." >&2
    exit 2
  fi
  for ros_package in pose_simulation pose_simulation_interfaces simulation_interfaces; do
    if ! ros2 pkg prefix "$ros_package" >/dev/null 2>&1; then
      printf '%s\n' \
        "error: required ROS package is not installed: $ros_package." \
        "Build PoseSimulation and its typed interfaces, then source the workspace setup.bash" \
        "or set POSESIM_ROS_SETUP." >&2
      exit 2
    fi
  done

  # Pass the already validated configuration layers to the child ROS node.
  # This does not pass or source any Simulation-owned setting.
  export POSESIM_CONFIG="$config_path"
  export POSESIM_PROFILE_CONFIG="$profile_path"
  for override in "${cli_overrides[@]}"; do
    export "$override"
  done

  if [[ ! -x "$simulation_runner" ]]; then
    printf '%s\n' \
      "error: Simulation runner was not found: $simulation_runner" \
      "Expected apps/Simulation/run_operator_view.sh next to PoseSimulation." >&2
    exit 2
  fi

  stop_previous_pose_simulation() {
    local current_pid="$$"
    local current_pgid
    local owner_pid=""
    local previous_simulation_pid=""
    local process_args
    local pid
    local pgid
    local _attempt
    current_pgid="$(ps -o pgid= -p "$current_pid" | tr -d ' ')"

    # Prefer the state file for a previous wrapper that was killed abruptly.
    # Validate every PID's command line before sending a signal because PIDs
    # can be reused after an unclean shutdown.
    if [[ -f "$run_state_file" ]]; then
      read -r owner_pid previous_simulation_pid <"$run_state_file" || true
      if [[ "$owner_pid" =~ ^[0-9]+$ && "$owner_pid" != "$current_pid" ]]; then
        process_args="$(ps -p "$owner_pid" -o args= 2>/dev/null || true)"
        if [[ "$process_args" == *"$ROOT_DIR/run_pose_simulation.sh"* ]]; then
          kill -TERM "$owner_pid" >/dev/null 2>&1 || true
        elif [[ "$previous_simulation_pid" =~ ^[0-9]+$ ]]; then
          process_args="$(ps -p "$previous_simulation_pid" -o args= 2>/dev/null || true)"
          if [[ "$process_args" == *"$simulation_runner"* ]]; then
            kill -TERM -- "-$previous_simulation_pid" >/dev/null 2>&1 || kill -TERM "$previous_simulation_pid" >/dev/null 2>&1 || true
          fi
        fi
      fi
    fi

    # Also find older wrappers that predate the state file or whose state was
    # lost.  The awk guard excludes this invocation and its own search command.
    while read -r pid; do
      [[ "$pid" =~ ^[0-9]+$ && "$pid" != "$current_pid" ]] || continue
      kill -TERM "$pid" >/dev/null 2>&1 || true
    done < <(
      ps -eo pid=,args= | awk -v script="$ROOT_DIR/run_pose_simulation.sh" -v current="$current_pid" \
        'index($0, script) > 0 && $1 != current && $0 !~ /awk -v script/ {print $1}'
    )

    for _attempt in {1..30}; do
      local previous_wrapper_alive=false
      while read -r pid; do
        if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" >/dev/null 2>&1; then
          previous_wrapper_alive=true
          break
        fi
      done < <(
        ps -eo pid=,args= | awk -v script="$ROOT_DIR/run_pose_simulation.sh" -v current="$current_pid" \
          'index($0, script) > 0 && $1 != current && $0 !~ /awk -v script/ {print $1}'
      )
      [[ "$previous_wrapper_alive" == false ]] && break
      sleep 0.1
    done

    # A wrapper can die after launching its UI group.  Remove only stale
    # PoseSimulation UI groups; unrelated RViz/rqt processes remain untouched.
    while read -r pgid; do
      [[ "$pgid" =~ ^[0-9]+$ && "$pgid" != "$current_pgid" && "$pgid" != "0" ]] || continue
      kill -TERM -- "-$pgid" >/dev/null 2>&1 || true
    done < <(
      ps -eo pid=,pgid=,args= | awk -v root="$ROOT_DIR" -v current="$current_pgid" '
        $2 != current &&
        (index($0, root "/install/pose_simulation/lib/pose_simulation/pose_visualization") > 0 ||
         index($0, "pose_simulation_rqt.pose_simulation_plugin.PoseSimulationPlugin") > 0 ||
         index($0, root "/install/pose_simulation/share/pose_simulation/config/pose_simulation.rviz") > 0) {
          print $2
        }
      ' | sort -u
    )
  }

  stop_previous_pose_simulation
  printf '%s\n' "$$" >"$run_state_file"

  cleanup_children() {
    local _attempt
    if [[ -n "$pose_launch_pid" ]] && kill -0 "$pose_launch_pid" >/dev/null 2>&1; then
      # ros2 launch keeps rqt_gui and rviz2 in the same process group.  Signal
      # that group first so the UI grandchildren do not survive the wrapper.
      kill -INT -- "-$pose_launch_pid" >/dev/null 2>&1 || kill -INT "$pose_launch_pid" >/dev/null 2>&1 || true
      for _attempt in {1..50}; do
        kill -0 "$pose_launch_pid" >/dev/null 2>&1 || break
        sleep 0.1
      done
      kill -TERM -- "-$pose_launch_pid" >/dev/null 2>&1 || kill -TERM "$pose_launch_pid" >/dev/null 2>&1 || true
      kill -KILL -- "-$pose_launch_pid" >/dev/null 2>&1 || kill -KILL "$pose_launch_pid" >/dev/null 2>&1 || true
    fi
    wait "$pose_launch_pid" >/dev/null 2>&1 || true
    if [[ -n "$simulation_pid" ]] && kill -0 "$simulation_pid" >/dev/null 2>&1; then
      kill -TERM "$simulation_pid" >/dev/null 2>&1 || true
      for _attempt in {1..50}; do
        kill -0 "$simulation_pid" >/dev/null 2>&1 || break
        sleep 0.1
      done
      kill -KILL "$simulation_pid" >/dev/null 2>&1 || true
    fi
    wait "$simulation_pid" >/dev/null 2>&1 || true
    if [[ -f "$run_state_file" ]] && grep -q "^$$\b" "$run_state_file"; then
      rm -f "$run_state_file"
    fi
  }
  trap 'exit 130' INT
  trap 'exit 143' TERM
  trap cleanup_children EXIT

  : >"$simulation_log"
  # Keep YOLO and the detection overlay enabled.  --headless only disables
  # Gazebo/rqt GUI; --no-yolo would set overlay_publish_hz to zero and leave
  # the Camera tab's detection_image topics without publishers.
  setsid "$simulation_runner" --headless >"$simulation_log" 2>&1 &
  simulation_pid="$!"
  printf '%s %s\n' "$$" "$simulation_pid" >"$run_state_file"
  sleep 1
  if ! kill -0 "$simulation_pid" >/dev/null 2>&1; then
    printf '%s\n' \
      "error: headless apps/Simulation/Gazebo/PX4 startup failed." \
      "Inspect: $simulation_log" >&2
    exit 2
  fi

  printf '%s\n' \
    "Headless apps/Simulation/Gazebo/PX4 started (log: $simulation_log)." \
    "Starting PoseSimulation read-only node, visualization, rqt, and RViz."
  ros2 launch pose_simulation pose_simulation_ui.launch.py &
  pose_launch_pid="$!"
  set +e
  wait "$pose_launch_pid"
  pose_launch_status="$?"
  set -e
  exit "$pose_launch_status"
fi
