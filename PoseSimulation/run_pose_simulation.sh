#!/usr/bin/env bash
set -euo pipefail

# This entrypoint validates the independent configuration and, when asked,
# runs the read-only external-input probe.  It never sources, starts, stops,
# or includes the independent Simulation application.
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
config_path="${POSESIM_CONFIG:-$ROOT_DIR/config/posesimulation.env}"
profile_path="${POSESIM_PROFILE_CONFIG:-}"
mode="check"
timeout_s="5"
cli_overrides=()

usage() {
  cat <<'USAGE'
Usage:
  ./run_pose_simulation.sh [--check-config | --print-config | --check-inputs] [options]

Validates the independent PoseSimulation configuration contract.  The
--check-inputs mode additionally subscribes to external ROS 2 source topics;
it does not start ROS, Gazebo, PX4, rqt, RViz, or any source application.

Options:
  --check-config          Validate the resolved configuration (default).
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
  POSESIM_*                Explicit configuration overrides.

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
export PYTHONPATH="$ROOT_DIR${PYTHONPATH:+:$PYTHONPATH}"

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
fi
