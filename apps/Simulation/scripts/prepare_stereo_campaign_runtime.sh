#!/usr/bin/env bash
set -euo pipefail

SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CALLER_CWD="$(pwd)"
ROOT_DIR="$SIM_ROOT"
SIM_CONFIG="${SIM_CONFIG:-$SIM_ROOT/config/simulation.env}"
SIM_SCENARIO_CONFIG=""

# Re-read the operator-edited file at button time and let the shared loader
# export its pinned YOLO runtime versions. No source text is interpolated.
# shellcheck disable=SC1091
source "$SIM_ROOT/scripts/load_sim_config.sh"
load_sim_config_preserving_env

YOLO_PYTHON_SETTING="${STEREO_V1_1_YOLO_PYTHON:-.venv/bin/python}"
case "$YOLO_PYTHON_SETTING" in
  /*)
    printf '%s\n' "error: STEREO_V1_1_YOLO_PYTHON must be repo-relative." >&2
    exit 2
    ;;
esac
YOLO_PYTHON_PATH="$(cd "$SIM_ROOT" && realpath -m --no-symlinks "$YOLO_PYTHON_SETTING")"
case "$YOLO_PYTHON_PATH" in
  "$SIM_ROOT"/*) ;;
  *)
    printf '%s\n' "error: STEREO_V1_1_YOLO_PYTHON escapes apps/Simulation." >&2
    exit 2
    ;;
esac
YOLO_RUNTIME_DIR="$(dirname "$(dirname "$YOLO_PYTHON_PATH")")"
exec bash "$SIM_ROOT/scripts/ensure_yolo_runtime.sh" --isolated --venv-dir "$YOLO_RUNTIME_DIR"
