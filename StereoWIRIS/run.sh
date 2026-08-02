#!/usr/bin/env bash
set -eo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SASL_ROOT="$(cd "$APP_DIR/.." && pwd)"
SASL_ROS_WORKSPACE="${SASL_ROS_WORKSPACE:-$SASL_ROOT/workspace/main}"
PX4_MSGS_SOURCE_DIR="$SASL_ROS_WORKSPACE/src/px4_msgs"
PX4_MSGS_OVERLAY_SETUP="$SASL_ROS_WORKSPACE/install/setup.bash"
PX4_MSGS_OVERLAY_MANIFEST="$SASL_ROS_WORKSPACE/install/share/sasl_ros_workspace/px4_msgs-overlay.env"
PX4_MSGS_REF="${PX4_MSGS_REF:-86d8239e962f6939e05c3737784f60c02fa884db}"
export PATH="$HOME/.local/bin:$PATH"
export LD_LIBRARY_PATH="$HOME/.local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-$APP_DIR/.cache/matplotlib}"
mkdir -p "$MPLCONFIGDIR"

usage() {
  cat <<'USAGE'
Usage:
  StereoWIRIS/run.sh [--check] [app args...]

Starts the StereoWIRIS GUI application.

Options:
  --check   Verify Python and import paths without starting the GUI.
  --help    Show this help.
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
      "Run: cd $SASL_ROOT && scripts/bootstrap_ros_workspace.sh" >&2
    return 2
  fi
  expected_commit="$(git -C "$PX4_MSGS_SOURCE_DIR" rev-parse "${PX4_MSGS_REF}^{commit}" 2>/dev/null || true)"
  actual_commit="$(git -C "$PX4_MSGS_SOURCE_DIR" rev-parse HEAD 2>/dev/null || true)"
  if [[ -z "$expected_commit" || "$actual_commit" != "$expected_commit" ]]; then
    printf '%s\n' \
      "error: canonical px4_msgs revision mismatch." \
      "required: $PX4_MSGS_REF" \
      "actual: ${actual_commit:-unavailable}" \
      "Repair with: cd $SASL_ROOT && scripts/bootstrap_ros_workspace.sh" >&2
    return 2
  fi
  if [[ ! -f "$PX4_MSGS_OVERLAY_SETUP" || ! -f "$PX4_MSGS_OVERLAY_MANIFEST" ]]; then
    printf '%s\n' \
      "error: canonical px4_msgs overlay is incomplete: $SASL_ROS_WORKSPACE/install" \
      "Run: cd $SASL_ROOT && scripts/bootstrap_ros_workspace.sh" >&2
    return 2
  fi
  overlay_commit="$(sed -n 's/^PX4_MSGS_COMMIT=//p' "$PX4_MSGS_OVERLAY_MANIFEST" | head -n 1)"
  if [[ "$overlay_commit" != "$actual_commit" ]]; then
    printf '%s\n' \
      "error: canonical px4_msgs overlay is stale." \
      "source: $actual_commit" \
      "overlay: ${overlay_commit:-unavailable}" \
      "Rebuild with: cd $SASL_ROOT && scripts/bootstrap_ros_workspace.sh" >&2
    return 2
  fi
}

if ! require_canonical_px4_msgs_overlay; then
  exit 2
fi

# ROS setup scripts may reference unset variables, so do not enable nounset here.
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source "$PX4_MSGS_OVERLAY_SETUP"

resolve_python() {
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

PYTHON_BIN="${PYTHON_BIN:-}"
if [[ -z "$PYTHON_BIN" ]]; then
  PYTHON_BIN="$(resolve_python "$APP_DIR/.venv/bin/python" "$SASL_ROOT/Simulation/.venv/bin/python" python3 python)"
fi

case "${1:-}" in
  --check)
    export PYTHONPATH="$APP_DIR/compat:$APP_DIR:$SASL_ROOT${PYTHONPATH:+:$PYTHONPATH}"
    "$PYTHON_BIN" - <<'PY'
from pathlib import Path
import sys

import cv2
import numpy as np
import pyarrow
import xlsxwriter
import yaml

import sasl_common.stereo_geometry
import wiris_stereo_distance.gui

print("StereoWIRIS runtime check ok")
print(f"python={sys.executable}")
print(f"sasl_root={Path.cwd().resolve()}")
print(
    f"opencv={cv2.__version__}, numpy={np.__version__}, yaml={yaml.__version__}, "
    f"xlsxwriter={xlsxwriter.__version__}, pyarrow={pyarrow.__version__}"
)
PY
    exit 0
    ;;
esac

bash "$APP_DIR/scripts/bootstrap_network_tools.sh" || true

if [[ "${STEREO_BOOTSTRAP_CALIBRATION_DEPS:-0}" == "1" ]]; then
  PYTHON_BIN="$PYTHON_BIN" bash "$APP_DIR/scripts/bootstrap_calibration_deps.sh"
fi

cd "$APP_DIR"
export PYTHONPATH="$APP_DIR/compat:$APP_DIR:$SASL_ROOT${PYTHONPATH:+:$PYTHONPATH}"
exec "$PYTHON_BIN" "$APP_DIR/main.py" "$@"
