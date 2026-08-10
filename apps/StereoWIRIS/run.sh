#!/usr/bin/env bash
set -eo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SASL_ROOT="$(cd "$APP_DIR/../.." && pwd)"
SASL_ROS_WORKSPACE="${SASL_ROS_WORKSPACE:-$SASL_ROOT/workspace/main}"
PX4_MSGS_SOURCE_DIR="$SASL_ROS_WORKSPACE/src/px4_msgs"
PX4_MSGS_OVERLAY_SETUP="$SASL_ROS_WORKSPACE/install/setup.bash"
PX4_MSGS_OVERLAY_MANIFEST="$SASL_ROS_WORKSPACE/install/share/sasl_ros_workspace/px4_msgs-overlay.env"
PX4_MSGS_REF="${PX4_MSGS_REF:-86d8239e962f6939e05c3737784f60c02fa884db}"
export PATH="$HOME/.local/bin:$PATH"
export LD_LIBRARY_PATH="$HOME/.local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-$APP_DIR/.cache/matplotlib}"
mkdir -p "$MPLCONFIGDIR"

workstation_ui_requested() {
  local previous=""
  local argument
  for argument in "$@"; do
    if [[ "$argument" == "--ui=workstation" ]]; then
      return 0
    fi
    if [[ "$previous" == "--ui" && "$argument" == "workstation" ]]; then
      return 0
    fi
    previous="$argument"
  done
  return 1
}

mavlink_only_requested() {
  local override mode argument

  # The launcher may be used from CI or a field script where the persisted
  # settings file is intentionally not modified.  An explicit runtime flag
  # therefore takes precedence over the saved mode.
  for argument in "$@"; do
    if [[ "$argument" == "--mavlink-only" ]]; then
      return 0
    fi
  done

  override="${SASL_MAVLINK_ONLY:-}"
  if [[ "$override" == "1" ]]; then
    return 0
  fi
  if [[ "$override" == "0" ]]; then
    return 1
  fi

  mode="${SASL_PIXHAWK_ETHERNET_MODE:-}"
  if [[ -n "$mode" ]]; then
    [[ "$mode" == "mavlink" ]]
    return
  fi

  # Legacy persists its mode in this repository-local JSON file.  Keep the
  # check dependency-free so the decision can be made before any ROS setup is
  # sourced.  The application validates the JSON again in Python.
  if [[ -f "$APP_DIR/stereo_distance_settings.json" ]] && \
    grep -Eq '"pixhawk_ethernet_mode"[[:space:]]*:[[:space:]]*"mavlink"' \
      "$APP_DIR/stereo_distance_settings.json"; then
    return 0
  fi

  return 1
}

WORKSTATION_UI_REQUESTED=0
if workstation_ui_requested "$@"; then
  WORKSTATION_UI_REQUESTED=1
fi
MAVLINK_ONLY_REQUESTED=0
if [[ "$WORKSTATION_UI_REQUESTED" != "1" ]] && mavlink_only_requested "$@"; then
  MAVLINK_ONLY_REQUESTED=1
  export SASL_MAVLINK_ONLY=1
fi
CHECK_REQUESTED=0
for argument in "$@"; do
  if [[ "$argument" == "--check" ]]; then
    CHECK_REQUESTED=1
    break
  fi
done

usage() {
  cat <<'USAGE'
Usage:
  apps/StereoWIRIS/run.sh [--check] [--mavlink-only] [--ui legacy|workstation] [app args...]

Starts the StereoWIRIS GUI application.

Options:
  --check   Verify Python and import paths without starting the GUI.
  --mavlink-only  Start Legacy without ROS2/uXRCE-DDS dependencies.
  --ui      Select the single operational UI for this process; default is legacy.
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
      "Run: cd $SASL_ROOT && tools/bootstrap_ros_workspace.sh" >&2
    return 2
  fi
  expected_commit="$(git -C "$PX4_MSGS_SOURCE_DIR" rev-parse "${PX4_MSGS_REF}^{commit}" 2>/dev/null || true)"
  actual_commit="$(git -C "$PX4_MSGS_SOURCE_DIR" rev-parse HEAD 2>/dev/null || true)"
  if [[ -z "$expected_commit" || "$actual_commit" != "$expected_commit" ]]; then
    printf '%s\n' \
      "error: canonical px4_msgs revision mismatch." \
      "required: $PX4_MSGS_REF" \
      "actual: ${actual_commit:-unavailable}" \
      "Repair with: cd $SASL_ROOT && tools/bootstrap_ros_workspace.sh" >&2
    return 2
  fi
  if [[ ! -f "$PX4_MSGS_OVERLAY_SETUP" || ! -f "$PX4_MSGS_OVERLAY_MANIFEST" ]]; then
    printf '%s\n' \
      "error: canonical px4_msgs overlay is incomplete: $SASL_ROS_WORKSPACE/install" \
      "Run: cd $SASL_ROOT && tools/bootstrap_ros_workspace.sh" >&2
    return 2
  fi
  overlay_commit="$(sed -n 's/^PX4_MSGS_COMMIT=//p' "$PX4_MSGS_OVERLAY_MANIFEST" | head -n 1)"
  if [[ "$overlay_commit" != "$actual_commit" ]]; then
    printf '%s\n' \
      "error: canonical px4_msgs overlay is stale." \
      "source: $actual_commit" \
      "overlay: ${overlay_commit:-unavailable}" \
      "Rebuild with: cd $SASL_ROOT && tools/bootstrap_ros_workspace.sh" >&2
    return 2
  fi
}

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
  PYTHON_BIN="$(resolve_python "$APP_DIR/.venv/bin/python" "$SASL_ROOT/apps/Simulation/.venv/bin/python" "$SASL_ROOT/.local/legacy/Simulation/.venv/bin/python" python3 python)"
fi

if [[ "$WORKSTATION_UI_REQUESTED" != "1" && "$MAVLINK_ONLY_REQUESTED" != "1" ]]; then
  if ! require_canonical_px4_msgs_overlay; then
    exit 2
  fi

  # ROS setup scripts may reference unset variables, so do not enable nounset here.
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1090
  source "$PX4_MSGS_OVERLAY_SETUP"
fi

if [[ "$CHECK_REQUESTED" == "1" ]]; then
    export PYTHONPATH="$APP_DIR/compat:$APP_DIR:$SASL_ROOT/apps:$SASL_ROOT/common/python:$SASL_ROOT${PYTHONPATH:+:$PYTHONPATH}"
    if [[ "$WORKSTATION_UI_REQUESTED" == "1" ]]; then
      "$PYTHON_BIN" - <<'PY'
from pathlib import Path
import sys

import cv2
import numpy as np
import pyarrow
import xlsxwriter
import yaml

import sasl_common.lifecycle
import Workstation.standalone_gui

print("StereoWIRIS Workstation runtime check ok")
print(f"python={sys.executable}")
print(f"sasl_root={Path.cwd().resolve()}")
print(
    f"opencv={cv2.__version__}, numpy={np.__version__}, yaml={yaml.__version__}, "
    f"xlsxwriter={xlsxwriter.__version__}, pyarrow={pyarrow.__version__}"
)
PY
    else
      "$PYTHON_BIN" - <<'PY'
from pathlib import Path
import sys

import cv2
import numpy as np
import pyarrow
import xlsxwriter
import yaml

import sasl_common.stereo_geometry
import sasl_common.lifecycle
import wiris_stereo_distance.gui
import wiris_stereo_distance.legacy_gui
import Workstation.standalone_gui

print("StereoWIRIS runtime check ok")
print(f"python={sys.executable}")
print(f"sasl_root={Path.cwd().resolve()}")
print(
    f"opencv={cv2.__version__}, numpy={np.__version__}, yaml={yaml.__version__}, "
    f"xlsxwriter={xlsxwriter.__version__}, pyarrow={pyarrow.__version__}"
)
PY
    fi
    exit 0
fi

bash "$APP_DIR/scripts/bootstrap_network_tools.sh" || true

if [[ "${STEREO_BOOTSTRAP_CALIBRATION_DEPS:-0}" == "1" ]]; then
  PYTHON_BIN="$PYTHON_BIN" bash "$APP_DIR/scripts/bootstrap_calibration_deps.sh"
fi

cd "$APP_DIR"
export PYTHONPATH="$APP_DIR/compat:$APP_DIR:$SASL_ROOT/apps:$SASL_ROOT/common/python:$SASL_ROOT${PYTHONPATH:+:$PYTHONPATH}"
exec "$PYTHON_BIN" "$APP_DIR/main.py" "$@"
