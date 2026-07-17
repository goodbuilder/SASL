#!/usr/bin/env bash
set -eo pipefail

# ROS setup scripts may reference unset variables, so do not enable nounset here.
source /opt/ros/humble/setup.bash

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SASL_ROOT="$(cd "$APP_DIR/.." && pwd)"
export PATH="$HOME/.local/bin:$PATH"
export LD_LIBRARY_PATH="$HOME/.local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
PX4_WORKSPACE_SETUP="${PX4_WORKSPACE_SETUP:-}"
if [[ -z "$PX4_WORKSPACE_SETUP" ]]; then
  for candidate in \
    "$SASL_ROOT/workspace/main/install/setup.bash" \
    "$SASL_ROOT/px4_msgs_ws/install/setup.bash" \
    "$HOME/px4_msgs_ws/install/setup.bash"; do
    if [[ -f "$candidate" ]]; then
      PX4_WORKSPACE_SETUP="$candidate"
      break
    fi
  done
fi
if [[ -f "$PX4_WORKSPACE_SETUP" ]]; then
  # shellcheck disable=SC1090
  source "$PX4_WORKSPACE_SETUP"
fi

usage() {
  cat <<'USAGE'
Usage:
  StereoDistance/run.sh [--check] [app args...]

Starts the StereoDistance GUI application.

Options:
  --check   Verify Python and import paths without starting the GUI.
  --help    Show this help.
USAGE
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
  PYTHON_BIN="$(resolve_python "$APP_DIR/.venv/bin/python" "$SASL_ROOT/Simulation/.venv/bin/python" python3 python)"
fi

case "${1:-}" in
  --help|-h)
    usage
    exit 0
    ;;
  --check)
    export PYTHONPATH="$APP_DIR:$SASL_ROOT${PYTHONPATH:+:$PYTHONPATH}"
    "$PYTHON_BIN" - <<'PY'
from pathlib import Path
import sys

import sasl_common.truth_coordinates
import wiris_stereo_distance.gui

print("StereoDistance runtime check ok")
print(f"python={sys.executable}")
print(f"sasl_root={Path.cwd().resolve()}")
PY
    exit 0
    ;;
esac

bash "$APP_DIR/scripts/bootstrap_network_tools.sh" || true

if [[ "${STEREO_BOOTSTRAP_CALIBRATION_DEPS:-0}" == "1" ]]; then
  PYTHON_BIN="$PYTHON_BIN" bash "$APP_DIR/scripts/bootstrap_calibration_deps.sh"
fi

cd "$APP_DIR"
export PYTHONPATH="$APP_DIR:$SASL_ROOT${PYTHONPATH:+:$PYTHONPATH}"
exec "$PYTHON_BIN" "$APP_DIR/main.py" "$@"
