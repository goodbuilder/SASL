#!/usr/bin/env bash
set -eo pipefail

# ROS setup scripts may reference unset variables, so do not enable nounset here.
source /opt/ros/humble/setup.bash
source /home/moontaehyung/Capstone/workspace/main/install/setup.bash

APP_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$APP_DIR/../.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-/home/moontaehyung/Capstone/.venv/bin/python}"

"$APP_DIR/scripts/bootstrap_network_tools.sh" || true

if [[ "${STEREO_BOOTSTRAP_CALIBRATION_DEPS:-0}" == "1" ]]; then
  PYTHON_BIN="$PYTHON_BIN" "$APP_DIR/scripts/bootstrap_calibration_deps.sh"
fi

cd "$REPO_ROOT"
exec "$PYTHON_BIN" apps/StereoDistance/main.py "$@"
