#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="${SASL_ROS_WORKSPACE:-$ROOT_DIR/workspace/main}"
SRC_DIR="$WORKSPACE_DIR/src"
PX4_MSGS_DIR="$SRC_DIR/px4_msgs"
PX4_MSGS_URL="${PX4_MSGS_URL:-https://github.com/PX4/px4_msgs.git}"
PX4_MSGS_REF="${PX4_MSGS_REF:-86d8239e962f6939e05c3737784f60c02fa884db}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_KEY="$(printf '%s' "$WORKSPACE_DIR" | cksum | awk '{print $1}')"
CACHE_ROOT="${SASL_ROS_CACHE:-$HOME/.cache/sasl_ros_workspace/$WORKSPACE_KEY}"
BUILD_BASE="$CACHE_ROOT/build"
LOG_BASE="$CACHE_ROOT/log"
INSTALL_BASE="$WORKSPACE_DIR/install"
PX4_MSGS_MANIFEST="$INSTALL_BASE/share/sasl_ros_workspace/px4_msgs-overlay.env"

usage() {
  cat <<'USAGE'
Usage:
  scripts/bootstrap_ros_workspace.sh [--no-build]

Creates or updates the repo-local ROS workspace used by StereoWIRIS and Simulation:
  workspace/main/src/px4_msgs

The script pins px4_msgs to the PX4_MSGS_REF commit and builds the
px4_msgs package unless --no-build is passed.

Environment overrides:
  SASL_ROS_WORKSPACE  Workspace path. Default: <SASL>/workspace/main
  PX4_MSGS_URL        px4_msgs git URL.
  PX4_MSGS_REF        px4_msgs commit/tag/branch to checkout.
  ROS_SETUP           ROS setup file. Default: /opt/ros/humble/setup.bash
  SASL_ROS_CACHE      Build/log cache root. Default: ~/.cache/sasl_ros_workspace/<workspace-key>
USAGE
}

build_workspace=1
case "${1:-}" in
  --help|-h)
    usage
    exit 0
    ;;
  --no-build)
    build_workspace=0
    ;;
  "")
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

mkdir -p "$SRC_DIR"

if [[ -e "$PX4_MSGS_DIR" && ! -d "$PX4_MSGS_DIR/.git" ]]; then
  echo "error: canonical px4_msgs path exists but is not a git checkout: $PX4_MSGS_DIR" >&2
  echo "Move that path aside, then rerun scripts/bootstrap_ros_workspace.sh." >&2
  exit 2
fi

if [[ -d "$PX4_MSGS_DIR/.git" ]]; then
  git -C "$PX4_MSGS_DIR" fetch --tags origin
else
  git clone "$PX4_MSGS_URL" "$PX4_MSGS_DIR"
fi

git -C "$PX4_MSGS_DIR" checkout --detach "$PX4_MSGS_REF"
EXPECTED_PX4_MSGS_COMMIT="$(git -C "$PX4_MSGS_DIR" rev-parse "${PX4_MSGS_REF}^{commit}")"
ACTUAL_PX4_MSGS_COMMIT="$(git -C "$PX4_MSGS_DIR" rev-parse HEAD)"
if [[ "$ACTUAL_PX4_MSGS_COMMIT" != "$EXPECTED_PX4_MSGS_COMMIT" ]]; then
  echo "error: px4_msgs checkout does not match requested revision: $PX4_MSGS_REF" >&2
  exit 2
fi

if [[ "$build_workspace" -eq 0 ]]; then
  echo "px4_msgs prepared at $PX4_MSGS_DIR"
  echo "Skipped build because --no-build was passed."
  exit 0
fi

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup file not found: $ROS_SETUP" >&2
  exit 1
fi

# ROS setup scripts may reference unset variables.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u

cd "$WORKSPACE_DIR"
mkdir -p "$BUILD_BASE" "$LOG_BASE"
rm -rf "$BUILD_BASE/px4_msgs" "$INSTALL_BASE/px4_msgs" "$WORKSPACE_DIR/build/px4_msgs"
colcon --log-base "$LOG_BASE" build \
  --packages-select px4_msgs \
  --build-base "$BUILD_BASE" \
  --install-base "$INSTALL_BASE"

mkdir -p "$(dirname "$PX4_MSGS_MANIFEST")"
PX4_MSGS_MANIFEST_TMP="${PX4_MSGS_MANIFEST}.tmp.$$"
printf 'PX4_MSGS_COMMIT=%s\nPX4_MSGS_REF=%s\n' \
  "$ACTUAL_PX4_MSGS_COMMIT" "$PX4_MSGS_REF" >"$PX4_MSGS_MANIFEST_TMP"
mv "$PX4_MSGS_MANIFEST_TMP" "$PX4_MSGS_MANIFEST"

echo "ROS workspace ready: $WORKSPACE_DIR"
echo "Build cache: $CACHE_ROOT"
echo "To use it: source $INSTALL_BASE/setup.bash"
