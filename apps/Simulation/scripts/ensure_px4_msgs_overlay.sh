#!/usr/bin/env bash
set -euo pipefail

SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SASL_ROOT="$(cd "$SIM_ROOT/../.." && pwd)"
SASL_ROS_WORKSPACE="${SASL_ROS_WORKSPACE:-$SASL_ROOT/workspace/main}"
PINNED_REF="${PX4_MSGS_REF:-86d8239e962f6939e05c3737784f60c02fa884db}"
SOURCE_IN_REPO="$SASL_ROOT/workspace/main/src/px4_msgs"
PX4_MSGS_SOURCE_DIR="$SASL_ROS_WORKSPACE/src/px4_msgs"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

if [[ ! -d "$PX4_MSGS_SOURCE_DIR" ]]; then
  if [[ ! -d "$SOURCE_IN_REPO" ]]; then
    printf '%s\n' "error: pinned px4_msgs source is missing: $SOURCE_IN_REPO" \
      "Restore the repo-owned workspace source or run tools/bootstrap_ros_workspace.sh once." >&2
    exit 2
  fi
  mkdir -p "$(dirname "$PX4_MSGS_SOURCE_DIR")"
  ln -s "$SOURCE_IN_REPO" "$PX4_MSGS_SOURCE_DIR"
fi

if [[ ! -f "$PX4_MSGS_SOURCE_DIR/package.xml" ]]; then
  printf '%s\n' "error: px4_msgs package source is incomplete: $PX4_MSGS_SOURCE_DIR" >&2
  exit 2
fi
actual_commit="$(git -C "$PX4_MSGS_SOURCE_DIR" rev-parse HEAD 2>/dev/null || true)"
expected_commit="$(git -C "$PX4_MSGS_SOURCE_DIR" rev-parse "${PINNED_REF}^{commit}" 2>/dev/null || true)"
if [[ -z "$expected_commit" || "$actual_commit" != "$expected_commit" ]]; then
  printf '%s\n' "error: px4_msgs source must be pinned to $PINNED_REF (actual ${actual_commit:-unavailable})." \
    "No checkout or source reset was performed." >&2
  exit 2
fi

OVERLAY_SETUP="$SASL_ROS_WORKSPACE/install/setup.bash"
PACKAGE_SETUP="$SASL_ROS_WORKSPACE/install/px4_msgs/share/px4_msgs/local_setup.bash"
OVERLAY_MANIFEST="$SASL_ROS_WORKSPACE/install/share/sasl_ros_workspace/px4_msgs-overlay.env"
if [[ ! -f "$OVERLAY_SETUP" || ! -f "$PACKAGE_SETUP" ]]; then
  if [[ ! -f "$ROS_SETUP" ]]; then
    printf '%s\n' "error: ROS setup not found: $ROS_SETUP" >&2
    exit 2
  fi
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  set -u
  workspace_key="$(printf '%s' "$SASL_ROS_WORKSPACE" | sha256sum | cut -c1-16)"
  # Keep caches separate for different checkouts/workspaces. A shared fixed
  # /tmp build directory can retain a CMake install prefix from another clone
  # and report success while installing the interfaces somewhere else.
  build_base="${SASL_ROS_BUILD_BASE:-/tmp/sasl-px4-msgs-build-${workspace_key}}"
  log_base="${SASL_ROS_LOG_BASE:-/tmp/sasl-px4-msgs-log-${workspace_key}}"
  mkdir -p "$build_base" "$log_base"
  colcon --log-base "$log_base" build \
    --base-paths "$PX4_MSGS_SOURCE_DIR" \
    --build-base "$build_base" \
    --install-base "$SASL_ROS_WORKSPACE/install" \
    --packages-select px4_msgs
fi

if [[ ! -f "$OVERLAY_SETUP" || ! -f "$PACKAGE_SETUP" ]]; then
  printf '%s\n' "error: px4_msgs overlay build did not produce the required setup files." >&2
  exit 2
fi
mkdir -p "$(dirname "$OVERLAY_MANIFEST")"
manifest_tmp="$OVERLAY_MANIFEST.tmp.$$"
printf 'PX4_MSGS_COMMIT=%s\nPX4_MSGS_REF=%s\n' "$actual_commit" "$PINNED_REF" >"$manifest_tmp"
mv "$manifest_tmp" "$OVERLAY_MANIFEST"
printf '%s\n' "px4_msgs overlay ready at a path selected by SASL_ROS_WORKSPACE."
