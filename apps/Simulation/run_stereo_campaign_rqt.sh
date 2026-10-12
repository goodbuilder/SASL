#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INTEGRATION_ROOT="$(cd "$ROOT_DIR/../.." && pwd)"
COMMON_ROS="$INTEGRATION_ROOT/common/ros"
SASL_ROS_WORKSPACE="${SASL_ROS_WORKSPACE:-$INTEGRATION_ROOT/workspace/main}"
export SASL_ROS_WORKSPACE

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf '%s\n' "error: ROS 2 Humble setup not found at /opt/ros/humble/setup.bash" >&2
  exit 2
fi
cd "$ROOT_DIR"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

# This uses the repo-owned pinned px4_msgs source, builds only if the overlay
# files are missing, and never deletes an existing workspace build.
bash "$ROOT_DIR/scripts/ensure_px4_msgs_overlay.sh"

set +u
# shellcheck disable=SC1091
source "$SASL_ROS_WORKSPACE/install/setup.bash"
if [[ -f "$SASL_ROS_WORKSPACE/install/px4_msgs/share/px4_msgs/local_setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$SASL_ROS_WORKSPACE/install/px4_msgs/share/px4_msgs/local_setup.bash"
fi
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT_DIR/install/setup.bash"
fi
set -u

SIM_CONFIG="${SIM_CONFIG:-$ROOT_DIR/config/simulation.env}"
export SIM_CONFIG
export PYTHONPATH="$ROOT_DIR:$ROOT_DIR/stereo_ground_sim:$ROOT_DIR/python_patches${PYTHONPATH:+:$PYTHONPATH}"

installed_plugin_xml="$ROOT_DIR/install/stereo_ground_sim/share/stereo_ground_sim/plugin.xml"
if [[ ! -f "$installed_plugin_xml" ]] || ! rg -q 'name="CampaignPanel"' "$installed_plugin_xml"; then
  build_base="${SASL_CAMPAIGN_COLCON_BUILD_BASE:-$(mktemp -d /tmp/sasl-campaign-colcon.XXXXXX)}"
  printf '%s\n' "Building the campaign RQT plugin into the existing Simulation install…"
  colcon build \
    --base-paths "$ROOT_DIR" "$COMMON_ROS/simulation_interfaces" \
    --build-base "$build_base" \
    --packages-select simulation_interfaces stereo_ground_sim stereo_ground_sim_overlay
  # shellcheck disable=SC1091
  source "$ROOT_DIR/install/setup.bash"
fi

if [[ ! -f "$ROOT_DIR/config/campaign_control.perspective" ]]; then
  printf '%s\n' "error: campaign RQT perspective is missing." >&2
  exit 2
fi

exec ros2 run rqt_gui rqt_gui \
  --clear-config \
  --force-discover \
  --perspective-file "$ROOT_DIR/config/campaign_control.perspective"
