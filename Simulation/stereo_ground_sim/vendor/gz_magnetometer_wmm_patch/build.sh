#!/usr/bin/env bash
# Builds a drop-in replacement for gz-sim's installed
# gz-sim-magnetometer-system plugin, with its built-in geomagnetic lookup
# table (originally a stock -60..60 lat, coarse snapshot -- see
# px4_wmm_geo_magnetic_tables.hh) replaced by PX4-Autopilot's own WMM table,
# so the simulated magnetometer signal and PX4's independently-computed
# EKF2_MAG_DECL agree at a given lat/lon.
#
# The system-installed gz-sim-magnetometer-system plugin is left untouched;
# this only produces a local .so that stereo_ground_sim's launch points at
# via an absolute path (world SDF plugin filename gets rewritten to it), so
# no system package is modified. Rebuilds only if the source is newer than
# the last build.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
OUT_NAME="libgz-sim-magnetometer-system.so"
OUT_PATH="$BUILD_DIR/$OUT_NAME"

if [[ -f "$OUT_PATH" && "$OUT_PATH" -nt "$SCRIPT_DIR/Magnetometer.cc" \
      && "$OUT_PATH" -nt "$SCRIPT_DIR/px4_wmm_geo_magnetic_tables.hh" \
      && "$OUT_PATH" -nt "$SCRIPT_DIR/Magnetometer.hh" ]]; then
  echo "$OUT_PATH is up to date."
  exit 0
fi

mkdir -p "$BUILD_DIR"

CFLAGS=$(pkg-config --cflags gz-sim8 gz-common5 gz-plugin2 gz-sensors8 \
  gz-sensors8-magnetometer gz-transport13 gz-msgs10 sdformat14 gz-math7)
LIBS=$(pkg-config --libs gz-sim8 gz-common5 gz-plugin2 gz-sensors8 \
  gz-sensors8-magnetometer gz-transport13 gz-msgs10 sdformat14 gz-math7)

g++ -std=c++17 -O2 -fPIC -shared \
  -I "$SCRIPT_DIR" \
  $CFLAGS \
  -o "$OUT_PATH" \
  "$SCRIPT_DIR/Magnetometer.cc" \
  $LIBS

echo "Built $OUT_PATH"
