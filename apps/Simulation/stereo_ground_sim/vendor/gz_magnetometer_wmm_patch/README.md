# gz-sim magnetometer WMM table patch

`Magnetometer.cc`/`Magnetometer.hh` are gz-sim 8.11.0's
`src/systems/magnetometer/` sources (Apache-2.0, Open Source Robotics
Foundation), vendored here with one change: the built-in geomagnetic lookup
table (`-60..60` latitude, an older/coarser WMM snapshot) is replaced by
`px4_wmm_geo_magnetic_tables.hh`, copied verbatim from PX4-Autopilot's
`src/lib/world_magnetic_model/geo_magnetic_tables.hpp` (BSD-3-Clause, PX4
Development Team; checkout at SHA `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`).

## Why

gz-sim's `Magnetometer` system recomputes declination/inclination/strength
from lat/lon on every update using its own vendored table, ignoring the
world SDF's `<magnetic_field>` after the first tick. That table disagreed
with PX4's own WMM table (which PX4 uses for `EKF2_MAG_DECL`) by ~0.85° of
declination at this world's `spherical_coordinates` lat/lon, which showed up
as a small but real bias in Attitude/Full ray heading. Replacing gz-sim's
table with PX4's makes both sides reference the same WMM data. This does not
touch EKF2 parameters, coordinate transforms, or PX4/GZBridge source.

## What this does *not* do

The system-installed `gz-sim-magnetometer-system` plugin package is left
untouched. `build.sh` compiles this patched source into a local `.so`
(`build/`, gitignored) linked against the already-installed system
`libgz-sim8.so` and friends, and `_generate_world()` in
`launch/stereo_ground_sim.launch.py` points the world SDF's magnetometer
plugin at that `.so` by absolute path (via
`STEREO_GROUND_SIM_MAGNETOMETER_PLUGIN_PATH`, set by
`run_operator_view.sh`) instead of gz-sim's stock one. Deleting `build/` or
unsetting that env var reverts to gz-sim's stock plugin with no other
changes needed.

## Updating

If PX4-Autopilot's WMM table is regenerated (e.g. after fetching a newer
NOAA/WMM release via `fetch_noaa_table.py`), re-copy
`src/lib/world_magnetic_model/geo_magnetic_tables.hpp` over
`px4_wmm_geo_magnetic_tables.hh` here (the `SAMPLING_RES`/`LAT_DIM`/`LON_DIM`
header lines are stripped out since `Magnetometer.cc` declares its own,
already matching this table's `-90..90` latitude range) and rerun
`build.sh`.
