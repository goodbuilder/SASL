# Simulation Scenarios

Scenario files replace fixed UAM-only assumptions with explicit geometry and
target metadata. They are shell env files loaded after the base config files:

```bash
SIM_SCENARIO_CONFIG=small_drone_close_2m_4x.env ./run_operator_view.sh
SIM_SCENARIO_CONFIG=small_drone_close_2m_4x.env ./run_auto_mission.sh
```

Command-line arguments and explicitly exported environment variables still
override values from a scenario file.

Required metadata for new scenarios:

- `SIM_SCENARIO_NAME`
- `SIM_TARGET_CLASS`
- `SIM_TARGET_SIZE_M`
- `SIM_RANGE_MIN_M`
- `SIM_RANGE_MAX_M`
- `SIM_ZOOM_INDEX`
- `CAMERA_BASELINE_M`
- `CAMERA_HEIGHT_M`
- `CAMERA_HORIZONTAL_FOV_RAD`

Coordinate contract: PX4 mission inputs remain local NED
`north/east/altitude`; stereo measurement and correction internals remain
stereo-center RDF `right/down/forward`.
