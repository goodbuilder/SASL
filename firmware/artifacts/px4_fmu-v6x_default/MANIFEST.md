# px4_fmu-v6x_default Custom Firmware

Tracked firmware artifact:

```text
firmware/artifacts/px4_fmu-v6x_default/px4_fmu-v6x_default.px4
```

Purpose:

- PX4 FMU v6X build with SASL GPS/MAG uXRCE-DDS export patches applied.
- Exports `/fmu/out/sensor_gps_0`, `/fmu/out/sensor_gps_1`,
  `/fmu/out/sensor_mag_0..4`, and `/fmu/out/vehicle_magnetometer`.

Source build record:

- PX4 git describe: `v1.17.0-alpha1-1214-g7b1e54889d-dirty`
- Build timestamp: `2026-06-24 06:10:22 KST`
- Original local artifact path:
  `/home/moontaehyung/Capstone/source/PX4-Autopilot/build/px4_fmu-v6x_default/px4_fmu-v6x_default.px4`

Integrity:

```text
SHA256 63410eca73fc864770afee773e6e41bd6b7787fd802a3df1f7ff73720b699ea8  px4_fmu-v6x_default.px4
```

QGroundControl upload:

Use QGC's custom firmware file flow and select this `.px4` file. Do not choose
the automatic PX4 stable firmware install path, because that overwrites the SASL
custom DDS export firmware with stock PX4.
