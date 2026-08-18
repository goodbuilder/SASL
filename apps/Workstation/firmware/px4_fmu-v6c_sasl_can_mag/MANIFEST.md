# px4_fmu-v6c_sasl_can_mag Custom Firmware

Tracked firmware artifact:

```text
apps/Workstation/firmware/px4_fmu-v6c_sasl_can_mag/px4_fmu-v6c_default.px4
```

For Pixhawk 6C / 6C Mini (both use the `px4_fmu-v6c_default` build target --
same MCU/firmware, Mini is a smaller form factor of the same reference
design).

Purpose: same `SASL_CAN_MAG` MAVLink-only CAN-magnetometer telemetry path as
`../px4_fmu-v6x_sasl_can_mag/`. See that MANIFEST.md for the full design
description, data flow, field/unit definitions, and Workstation-side
counterpart -- the source changes are shared (`src/drivers/uavcan/sensors/mag.*`,
`msg/SaslCanMag.msg`, `src/modules/mavlink/streams/SASL_CAN_MAG.hpp`,
`mavlink_messages.cpp`, `message_definitions/v1.0/common.xml`); only the
board target differs.

Board-specific notes:

- `boards/px4/fmu-v6c/default.px4board` was **not** modified -- this build
  fit without disabling `fw_autotune_attitude_control` (v6c has fewer
  default drivers than v6x, more flash headroom).
- Confirmed 2 CAN interfaces (`UAVCAN_NUM_IFACES=2` in the build), so the
  `can_port` 1=CAN1/2=CAN2 mapping applies the same as on v6x.

Source diff (patches): same core diff as v6x, see
`../px4_fmu-v6x_sasl_can_mag/MANIFEST.md` for the full list and apply
instructions -- `../patches/px4-v1.17-sasl-can-mag-core.patch` and
`../patches/px4-v1.17-sasl-can-mag-mavlink-dialect.patch` apply here
unchanged; skip `px4-v1.17-sasl-can-mag-v6x-flash-fit.patch`, it is v6x-only.

Source build record:

- PX4 git describe: `v1.17.0-alpha1-1214-g7b1e54889d-dirty` (same commit/tree
  as the v6x build, built same session)
- Flash usage: 1,943,304 / 1,966,080 bytes (98.84%)

Integrity:

```text
SHA256 c3fc7d2015ff6c8c78c885de22a0a5d36d92a4304b212197886e470d299e79ee  px4_fmu-v6c_default.px4
```

QGroundControl upload: use the Custom firmware file flow, not automatic
stable install (see sibling MANIFEST for why).

Not yet verified on real hardware: same list as
`../px4_fmu-v6x_sasl_can_mag/MANIFEST.md` -- this build has not been flashed
or bench-tested at all yet.
