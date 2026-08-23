# px4_fmu-v6x_sasl_can_mag Custom Firmware

Tracked firmware artifact:

```text
apps/Workstation/firmware/px4_fmu-v6x_sasl_can_mag/px4_fmu-v6x_default.px4
```

Purpose:

- PX4 FMU v6X build that adds a MAVLink-only telemetry path for the two
  DroneCAN (UAVCAN) RM3100 magnetometers, in parallel to the existing
  `sensor_mag` -> EKF2 path (which is untouched).
- New private MAVLink message `SASL_CAN_MAG` (id 180, added to
  `message_definitions/v1.0/common.xml`) carries `time_usec`, `device_id`,
  `can_port` (1=CAN1, 2=CAN2, from `getIfaceIndex()`), `node_id`, and
  `xmag`/`ymag`/`zmag` in Gauss (same sensor-frame vector fed to
  `PX4Magnetometer::update()`, not rotated).
- Fed by a new `sasl_can_mag` uORB topic, published from
  `UavcanMagnetometerBridge::mag_sub_cb`/`mag2_sub_cb`
  (`src/drivers/uavcan/sensors/mag.cpp`), streamed by a new
  `MavlinkStreamSaslCanMag` (`src/modules/mavlink/streams/SASL_CAN_MAG.hpp`).
- No ROS2/DDS/uXRCE-DDS involved in this path -- MAVLink only, per project
  requirement. (This build otherwise still carries the earlier SASL GPS/MAG
  uXRCE-DDS export patches from the `px4_fmu-v6x_default` artifact next to
  this one; that DDS path is unrelated/unused by this telemetry feature.)
- `CONFIG_MODULES_FW_AUTOTUNE_ATTITUDE_CONTROL` disabled in
  `boards/px4/fmu-v6x/default.px4board` to free ~10KB of flash headroom --
  the board was already at ~100% flash before this feature. This vehicle is
  airframe `4001_quad_x` (pure multirotor); fixed-wing autotune is unused on
  it. **This board default is shared board-wide, so any future fixed-wing/VTOL
  build from this source tree needs that option re-enabled first.**

Source build record:

- PX4 git describe: `v1.17.0-alpha1-1214-g7b1e54889d-dirty`
- Build host tree: external PX4 checkout at the recorded revision (machine-local path omitted)
- Build timestamp: see `.px4` JSON `build_time` field
- Files changed from that commit (outer repo):
  - `src/drivers/uavcan/sensors/mag.cpp`, `mag.hpp`
  - `msg/SaslCanMag.msg` (new), `msg/CMakeLists.txt`
  - `src/modules/mavlink/streams/SASL_CAN_MAG.hpp` (new)
  - `src/modules/mavlink/mavlink_messages.cpp`
  - `src/modules/mavlink/mavlink_main.cpp` (null-slot guard for the MAVLink
    signing callback's instance lookup)
  - `boards/px4/fmu-v6x/default.px4board`
  - (pre-existing, unrelated: `src/modules/uxrce_dds_client/*` DDS export patches)
- Files changed in the `mavlink` submodule:
  - `message_definitions/v1.0/common.xml` (+`SASL_CAN_MAG` message, id 180)

Source diff (patches):

Until now this diff existed only as uncommitted working-tree state in the
build host tree above -- unrecoverable if that checkout is lost. It is now
also captured as patches under `../patches/`:

- `px4-v1.17-sasl-can-mag-core.patch` -- the driver/msg/mavlink-stream diff
  (`mag.cpp`, `mag.hpp`, `SaslCanMag.msg`, `msg/CMakeLists.txt`,
  `SASL_CAN_MAG.hpp`, `mavlink_messages.cpp`). Applies to both v6x and v6c.
- `px4-v1.17-sasl-can-mag-mavlink-dialect.patch` -- the `common.xml` addition.
  Apply inside the `mavlink` submodule (`src/modules/mavlink/mavlink`), not
  the outer PX4-Autopilot tree.
- `px4-v1.17-mavlink-signing-null-instance.patch` -- guards the MAVLink
  signing callback's instance lookup against null slots left by instance
  stop/start or startup failure. Without this guard, `mavlink_rcv_if*` can
  hard-fault in `Mavlink::get_instance_for_status()`.
- `px4-v1.17-sasl-can-mag-v6x-flash-fit.patch` -- the
  `CONFIG_MODULES_FW_AUTOTUNE_ATTITUDE_CONTROL` removal from
  `boards/px4/fmu-v6x/default.px4board`. **v6x only** -- see v6c's MANIFEST,
  its board default did not need this.

Apply from a PX4-Autopilot `v1.17.0-alpha1-1214-g7b1e54889d` checkout:

```sh
SASL_ROOT=/path/to/SASL
cd /path/to/PX4-Autopilot
git apply "$SASL_ROOT/apps/Workstation/firmware/patches/px4-v1.17-sasl-can-mag-core.patch"
git apply "$SASL_ROOT/apps/Workstation/firmware/patches/px4-v1.17-mavlink-signing-null-instance.patch"
git apply "$SASL_ROOT/apps/Workstation/firmware/patches/px4-v1.17-sasl-can-mag-v6x-flash-fit.patch"
(cd src/modules/mavlink/mavlink && git apply "$SASL_ROOT/apps/Workstation/firmware/patches/px4-v1.17-sasl-can-mag-mavlink-dialect.patch")
make px4_fmu-v6x_default
```

Integrity:

```text
SHA256 3ef5736aabdaec9047cf97b0cd5eefa2cfc074672c3ae43776c3f0e12d2349ce  px4_fmu-v6x_default.px4
```

Flash usage: 1,956,228 / 1,966,080 bytes (99.50%).

QGroundControl upload:

Use QGC's custom firmware file flow and select this `.px4` file. Do not
choose the automatic PX4 stable firmware install path -- that overwrites
this custom build with stock PX4 and silently removes both this telemetry
path and the SASL DDS export patches (this appears to be what happened to
the vehicle before this investigation: `ver all` on the bench showed plain
`Release 1.17.0` stable, not this "dirty" custom build).

Workstation-side counterpart:

- `workstation_core/mavlink_dialects/` -- vendored pymavlink dialect
  (`sasl_can_mag_dialect.py`, generated from the same `common.xml`) so the
  app can decode `SASL_CAN_MAG`. Registered and applied in
  `PixhawkMavlinkSubscriber.start()`.
- `workstation_core/pixhawk_attitude.py` -- `SASL_CAN_MAG` added to
  `MAVLINK_MESSAGE_IDS`/`MAVLINK_RM3100_STREAM_REQUESTS` (20 Hz). Existing
  `_can_mag_sensor_state_from_mavlink` parser needed no changes; field names
  (`xmag`/`ymag`/`zmag`/`can_port`/`device_id`/`node_id`/`time_usec`) already
  matched.

## 2026-08-23 MAVLink sparse-instance Hard Fault correction

Seven P1 hard-fault records had the same signature: task
`mavlink_rcv_if2`, PC `0x080efbea`, and `r4=0`. Symbolizing that PC against
the matching v6X ELF resolved it to `Mavlink::get_instance_for_status()`;
the faulting instruction dereferenced the null `Mavlink *` loaded from an
empty `mavlink_module_instances` slot. PX4's MAVLink-signing callback added
the lookup without the null check that the neighboring instance lookup
functions already use.

The tracked firmware now includes
`px4-v1.17-mavlink-signing-null-instance.patch`, which skips null slots and
compares the callback status directly with `inst->get_status()`. The matching
Workstation USB recovery command no longer runs `mavlink stop`; stop/start can
create a sparse instance array while another receiver remains live. Rebuilt
machine code contains a `cbz` guard before the instance status address is
formed.

The 2026-08-24 field recurrence also exposed two competing startup owners:
`MAV_2_CONFIG=1000` created the PX4 `rc.serial` Ethernet instance while SD-card
`extras.txt` issued another manual `mavlink start`. Depending on boot timing,
the second bind either failed or produced simultaneous unicast and broadcast
copies. Workstation now treats `MAV_2_CONFIG` as the sole MAVLink-instance
owner; `extras.txt` contains only `ifup`/`ifconfig`, and recovery persists that
state then reboots. This operational correction does not change this firmware
artifact or its SHA-256.

This correction is build-verified. P1/P2 hardware verification must still
prove: no new fault log, both Ethernet heartbeats remain fresh, and message
180 reports independent `can_port=1` and `can_port=2` samples.

## 2026-08-18 Hardware Finding - Pixhawk 1 (left/P1)

Flashed via QGC's custom firmware file flow (Ethernet MAVLink was already
provisioned on this vehicle from earlier work -- SD card `net.cfg`/
`extras.txt` and the saved `MAV_2_CONFIG=1000` param survived the reflash,
so it reconnected over ETH without needing to redo USB bootstrap).

Confirmed:

- PX4 accepts `MAV_CMD_SET_MESSAGE_INTERVAL` for the private message id
  (180) from Workstation's stream-request path -- `SASL_CAN_MAG` streams
  live once Workstation connects, no firmware-side rejection.
- CAN1/CAN2 magnet-proximity isolation: holding a magnet near one CAN port's
  RM3100 moves only that port's panel reading; the other port's reading
  stays put. Checked both directions (CAN1-only, then CAN2-only).
- Workstation-side dedicated-sample priority (the `_mag_slot_state` fix in
  `sensor_presentation_controller.py`) holds under live traffic: the CAN2
  label stays on the `SASL_CAN_MAG` reading and does not flicker back to the
  concurrent, port-ambiguous `SCALED_IMU2` stream.

Still not yet verified on real hardware:

- node_id/device_id distinguish the two sensors correctly over the air
  (isolation was confirmed by physical magnet placement, not yet by reading
  back node_id/device_id values themselves)
- Independent per-port timestamps hold up under sustained streaming
- No regression to `sensor_mag`/EKF/`SCALED_IMU*` (code review says untouched,
  but not flight-verified after this specific build)
- Pixhawk 2 (right/P2) has not been flashed yet -- see
  `../px4_fmu-v6c_sasl_can_mag/MANIFEST.md` or reflash this same v6x build if
  P2 is also v6X hardware
