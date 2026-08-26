# PX4 v1.17.0 Stock Firmware for SASL Workstation

## Purpose

This directory pins the official PX4 `v1.17.0` stock FMUv6X firmware and the
smallest deterministic parameter/network overlays needed by the current P1/P2
Workstation rig. It is a reproducible alternative to the SASL custom firmware,
not a full vehicle parameter backup.

The firmware is only for the `px4_fmu-v6x_default` target (`board_id=53`). Do
not flash it onto an FMUv6C/Pixhawk 6C or another board target.

## Files

- `px4_fmu-v6x_default.px4`: unchanged official PX4 `v1.17.0` release asset.
- `release-asset.json`: official URL, Git identity, board metadata, size, and
  independently checked SHA-256.
- `p1_workstation.params`: QGroundControl parameter overlay for P1/Left.
- `p2_workstation.params`: QGroundControl parameter overlay for P2/Right.
- `stockPX4.md`: consolidated Korean parameter and SD network-file guide.
- `network/p1/{net.cfg,extras.txt}`: P1 `10.0.0.2/24` SD-card network files.
- `network/p2/{net.cfg,extras.txt}`: P2 `10.0.0.3/24` SD-card network files.
- `SHA256SUMS`: integrity list for the firmware and operator input files.

## Firmware provenance

```text
Repository:    PX4/PX4-Autopilot
Release:       v1.17.0
Target:        px4_fmu-v6x_default
Git commit:    d6f12ad1c4f70ad3230afd7d86e971421e02fef4
Asset bytes:   1,824,460
SHA-256:       8f2c42787163b71e00758205ab0bed9df1669fd26d7a9f42d6f995cbabba0d7f
Download URL:  https://github.com/PX4/PX4-Autopilot/releases/download/v1.17.0/px4_fmu-v6x_default.px4
```

The GitHub release API advertised the same SHA-256 before the asset was copied
into this repository. The embedded PX4 package reports `PX4FWv1`, `board_id=53`,
`git_identity=v1.17.0`, and the commit above.

Primary references:

- PX4 release: <https://github.com/PX4/PX4-Autopilot/releases/tag/v1.17.0>
- PX4 v1.17 Ethernet setup: <https://docs.px4.io/v1.17/en/advanced_config/ethernet_setup>
- QGroundControl parameter format: <https://github.com/mavlink/qgroundcontrol/blob/master/docs/en/qgc-dev-guide/file_formats/parameters.md>

## Parameter overlay contract

Both `.params` files follow the QGroundControl five-column format:

```text
Vehicle-Id  Component-Id  Name  Value  MAV_PARAM_TYPE
```

All included values use MAVLink `MAV_PARAM_TYPE_INT32` (`6`). The overlays
contain transport and current rig sensor-enablement values only:

| Parameter | Stock default | P1 | P2 | Purpose |
|---|---:|---:|---:|---|
| `MAV_SYS_ID` | `1` | `1` | `2` | Unique P1/P2 MAVLink identity |
| `MAV_PROTO_VER` | `2` | `2` | `2` | MAVLink 2 |
| `MAV_2_CONFIG` | `0` | `1000` | `1000` | Let `rc.serial` own the Ethernet instance |
| `MAV_2_MODE` | `0` | `0` | `0` | Normal/GCS stream profile |
| `MAV_2_BROADCAST` | `0` | `1` | `1` | Network discovery/heartbeat broadcast |
| `MAV_2_RADIO_CTL` | `1` | `0` | `0` | Disable SiK-radio throttling on Ethernet |
| `MAV_2_RATE` | `0` | `2000000` | `2000000` | Workstation stream ceiling in bytes/s |
| `MAV_2_REMOTE_PRT` | `0` | `14550` | `14551` | Workstation destination port |
| `MAV_2_UDP_PRT` | `0` | `14550` | `14551` | PX4 network port |
| `GPS_1_CONFIG` | `201` | `201` | `201` | Main serial F9P on GPS1 |
| `GPS_1_PROTOCOL` | `1` | `1` | `1` | u-blox protocol |
| `GPS_2_CONFIG` | `0` | `202` | `202` | Secondary serial F9P on GPS2 |
| `GPS_2_PROTOCOL` | `1` | `1` | `1` | u-blox protocol |
| `UAVCAN_ENABLE` | `0` | `2` | `2` | DroneCAN sensor automatic configuration |
| `UAVCAN_BITRATE` | `1000000` | `1000000` | `1000000` | Current RM3100 bus bitrate |
| `UAVCAN_SUB_MAG` | `1` | `1` | `1` | DroneCAN magnetometer subscription |

Values shown as stock defaults are from the PX4 v1.17 parameter contract. The
unchanged values are included deliberately so the overlay records the complete
Workstation boundary rather than relying on an implicit default.
The downloaded firmware's embedded `parameter_xml` was also decoded and
checked: all 16 overlay parameters exist as `INT32`, and their embedded defaults
match the table above.

The files intentionally exclude:

- `CAL_ACC*`, `CAL_GYRO*`, `CAL_MAG*`, and barometer calibration;
- airframe, mixer/control-allocation, actuator, battery, RC, and safety values;
- board mounting rotation and sensor offsets;
- the private `SASL_CAN_MAG` message, which stock firmware cannot enable with
  a parameter.

Export a full backup before flashing. After stock firmware is installed, run
the normal QGroundControl sensor/compass calibration and verify the discovered
device IDs instead of copying `CAL_*_ID` values from another board.

## Apply order

1. Connect exactly one Pixhawk by USB and save its complete parameter backup.
2. In QGroundControl choose the custom firmware file flow and select the
   bundled `px4_fmu-v6x_default.px4`. This uses an official stock image while
   pinning the exact checked bytes.
3. Reconnect by USB. In **Vehicle Setup > Parameters > Tools > Load from file
   for review**, select the matching `p1_workstation.params` or
   `p2_workstation.params`, review the diff, and apply it.
4. Copy the matching `network/<side>/net.cfg` to `/fs/microsd/net.cfg` and
   `network/<side>/extras.txt` to `/fs/microsd/etc/extras.txt` using the PX4
   console or Workstation `ETH 저장` flow. Run `netman update -i eth0` after
   copying `net.cfg`; PX4 persists the values and reboots automatically.
5. Save parameters before the `netman update` reboot. Do not add a manual
   `mavlink start` or `mavlink stop`; `MAV_2_CONFIG=1000` is the sole
   MAVLink-instance owner.
6. Repeat for the other board only after the first side passes verification.

P2's overlay rows deliberately use input Vehicle-Id `1`: a freshly flashed
stock board initially boots as MAVLink system `1`. Load the file with only P2
connected; `MAV_SYS_ID=2` takes effect after reboot.

## Verification gates

Before a Measurement is approved, verify all of the following:

```text
P1: IP 10.0.0.2, MAVLink system 1, UDP 14550
P2: IP 10.0.0.3, MAVLink system 2, UDP 14551
PC: IP 10.0.0.10/24
```

- `sha256sum -c apps/Workstation/firmware/stock/SHA256SUMS` passes.
- `netman show` reports the expected persistent `DEVICE`, `BOOTPROTO`,
  `IPADDR`, `NETMASK`, and `ROUTER` values. `netman update` consumes and
  deletes the SD-card `net.cfg` by design.
- Each board answers ping from the PC and has a distinct MAC address.
- Only one Ethernet MAVLink instance exists per board in `mavlink status`.
- Workstation receives P1/P2 `HEARTBEAT`, `ATTITUDE`, `GPS_RAW_INT`, and
  `GPS2_RAW` at the requested rates.
- RTCM injection reaches both PX4 receivers and the selected P1/P2 GNSS rows
  produce the expected fix/age evidence.
- QGroundControl calibration completes and PX4 uses the intended RM3100 for
  estimator attitude.
- One start/stop/save Measurement round trip seals successfully.

Stock PX4 does not emit the private `SASL_CAN_MAG` stream. Workstation's
per-CAN `can_port`, `node_id`, and `device_id` view therefore remains
unavailable even though the normal `sensor_mag -> EKF2 -> ATTITUDE` path can
use the RM3100.

## Rollback

If stock validation fails, reconnect only the affected board by USB, reflash
the matching tracked custom artifact under
`apps/Workstation/firmware/px4_fmu-v6x_sasl_can_mag/`, restore that board's
full pre-flash parameter backup, restore its side-specific network files, and
repeat the heartbeat/ATTITUDE/GPS/RTCM checks before reconnecting both boards.

## Coordinate contract

This bundle changes no coordinate calculation, frame, axis order, or unit.
Standard PX4 `ATTITUDE` remains body-FRD relative to world-NED; GPS messages
remain WGS84 latitude/longitude with the PX4/MAVLink altitude fields consumed
by Workstation. The files only configure transport endpoints and enable the
existing serial-GNSS/DroneCAN producers.
