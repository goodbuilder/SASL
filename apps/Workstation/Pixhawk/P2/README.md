# P2 Pixhawk configuration snapshot

This directory separates the new P2 board's pre-configuration readback,
applied configuration, and files derived for the intended P2 Ethernet role.

- Root-level artifacts are actual device readback or explicit missing-file
  records.
- proposed/ contains P2-specific files derived from the proven P1 structure
  and the Workstation P2 contract. The approved values were applied and
  post-reboot parameter readback passed.
- P1 parameters_backup.bson is never copied here because it contains P1 sensor
  calibration and other board-specific values.
- parameters_backup.bson is the new P2 board's own byte-exact microSD backup.
- ethernet_params.json records the actual factory-like values read from P2.
- ethernet_params.after_config.json records write ACKs, storage ACK, reboot,
  post-reboot readback, and the before/after parameter backups.
- ethernet_runtime.json records the verified USB state and the Ethernet field
  boundary that remains untested because the host USB-Ethernet NIC is absent.

The physically assigned P2 was observed with PX4 GUID
0006000000003837393834335109002d004a and factory-like MAVLink system ID 1.
The configured P2 identity is system ID 2, IP 10.0.0.3, and UDP port 14551.

The P2 board was disarmed throughout configuration. P1 calibration and
board-specific values were never copied.

## Current diagnostic record — 2026-09-04

The current live diagnosis is recorded in
[`ERROR_DIAGNOSIS_20260904.md`](ERROR_DIAGNOSIS_20260904.md).

Summary: the P2 PX4 application and USB MAVLink heartbeat are healthy, but
`eth0` remains `DOWN` and `ifup eth0` fails. PX4 also reports a compass
inconsistency of about 133 degrees, GPS speed accuracy below the configured
preflight threshold, and `EKF2_GPS_CTRL=2`, which disables latitude/longitude
and 3D velocity fusion. P2 is therefore not yet qualified for all-Ethernet
operation or long-duration P1/P2 telemetry verification. No persistent
parameter/file write or reboot was performed during this diagnosis.
