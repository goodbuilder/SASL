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

## 2026-09-06 parameter backup after yaw-delta tuning

- `parameters-full-20260906T0939KST.params`: complete onboard parameter table
  (1183 params) read over USB with `PARAM_REQUEST_LIST`, QGC overlay format.
  Read-only capture, board disarmed. `EKF2_GPS_CTRL` was `7` at capture time
  (the `2` noted in the 2026-09-04 diagnosis had been corrected); it was later
  set to `0` on both boards because `EKF2_HEAD_NOISE=0.02` also acts as the
  mag-heading consistency gate and GPS NE aiding made heading fusion drop out
  after the rig was moved — see `../../Pixhawk.md` section 12. The
  `yaw-tune-*.params` overlay carries the current value (`0`).
- `yaw-tune-20260906T0939KST.params`: only the parameters changed or pinned on
  2026-09-06. The `CAL_MAG2_YAW=-5.48` trim used earlier that day was removed by
  operator decision (it was pose-dependent); the overlay now carries
  `CAL_MAG2_YAW=0`, `CAL_MAG2_ROT=0`, and the raw P2-P1 ΔYaw is about -7.3°.
  Common values match P1's overlay. Apply with PARAM_SET,
  then PARAM_SAVE and reboot. Never set an unused `CAL_MAGn_PRIO` to 0 — see
  `../../Pixhawk.md` section 12.
