# Change Brief

## Operator outcome

For every observable magnetometer row in the Pixhawk `Magn` area, the operator can see both
the raw sensor-axis yaw indication and the PX4 yaw reference produced while a magnetometer is
actually selected by priority. The display must make the two meanings unmistakable.

## Motivation

Priority A/B tests currently require manually recording the raw magnetic vector, changing
`CAL_MAG*_PRIO`, and reading `ATTITUDE.yaw`. A side-by-side readout will shorten that diagnostic
loop without turning raw telemetry into an unverified estimator result.

## Included scope

- Add read-only `RAW yaw` to the existing IMU `SCALED_IMU/2/3` table and dedicated CAN table.
- Add read-only `PX4 yaw (selected)` and a signed difference where the row's device identity is
  proven to be the active `vehicle_magnetometer` device.
- Keep the current raw x/y/z, norm, unit, age, topic, identity, and selection evidence visible.
- Mark rows without a proven identity, without a complete horizontal vector, or without fresh
  ATTITUDE as unavailable rather than copying a value from another sensor.
- Document that the PX4 value is the firmware estimator output and that raw yaw is diagnostic.

## Excluded scope and non-goals

- No automatic priority changes, calibration writes, reset actions, or new MAVLink requests.
- No host-side claim that a standard `SCALED_IMU*` row reproduces PX4's complete EKF for an
  unidentifiable physical sensor.
- No new persistence/schema field, measurement semantic, or alignment verdict.
- No changes to PX4 firmware, stream ownership, worker scheduling, or unrelated workspaces.

## Acceptance summary

The two existing magnetometer tables retain stable geometry and refresh in place. A complete
row displays a normalized signed raw sensor-axis azimuth. A row with a matching active device
displays the current PX4 `ATTITUDE.yaw` as the corrected/tilt-aware reference and its wrapped
delta; all other rows show an explicit non-selected or identity-unproven state. Focused logic,
evidence, UI, stale/missing, selection, and layout tests pass.

## Approved scope amendment

The P2 USB observation confirms that `SCALED_IMU/2/3` currently correspond to BMM150, GPS1
IST8310, and GPS2 IST8310, while RM3100 is exposed through the existing dedicated CAN telemetry
path as separate device instances. To make RM3100 visible in the requested `IMU magnetometer
(MAVLink)` view, add those existing dedicated-CAN RM3100 rows to that view as an additive
projection. Keep the dedicated CAN table unchanged, preserve source/topic/device evidence, and
reuse the same RAW yaw, exact active-device PX4 yaw, wrapped delta, stale, and unavailable-state
rules. This amendment adds no priority change, MAVLink stream request, firmware write, persistence,
or measurement/alignment behavior.

The user approved this amendment on 2026-09-04 and selected the stable-firmware-compatible
`DEBUG_VECT` path rather than the private `SASL_CAN_MAG` message. The implementation therefore
uses only the existing `DEBUG_VECT` `CAN1_MAG`/`CAN2_MAG` compatibility rows and makes no new
private-message stream request.

## Approved scope amendment — Stable PX4 Shell RM3100 readback

Stable PX4 remains the only Pixhawk firmware. The Workstation adds a read-only polling adapter for
the existing PX4 Shell `listener sensor_mag` path over `SERIAL_CONTROL`. It filters the returned
per-instance samples by known RM3100 device IDs, retains the source as `px4-shell/sensor_mag`, and
projects those readings into the existing RM3100 CAN1/CAN2 rows. It also reads the active
`vehicle_magnetometer` identity so the established exact-device/fresh-ATTITUDE yaw rule remains
unchanged. No custom firmware, private stream request, MAVLink relabeling, or persistence schema
change is included.
