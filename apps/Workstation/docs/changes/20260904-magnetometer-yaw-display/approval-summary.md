# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

The existing Pixhawk `Magn` tables will gain two read-only diagnostics for each row: a raw
sensor-axis yaw and, when the active physical device is proven, the actual PX4 `ATTITUDE.yaw`
reference plus its signed difference. Existing x/y/z, norm, units, age, topic, device identity,
priority controls, calibration readback, and all other workflows stay the same.

## Operator workflow before and after

Before, the operator compares the raw vector and the PX4 yaw manually after each priority A/B
change. After, the same existing procedure shows the comparison in the table. The operator still
has to change priority and verify the active device; the display never changes priority itself.

## Success, delay, rejection, failure, overload, and recovery

Success is a stable row-level RAW value and an exact selected-device PX4 reference. Missing or
near-zero vector, stale ATTITUDE, fallback/unmatched identity, Shell timeout/malformed output, or
disconnected telemetry produces an explicit unavailable/non-selected state. The existing bounded
Shell worker is reused at diagnostic cadence; manual Shell/PARAM work yields the poll, and reconnect
restores polling.

## Data and compatibility effects

The change is in-memory and additive. No saved field, measurement schema, firmware parameter, or
migration changes. Existing legacy candidates and fake/test owners remain readable.

## Performance priorities and allowed degradation

GUI responsiveness and existing telemetry cadence have priority. Only constant-time finite math,
angle wrapping, and text formatting are added to the current refresh. Shell polling is bounded at
one read per side every 2 seconds with a 4-second timeout; there is no high-rate stream request or
unbounded retry.

## Visual and interaction changes, including dynamic-layout stability

Reuse the existing compact table/Inspector style. Values use fixed signed degree formatting and
existing table update-in-place behavior. Columns, row selection, focus, scroll, splitter, and
window geometry must remain stable across signs, rollovers, stale states, and rapid refreshes.

## How acceptance will be demonstrated

- Pure tests prove raw angle, wrapping, invalid-vector, and stale/reference gates.
- Evidence tests prove exact active-device correlation and no false copied PX4 value.
- Qt tests prove both tables/Inspector display and geometry/selection stability.
- Existing Pixhawk telemetry/calibration/UI regressions and the documented launcher check pass.
- Real priority A/B behavior remains hardware/manual evidence, not a fake-test claim.

## Cost, risks, assumptions, rollback, and field limits

The key risk is presenting raw sensor-axis azimuth as a physical heading. Labels and the Inspector
must state that RAW is diagnostic. PX4 yaw is treated as the firmware's calibrated/rotation-aware,
tilt-aware estimator reference, but may include fusion/history and is shown only for the proven
active device. Rollback is additive source removal. Real-rig identity/fallback behavior remains
unverified until hardware is available.

## Exact approved scope

Add the RAW yaw and exact active-device PX4 yaw/delta readouts to the existing IMU and dedicated
CAN magnetometer tables, with no priority/firmware/stream/persistence/workflow changes. The user
approved this exact scope and confirmed `apps/Workstation/README.md` (`README.md` relative to this
workspace) on 2026-09-04.

## Approved scope amendment

The P2 USB capture showed RM3100 as dedicated CAN magnetometer devices, separate from the three
standard `SCALED_IMU*` streams. The proposed amendment is to include the existing dedicated-CAN
RM3100 rows in the `IMU magnetometer (MAVLink)` table as an additive view, while retaining the
dedicated CAN table and all existing controls. The RM3100 rows will show the same producer-axis
RAW yaw and, only for an exact active-device match, PX4 `ATTITUDE.yaw` plus wrapped delta.

No priority, firmware, private-message stream, persistence, worker, or measurement behavior changes
are included. The user approved this amendment on 2026-09-04 and selected the stable-firmware
`DEBUG_VECT` path (`CAN1_MAG`/`CAN2_MAG`).

## Approved scope amendment — Stable PX4 Shell RM3100 readback

The connected stock/stable PX4 does not emit the named `DEBUG_VECT` RM3100 samples reliably, so
the operator must still be able to observe the real RM3100 instances without flashing custom
firmware. The Workstation will add a read-only, bounded PX4 Shell polling path over the existing
`SERIAL_CONTROL` owner. It will query `listener sensor_mag` instances, retain only known RM3100
device IDs, and feed those samples into the same two RM3100 rows already used by the IMU view and
the dedicated CAN view.

The polling path will also read the active `vehicle_magnetometer` device identity so the existing
exact-device/fresh-ATTITUDE gate can produce the five expected yaw projections per Pixhawk:
three standard IMU rows plus RM3100 CAN1 and CAN2. Shell samples remain explicitly labelled
`px4-shell/sensor_mag`; they are not relabelled as MAVLink `DEBUG_VECT`.

Preserved: stable firmware, existing MAVLink streams, standard `SCALED_IMU*` rows, existing
DEBUG_VECT/SASL_CAN_MAG compatibility decoders, priority/calibration controls, persistence schema,
and all unrelated Workstation workflows. The polling cadence is bounded and read-only; duplicate
polls, timeout, malformed output, disconnect, stale result, and shutdown must fail visibly without
blocking or replacing the MAVLink receive owner. This amendment was explicitly approved by the
user on 2026-09-05 with “수정 시작”.
