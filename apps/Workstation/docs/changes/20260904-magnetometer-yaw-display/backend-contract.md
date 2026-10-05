# Backend Contract

## Commands and validation

No command or validation contract changes. Existing `CAL_MAG*_PRIO` edits and active-device
verification remain outside this display slice.

## State, ownership, and concurrency

`EvidenceAdapter` remains the read-model owner for table rows. It reads existing `latest_mag_sensors`,
`latest_can_mag_sensors`, `latest_extrinsics`, and active-device evidence without mutating them.
The existing per-camera Shell worker owns one bounded `sensor_mag`/`vehicle_magnetometer` readback
at a time; the GUI refresh remains single-threaded and bounded.

## Worker resource budgets, scheduling, and backpressure

The existing Shell worker is reused with a 2 s poll cadence and a 4 s command timeout. There is no
new queue, high-rate stream request, or unbounded retry. Manual Shell/PARAM work wins the same
per-camera exclusive slot; a poll yields while that slot is busy.

## External boundaries and workers

The read-only external boundary is PX4 Shell over the existing MAVLink `SERIAL_CONTROL` owner:
`listener sensor_mag -i 0..4 -n 1` supplies per-instance RM3100 vectors/device IDs and
`listener vehicle_magnetometer -n 1` supplies the selected device ID. PX4 remains the source of
calibration, sensor selection, and EKF ATTITUDE; Shell samples retain `px4-shell/sensor_mag`
provenance and are not relabelled as DEBUG_VECT.

## Results, errors, timeout, retry, and cancellation

Normal result is an additive row mapping. Missing/malformed/partial Shell output leaves the last
sample to age and emits a throttled diagnostic; timeout/disconnect never blocks the GUI or replaces
the MAVLink receive owner. A late ATTITUDE or active-device update may change only the reference
cell on the next refresh. Reconnect restarts polling automatically.

## Persistence, restart, and shutdown

No persistence or migration. Restart/shutdown clears the same in-memory evidence and existing
connection lifecycle owns recovery.
