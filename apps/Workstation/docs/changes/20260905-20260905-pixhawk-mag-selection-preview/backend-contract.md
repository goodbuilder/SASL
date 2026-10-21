# Backend Contract

## Commands and validation

Add a pure preview function that accepts CAL_MAG params, live `sensor_mag` instance→device IDs,
and optional role priority overrides. It returns the predicted stock-selector candidate without
claiming active health. Existing priority write validation and disarm gate are unchanged.

The `mag_apply` worker now reads only `UAVCAN_ENABLE`, `UAVCAN_SUB_MAG`,
`CAL_MAG0..3_ID`, and `CAL_MAG0..3_PRIO` before/after a priority write. Full REAL32 calibration
parameters remain owned by the explicit MAG refresh path; they are not needed to apply a priority.

## State, ownership, and concurrency

The panel owns a per-side latest sensor instance read model; the Shell worker remains its only
producer. The PARAM worker remains the only writer of CAL_MAG priority. Workspace refresh reads
snapshots only.

## Worker resource budgets, scheduling, and backpressure

## External boundaries and workers

## Results, errors, timeout, retry, and cancellation

Missing topology produces an empty/waiting preview rather than a failed PARAM operation. Existing
worker timeouts, busy state, and late-result handling remain unchanged.

Follow-up: integer `PARAM_SET` is resent up to three total attempts only when the matching
`PARAM_VALUE` response times out. A received value mismatch is returned immediately without retry.
The worker checks final readback against every requested configured slot. If a write or final
readback fails after earlier slots changed, it reports the partial outcome, invalidates cached
MAG readback, and lets the normal visible/explicit refresh path reconcile it. The disarm gate,
exclusive per-camera command owner, and Shell active-device verification remain.

The worker also validates requested priorities against the captured live instance-to-device
snapshot. Explicit priority-0 lower instances preceding an enabled higher candidate produce a
v1.17 voter-gap error before any write. Empty/incomplete topology does not invent a firmware gap.

## Persistence, restart, and shutdown
