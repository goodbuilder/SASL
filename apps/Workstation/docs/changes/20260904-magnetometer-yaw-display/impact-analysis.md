# Impact Analysis

## Preserved invariants

- No priority write, calibration reset, new MAVLink stream request, or firmware behavior changes.
- Raw producer axes, units, topic, age, device identity, and selection evidence remain intact.
- PX4 yaw is never presented as proof for an unmatched sensor.
- Existing table selection, Inspector, connection, PARAM, measurement, recording, and alignment
  workflows remain unchanged.

## Affected layers and components

- Pure logic: new finite-vector/raw-yaw and wrapped-delta calculation, with explicit invalid gates.
- Evidence adapter: additive row fields for raw yaw and selected PX4 reference state.
- Pixhawk workspace: additive columns and Inspector text.
- Tests: magnetometer workspace/evidence plus focused pure-logic tests.
- Documentation: packet; after approval, confirmed Workstation README and relevant UI product docs.

## Feature dependency, ownership, and coupling changes

No owner transfer. The Pixhawk evidence adapter reads the existing public attitude and active-device
read models; it does not access another feature's widgets or workers. Calibration/priority remains
owned by the existing PX4 MAG manager and firmware.

## Worker CPU/GPU resource impact

The existing per-camera Shell worker is reused at most once every 2 s, with a 4 s timeout and no
overlapping poll/manual command on a side. No new queue, GPU, or accelerator memory is introduced;
the GUI only receives the bounded completion callback.

## Compatibility and migration

Additive in-memory fields only; no saved schema or migration. Legacy/fake panel owners that omit
attitude or device identity continue to render raw tables with explicit unavailable PX4 fields.

## Rollback and recovery

Revert the additive pure logic, evidence fields, columns, and tests. Runtime recovery follows the
existing telemetry reconnect and PARAM readback paths.

## Risks and assumptions

- Raw `SCALED_IMU*` x/y/z may not identify the physical CAL_MAG slot; no calibration is guessed.
- PX4 `ATTITUDE.yaw` may be influenced by estimator fusion/history and is not guaranteed to be
  solely a one-sensor instantaneous atan2 result; the UI labels it as firmware reference.
- Exact active-device/ATTITUDE matching depends on available telemetry; real hardware verification
  remains required.
- Shell `SERIAL_CONTROL` availability and latency remain transport/firmware boundary conditions;
  failure is visible as aging/unavailable evidence rather than a synthetic live sample.
