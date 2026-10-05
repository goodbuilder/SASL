# Workflow Impact

## Affected operators and workflow IDs

- Workflow 9: PX4/attitude verification.
- Workflow 12: diagnostics and fault investigation.
- The priority A/B diagnostic procedure in `Pixhawk.md` remains the operator's authority for
  changing one MAG priority at a time and verifying the actual device.

## Current workflow

The operator observes a raw MAG vector, changes one priority externally/through the existing
calibration action, waits for the selected device readback, and separately reads ATTITUDE yaw.
The operator manually compares those values across sensors and records the result.

## Proposed workflow

The operator opens the same existing `Magn` tables. Each live complete row immediately shows its
raw sensor-axis azimuth. The row that matches the verified active device additionally shows PX4
ATTITUDE yaw and the wrapped difference. The operator still changes priority and verifies active
device identity exactly as before; the new fields are observation aids only.

## Preserved, added, removed, and reordered steps

- Preserved: connect, qualification, PARAM refresh, disarm requirement, priority edit, active
  device verification, ATTITUDE freshness checks, and manual A/B procedure.
- Added: the existing Pixhawk connection performs a bounded read-only Shell observation of
  `sensor_mag` and `vehicle_magnetometer`, so stable firmware can populate RM3100 rows without a
  custom firmware flash.
- Added: read the resulting RM3100 rows and yaw diagnostics in the existing tables.
- Removed/reordered: none.

## Decisions, confirmations, handoffs, waits, and artifacts

Raw vector -> host diagnostic formula; active device identity + ATTITUDE -> PX4 reference; the
Inspector receives the same live read model. No command handoff or new artifact is introduced.
The value is not considered a successful priority change; success remains active-device
readback plus fresh telemetry.

## Interruption, cancellation, retry, restart, and resume

Disconnect/reconnect stops and restarts the bounded Shell observation. A poll timeout, malformed
response, or concurrent manual Shell/PARAM command leaves evidence aging and retries on the next
cadence; no modal action or priority write is introduced.

## Degraded, error, and recovery behavior

Incomplete x/y/z or a near-zero horizontal vector yields `--` with a diagnostic-unavailable
reason. Missing/stale ATTITUDE yields no PX4 reference. An unmatched or inferred-only device
identity never receives the active PX4 yaw; the row says `미선택`/`identity 확인 대기`. Fresh
telemetry and verified active-device readback recover the fields automatically.

## Completion and next actions

The workflow completes when the operator can distinguish raw sensor-axis yaw from the selected
PX4 estimator yaw without changing existing controls. For a priority experiment, the next action
remains recording the selected device ID, priority, and observed PX4 yaw.

## Training, operating-procedure, and compatibility impact

Only a short legend/README note is needed: RAW is diagnostic sensor-axis azimuth; PX4 yaw is the
firmware estimator reference for the verified active sensor; Shell readback is diagnostic-rate and
retains its source. No firmware, saved data, or operator permission change is required.

## Workflow-to-screen, command, state, and regression traceability

`Magn` table -> `EvidenceAdapter` row projection -> pure yaw formatting/calculation -> existing
Pixhawk workspace model/Inspector. Priority, parameter, and stream commands remain unchanged.
Regression covers normal, missing, stale, non-selected, inferred identity, rollover, and rapid
refresh cases plus existing Pixhawk calibration/telemetry workflows.
