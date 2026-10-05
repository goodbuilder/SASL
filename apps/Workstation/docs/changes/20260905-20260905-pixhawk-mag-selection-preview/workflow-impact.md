# Workflow Impact

## Affected operators and workflow IDs

- Pixhawk MAG observation and priority workflow: refresh → inspect → edit priority → Apply & Verify.
- Hardware topology refresh after a GPS/MAG physical change.

## Current workflow

The operator refreshes PARAM state, edits role priorities, applies one side, and waits for
active-device/ATTITUDE verification. Device identity is inferred from CAL_MAG slots and the
operator separately uses raw MAG rows or Shell logs to understand instance order.

## Proposed workflow

The operator refreshes the side, reads the topology preview, edits any role priorities, and
sees the projected priority candidate update immediately. Apply & Verify still writes the
requested priority values and waits for the actual selected ID and fresh ATTITUDE; the preview
does not block a requested CAN2 priority.

## Preserved, added, removed, and reordered steps

- Preserved: disarm requirement, independent P1/P2 controls, manual PARAM refresh, priority
  writes, save/reboot controls, and fresh active-device/ATTITUDE verification.
- Added: instance/device/role/slot preview and candidate-vs-actual display.
- Removed: none. No automatic topology or calibration mutation is introduced.

## Decisions, confirmations, handoffs, waits, and artifacts

The Shell polling read model owns live instance identity; the PARAM worker owns CAL_MAG values
and priority writes; the workspace only renders their public read models. A physical removal is
followed by Refresh MAG, not by an automatic slot remap.

## Interruption, cancellation, retry, restart, and resume

Existing worker busy/timeout/error behavior remains. A topology preview may temporarily show
waiting or partial instances during reconnect; it must not reset combo selection or outer scroll.

## Degraded, error, and recovery behavior

Missing Shell topology leaves the preview in `Topology readback waiting` while PARAM and active
status remain available. A mismatch is rendered as candidate versus actual, not as a blocked
action or a fabricated success.

## Completion and next actions

Completion requires focused model/UI tests, launcher checks, and a real/manual runtime check
that a five-instance CAN2 candidate differs from the actual active sensor without blocking the
Apply action.

## Training, operating-procedure, and compatibility impact

## Workflow-to-screen, command, state, and regression traceability
