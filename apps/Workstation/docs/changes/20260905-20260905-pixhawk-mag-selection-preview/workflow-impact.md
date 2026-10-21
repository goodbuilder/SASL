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

## Follow-up workflow

The operator edits one or more P1/P2 priorities. Background status refresh updates actual
readback but preserves staged edits and marks them as pending. The operator can explicitly
discard the draft back to the latest confirmed PARAM values or apply it. While a PARAM worker is
active, that side's controls are locked. A confirmed voter-gap configuration blocks Apply with
the affected lower instance numbers and the `Min (1)` remedy. Missing topology leaves Apply
available but shows that the firmware guard could not be evaluated.

Apply retries a lost PARAM acknowledgement, confirms final readback, then retains the existing
independent active-device/fresh-ATTITUDE verification. On failure, staged values remain visible,
cached status is invalidated, and a fresh readback is requested by the normal workspace refresh
path. Saving and rebooting remain explicit separate operations; Apply never performs either
automatically.

## Preserved, added, removed, and reordered steps

- Preserved: disarm requirement, independent P1/P2 controls, manual PARAM refresh, priority
  writes, save/reboot controls, and fresh active-device/ATTITUDE verification.
- Added: instance/device/role/slot preview and candidate-vs-actual display.
- Added in the follow-up: per-side staged edit state, explicit discard, bounded PARAM_SET retry,
  partial-write reconciliation, and a v1.17 sparse-voter guard.
- Removed: none. No automatic topology or calibration mutation is introduced.

## Decisions, confirmations, handoffs, waits, and artifacts

The Shell polling read model owns live instance identity; the PARAM worker owns CAL_MAG values
and priority writes; the workspace only renders their public read models. A physical removal is
followed by Refresh MAG, not by an automatic slot remap.

## Interruption, cancellation, retry, restart, and resume

Existing worker busy/timeout/error behavior remains. A topology preview may temporarily show
waiting or partial instances during reconnect; it must not reset staged combo selection or outer
scroll. Failed writes retain drafts and require a fresh readback before actual values are presented
as current.

## Degraded, error, and recovery behavior

Missing Shell topology leaves the preview in `Topology readback waiting` while PARAM and active
status remain available; the firmware-specific guard is then unavailable and Apply keeps its
existing gates. A confirmed v1.17 voter gap is shown with explicit instance IDs and blocks Apply.
An active-device mismatch remains candidate-versus-actual evidence, not fabricated success.

## Completion and next actions

Completion requires focused model/UI tests, launcher checks, and a real/manual runtime check
that a five-instance CAN2 candidate differs from the actual active sensor without blocking the
Apply action.

## Training, operating-procedure, and compatibility impact

## Workflow-to-screen, command, state, and regression traceability
