# Workflow Impact

## Affected operators and workflow IDs

Field operator; camera installation alignment using P1/P2/Drone GNSS.

## Current workflow

Open Telemetry, read P1/P2/Drone GNSS and F/R/D/Rg, mentally derive the angle, then adjust the
camera installation.

## Proposed workflow

Open Telemetry, read the existing three GNSS positions, then use the adjacent live
`Baseline↔Drone angle` and `90° error` values. Move/rotate the camera installation until the angle
is 90°, while keeping the existing source freshness evidence visible.

## Preserved, added, removed, and reordered steps

One read-only angle observation is added after GNSS position inspection. No step is removed,
reordered, automated, or made blocking.

## Decisions, confirmations, handoffs, waits, and artifacts

No confirmation, handoff, wait, or new artifact is introduced. The UI follows the existing 4 Hz
visible-workspace poll.

## Interruption, cancellation, retry, restart, and resume

Not applicable to a read-only projection. Disconnect/reconnect naturally moves the derived display
through WAITING/STALE/PARTIAL/CURRENT without owning device recovery.

## Degraded, error, and recovery behavior

One missing Pixhawk leaves the other angle independently observable. Missing baseline direction
shows WAITING. Stale finite values remain visible with stale text, age, and muted/dashed geometry.

## Completion and next actions

There is no completion action or verdict. The operator decides when to take the existing physical
alignment sample or continue adjusting hardware.

## Training, operating-procedure, and compatibility impact

Document the angle as the acute horizontal line angle between P1→P2 and midpoint→Drone. 90° means
the Drone lies on the perpendicular through the P1/P2 GNSS midpoint. Existing files, settings,
commands, and procedures remain compatible.

## Workflow-to-screen, command, state, and regression traceability

`Devices > Telemetry` -> existing DroneTelemetryController GNSS snapshot -> new read-only table
columns. No command path. Regression covers GNSS source selection, missing/stale/zero-vector cases,
90° target/error formatting, rapid refresh, table geometry, focus/scroll, and shutdown.
