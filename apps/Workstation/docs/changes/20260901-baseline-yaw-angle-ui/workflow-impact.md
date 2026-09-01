# Workflow Impact

## Affected operators and workflow IDs

Field operator; physical baseline and Roll/Pitch/Yaw alignment steps 5–7.

## Current workflow

Open Pixhawk, read yaw and baseline in separate regions, mentally derive direction, then continue
the physical adjustment/sample procedure.

## Proposed workflow

Open Pixhawk, read the existing raw values, then use the adjacent live baseline/yaw summary and top
view to observe P1/P2 horizontal direction. Continue the existing physical adjustment and evidence
sampling procedure unchanged.

## Preserved, added, removed, and reordered steps

One read-only observation is added after raw baseline inspection. No step is removed, reordered,
automated, or made blocking.

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

Document `+` as clockwise/right of Forward and `-` as counter-clockwise/left. Existing files,
settings, commands, and procedures remain compatible.

## Workflow-to-screen, command, state, and regression traceability

`Devices > Pixhawk` -> existing evidence adapter -> new read-only summary/plot. No command path.
Regression covers raw attitude/baseline presentation, independent side loss, stale recovery,
role/source selection, rapid refresh, and shutdown.
