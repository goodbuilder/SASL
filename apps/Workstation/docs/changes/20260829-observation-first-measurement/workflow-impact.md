# Workflow Impact

## Affected operators and workflow IDs

Field operator; Measurement Start/Stop, source recovery, recording and storage degradation.

## Current workflow

Start waits for camera readiness; zero ready sides fail the run. Recording locks recovery commands.
Stop may wait for finalization evidence before IDLE.

## Proposed workflow

Start opens RUNNING immediately. Every source independently joins/leaves. Busy resources join after
release; failed cameras require manual reactivation. Stop freezes the run boundary immediately and
late finalization remains attached to the old generation.

## Preserved, added, removed, and reordered steps

Preserve exact result gates and operator Stop. Add source transition evidence and MP4 segments.
Remove camera/storage/media readiness from admission and integrity from completion.

## Decisions, confirmations, handoffs, waits, and artifacts

No confirmation. Camera rejoin creates a new segment. OS opens saved files. `SAVED` means path
exists, not playable or integrity-verified.

## Interruption, cancellation, retry, restart, and resume

No automatic camera retry or restart adoption. Essential structured rows retry in bounded memory;
relaunch starts a fresh run and leaves old bytes untouched.

## Degraded, error, and recovery behavior

Unavailable sources and persistence drops stay visible but local. Mapping/config/calibration
mutations remain blocked during Measurement.

## Completion and next actions

New Measurement is always admissible after the prior stop boundary, regardless of old evidence.

## Training, operating-procedure, and compatibility impact

Operator may start first and recover sources during observation. Existing rows require no migration.

## Workflow-to-screen, command, state, and regression traceability

Measurement Run labels, existing source status cards, recovery commands, Data storage status and
Open Data Folder cover the workflow; no new page or table.
