# Workflow Impact

## Affected operators and workflow IDs

- `WF-RECORD-GNSS-BASELINE-CENTER`: Measurement operator records live source/derived evidence.
- `WF-DATA-REVIEW-LATEST-XLSX`: researcher reviews latest Measurement source sheets.

## Current workflow

Start recording -> three raw PX4 GNSS timelines append independently -> live UI derives F/R/D/Rg
-> stop -> operator must offline-join three raw files to reproduce the display.

## Proposed workflow

Start recording -> primary Drone GNSS callback snapshots just-used selected P1/P2 plus Drone -> pure
estimator produces CURRENT/STALE/WAITING -> one non-blocking bulk append to dedicated JSONL -> stop
checkpoint settles normally -> Latest XLSX exposes exact-generation `GnssBaselineCenter` rows.

## Preserved, added, removed, and reordered steps

- Preserved: start/stop, all raw streams, UI display, RTCM, Measurement, manual recording, file
  ownership, store checkpoint, latest-XLSX generation/open commands.
- Added: automatic derived-row append and XLSX sheet; no operator step.
- Removed/reordered: none.

## Decisions, confirmations, handoffs, waits, and artifacts

- Trigger is primary Drone GPS receipt, not UI refresh. Exactly one attempt per callback.
- Writer admission is not success; Data Status and the actual JSONL bytes are authoritative.
- Artifact: `<data-root>/gnss_baseline_center.jsonl`; derived view: `GnssBaselineCenter` sheet.

## Interruption, cancellation, retry, restart, and resume

- No feature-specific lifecycle. Existing store close/checkpoint drains admitted rows.
- Application restart appends future records to the same fixed file; existing bytes are untouched.
- Failed bulk stream uses existing `Retry Storage`; no private retry or memory backlog is added.

## Degraded, error, and recovery behavior

- Missing P1/P2: append WAITING row with source/reason and null coordinates.
- Stale/skewed: append finite STALE row with actual ages/delta.
- Writer/queue failure: row may be dropped, stream becomes DEGRADED/FAILED, other sources continue.
- Decoded non-finite GNSS fields: store null with explicit WAITING/STALE; any other malformed or
  unsupported serialization input rejects only that row.
- XLSX failure: JSONL remains usable; existing Update Latest XLSX retry applies.

## Completion and next actions

Recording completion does not require this optional bulk stream to be lossless. The operator checks
Data Status dropped/error counts when completeness matters and uses JSONL as source of truth.

## Training, operating-procedure, and compatibility impact

Operators gain one file and one XLSX sheet. They must not compare these columns to camera
`baseline_center_*` without reading the explicit frame. Existing readers that ignore unknown files
and sheets remain compatible.

## Workflow-to-screen, command, state, and regression traceability

| Behavior | Owner/evidence | Test |
|---|---|---|
| One row per primary Drone callback | Sensor -> Recording controller -> fixed stream | trigger/duplicate test |
| No recording, no row | recording gate | negative test |
| WAITING/STALE/CURRENT exact payload | pure estimator + independent source fixture | row contract tests |
| Writer failure isolated | MeasurementDataStore bulk stream | failure/drop tests |
| Latest generation only | Latest XLSX sheet spec | old/new generation fixture |
| Raw/Measurement/Parquet unchanged | existing schemas | affected/full regression |
