# Workflow Impact

## Affected operators and workflow IDs

- Operator: Workstation measurement operator/analyst.
- Workflows: `Measurement Start/Stop`, `Data Status`, `Latest Measurement XLSX Review`.

## Current workflow

Stop Measurement, open the data folder, then assemble JSONL rows externally by generation
and time. Missing devices and storage degradation do not block the next Measurement.

## Proposed workflow

Stop Measurement; the application queues a latest-generation export after storage settles.
Data Status shows waiting/generating/ready/stale/failed state. The operator may update or
open the workbook. JSONL remains available for authoritative analysis.

## Preserved, added, removed, and reordered steps

- Preserved: immediate `IDLE`, immediate next-run admission, fixed JSONL append, file-manager access.
- Added: background export request, status, manual update and open actions.
- Removed/reordered: none; export follows rather than precedes stop completion.

## Decisions, confirmations, handoffs, waits, and artifacts

The stop controller hands generation/boundary metadata to the storage export owner. One
coalescing owner produces `<data-root>/latest_measurement.xlsx`; no per-run XLSX archive is made.

## Interruption, cancellation, retry, restart, and resume

An active/new recording defers or cancels expensive generation. A later completed
generation supersedes an older pending target. Manual update discovers the current-process
last generation, then a persisted stop event, then a latest saved identity with an explicit
inferred-boundary warning. Shutdown cancels and joins the export worker before storage closes.

## Degraded, error, and recovery behavior

Missing rows become `#N/A`/zero counts; malformed rows are listed in `Issues`. A checkpoint
timeout produces an explicitly partial workbook and retry. Generation/validation/replace
failure leaves the prior workbook intact and exposes the error for manual retry.

## Completion and next actions

Success is a validated app-owned XLSX whose published generation equals the requested latest
generation. The operator may open it with the system spreadsheet application or continue work.

## Training, operating-procedure, and compatibility impact

Operators learn two compact Data actions and that XLSX is a disposable derived view. Existing
JSONL/legacy review procedures remain valid.

## Workflow-to-screen, command, state, and regression traceability

`MeasurementRunController.finish_measurement_run` -> export request -> data checkpoint ->
workbook writer -> atomic publication -> Data Status snapshot/actions. Tests cover each handoff.
