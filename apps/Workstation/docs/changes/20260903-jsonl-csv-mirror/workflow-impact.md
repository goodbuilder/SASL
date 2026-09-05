# Workflow Impact

## Affected operators and workflow IDs

Workflow `WS-DATA-CSV-MIRROR`: Workstation operator collecting a sessionless Measurement or
Manual-record evidence set and later opening stream data for inspection or analysis.

## Current workflow

The operator starts the observation, optionally uses `Data > Status` to inspect JSONL writer health,
stops or continues the observation, then opens the data folder and uses JSONL-aware tooling or the
derived XLSX. Each stream is separated by a fixed JSONL file.

## Proposed workflow

The operator performs the same start/stop and source-recovery workflow. While the observation runs,
the storage owner appends a matching CSV sidecar for every JSONL record. Afterward the operator can
open `events.csv`, `measurements.csv`, `camera_left.csv`, and the other same-stem files directly.
`Data > Status` shows both paths and whether the CSV mirror is current, retrying, failed, or dropped.

## Preserved, added, removed, and reordered steps

- Preserved: Measurement start/stop, source joining/leaving, queue priorities, JSONL paths, XLSX
  generation, folder access, Retry Storage, and all device workflows.
- Added: automatic CSV sidecar append; paired CSV path/state visibility in the existing stream table.
- Removed: none. No new mandatory operator confirmation or modal dialog is introduced.
- Reordered: none. CSV work follows the existing JSONL persistence attempt and never precedes the
  authoritative append.

## Decisions, confirmations, handoffs, waits, and artifacts

The operator action remains the existing Measurement/recording action. The storage owner receives
the same canonical record, writes JSONL first, then mirrors it to the stream CSV. Completion evidence
is the persisted row pair and the stream snapshot; a UI click or queue admission alone is not proof.
Artifacts are the 16 `.jsonl` files plus 16 same-stem `.csv` files under the resolved data root.

## Interruption, cancellation, retry, restart, and resume

If JSONL fails, the existing JSONL retry policy applies and no CSV row is published. If JSONL
succeeds but CSV fails, the JSONL row remains authoritative, the CSV mirror enters its bounded retry
path, and the UI reports degradation. Retry Storage reopens failed mirror files. Store restart
continues append-only CSV files with the fixed header; JSONL bytes are never rewritten.

## Degraded, error, and recovery behavior

A missing or foreign/invalid CSV destination is a CSV-only failure. The operator can continue the
Measurement and use JSONL/XLSX, fix or remove the conflicting destination outside the active run,
then press Retry Storage. Bounded CSV retry overflow increments an explicit CSV drop counter and
does not block a producer. Full-storage, permission, malformed-row, and shutdown cases remain
visible in storage diagnostics.

## Completion and next actions

When the operator opens the data folder, CSV files are ready for rows successfully mirrored so far.
They may use CSV for quick inspection, JSONL for authoritative recovery/audit, and XLSX for the
existing multi-sheet review. A future full historical backfill is outside this packet.

## Training, operating-procedure, and compatibility impact

The only new instruction is that same-stem CSV files are convenience mirrors, not independent
truth. Existing JSONL/XLSX workflows remain valid. New CSV consumers must treat `record_json` as
the complete record and must not infer a coordinate-frame conversion from CSV column names.

## Workflow-to-screen, command, state, and regression traceability

`Data > Status` → existing `MeasurementDataStore` snapshot → per-stream JSONL/CSV state → files in
the resolved data root. Regression covers fresh append, restart append, malformed input, JSONL
failure, CSV failure/retry, bounded overflow, close, UI path/state rendering, and preserved
Measurement availability.
