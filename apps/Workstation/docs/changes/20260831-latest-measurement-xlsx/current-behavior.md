# Current Behavior

## Version and worktree context

- Baseline commit: `7242452fe50006a61c9bfa2e012120c9c677571c`.
- The Workstation tree is heavily modified by existing approved/user-owned work,
  including the storage controller, measurement run controller, Data workspace,
  lifecycle, tests and product documents. Those changes are preserved.

## Reproduction

1. Start and stop a sessionless Measurement.
2. Open `Data > Status` and the resolved data folder.
3. Observe fixed JSONL files and binary subdirectories, but no newly generated XLSX.

The inspected representative root contained 43,132 valid top-level JSONL rows. Its
latest stopped generation was exactly joinable through `runtime_generation` in
events/artifacts/native evidence and `measurement_id` in result/sensor streams.

## Observed frontend behavior

`Data > Status` shows writer state, per-stream paths/errors/drops and the existing
`Open Data Folder`, `Open Legacy Recordings Folder`, and `Retry Storage` actions.
The sessionless Review tab is removed and there is no export status or action.

## Observed backend and persistence behavior

One `MeasurementDataStore` daemon writer appends 15 fixed JSONL streams through
bounded priority/bulk queues. Measurement stop returns to `IDLE`, appends a stop event,
requests an asynchronous flush and immediately permits the next run. JSONL never rotates.

Legacy session code can build `review.xlsx`, but production sets
`_measurement_session_hierarchy_enabled=False`, so that path is dormant and incompatible
with the current type-owned root.

## Documentation, code, test, and runtime conflicts

The current product contract explicitly says no automatic `review.xlsx`; this approved
change supersedes only that artifact statement with a distinct, latest-only derived
`latest_measurement.xlsx`. It does not supersede the sessionless availability-first rule.
