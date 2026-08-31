# Impact Analysis

## Preserved invariants

- JSONL is authoritative, append-only and never rewritten by export.
- Storage/export errors never gate Measurement or device owners.
- One data store owns append handles; one exporter owns the generated workbook.
- Existing user changes, legacy Sessions and `review.xlsx` remain untouched.

## Affected layers and components

- Domain/persistence: generation discovery/filtering, safe Excel serialization and checkpoint.
- Controller/state: one coalesced exporter status and cancellation lifecycle.
- Workflow/UI: Measurement stop request and Data Status actions/status.
- Documentation/tests: recording schema, Data workflow and regression fixtures.

## Feature dependency, ownership, and coupling changes

Data storage owns export; Measurement only submits immutable boundary metadata and Data UI only
uses public request/snapshot methods. No device, method worker or widget private state is accessed.

## Worker CPU/GPU resource impact

Baseline has no XLSX worker. Proposed maximum is one bounded CPU/disk export worker, no GPU,
no nested pool and no parallel export. It waits for an idle recording window, coalesces targets,
uses streaming/constant-memory output and cooperatively cancels on new recording/shutdown.

## Compatibility and migration

No schema migration or dependency change. A new root-level derived file is additive. An existing
unmarked same-name file is treated as user-owned and never overwritten.

## Rollback and recovery

Remove the UI/controller/export additions; JSONL remains complete and usable. A generated XLSX
may be deleted independently. Failed temporary files are uniquely named and cleaned by their owner.

## Risks and assumptions

Excel row/column/cell limits require deterministic partitioning. Timestamp precision requires
large integer text cells. Late rows may briefly mark the workbook stale before regeneration.
The approved target is the latest Measurement only, not a complete historical archive.
