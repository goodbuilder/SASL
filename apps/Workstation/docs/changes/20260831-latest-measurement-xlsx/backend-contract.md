# Backend Contract

## Commands and validation

`request_latest_measurement_xlsx(generation=None, started_utc_ns=None,
stopped_utc_ns=None, source_states=None, automatic=False) -> bool` accepts/coalesces a request.
`latest_measurement_xlsx_snapshot() -> mapping` is the read model.
`cancel_latest_measurement_xlsx_export(timeout_s=...) -> bool` owns cancellation/shutdown.

An explicit generation must be a nonempty safe token. Manual discovery prefers the current-process
last completed generation, then latest persisted `measurement_stopped`, then latest saved identity
with `completion_evidence=#N/A`.

## State, ownership, and concurrency

One DataStorageController-owned coordinator permits one worker and one pending latest target.
States are `NOT_CREATED`, `WAITING`, `GENERATING`, `READY`, `STALE`, `FAILED`, `CANCELLED`.
Duplicate requests coalesce; a newer completed generation supersedes an older pending target.

## Worker resource budgets, scheduling, and backpressure

At most one low-priority CPU/disk worker, streaming rows with XlsxWriter constant-memory mode.
Automatic work waits for two seconds of matching-row quiet and for recording idle. No unbounded
job queue exists. A new recording or shutdown requests cooperative cancellation.

## External boundaries and workers

The only external boundary is the filesystem and XLSX ZIP/XML format. No network/device/GPU work.
A background-only store checkpoint waits up to five seconds for pre-cutoff admitted rows to settle,
fsyncs handles, and returns complete-line byte limits plus pending/drop/error evidence.

## Results, errors, timeout, retry, and cancellation

Checkpoint timeout still permits a partial, explicitly flagged workbook and schedules retry.
Malformed JSON is recorded in `Issues`; unsupported values become `#N/A`. Hard read/write/validate
or ownership collision failure does not publish. Cancellation deletes only the owned temp file.

## Persistence, restart, and shutdown

Output is `<resolved-root>/latest_measurement.xlsx`, schema
`workstation-latest-measurement-xlsx-v1`. A validated temp file is fsynced and atomically replaced.
Only a missing or schema-marked destination is replaceable. Shutdown stops exporter before closing
the JSONL writer. No sidecar, migration, JSONL mutation or startup gate is introduced.
