# Backend Contract

## Commands and validation

No command change. `workstation_calibration_capture_snapshot()` returns JSON-safe
`workstation-calibration-capture-status-v1` evidence; command acceptance never increments success.

## State, ownership, and concurrency

Controller owns run counters and active-root identity. A request is counted only when an existing
checkerboard validation worker is accepted. Waiting ticks do not count. Root change/restart resets
run counters; persisted set inventory remains recoverable.

## Worker resource budgets, scheduling, and backpressure

Existing preview and capture workers are unchanged. Snapshot does no blocking device I/O and uses
directory-stamp caching to avoid full 4 Hz rescans.

## External boundaries and workers

Preview status comes from the newest accepted `CheckerboardPreviewWorker` result. Capture inventory
uses existing left/right files and `load_calibration_pair_usage()`.

## Results, errors, timeout, retry, and cancellation

Counts are integers when inventory is READY/EMPTY and `None` when unreadable. Auto waiting/duplicate
does not increment; partial saves are K,D-only; no saved side or worker failure increments failure.
Late completion is reflected in persistent inventory regardless of Auto Stop.

## Persistence, restart, and shutdown

Snapshot/run counters are not persisted. Files and capture metadata remain unchanged. Restart scans
the active root. No new shutdown resource exists.
