# Backend Contract

## Commands and validation

`solve_workstation_optical_pose_gnss_mountain()` remains the authoritative solver. It
continues requiring paired frozen Mountain points, exact operating K,D, and the existing
GNSS baseline/fix gates. It now projects the solved candidate's OpenCV relative R/t as
an additive named mapping derived from the immutable candidate poses.

## State, ownership, and concurrency

The controller owns the candidate and snapshot. The UI never calculates or mutates R/t.
No lifecycle, lock, queue, or concurrency behavior changes.

## Worker resource budgets, scheduling, and backpressure

Not applicable; no worker or resource path changes.

## External boundaries and workers

No external API, device, socket, GNSS, or solver boundary changes.

## Results, errors, timeout, retry, and cancellation

The existing candidate result and failure mapping remain unchanged. The optional display
projection is finite/shape-checked by the domain pose contract; a presentation caller
without the mapping remains compatible.

## Persistence, restart, and shutdown

The additive snapshot/event projection is JSON-safe and does not change settings or
calibration files. Existing append-only Alignment events remain compatible; no migration,
shutdown, or restart behavior changes.
