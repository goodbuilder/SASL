# Backend Contract

## Commands and validation

`begin_measurement_run` freezes run identity/time and enters RUNNING before source requests.
Source readiness is never an admission input. Mapping/config/calibration validation remains local.

## State, ownership, and concurrency

Run snapshot owns a source map and video segment list. Workers freeze generation/segment tokens;
late callbacks cannot read the current global run identity. Stop freezes the run boundary before
asynchronous teardown.

## Worker resource budgets, scheduling, and backpressure

No new media worker. Essential retry reserve: FIFO, 256 encoded records and 8 MiB aggregate,
1/2/4/.../30 s retry; overflow rejects newest and increments drops. Video/high-rate bulk is not
buffered by this reserve.

## External boundaries and workers

Busy camera resources auto-join after release. Failed cameras rejoin only through manual side
reactivation and a newer-generation current frame. P1/P2/GNSS/NTRIP/Target connection changes are
allowed; mutations remain blocked.

## Results, errors, timeout, retry, and cancellation

Source timeout marks only that source unavailable. Existing method gates determine results. Camera
rejoin allocates a new non-overwriting MP4 segment. EOS/teardown is bounded; artifact append result
does not control resource release.

## Persistence, restart, and shutdown

Additive generation/source/segment records only. `SAVED` is path existence. No hash, probe, decode,
manifest, catalog, revision, revalidation or restart adoption.
