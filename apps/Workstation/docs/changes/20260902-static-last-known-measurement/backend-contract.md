# Backend Contract

## Commands and validation

No command is added. Existing measurement readiness requires exact operating K,D, finite retained
or live left/right attitude, valid retained or live 3D GPS positions, and a finite positive NED
baseline.

## State, ownership, and concurrency

The Measurement owner consumes immutable last-valid samples from the existing Pixhawk/GNSS
owners. It does not connect devices or overwrite their authoritative live status. A valid newer
sample atomically supersedes its retained predecessor for subsequent pairs.

Attitude and GPS caches are per-side and process-local. A source-role selection change invalidates
the affected GPS cache before accepting values from the new source. Cache objects preserve their
original timestamps and device identity.

## Worker resource budgets, scheduling, and backpressure

No worker, thread, process, queue, GPU allocation, or backpressure policy changes.

## External boundaries and workers

Existing Pixhawk/GNSS receiver read models remain authoritative. Measurement owns only immutable
last-valid references/copies used for readiness and pair construction.

## Results, errors, timeout, retry, and cancellation

Explicit disconnect or freshness timeout after one valid sample selects retained-static input
automatically. No prior valid sample, invalid timestamp, non-finite position, or non-positive
P1-to-P2 baseline remains a hard rejection. A valid newer sample returns only its affected
component to live state; mixed live/retained component state must remain explicit.

## Persistence, restart, and shutdown

Retained values are process-local and cleared on shutdown/restart. Historical evidence is never
loaded as active geometry. No new retained-mode marker, age field, or dedicated fallback event is
added. Existing exact attitude/GPS payloads, real timestamps, method identity, and geometry hashes
remain unchanged; their removal or rewriting is outside the acceptable evidence contract.
