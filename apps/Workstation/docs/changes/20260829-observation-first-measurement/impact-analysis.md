# Impact Analysis

## Preserved invariants

One owner per device/worker; generation isolation; no path overwrite; exact result gates; all
NED/FRD/RDF, units, signs and timestamp provenance remain unchanged.

## Affected layers and components

Run controller, camera/recording lifecycle, data-store retry reserve, command enablement, status
projection, additive event/artifact schema and product documentation.

## Feature dependency, ownership, and coupling changes

Run owns only the observation interval and source read model. Device owners publish transitions;
Recording owns segment paths; DataStorage owns retry. No widget accesses worker internals.

## Worker CPU/GPU resource impact

No new decoder/encoder or full-file verification worker. Retry reserve is capped at 256 records and
8 MiB. Existing YOLO priority and decoded-recorder backpressure remain.

## Compatibility and migration

Additive snapshots/JSONL fields only; no existing byte rewrite or startup scan.

## Rollback and recovery

Revert source-transition admission and segment/retry additions. Produced MP4/JSONL remain ordinary
self-describing evidence.

## Risks and assumptions

RUNNING can contain no usable source and therefore no valid result. `SAVED` proves existence only.
Process crash, OS failure and OOM cannot be made observable by an in-process command.
