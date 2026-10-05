# Impact Analysis

## Preserved invariants

OpenCV optical RDF/pixel contract, native lossless saved raster, timing/NAL/pose gates, one capture
worker slot, explicit Photo set selection and Apply quality gates remain unchanged.

## Affected layers and components

Calibration session inventory, calibration workflow controller snapshot, backend facade/protocol,
EvidenceAdapter, CalibrationWorkspace presentation, tests and Calibration operator documents.

## Feature dependency, ownership, and coupling changes

Calibration controller remains the sole capture status owner. Workspace consumes a public
read-only snapshot and existing preview signal; it does not read another feature's private state.

## Worker CPU/GPU resource impact

No new worker, queue, decoder or GPU demand. A second visible projection renders the already
produced 2 Hz overlay; inventory is cached and invalidated by root/directory stamps and completion.

## Compatibility and migration

One additive facade and ephemeral mapping schema. No setting or persisted capture schema changes;
legacy photo sets are read through existing pair-usage rules.

## Rollback and recovery

Remove the new snapshot/facade/widgets/docs/tests. Persisted calibration files remain untouched.

## Risks and assumptions

Active timing/NAL packets may change waiting text; the snapshot carries their authoritative text
without altering gates. Real-camera cadence and readability require hardware/manual confirmation.
