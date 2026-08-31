# Implementation Plan

## Approval status

Approved at `2026-08-31T16:15:22+09:00` by the user's `Implement the plan.` request.

## Slices and dependencies

1. Pure generation selection/flattening/Excel serialization and atomic publication.
2. Store read checkpoint and one coalesced cancellable export owner.
3. Measurement stop automatic request and Data Status request/snapshot/open workflow.
4. Restart/shutdown, documentation, load and full regression verification.

## File and component map

- New export core/worker module under `workstation_core`.
- Extend MeasurementDataStore and DataStorageController without changing enqueue return semantics.
- Narrow MeasurementRunController, Data workspace and lifecycle handoffs.
- Focused exporter/store/controller/UI tests plus authoritative product docs.

## Migration and rollback steps

No migration. Existing roots are read only; first publication refuses a foreign same-name file.
Rollback removes the additive feature and leaves every source artifact untouched.

## Stopping conditions

Stop and return to approval if implementation requires blocking Measurement, rewriting JSONL,
adding a dependency, restoring Session ownership, embedding binaries, or changing device/math policy.
