# Implementation Plan

## Approval status

Approved by the user on 2026-09-01; implementation is authorized for this packet only.

## Slices and dependencies

1. Add and independently test the pure horizontal geometry contract.
2. Add and independently test the source/freshness EvidenceAdapter snapshot.
3. Add the fixed-height plot and Pixhawk workspace values; test normal, partial, stale, waiting,
   rollover, and rapid-refresh geometry stability.
4. Update lasting product/README contracts.
5. Run focused, dependent, full project, offscreen layout, and launcher checks; record residual
   hardware/manual conditions.

## File and component map

- Domain/evidence: `workstation_core/physical_alignment.py`, `evidence.py`.
- Presentation: `telemetry_plots.py`, `workspaces.py`.
- Tests/docs: focused Pixhawk/plot tests, `README.md`, UI product documents, this packet.

## Migration and rollback steps

No migration. Rollback the additive domain/read-model/presentation fields and matching docs/tests.

## Stopping conditions

Stop and return to approval if a new setting, command, persistence/schema field, worker/device
change, measurement-semantic change, or ownership transfer becomes necessary.
