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

## 2026-09-03 corrected implementation plan

1. Add a GNSS-only horizontal baseline-to-midpoint-Drone angle property and absolute 90° error to
   the existing GNSS baseline-center read mapping; use `atan2(|Forward|, |Right|)`.
2. Add independent cardinal/90°/zero-vector tests and preserve the existing F/R/D/Rg record
   contract.
3. Add the two read-only Telemetry table columns, fixed formatting, tooltips, and stable geometry
   checks for CURRENT/STALE/WAITING and long values.
4. Remove the unneeded Drone ATTITUDE/yaw amendment from this packet's presentation/read model;
   keep the existing P1/P2 yaw diagnostic separate and unchanged.
5. Update the Workstation README, numbered UI contracts, and this packet; run focused, dependent,
   full Workstation, and launcher verification.
