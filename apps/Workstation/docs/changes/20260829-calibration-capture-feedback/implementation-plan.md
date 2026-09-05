# Implementation Plan

## Approval status

Approved 2026-08-29. Implementation is allowed for this packet only.

## Slices and dependencies

1. Pure current-root inventory and edge-case tests.
2. Controller-owned run state and additive snapshot contract.
3. Backend/protocol/evidence handoff.
4. Primary preview and fixed feedback rows.
5. Documentation, focused/full regression, screenshots and application smoke.

## File and component map

Primary code: `calibration_sessions.py`, `calibration_controller.py`, `standalone_backend.py`,
`protocol.py`, `evidence.py`, `workspaces.py`. Tests stay in existing Calibration/contract suites.

## Migration and rollback steps

No migration. Roll back this packet's additive code/docs/tests only; do not rewrite user data.

## Stopping conditions

Stop for new capture policy, persisted schema, worker/resource allocation, destructive operation,
or cross-feature ownership change; those require amended approval.
