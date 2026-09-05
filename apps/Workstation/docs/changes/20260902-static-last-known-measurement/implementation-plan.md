# Implementation Plan

## Approval status

Approved by the user on 2026-09-02. Implementation may proceed after the README target is
confirmed.

## Slices and dependencies

1. Add/normalize per-side process-local last-valid attitude selection beside the existing GPS
   cache and remove the current-Measurement-only attitude rejection.
2. Route readiness and attempt construction through the same selected values without changing
   payload fields or timestamps.
3. Add focused pure/controller tests for normal, stale, disconnect, invalid, replacement,
   source-change, restart-equivalent, and evidence-preservation cases.
4. Verify ready-method UI projection without adding presentation elements.
5. Update lasting product docs, run focused and relevant regression suites, then launcher/runtime
   checks.

## File and component map

- `measurement_view_controller.py`: selection/cache/readiness and attempt handoff.
- `tests/test_measurement_methods.py`: readiness/cache tests.
- Other focused evidence/UI tests only if the handoff requires them.
- `docs/ui/01_USER_WORKFLOW.md`, `06_STATUS_MODEL.md`, `07_COMMAND_MODEL.md`, and audit amendment.
- `README.md` only if user-facing behavior documentation is not fully covered by numbered docs.

## Migration and rollback steps

No migration. Rollback restores current-Measurement attitude rejection and removes the new
process-local attitude cache; existing GPS/evidence formats remain compatible.

## Stopping conditions

Stop and return to approval if implementation requires a new UI control/status, persistent
setting, evidence-schema field, device-owner mutation, timestamp rewrite, coordinate conversion,
or broader refactor.
