# Implementation Plan

## Approval status

Approved by the user on 2026-08-29 after the decision-complete plan.
Software implementation and automated/visual verification are complete. Real post-restart
MAVLink/LIVE topology checks remain in `verification.md`.

## Slices and dependencies

1. Pure candidate/route/assignment contracts and fixtures.
2. Additive settings persistence and legacy migration.
3. Bounded discovery worker and controller route preflight/partial connect.
4. Facade/evidence and Pixhawk workspace workflow.
5. Focused tests, product docs, full regression and visual/hardware evidence.

## File and component map

- `workstation_core/network.py`: read-only discovery and route oracle.
- `device_runtime_workers.py`, `device_runtime_controller.py`: worker/orchestration.
- settings/facade/evidence/workspace and focused tests/docs.

## Migration and rollback steps

Add default-empty assignment fields. Do not rewrite settings until operator Assign. Rollback removes
fields/preflight/UI while leaving existing URL/peer settings valid.

## Stopping conditions

Stop on any required host network mutation, new PX4 mutation, cross-feature Camera owner access,
or need to weaken liveness/sysid/RTCM gates; those require amended approval.
