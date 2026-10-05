# Implementation Plan

## Approval status

Approved by the user on 2026-08-29 after the decision-complete plan.
Software implementation and automated/visual verification are complete. Real post-restart
MAVLink/LIVE topology checks remain in `verification.md`.

The 2026-08-29 explicit NetworkManager role-application amendment was approved and implemented.
Pure/controller/UI/full-regression and visual checks pass; real mutating one-/multi-NIC device
moves remain the field verification boundary.

## Slices and dependencies

1. Pure candidate/route/assignment contracts and fixtures.
2. Additive settings persistence and legacy migration.
3. Bounded discovery worker and controller route preflight/partial connect.
4. Facade/evidence and Pixhawk workspace workflow.
5. Focused tests, product docs, full regression and visual/hardware evidence.

Amendment slices after revised approval:

1. Pure role-set union/delta, stable identity, transaction and rollback contracts.
2. Additive role-intent/verified-state persistence and legacy migration that never adopts unrelated
   NetworkManager entries as Workstation-owned.
3. Bounded privileged NetworkManager adapter with exact snapshot, app-owned delta, dual readback,
   cancellation/timeout and compensating rollback.
4. One coordinator using public Camera/Pixhawk owner contracts to snapshot, stop, apply, reconnect
   and verify affected owners while unrelated NICs continue.
5. Native confirmation, bounded progress/failure/recovery presentation and stable candidate-role
   table/side summaries.
6. Focused fake/real-boundary tests, full regression, launcher/visual checks and one-/multi-NIC
   hardware moves with active Camera/Pixhawk and rollback evidence.

## File and component map

- `workstation_core/network.py`: read-only discovery and route oracle.
- `device_runtime_workers.py`, `device_runtime_controller.py`: worker/orchestration.
- settings/facade/evidence/workspace and focused tests/docs.

The amendment may extend the network core/worker/controller and settings/evidence/workspace owners,
plus narrow application coordinator/facade contracts for existing Camera/Pixhawk stop/start. No
Camera decoder, MAVLink parser, PX4 protocol, Measurement algorithm or unrelated worker rewrite is
authorized.

## Migration and rollback steps

Add default-empty assignment fields. Do not rewrite settings until operator Assign. Rollback removes
fields/preflight/UI while leaving existing URL/peer settings valid.

The amendment adds version-tolerant role intent and Workstation-owned entry provenance. Existing
manual NetworkManager profiles remain external and are preserved. Software rollback disables the
new Apply path and retains completed read-only Find/Assign/Connect; it never bulk-restores or
deletes unknown host profiles.

## Stopping conditions

Stop and return to amended approval on any need to mutate PX4, weaken liveness/sysid/RTCM or
recording locks, rewrite Camera/Pixhawk private internals, remove unrelated host settings, apply at
startup without confirmation, or broaden beyond source/target role owners.
