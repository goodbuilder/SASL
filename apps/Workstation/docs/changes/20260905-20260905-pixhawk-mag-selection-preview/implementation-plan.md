# Implementation Plan

## Approval status

Approved by user in conversation on 2026-09-05. Scope: selection preview and topology read model;
no CAN2 blocking, no automatic priority/slot/calibration mutation.

## Slices and dependencies

1. Add pure selection-preview domain model and tests.
2. Retain all parsed `sensor_mag` instance identities in the panel read model.
3. Add bounded P1/P2 preview rendering and combo-change updates.
4. Run focused tests, launcher checks, and inspect the real Workstation startup boundary.

## File and component map

- `workstation_core/px4_mag_manager.py`: preview contract and role/device helpers.
- `standalone_gui.py`: per-side live sensor instance snapshot.
- `workspaces.py`: preview controls/rendering.
- `tests/test_pixhawk_magnetometer_workspace.py` and a focused domain test: evidence.
- `docs/ui/01_USER_WORKFLOW.md`, `docs/ui/03_WORKSPACE_LAYOUT.md`, README: lasting contract.

## Migration and rollback steps

No data migration. Rollback is source revert; existing PARAM values and manual physical topology
remain operator-owned.

## Stopping conditions

Stop if implementing the preview requires automatic CAL_MAG ID remapping, a new telemetry
protocol, or a firmware change; those are outside this packet.
