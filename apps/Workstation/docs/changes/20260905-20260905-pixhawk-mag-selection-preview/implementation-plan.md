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

## 2026-10-12 approved follow-up slices

1. Add pure v1.17 sparse-voter-gap detection to the existing preview contract; preserve the
   `sensor_mag` instance index and CAL_MAG device-ID mapping.
2. Make visible P1/P2 priority combos staged per-side/per-slot editor state. Add explicit
   Discard, preserve drafts across 4 Hz refresh, and disable only the busy side's controls.
3. Enforce a proven gap in the parameter worker before writes; retry missing PARAM_SET ACKs,
   check final readback, and reconcile stale cache after failed/partial application.
4. Update current Workstation/Pixhawk operational docs and regression tests.

Source map: `workstation_core/px4_mag_manager.py`, `workspace_views/pixhawk_parts/magnetometers.py`,
`workspace_views/pixhawk.py`, `workers/device_runtime_workers.py`,
`controllers/device_runtime_controller.py`, `workstation_core/pixhawk_link/wire_commands.py`,
related Workstation tests, `README.md`, `Pixhawk.md`, and `docs/ui/01_USER_WORKFLOW.md` /
`docs/ui/07_COMMAND_MODEL.md`.

The user's “수정해” instruction on 2026-10-12 explicitly approves this follow-up scope; no second
approval gate is needed. Hardware parameter changes, firmware changes, and automatic save/reboot
are excluded.
