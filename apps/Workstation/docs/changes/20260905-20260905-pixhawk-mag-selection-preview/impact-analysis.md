# Impact Analysis

## Preserved invariants

- No automatic priority changes or slot/calibration remapping.
- One PARAM/Shell receive owner per Pixhawk side remains unchanged.
- CAN2 remains editable when outside the stock selector range; only a proven v1.17 sparse-voter
  gap in the active in-range priority candidate blocks Apply.
- Active selection success still requires fresh PX4 device identity and ATTITUDE evidence.
- Apply never saves or reboots PX4.

## Affected layers and components

- Domain: `workstation_core/px4_mag_manager.py` selection/topology preview contract.
- Device boundary: `standalone_gui.py` retains the parsed all-instance Shell snapshot.
- Workspace: `workspaces.py` renders candidate/actual topology and updates it in place.
- Workspace editor: `MagnetometerView` owns non-persistent per-side/per-slot drafts; readback
  updates only clean controls, and explicit Discard restores confirmed values.
- The concurrent Qt Designer packet now supplies the MagnetometerView widgets; this follow-up
  changes the view's signal/state behavior and leaves the `.ui` form/resource owner unchanged.
- PARAM boundary: `wire_commands.py` retries only a missing PARAM_VALUE response; worker final
  readback and failure recovery expose partial writes.
- Tests: domain preview, workspace payload/format, and existing active verification tests.
- Product docs: Pixhawk workflow and workspace layout priority sections.

## Feature dependency, ownership, and coupling changes

The Shell owner remains responsible for live sensor instances. The PARAM worker remains the
sole owner of CAL_MAG PARAM reads/writes. The Pixhawk workspace consumes both read models through
the existing panel boundary; draft state is view-local and never persisted. The controller
continues to sequence jobs and invalidate stale readback after apply failure. Device and socket
ownership do not change.

## Worker CPU/GPU resource impact

## Compatibility and migration

Existing settings and CAL_MAG values are untouched. Firmware versions without a live Shell
topology still render PARAM/actual fields and leave the preview waiting. No parameter migration.

## Rollback and recovery

## Risks and assumptions

Radio SERIAL_CONTROL responses can be partial or delayed, so preview identity is labeled as
topology readback and actual selected identity remains authoritative. Priority prediction cannot
prove health/confidence tie-breaking; Apply & Verify remains the final evidence.
