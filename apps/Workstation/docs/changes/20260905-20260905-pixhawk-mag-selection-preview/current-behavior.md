# Current Behavior

## Version and worktree context

The Workstation uses the `workspaces.py` Pixhawk `MAG Priority / active selection` group and
the shared `Px4MagManager`/`PixhawkParameterWorker` path. The worktree already contains the
completed 20260904 magnetometer-yaw display change and substantial unrelated user changes;
this packet owns only the new selection-preview slice.

## Reproduction

With CAN1 connected, the stable PX4 1.17 runtime publishes five raw magnetometer instances:
internal `4395041`, GPS1 `396809`, GPS2 `396817`, CAN1 RM3100 (`8944899` on P1 or `8944643`
on P2), and CAN2 RM3100 `8944387`. CAN2 is instance 4, while stock PX4's vehicle selector
only consumes instances 0..3.

With CAN1 removed, CAN2 becomes instance 3. The working field configuration used minimum
priority 1 for preceding slots and priority 100 for CAN2, then verified
`vehicle_magnetometer.device_id=8944387` and live ATTITUDE.

## Observed frontend behavior

The current cards expose five role priority combos and an Apply & Verify button. They show
PARAM/active verification status, but not the raw instance-to-device map or the selection
candidate implied by the current combo values. The controls are not topology-blocked, which
is preserved by this change.

## Observed backend and persistence behavior

`_run_mag_apply()` writes only the `CAL_MAG{slot}_PRIO` associated with the currently read
CAL_MAG device IDs. `build_mag_status()` computes the highest positive CAL_MAG priority but
does not know live `sensor_mag` instance indices. Shell parsing already receives all instances
with `known_only=False`, but the instance map is not retained as a public UI read model.

## Documentation, code, test, and runtime conflicts

The older `Px4MagManager.use_rm3100_for_yaw()` helper sets non-target slots to zero; this
packet does not change that legacy helper's command semantics. The new UI preview must not
claim that PARAM priority alone proves active selection. Existing tests cover active-device
verification, but not five-instance topology prediction or the post-removal instance-3 case.

## 2026-10-12 follow-up baseline

The visible Pixhawk workspace refreshes at the 4 Hz status cadence. When cached MAG status exists,
`_refresh_mag_priority_group()` writes cached slot priorities into every combo on each refresh.
`mag_priority_payload()` then reads those same combo values when Apply is invoked. There is no
staged/dirty editor state, so a changed value can be silently reverted before Apply. The existing
UI test reads the payload immediately after an edit and does not refresh between those actions.

The integer PARAM write sends once and returns `None` when no matching `PARAM_VALUE` arrives;
unlike PARAM reads, it has no retry. The worker stops at the first mismatch, so earlier slots can
remain changed. The Apply failure handler leaves cached MAG status intact, which can present old
values after a partial write. MAVLink specifies retrying PARAM_SET when its response times out.

The existing PX4 v1.17 preview predicts highest priority but does not report the sparse voter
failure when a higher `sensor_mag` instance is enabled while a known lower instance is priority 0.
The authoritative P1/P2 notes document this condition and the `Min (1)` filler workaround. No
current hardware session was opened for this follow-up; firmware/runtime acceptance remains
hardware-manual and outside this code-only authorization.

## Concurrent UI form migration observed during implementation

While this follow-up was in progress, the active `20261012-qt-designer-ui` packet converted the
Pixhawk MagnetometerView markup to the shared `pixhawk_magnetometers.ui` form. The MAG fix now
binds draft/busy behavior to that form's existing controls; it does not edit the form resources or
the other packet's Settings/MainWindow work. The focused Qt tests ran against this form-based
worktree. Live visual review of the migration remains unverified.
