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
