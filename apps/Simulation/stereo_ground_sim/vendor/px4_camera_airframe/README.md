# Camera Pixhawk PX4 airframe (4090_camera_px4)

`4090_camera_px4` is a copy of PX4-Autopilot's stock
`ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500` (the target drone's
airframe, `PX4_SYS_AUTOSTART=4001`), with two additions:
`param set-default EKF2_MAG_TYPE 1`, and a `mavlink start`/`mavlink stream`
pair that pushes this camera's ATTITUDE to the operator rqt panel (see
"Operator MAVLink ATTITUDE diagnostics link" below).

## Why a separate file

The left/right stereo camera Pixhawks (`left_stereo_camera`/
`right_stereo_camera` in `worlds/stereo_ground_drone.sdf`) are fixed mounts
that never maneuver, unlike the target drone which actually flies. EKF2's
default `MAG_TYPE=Automatic` switches to 3-axis magnetometer fusion once
considered in-flight, which also tries to estimate a body-fixed hard-iron
bias state -- but that state is only observable if the vehicle's attitude
actually changes. A camera that never maneuvers can leave it unstable
(`estimator_sensor_bias.mag_bias_stable=0`). `EKF2_MAG_TYPE=1` keeps these
fixed cameras on 2D magnetic-heading fusion rather than 3-axis fusion. It
does not repair a malformed simulated sensor vector: the Gazebo ENU -> sensor
FLU -> PX4 body-FRD contract is prepared and build-stamped separately by
`run_operator_view.sh`.

Rather than branching on `$px4_instance` inside the target drone's own
`4001_gz_x500` airframe, the two camera Pixhawks boot on this separate
`PX4_SYS_AUTOSTART=4090` airframe instead, so the target drone (which does
maneuver, and could legitimately benefit from hard-iron bias learning) stays
on `Automatic` unmodified.

## Operator MAVLink ATTITUDE diagnostics link

`operator_panel.py`'s Sensors tab shows a "MAVLink ATTITUDE (raw)" row next
to "Published VehicleAttitude (actual)" as an independent cross-check
(`Δyaw ROS2-MAVLink`): one path reads the ROS2 `VehicleAttitude` topic, the
other reads a raw MAVLink `ATTITUDE` message, so a bug specific to either
path is visible as a nonzero delta instead of silently agreeing with itself.
That raw stream previously never arrived: PX4-Autopilot's own
`px4-rc.mavlink` only opens per-instance GCS/offboard/onboard links on ports
that are each a function of `$px4_instance` (e.g. `18570+instance`,
`14580+instance`), never the fixed `udpin:0.0.0.0:14551` the panel's
`MavlinkAttitudeReader` listens on.

This file adds one more `mavlink start` instance, on a local port
(`14650+$px4_instance`, distinct from `px4-rc.mavlink`'s own port
allocations) that pushes to remote port `14551` -- the same port for both
camera instances, since UDP happily accepts multiple senders on one
receiving port and `MAV_SYS_ID` (set to `$px4_instance+1` by `rcS`: left=2,
right=3) is exactly how the panel's `MAVLINK_ATTITUDE_SIDE_BY_SYSID` already
tells them apart. `mavlink stream -s ATTITUDE` then requests just that one
message at 50 Hz, instead of a full stream-mode preset.

## What this does *not* do

The system PX4-Autopilot checkout's `4001_gz_x500` is untouched (reverted
back to stock). `run_operator_view.sh` copies this file into
`PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4090_camera_px4`
before every `make px4_sitl_default` build (PX4-Autopilot is gitignored, so
nothing there is committed by SASL git), and passes
`PX4_SYS_AUTOSTART=4090` only for the left/right camera PX4 instances; the
target instance (0) still gets `4001`.
