# Camera Pixhawk PX4 airframe (4090_camera_px4)

`4090_camera_px4` is a copy of PX4-Autopilot's stock
`ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500` (the target drone's
airframe, `PX4_SYS_AUTOSTART=4001`), with one addition:
`param set-default EKF2_MAG_TYPE 1`.

## Why a separate file

The left/right stereo camera Pixhawks (`left_stereo_camera`/
`right_stereo_camera` in `worlds/stereo_ground_drone.sdf`) are fixed mounts
that never maneuver, unlike the target drone which actually flies. EKF2's
default `MAG_TYPE=Automatic` switches to 3-axis magnetometer fusion once
considered in-flight, which also tries to estimate a body-fixed hard-iron
bias state -- but that state is only observable if the vehicle's attitude
actually changes. A camera that never maneuvers leaves it stuck wherever it
happened to be after the brief startup transient (confirmed via
`estimator_sensor_bias`: `mag_bias_stable=0`, non-zero `mag_bias[0..2]`),
which was the source of a steady, per-camera heading bias feeding the
Attitude/Full Ray triangulation. `EKF2_MAG_TYPE=1` (Magnetic heading) fuses
only 2D heading and never attempts hard-iron bias learning, avoiding this.

Rather than branching on `$px4_instance` inside the target drone's own
`4001_gz_x500` airframe, the two camera Pixhawks boot on this separate
`PX4_SYS_AUTOSTART=4090` airframe instead, so the target drone (which does
maneuver, and could legitimately benefit from hard-iron bias learning) stays
on `Automatic` unmodified.

## What this does *not* do

The system PX4-Autopilot checkout's `4001_gz_x500` is untouched (reverted
back to stock). `run_operator_view.sh` copies this file into
`PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4090_camera_px4`
before every `make px4_sitl_default` build (PX4-Autopilot is gitignored, so
nothing there is committed by SASL git), and passes
`PX4_SYS_AUTOSTART=4090` only for the left/right camera PX4 instances; the
target instance (0) still gets `4001`.
