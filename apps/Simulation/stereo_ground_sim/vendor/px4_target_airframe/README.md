# PX4 SITL target airframe

`4001_gz_x500` preserves PX4 v1.17.0's stock Gazebo x500 parameters and
adds one lockstep-specific setting: `UXRCE_DDS_SYNCT=0`. The preflight run
showed that the target instance retained `UXRCE_DDS_SYNCT=1`, connected to
MicroXRCE-DDS, then never created its output writers. The camera instances
created writers with the parameter set to `0`. The simulation uses Gazebo
lockstep time, and the recorder stores the PX4 timestamp, Gazebo simulation
timestamp, and host monotonic receive time as separate domains; it does not
convert one into another.

`run_operator_view.sh` installs this file as
`PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500`
before the SITL build and copies it into the build output so PX4 instances use
the repository-owned setup even when an incremental ROMFS sync is skipped.
