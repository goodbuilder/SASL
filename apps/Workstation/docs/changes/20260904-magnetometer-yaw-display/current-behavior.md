# Current Behavior

## Version and worktree context

- Workstation root: `/home/moontaehyung/바탕화면/SASL-20260821/apps/Workstation`.
- Current baseline ref: `3288aee`.
- The worktree contains extensive pre-existing user-owned changes, including all directly
  relevant Workstation files and tests. This packet uses narrow additive edits only.
- Existing active packets are unrelated; this packet owns only the per-MAG yaw presentation.

## Reproduction

1. Open `Devices > Pixhawk` and inspect `IMU magnetometer (MAVLink)` or
   `CAN magnetometer (dedicated MAVLink)`.
2. Observe xmag/ymag/zmag, norm, state, age, and topic/device evidence.
3. Change a `CAL_MAG*_PRIO` through the existing guarded PX4 calibration workflow, verify the
   active device, and read the separate ATTITUDE yaw manually.

## Observed frontend behavior

The Workstation tables show three fixed IMU stream rows per Pixhawk and two fixed dedicated CAN
rows per Pixhawk. They have no yaw column. Clicking a row opens an Inspector snapshot containing
the raw vector, norm, freshness, producer-axis contract, verdict, and source.

## Observed backend and persistence behavior

`EvidenceAdapter.magnetometers()` projects standard `SCALED_IMU*` xmag/ymag/zmag in producer
sensor axes and mG; `can_magnetometers()` projects only dedicated CAN telemetry in producer axes
and G. The adapter deliberately performs no NED/ENU/FRD conversion. `EvidenceAdapter.attitude()`
projects the current PX4 ATTITUDE roll/pitch/yaw, while the panel tracks
`latest_vehicle_mag_device_ids` for the active selected device. Standard SCALED_IMU rows do not
carry a proven physical device ID; dedicated CAN rows may carry one.

PX4's local source applies the selected magnetometer calibration correction and sensor-to-body
rotation before publishing `vehicle_magnetometer`; EKF yaw uses the selected/fused data together
with attitude/tilt state. Therefore `ATTITUDE.yaw` is the authoritative selected/estimator
reference, but it is not a direct field inside each raw `Magn` vector.

## P2 USB hardware evidence relevant to the amendment

The read-only P2 USB capture identified standard `SCALED_IMU/2/3` samples as BMM150, GPS1
IST8310, and GPS2 IST8310. It identified two additional RM3100 instances on the dedicated CAN
path. The active `vehicle_magnetometer` at capture time was GPS2, so the RM3100 rows must not
inherit PX4 yaw merely because they are present; the existing exact-device correlation gate must
decide whether PX4 yaw is shown.

## Documentation, code, test, and runtime conflicts

The existing product documents correctly require producer frame/unit clarity and explicitly say
the current raw MAG presentation has no coordinate conversion. The requested display must preserve
that contract. A host `atan2` value is only a sensor-axis diagnostic unless identity, calibration,
rotation, timing, and estimator context are all proven; displaying it as an unconditional physical
heading would conflict with the current contract.

## Stable-firmware readback gap

The current stock/stable PX4 captures contain `SCALED_IMU*` samples and no named `DEBUG_VECT`
`CAN1_MAG`/`CAN2_MAG` samples. The Workstation now uses the existing PX4 Shell owner to issue
`listener sensor_mag -i 0..4 -n 1` and `listener vehicle_magnetometer -n 1`, filters known RM3100
IDs, and feeds the bounded readback into the RM3100 rows. Ethernet/Radio Shell timeouts and
malformed/partial output remain failure states; no claim of high-rate RM3100 MAVLink telemetry is
made.
