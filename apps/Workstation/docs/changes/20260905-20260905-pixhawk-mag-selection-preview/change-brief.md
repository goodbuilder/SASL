# Change Brief

## Operator outcome

On the Pixhawk MAG Priority view, the operator can see which `sensor_mag` instance and
device ID each role represents, set any priority without the UI blocking the action, and
see the priority-based candidate beside the actual PX4 `vehicle_magnetometer` selection.

The screen must remain useful after a GPS or magnetometer is physically removed and the
operator refreshes the topology.

## Motivation

Stable PX4 exposes up to four `vehicle_magnetometer` selector inputs (`sensor_mag` instances
0..3), while a rig with the internal magnetometer, two GPS compasses, and two RM3100s can
publish five raw instances. The existing UI shows role priorities but does not expose the
instance-to-device mapping or the predicted candidate, so a priority write can be mistaken
for an active-source change.

## Included scope

- Add a read-only topology/identity preview to the existing P1/P2 MAG Priority cards.
- Show `sensor_mag` instance, device ID, role, CAL_MAG slot, priority candidate, and actual
  selected device from the existing Shell/PARAM read models.
- Keep all priority controls editable; do not block CAN2 or show a topology rejection dialog.
- Refresh the preview when live Shell topology, PARAM status, actual selected ID, or a combo
  value changes.
- Preserve the existing disarm, PARAM owner, save, reboot, and active-device/ATTITUDE
  verification contracts.

## Excluded scope and non-goals

- No automatic physical sensor removal, GPS selection, or firmware change.
- No automatic priority write, slot remap, calibration reset, or calibration-value migration.
- No change to the PX4 selector or to the meaning of `CAL_MAG*_PRIO`.
- No new generic dashboard or redesign of the Pixhawk workspace.

## Acceptance summary

For each side, the UI visibly distinguishes the requested priority candidate from the actual
PX4 active device. With five raw instances, a CAN2 priority of 100 may show CAN2 at instance 4
while actual PX4 selection remains another in-range sensor; after a physical sensor is removed
and topology is refreshed, the preview reflects the new instance. Existing Apply & Verify still
requires fresh active-device and ATTITUDE evidence for success.
