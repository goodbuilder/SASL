# PoseSimulation

`PoseSimulation` is an independent ROS 2 application for Phase 1 stereo-pose
diagnostics. It observes Gazebo poses and a PX4 raw GNSS position, converts the
inputs into an explicit coordinate contract, derives ideal rigid-parallel
stereo poses, and reports pose, optical-axis, relative-camera, and baseline
errors.

The application does not modify the `Simulation/` application. It has no
runtime import from `stereo_ground_sim`; existing Simulation topics and source
formats are only input contracts. `PoseSimulation` does not call `/set_pose`,
publish attitude commands, or apply a correction.

## Source hierarchy

- `target_reference_link` is the only absolute Ground Truth source.
- `vehicle_reference_link` is a configured fallback and is reported as such.
- A configured model-pose fallback is diagnostic only and is not equivalent to
  the target reference link.
- `/fmu/out/vehicle_gps_position` is an RTK operational reference candidate.
  Current `px4_msgs` exposes this topic as `SensorGps`; the node also accepts
  `VehicleGpsPosition` when available. `VehicleGlobalPosition` is not used as
  the RTK position input.
- Actual camera poses are observed optical RDF world poses from the camera
  diagnostic JSON fields, never the payload's desired pose fields.

The current Gazebo `optical_link` entity is authored in SDF FLU axes. The
verified fixed `R_link_from_optical` transform is therefore applied before an
actual pose is labeled `camera_optical_rdf`; the entity quaternion is not
treated as an RDF quaternion by identity assumption.

The node can also read raw Gazebo `Pose_V` streams directly with
`enable_gazebo_raw_pose_streams: true`. Configure the exact target reference
entity, parent model, and both optical entities before enabling that path; the
parser composes scoped link poses with the parent model world pose and retains
the Gazebo source stamp.

## Coordinate and reporting contract

The world frame is `gazebo_world_enu` (East, North, Up, metres), PX4 local is
NED, the stereo rig is `baseline_center_frd` (Front, Right, Down), and camera
optical frames are RDF (Right, Down, Forward). Rotations are named `R_B_A` and
map components from A into B. Quaternion boundaries use `xyzw`.

The reported Euler angles use fixed extrinsic Z-Y-X (`Rz(yaw) Ry(pitch)
Rx(roll)`, equivalent to intrinsic X-Y-Z) in degrees. Rotation-angle results
come from the clamped trace formula; Euler values are reporting fields only.
World Up defines the look-at roll reference. Near-vertical lines of sight use
the configured projected fallback axis and record the selected policy.

The default rig center is the midpoint of the two observed optical centers.
Consequently, an independent midpoint position error is reported as
`not_available` unless an authoritative nominal center is added to the
configuration. The default target alignment records the explicit
`zero_lever_arm_assumption`.

## Topics

All outputs are under `/simulation/pose_simulation`:

- `pose_error` — actual-vs-GT, actual-vs-RTK, and RTK-ideal-vs-GT-ideal pose and
  optical-axis diagnostics as self-describing JSON.
- `stereo_geometry` — relative stereo attitude and baseline diagnostics as
  self-describing JSON.
- `status` — source provenance, validity, timestamp policy, frame contract,
  and RTK state as self-describing JSON.
- `ideal_pose/ground_truth/{left,right}` and `ideal_pose/rtk/{left,right}` —
  ideal camera optical `PoseStamped` messages when the corresponding input is
  valid.
- `ideal_pose/{ground_truth,rtk}/rig` — ideal `baseline_center_frd` pose
  messages for the same two sources.
- `actual_pose/{left,right}` — observed optical `PoseStamped` re-publication
  for inspection only.

## Run

Build this package in a ROS 2 workspace that contains `px4_msgs`, source that
workspace, and run:

```bash
ros2 launch pose_simulation pose_simulation.launch.py
```

Before a real evaluation, set the WGS84 reference origin, altitude datum, and
nominal stereo extrinsics in `config/pose_simulation.yaml`. The checked-in
zero origin is a placeholder and is not a physical survey result.
