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

The launcher carries an independent copy of the reference world under
`worlds/` and the required `wiris_camera` and `x500_15m` model assets under
`models/`. These are copied inputs, not runtime imports from `Simulation`.
The copied world has no `target_reference_link`; standalone evaluation
therefore reports the `x500_15m` model as `MODEL_POSE_FALLBACK`, preserving the
Ground Truth hierarchy instead of relabeling the model as an authoritative
link.

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
- `visualization/markers` — atomic `visualization_msgs/MarkerArray` for the
  read-only RViz scene.
- `diagnostics` — standard `diagnostic_msgs/DiagnosticArray` generated from the
  same validated evaluation sample.

The UI adapter consumes the existing JSON and `PoseStamped` outputs. It does
not recalculate pose errors, perform WGS84 conversion, or issue pose commands.
The core has no independent sequence ID in this Phase, so
`evaluation_timestamp_ns` is used as the sample identity. If the three JSON
outputs or the actual pose timestamps cannot be paired, rqt shows
`PAIR ID MISMATCH`/`TIMESTAMP MISMATCH` and RViz deletes the prior marker set.

## Run

Build this package in a ROS 2 workspace that contains `px4_msgs`, source that
workspace, and run:

```bash
ros2 launch pose_simulation pose_simulation.launch.py
```

To launch the core with the read-only UI adapter, rqt, and RViz:

```bash
ros2 launch pose_simulation pose_simulation_ui.launch.py
```

The UI roles are intentionally separate:

- rqt (`PoseSimulationPlugin`) shows status, validity, source provenance,
  numeric errors, time alignment, diagnostics, and bounded trends in the five
  tabs `Overview`, `Pose Error`, `Stereo Geometry`, `Reference & Time`, and
  `Trends`.
- RViz uses `gazebo_world_enu` as Fixed Frame and consumes
  `/simulation/pose_simulation/visualization/markers` for actual optical poses,
  GT/RTK ideal poses, target/LOS rays, baseline geometry, and error labels.
  The checked-in display config is `config/pose_simulation.rviz`; marker
  namespaces separate `actual`, `ideal_gt`, `ideal_rtk`, `rays`, `error`,
  `baseline`, `target`, and `text` so each group can be toggled independently.

RTK markers are deleted when the current RTK sample is invalid or stale; a
previous valid RTK marker is never silently presented as current. The rqt
trend pause button pauses only display accumulation. The UI always displays
`DIAGNOSTIC ONLY — NO POSE COMMAND`; it has no pose controls, Interactive
Markers, `/set_pose` caller, or correction path.

UI launch arguments include `start_pose_simulation`, `start_visualization`,
`start_rqt`, `start_rviz`, `namespace`, `rviz_config`, `ui_config`, and
`visualization_config`. The checked-in UI config leaves all pass/fail
thresholds unspecified, so values are never labeled PASS by assumption. If a
project-approved warning/failure value is added under `ui_thresholds`, rqt
shows the configured policy without changing PoseSimulation calculations.

For the same build-and-run workflow as the Simulation entrypoints, use the
independent launcher from this directory:

```bash
./run_pose_simulation.sh
```

It builds only `PoseSimulation`, sources `/opt/ros/${ROS_DISTRO:-humble}` and
the optional `POSE_SIMULATION_ROS_WORKSPACE` overlay, then starts the node,
PoseSimulation visualization, the `PoseSimulation` rqt plugin, RViz, and the
copied Gazebo world with server + GUI. Use
`--no-build` when the local install is already current, or `--node-only` for
the diagnostic node without GUIs. `--gazebo-gui-only` opens only the client
against an already running server. The launcher never starts, imports,
modifies, or sends pose commands to `Simulation`.

Before a real evaluation, set the WGS84 reference origin, altitude datum, and
nominal stereo extrinsics in `config/pose_simulation.yaml`. The checked-in
zero origin is a placeholder and is not a physical survey result.
