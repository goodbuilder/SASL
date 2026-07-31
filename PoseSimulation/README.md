# PoseSimulation

`PoseSimulation` is an independent ROS 2 application for read-only stereo-pose
diagnostics. In M3, its primary `pose_simulation` node consumes the external
Ground Truth `PoseStamped` and immutable left/right optical-pose snapshots,
derives the GT ideal rigid-parallel rig, and publishes typed attitude,
optical-axis, relative-stereo, and baseline measurements. RTK, correction, UI,
and pose-command paths are not part of this node.

The application does not modify the `Simulation/` application. It has no
runtime import from `stereo_ground_sim`; existing Simulation topics and source
formats are only input contracts. `PoseSimulation` does not call `/set_pose`,
publish attitude commands, or apply a correction.

## Configuration phase

`PoseSimulation/config/posesimulation.env` is the independent application
configuration contract. It is parsed as data by `pose_simulation.config`; it
is never used to source Simulation-owned settings. The precedence is:

1. the checked-in `posesimulation.env`;
2. an optional `POSESIM_PROFILE_CONFIG` profile;
3. explicit caller `POSESIM_*` environment values;
4. `--set POSESIM_KEY=VALUE` command-line overrides.

The loader validates the read-only boundary, input/output topic ownership,
coordinate-frame contract, time policy, and RTK altitude-datum policy without
importing ROS or starting a source process. Empty metric thresholds mean
`UNSPECIFIED`, not `PASS`.

## M3 source contract

- `/simulation/ground_truth/vehicle_reference_link` is the configured M3
  Ground Truth input. M3 has no model-pose, PX4-pose, or planned-position
  fallback.
- Actual camera poses are observed optical RDF world poses from the camera
  diagnostic JSON fields, never the payload's desired pose fields.

The current Gazebo `optical_link` entity is authored in SDF FLU axes. The
verified fixed `R_link_from_optical` transform is therefore applied before an
actual pose is labeled `camera_optical_rdf`; the entity quaternion is not
treated as an RDF quaternion by identity assumption.

The repository may carry copied world/model assets for other read-only tools,
but the M3 node does not launch Gazebo, parse SDF, or derive a fallback pose
from them. Missing or invalid external GT remains invalid.

Camera snapshots are immutable for one run. An entity identity, SDF hash, or
pose change invalidates that side; the first valid snapshot is never silently
replaced.

## Coordinate and reporting contract

The world frame is `gazebo_world_enu` (East, North, Up, metres), PX4 local is
NED, the stereo rig is `baseline_center_frd` (Front, Right, Down), and camera
optical frames are RDF (Right, Down, Forward). Rotations are named `R_B_A` and
map components from A into B. Quaternion boundaries use `xyzw`.

The M1 pose contract is checked in at `config/pose_contract.yaml`. Optical RDF
error angles use `R_down(yaw) R_right(pitch) R_forward(roll)` in degrees;
rig-level FRD reporting uses `R_down(yaw) R_right(pitch) R_front(roll)`.
Rotation-angle results come from the clamped trace formula; Euler values are
reporting fields only and carry their convention and gimbal-lock status.
World Up defines the look-at roll reference. Near-vertical lines of sight use
only a projected previous Right axis; without one the ideal pose is invalid,
and no arbitrary fallback axis is selected.

The default rig center is the midpoint of the two observed optical centers.
Consequently, an independent midpoint position error is reported as
`not_available` unless an authoritative nominal center is added to the
configuration. The default target alignment records the explicit
`zero_lever_arm_assumption`.

## M3 topics

All M3 PoseSimulation outputs are under `/pose_simulation`:

- `pose_error` — `pose_simulation_interfaces/msg/PoseError`, containing both
  camera actual-vs-GT-ideal RPY/rotation and optical-axis errors.
- `stereo_geometry` — `pose_simulation_interfaces/msg/StereoGeometry`,
  containing relative attitude and baseline geometry.
- `status` — `pose_simulation_interfaces/msg/PoseSimulationStatus`, including
  source identity, contract hash, validity, and the read-only guard.
- `ideal_pose/ground_truth/{rig,left,right}` — standard `PoseStamped` ideal
  poses, all stamped with the same GT evaluation time.

The typed definitions are in the independent `pose_simulation_interfaces`
package. Invalid evaluations publish `valid=false` with NaN metric fields; the
node does not keep a previous valid metric as the current result. M3 does not
publish RTK ideal poses, markers, or `DiagnosticArray` messages.

## M3 runtime

Start Simulation separately, then run PoseSimulation:

```bash
source /opt/ros/humble/setup.bash
source <workspace>/install/setup.bash
ros2 launch pose_simulation pose_simulation.launch.py
```

The node subscribes only to the three configured `/simulation/...` inputs. A
valid GT sample triggers one monotonic `evaluation_id`; the GT source stamp is
the common output stamp and static camera residuals are not treated as
time-varying measurements. `PoseSimulationStatus` always reports
`read_only=true`, `pose_command_enabled=false`, and
`rtk_comparison_implemented=false`.

## Run

### M2 external-input probe

M2 adds a read-only input adapter and evaluation-time alignment probe.  It
does not calculate attitude errors, baseline metrics, RTK results, rqt/RViz
results, or any final measurement output.  It only validates and aligns the
following external interfaces:

- `/simulation/ground_truth/vehicle_reference_link` as
  `geometry_msgs/msg/PoseStamped`, using `header.stamp` and the
  `gazebo_world_enu` frame;
- `/simulation/camera_pose_diagnostics/left` and `/right` as
  `std_msgs/msg/String` with schema `camera_pose_comparison/v1`.

The current camera publisher reads Gazebo `pose_static` and publishes a
reliable, transient-local, depth-one snapshot.  PoseSimulation therefore
uses `POSESIM_CAMERA_POSE_MODE=static_snapshot`: the first valid left/right
optical pose is immutable for the run.  Repeated snapshots must keep the
same entity identity and generated SDF hash; a changed pose, entity, or hash
closes that camera input with an explicit invalid status.  The camera JSON's
observed entity quaternion is converted with the checked-in FLU-to-RDF
extrinsic; desired-pose fields are never used as measurements.

Source timestamps are authoritative measurement times.  ROS receive time is
monotonic bookkeeping only.  Static alignment is triggered by each valid GT
sample, uses that GT timestamp as the evaluation timestamp, and reports
camera time residuals as not applicable.  If the source clock cannot be
established as `gazebo_sim_time`, the sample is invalidated with
`CLOCK_DOMAIN_UNRESOLVED` rather than silently mixing clock domains.
GT receive freshness is checked separately against
`POSESIM_MAX_GT_SOURCE_AGE_MS`; static camera snapshots are not aged out by
this policy.

The supported input-probe entrypoint is:

```bash
cd PoseSimulation
./run_pose_simulation.sh --check-config
./run_pose_simulation.sh --print-config
./run_pose_simulation.sh --check-inputs --timeout-s 5
```

The runner preserves the caller's working directory, resolves the optional
profile path, validates the resolved snapshot, and never starts or stops
Gazebo, PX4, rqt, RViz, or any Simulation process. `--check-inputs` only
subscribes to external ROS 2 topics and returns `SOURCE_UNAVAILABLE` when the
ROS runtime or source application is not present. For a two-application
workflow, start Simulation in its own terminal using its own entrypoint, then
run the PoseSimulation probe in a separate terminal.

The probe uses a reliable/volatile depth-ten subscription for the GT topic and
a reliable/transient-local depth-one subscription for each camera snapshot,
matching the current source publishers.  It has no publisher and no
`/set_pose`, attitude-command, or correction path.

### Existing ROS runtime components

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

- rqt (`PoseSimulationPlugin`) is a read-only operator view with the tabs
  `Overview`, `Camera`, `Pose Error`, `Stereo Geometry`, `Reference & Time`,
  `Trends`, and `Diagnostics`. The header keeps the overall state, evaluation
  identity, simulation time, GT/left/right/frame badges, and a non-interactive
  `Read-only` badge. Invalid or unaligned values are shown as `—`, never as
  zero; raw enum values and nanosecond timestamps remain in `Diagnostics`.
- The Camera tab reads the external Simulation overlay streams
  `/left_camera/detection_image` and `/right_camera/detection_image` as
  `sensor_msgs/msg/Image` with sensor-data QoS. It uses a latest-frame-wins
  buffer and displays an explicit unavailable state when a source is absent;
  PoseSimulation does not create, alter, or overlay the image stream.
- Trends use a 2×2 layout for relative attitude, optical alignment, time
  alignment, and baseline stability. Invalid evaluations insert a visual gap
  and are not converted to zero. The baseline plot switches between metres
  and degrees so unlike units never share one axis.
- RViz uses `gazebo_world_enu` as Fixed Frame and consumes
  `/pose_simulation/visualization/markers` for actual optical poses,
  GT/RTK ideal poses, target/LOS rays, baseline geometry, and error labels.
  The checked-in display config is `config/pose_simulation.rviz`; marker
  namespaces separate `actual`, `ideal_gt`, `ideal_rtk`, `rays`, `error`,
  `baseline`, `target`, and `text` so each group can be toggled independently.

The RViz adapter consumes typed M3 results and the evaluation-stamped,
read-only PoseStamped topics `target/ground_truth`, `actual_pose/left`, and
`actual_pose/right` published by the PoseSimulation measurement node.  These
topics expose validated source data for visualization only; they do not alter
the measurement calculation or source Simulation.  Every MarkerArray is
assembled from one evaluation identity and uses deterministic namespace/ID
pairs.  Invalid or mismatched evaluations emit explicit DELETE markers rather
than retaining stale geometry.  Camera frustums are symbolic orientation
frustums, not calibrated FOV displays, because no public calibrated intrinsic
contract exists.

RTK markers are deleted when the current RTK sample is invalid or stale; a
previous valid RTK marker is never silently presented as current. The rqt
trend pause button pauses only screen refresh; subscriptions and the bounded
buffer continue. The UI displays a small
`Read-only` badge with the tooltip `Diagnostic only — no pose command
publisher`; it has no pose controls, Interactive Markers, `/set_pose` caller,
or correction path.

UI launch arguments include `start_pose_simulation`, `start_visualization`,
`start_rqt`, `start_rviz`, `namespace`, `rviz_config`, `ui_config`, and
`visualization_config`. The checked-in UI config leaves all pass/fail
thresholds unspecified, so values are never labeled PASS by assumption. If a
project-approved warning/failure value is added under `ui_thresholds`, rqt
shows the configured policy without changing PoseSimulation calculations.

Before a real evaluation, set the WGS84 reference origin, altitude datum, and
nominal stereo extrinsics in `config/pose_simulation.yaml`. The checked-in
zero origin is a placeholder and is not a physical survey result.
