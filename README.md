You are building a desktop UI prototype for a real-time stereo vision surveillance system.

## Project Goal

Build a UI-first MVP for a real-time stereo vision monitoring system using:

- Two Workswell WIRIS Enterprise cameras
- Only the 30x Optical Zoom visible stream is used
- No thermal stream is used
- Two Pixhawk units
- Pixhawk and WIRIS devices are connected to the PC over Ethernet
- The system will later use ROS2 for sensor and Pixhawk data
- The system estimates target distance, azimuth, and elevation using stereo vision
- YOLO detection will run after left/right camera verification
- Internal camera parameters are measured using checkerboard calibration
- Baseline can be manually entered or derived from GNSS
- The UI must emphasize confidence, synchronization quality, stereo quality, pose quality, and device state

For now, implement a UI prototype with disconnected placeholder state and clean architecture so backend modules can later be connected.

## Technology

Use Python with PyQt6.

Use a modular structure:

uam_stereo_ui/
  main.py
  ui/
    main_window.py
    tabs/
      live_tab.py
      device_tab.py
      calibration_tab.py
      control_tab.py
      events_tab.py
      settings_tab.py
      debug_tab.py
    widgets/
      status_badge.py
      confidence_bar.py
      device_card.py
      video_panel.py
      metric_card.py
  core/
    models.py
    state.py
    config.py

If PyQt6 is not available, use PySide6 with minimal changes.

## Notebook Handoff

The working handoff model is:

1. Push this `SASL` repository.
2. Clone or pull it on the notebook.
3. Recreate local ROS build outputs from tracked scripts instead of
   copying machine-local workspaces.

For the PX4 ROS message overlay used by `StereoWIRIS`, run:

```bash
cd SASL
scripts/bootstrap_ros_workspace.sh
```

This prepares `workspace/main/src/px4_msgs` at the pinned PX4 message
revision and builds the local overlay at `workspace/main/install/setup.bash`.
The source clone and generated overlay are intentionally not tracked by
this repository; they are reproducible local state. The script keeps the
ROS build/log cache under `~/.cache/sasl_ros_workspace/...` so ROS message
generation does not depend on the checkout path being ASCII-only.
`StereoWIRIS/run.sh` and `Simulation/run_operator_view.sh` consume this
single canonical overlay. The Simulation runner validates both the pinned
source revision and the bootstrap-generated overlay commit stamp before
starting; StereoWIRIS applies the same validation. Both print the bootstrap
command when the dependency is missing or stale. Do not create a second
`Simulation/px4_msgs` or `px4_msgs_ws` checkout; those machine-local fallback
workspaces are not runtime inputs.

If a future task requires PX4 firmware/DDS export changes, keep the
patch, submodule, fork reference, or setup script inside `SASL` so the
notebook clone can reproduce the same firmware source state.

## Simulation StereoDistanceNode boundaries

`Simulation/stereo_ground_sim/stereo_ground_sim/stereo_distance_node.py`
owns ROS subscriptions/publications, runtime workers, coordinate transforms,
and Gazebo command I/O.  Deterministic policies are kept outside that ROS node:

- `mission_timeline.py` calculates mission phases and scalar setpoints. Its
  altitude and vertical velocity contract is world ENU (`z` positive up); the
  approach distance is a scenario scalar, not a PX4 NED or camera-frame vector.
- `observation_contract.py` validates observation modes and builds immutable
  pixel/provenance records. Pixels use OpenCV `(u=right, v=down)` order, while
  its declared triangulation output remains baseline-center FRD `[F,R,D]`.
- `pair_evaluation_registry.py` exclusively owns the pair/measurement/world-ENU
  alignment join and bounded evaluated/failed tombstones. Terminal evaluation
  removes all three join entries atomically, preventing late Ground Truth from
  resurrecting a completed pair.
- `research_terminal_coordinator.py` owns per-epoch/pair terminal deduplication
  plus recorder submission/close failure state. Record payload construction
  remains schema-compatible while its lifecycle no longer belongs to the ROS
  node.
- `pair_work_scheduler.py` owns the measurement/GT-alignment ready queues and
  alternates the two work classes so a continuous image stream cannot starve
  delayed Ground Truth evaluation.
- `evaluation_pipeline.py` computes signed component, Euclidean position, and
  range errors only after estimates and Ground Truth share baseline-center FRD
  `[F,R,D]`; it performs no world/camera/PX4 frame conversion.
- `gazebo_target_controller.py` owns command throttling and Gazebo service
  execution. Mission distance maps to world ENU `y`, altitude to ENU `z/+up`,
  and yaw is encoded about world `+Z`.
- `ground_truth_provider.py` owns old-epoch rejection, bounded future records,
  reset quarantine, and timestamp-ordered release. It retains world ENU
  position and XYZW world-from-reference orientation without converting them.
- `stereo_measurement_pipeline.py` owns OpenCV optical pixel-to-ray and
  closest-ray triangulation. It accepts `(u=right,v=down)` observations plus
  world-ENU camera poses and emits baseline-center FRD `[F,R,D]` metres with
  the canonical geometry-policy diagnostics.
- `pose_source_provider.py` validates and owns PX4 attitude, raw GPS, and fused
  global-position caches. Pair snapshots preserve PX4 Hamilton WXYZ
  body-FRD-to-earth-NED semantics and native timestamps before later pose or
  baseline conversion.
- `pose_conversion_adapter.py` validates source age/epoch/skew and converts
  frozen PX4/GPS sources into camera poses. PX4 Hamilton WXYZ remains
  body-FRD-to-earth-NED; GPS separation is world ENU; camera outputs use
  world-ENU centres with OpenCV optical RDF basis; the UAM public reference is
  explicitly baseline-center FRD metres.
- `runtime_worker_lifecycle.py` owns creation, role assignment, and bounded
  joining of plot, measurement, and RViz worker threads.
- `runtime_worker_loops.py` owns condition waiting, fair measurement/late-GT
  dequeue, target-rate pacing, drain termination, latest-only RViz scheduling,
  and per-iteration fault containment. It delegates pair math and all frame
  conversion to the measurement/pose pipelines.
- `pair_measurement_orchestrator.py` owns the per-pair ordering of image decode,
  observation acquisition, and Nominal/Optical/GPS/diagnostic ray calls. Every
  ray variant receives the same pixels, dimensions, pair timestamp, and frozen
  pose-source snapshot.
- `visualization_payloads.py` serializes read-only metric snapshots into public
  `[forward,right,down,range]` plot arrays and never mutates measurement state.
  It also owns finite-safe JSON serialization for ray metrics and preserves
  FRD point arrays plus their source provenance.
- `research_record_policy.py` gates eligible experiment samples on evaluated,
  finite OpenCV pixels and Nominal/Corrected/GT baseline-center FRD vectors;
  a world-ENU or otherwise mismatched output frame cannot enter the sample set.
- `research_record_fields.py` owns stable CSV flattening for diagnostic FRD
  rays and camera-oracle world-ENU/XYZ plus orientation-XYZW fields. It only
  serializes existing values and performs no frame conversion. It also builds
  the Operational calibration contract, keeping observed pixels, PX4
  body-FRD-to-earth-NED attitude, baseline-center FRD positions, and world-ENU
  Ground Truth in separately named fields.
- `research_terminal_record_builder.py` owns the complete immutable terminal
  pair payload, including schema identity, queue counters, observation fields,
  Nominal/Corrected/GT FRD values, oracle/debug labels, and eligibility policy.
  The node only claims the terminal pair and submits the finished record.
- `rviz_geometry.py` constructs display-only camera frustum and ray primitives
  exclusively as world ENU metre points; optical basis vectors are consumed
  only inside the geometry builder. It also owns the explicit UAM
  baseline-center-FRD reference to world-ENU display-ray conversion.
- `rviz_publisher.py` exclusively serializes and publishes distance text,
  camera/FOV, ray, authoritative-drone, and ray-metrics ROS messages. Every
  Marker position is Gazebo world ENU metres; baseline-center FRD measurement
  topics remain outside this display publisher. The node keeps compatibility
  delegates while constructing the publisher owner during ROS setup.
- `yolo_model_runtime.py` exclusively owns the loaded model, model device,
  warmup/error/fallback state, class filtering, and single/pair inference.
  Detection outputs retain OpenCV top-left pixel order `(u=right,v=down)` and
  perform no stereo or world-frame conversion. Historical node attributes and
  methods remain compatibility properties/delegates backed by that owner.
- `observation_projection.py` projects world-ENU reference points through
  world-ENU camera poses into OpenCV `(u=right,v=down)` observations and owns
  the optional image-bounds gate plus synthetic bbox construction.
- `diagnostic_ray_pipeline.py` owns canonical-FRD validity checks, signed
  Attitude/Operational-minus-Optical component deltas, and the diagnostic
  source-composition order. Its pose inputs remain world ENU/OpenCV optical,
  its GPS baseline remains an ENU separation, and every public ray result is
  baseline-center FRD metres.
- `measurement_publication.py` owns canonical position, plot, detection, and
  profile-status ROS serialization in baseline-center FRD metres.
- `runtime_configuration.py`, `pair_ground_truth_coordinator.py`, and
  `camera_pose_coordinator.py` own profile loading, epoch/pair/GT routing, and
  camera/PX4/GPS intake without collapsing ENU/NED/FRD/RDF contracts.
- `research_recording_setup.py`, `research_terminal_facade.py`,
  `runtime_control_coordinator.py`, and `triangulation_facade.py` own recording
  setup, terminal submission, runtime lifecycle, and pipeline assembly.

`StereoDistanceNode` is now the ROS parameter, topic, callback, and lifecycle
assembly layer; its class body is approximately 1,065 lines. `Workstation` and
StereoWIRIS remain independently launched apps and do not import each other's
app implementation.

## Workstation application boundaries

`Workstation` and `StereoWIRIS` are separate applications. They may use
app-neutral contracts from `Workstation/workstation_core` or `sasl_common`, but neither
application may import the other application's UI, controllers, workers, or
runtime entry point. Similar behavior must be implemented and tested within
each application's own package. Launch `Workstation` with
`python -m Workstation`; `StereoWIRIS/main.py --ui workstation` is rejected
instead of crossing the application boundary.

`Workstation/standalone_gui.py` remains the PyQt panel assembly and
compatibility facade. Runtime responsibilities extracted from that facade are:

- `camera_time_sync_controller.py`: WIRIS STUS/GDTI, RTCP/NTP readiness,
  drift metrics, and time-sync presentation.
- `camera_stream_controller.py`: camera preview/detection start, retry, stop,
  and stream resource cleanup lifecycle.
- `wiris_control_controller.py`: workstation WIRIS SDK command, PC UTC, and
  optical-zoom orchestration.
- `app_lifecycle_controller.py`: bounded application shutdown and auxiliary
  worker cleanup.
- `recording_controller.py`: session state, recording rows and timelines,
  camera/PX4/GNSS metadata, save orchestration, and recording presentation.
- `session_workers.py`: background persistence of a completed session.
- `wiris_runtime_workers.py`: WIRIS clock, STIP, RTSP enablement, and stream
  reachability work.
- `device_runtime_workers.py`: camera-network discovery/configuration and
  Pixhawk discovery, connection, shell, parameter, RTCM, and setup work.
- `device_runtime_controller.py`: workstation camera-network and Pixhawk
  connection/setup worker lifecycles plus their UI state transitions.
- `gnss_runtime_workers.py`: GNSS base serial, NTRIP, and serial discovery
  work.
- `gnss_runtime_controller.py`: GNSS operation mode, base serial, NTRIP,
  survey-in, readiness, RTCM routing, and associated UI state transitions.
- `calibration_workers.py`: checkerboard capture, calibration, random
  validation, and preview detection work.
- `calibration_controller.py`: checkerboard capture, file scanning, random
  validation, calibration result application, and persistence workflow.
- `measurement_view_controller.py`: measurement-attempt assembly, explicit
  baseline/optical frame conversions, triangulation execution, and distance
  and alignment presentation.
- `sensor_presentation_controller.py`: Pixhawk attitude, GNSS, magnetometer,
  relative-heading, and sensor-health presentation without frame conversion.

The facade re-exports historical worker names and delegates historical panel
methods so existing integrations keep their public interface while ownership
stays in workstation modules. Recording coordinate fields retain their
declared contracts: PX4 attitude is body FRD relative to earth NED, camera
observations are OpenCV optical RDF, GNSS deltas are NED where explicitly
named, and public measurements are baseline/stereo-center FRD
`[forward,right,down]` metres. Timestamp fields distinguish monotonic host
time, Unix UTC, GPS UTC, and RTP/RTCP-derived time instead of aliasing them as a
single clock.

## Main Tabs

Create these top-level tabs:

1. Live
2. Device
3. Calibration
4. Control
5. Events
6. Settings
7. Debug

The UI should look like an engineering control system, not a consumer CCTV app.

Use dark theme.
Use compact panels.
Use clear status colors:
- GOOD: green
- DEGRADED: yellow/orange
- INVALID/ERROR: red
- UNKNOWN: gray

## 1. Live Tab

Purpose:
Real-time operation, stereo distance estimation, confidence judgment.

Layout:
- Top status bar
- Left camera video panel
- Right camera video panel
- Target info panel on the right
- Bottom panel for sync/stereo/pose/system status

Show:

Left / Right WIRIS 30x OZ visible video panels:
- Use placeholder black panels for now
- Reserve the overlay layer for backend YOLO bounding boxes
- Show camera role: LEFT CAMERA, RIGHT CAMERA
- Show FPS, latency, timestamp

Target Info:
- Target ID
- Distance in meters
- Azimuth in degrees
- Elevation in degrees
- Confidence bar
- Confidence breakdown:
  - Stereo
  - Sync
  - Pose
  - GNSS

Time Sync Status:
- Sync mode
- Left-Right camera offset
- Left camera - Left Pixhawk offset
- Right camera - Right Pixhawk offset
- Sync quality badge

Stereo Quality:
- Matches
- Inliers
- Reprojection error
- Stereo quality badge

Pose / Sensor Quality:
- Pose state
- Yaw stability
- Magnetic anomaly state
- Vibration state

System Mode:
- FULL
- DEGRADED
- SAFE

Also include a simple compass/top-view placeholder showing target direction relative to camera heading.

## 2. Device Tab

Purpose:
Hardware and sensor monitoring.

Sections:

A. WIRIS Status
For Left and Right:
- Assigned role
- IP
- Serial number
- Firmware version
- RTSP status
- TCP control status
- FPS
- Latency
- Packet loss

B. Pixhawk Status
For Left and Right:
- Assigned role
- Ethernet endpoint
- MAVLink/ROS2 status
- System ID
- Component ID
- Attitude rate
- GNSS rate

C. GNSS / RTK Status
For Left and Right:
- Fix type
- RTK state: FIX / FLOAT / NONE
- Satellite count
- Covariance

D. Attitude Sensors
For Left and Right:
- IMU status
- Gyro rate
- Accel rate
- EKF status
- Roll / Pitch / Yaw
- Roll / Pitch / Yaw variance
- Vibration level
- RM3100 status
- Magnetic field norm
- Magnetic anomaly flag

E. ROS2 Runtime Status
- Active topics
- Topic Hz
- Topic latency

## 3. Calibration Tab

Purpose:
Accuracy management.

Sections:

A. Left OZ Intrinsic
- K matrix
- Distortion coefficients
- Reprojection error
- Calibration date

B. Right OZ Intrinsic
- K matrix
- Distortion coefficients
- Reprojection error
- Calibration date

C. Stereo Extrinsic
- R matrix
- T vector
- Baseline

D. Baseline Mode
- Manual
- GNSS-derived

E. Time Offset Calibration
- Left-Right camera offset
- Left camera - Left Pixhawk offset
- Right camera - Right Pixhawk offset

F. Pose Alignment
- Camera-to-Pixhawk rotation offset
- Roll offset
- Pitch offset
- Yaw offset

G. Validation
- Known-distance error
- Stereo quality score

Show warning:
"OZ zoom / focus / resolution changes invalidate calibration."

Add buttons:
- Load calibration
- Save calibration
- Run checkerboard calibration
- Validate calibration

Buttons can be non-functional placeholders for now.

## 4. Control Tab

Purpose:
WIRIS camera operation control.

Sections:
- Left WIRIS Control
- Right WIRIS Control
- Apply to both toggle

Controls:
- Visible stream start / stop
- Zoom
- Focus mode
- Exposure mode
- Gain
- Record
- Snapshot
- Reboot camera

Safety:
- Lock stereo parameters toggle
- If lock is enabled, disable zoom/focus/resolution controls visually

## 5. Events Tab

Purpose:
Alerts, logs, session recording, replay.

Show event table with columns:
- Timestamp
- Event Type
- Target ID
- Distance
- Azimuth
- Elevation
- Confidence
- Reason
- Snapshot / Clip path

Event types:
- Target detected
- Target approaching
- Confidence drop
- GNSS loss
- Sync degraded
- Yaw unstable
- Stereo quality degraded
- Network loss
- Recording started
- Recording stopped

Functions:
- Record session
- Replay session
- Export logs
- Export calibration snapshot

Show an empty table until an event backend supplies rows.

## 6. Settings Tab

Settings must have subtabs:

A. Network
B. Mapping
C. Stereo
D. Video Receiver
E. Detection
F. ROS2
G. Advanced

### 6-A. Network

Inputs:

WIRIS Camera A:
- IP
- RTSP visible URL
- RTSP port
- TCP control port

WIRIS Camera B:
- IP
- RTSP visible URL
- RTSP port
- TCP control port

Pixhawk A:
- Ethernet endpoint
- UDP/TCP mode
- Port
- System ID
- Component ID

Pixhawk B:
- Ethernet endpoint
- UDP/TCP mode
- Port
- System ID
- Component ID

Buttons:
- Scan
- Test connection
- Save profile
- Load profile

Use default connection profile values:
- WIRIS A: 192.168.0.101
- WIRIS B: 192.168.0.102
- RTSP visible URL: rtsp://<ip>:8554/visible
- TCP control port: 2240
- Pixhawk A: 192.168.0.201:14550
- Pixhawk B: 192.168.0.202:14550

### 6-B. Mapping

Use simple swap buttons.

Camera mapping:
- Button: Swap Camera Left/Right
- Current:
  - Left Camera = WIRIS A
  - Right Camera = WIRIS B

Pixhawk mapping:
- Button: Swap Pixhawk Left/Right
- Current:
  - Left Pixhawk = Pixhawk A
  - Right Pixhawk = Pixhawk B

Display:
- Camera serial number
- Pixhawk system ID
- Assigned role

### 6-C. Stereo

Settings:

Calibration:
- Calibration file path
- Rectification map path
- Q matrix path

Baseline:
- Manual baseline value
- GNSS-derived baseline toggle

Image geometry:
- Image size
- OZ zoom lock value
- Focus lock value

Time Sync:
Sync source options:
- PC receive time
- RTP timestamp
- PC receive time + manual offset
- RTP timestamp + manual offset

Offsets:
- Left camera offset [ms]
- Right camera offset [ms]
- Left Pixhawk offset [ms]
- Right Pixhawk offset [ms]

Thresholds:
- Sync good threshold [ms]
- Sync degraded threshold [ms]
- Sync invalid threshold [ms]

### 6-D. Video Receiver

Receiver backend:
- OpenCV
- FFmpeg / PyAV
- GStreamer

RTSP transport:
- TCP
- UDP

Stream:
- Visible only
- Target FPS
- Expected resolution
- Reconnect on failure

Latency:
- Latency buffer [ms]
- Drop late frames ON/OFF
- Max frame queue size
- Decode timeout [ms]

Timestamp:
- Use RTP timestamp ON/OFF
- Expose receiver timestamp ON/OFF
- Timestamp fallback policy:
  - RTP first, PC fallback
  - PC only
  - Reject frame if RTP missing

Debug:
- Show frame index
- Show decode latency
- Show receiver backend status

Recommended defaults:
- MVP backend: FFmpeg or OpenCV
- Final backend: GStreamer
- MVP timestamp: PC receive time
- Final timestamp: RTP timestamp + manual offset

### 6-E. Detection

YOLO model:
- Model weights path
- Select .pt button
- Clear selection button

Inference runtime:
- Device policy: auto / cpu / cuda:0
- Confidence threshold
- Image size
- Execution mode: detect / track

Stereo gate:
- Require left/right verification
- Detection timing: after stereo sync / left-right independent

The UI stores the selected `.pt` path for the future YOLO backend. It does not import PyTorch or run inference in the UI process.

### 6-F. ROS2

Inputs:
- ROS_DOMAIN_ID
- Namespace
- Topic names

Pixhawk topics:
- /left_pixhawk/imu
- /right_pixhawk/imu
- /left_pixhawk/global_position
- /right_pixhawk/global_position
- /left_pixhawk/attitude
- /right_pixhawk/attitude

Application topics:
- /left_camera/image
- /right_camera/image
- /stereo/targets
- /system/status
- /events

Bridge options:
- MAVROS2 mode
- px4_ros_com / uXRCE-DDS mode
- Micro XRCE-DDS Agent endpoint

### 6-G. Advanced

Settings:
- Logging level
- Raw frame save ON/OFF
- ROS bag recording ON/OFF
- Sensor debug ON/OFF
- Confidence thresholds
- Yaw variance threshold
- Vibration threshold
- Network timeout threshold
- Safe mode policy
- Auto reconnect policy

## 7. Debug Tab

Purpose:
Development, experiment, and failure analysis.

Show:
- Time sync plot placeholder
- Left-right frame offset graph placeholder
- Camera-Pixhawk offset graph placeholder
- ROS2 topic frequency table
- Network latency graph placeholder
- Stereo match visualization placeholder
- Disparity visualization placeholder
- Inlier/outlier count
- Pose stream graph placeholder
- GNSS status history
- Raw logs
- Video receiver backend diagnostics

Use disconnected placeholder data until backend modules supply runtime values.

## Data Models

Create dataclasses in core/models.py:

- DeviceStatus
- CameraStatus
- PixhawkStatus
- GNSSStatus
- AttitudeSensorStatus
- ROS2TopicStatus
- TimeSyncStatus
- StereoQualityStatus
- TargetStatus
- EventRecord
- SystemConfig

Create state.py with functions returning an initial disconnected application state.

## Behavior

- Swap Camera Left/Right button must actually swap labels in Mapping tab and update Live tab labels.
- Swap Pixhawk Left/Right button must update mapping labels.
- Lock stereo parameters toggle must disable zoom/focus controls in Control tab.
- Time sync source selection must update displayed sync mode in Live tab.
- Video receiver backend selection must update displayed backend diagnostics in Debug tab.
- No real hardware connection is required yet.
- Use clean class structure so real backend can later be connected.

## Visual Style

- Dark theme
- Engineering dashboard style
- Avoid oversized empty UI
- Use cards, tables, badges, and compact controls
- Make status values visually scannable
- Use monospace font for matrices, topic names, timestamps, and IP addresses

## Deliverables

Return complete runnable code for the entire PyQt6 application.

Do not omit files.
Do not use pseudocode.
Do not require external image assets.
Use only standard Python libraries plus PyQt6.

## Purpose

SASL is a stereo-vision surveillance and measurement workspace for two WIRIS cameras and two Pixhawk units. It intentionally provides two functionally equivalent desktop applications with different UI/runtime integration boundaries: the ROS-capable Legacy `StereoWIRIS` UI and the MAVLink-focused standalone `Workstation` UI. Simulation, pose evaluation, firmware reproducibility material, and shared coordinate contracts live in the same repository; `ref/` is reference-only.

## Current Run Flow

Create one Python environment for the checkout and install the declared runtime:

```bash
python3 -m pip install -r requirements.txt
./run.sh
```

`./run.sh` starts Workstation. Run `StereoWIRIS/run.sh --ui legacy` for Legacy; Legacy additionally requires the pinned ROS 2/PX4 message overlay prepared by `scripts/bootstrap_ros_workspace.sh`. Use `StereoWIRIS/run.sh --check --ui workstation` or `--check --ui legacy` to validate the selected runtime before opening Qt. Development and CI dependencies are declared in `requirements/dev.txt`.

## Inputs/Outputs and Interfaces

- Hardware inputs include WIRIS visible RTSP/control streams, Pixhawk telemetry, GNSS/RTK, calibration profiles, and optional ROS 2 topics in Legacy.
- Public measurement vectors retain their declared frame and axis contract. Raw local GNSS baseline is NED `[North, East, Down]`; P1 body projection is FRD `[Forward, Right, Down]`; camera optical rays are RDF `[Right, Down, Forward]`; public stereo results use their declared stereo-center FRD schema.
- Selector observation exports use stereo-center RDF component order. Canonical `nominal_pose_right/down/forward_m` fields and compatibility `raw_right/down/forward_m` fields are exact aliases in that exported schema; neither is a public FRD array or a new coordinate conversion.
- Runtime recordings are written below `StereoWIRIS/recordings/` or `workstation_data/recordings/` and are not source files. Reusable evidence belongs in a minimized, anonymized `tests/fixtures/` dataset with its schema/frame contract.

## Operational Rules

- Camera and Pixhawk mapping changes are applied in-memory and reflected across relevant tabs.
- Stereo parameter lock disables zoom, focus, and resolution controls in the Control tab.
- Time sync source changes update the Live tab sync mode.
- Video receiver backend changes update Live status and Debug diagnostics.
- YOLO `.pt` model selection updates Live status and Debug diagnostics, but does not run inference yet.
- Workstation `Tasks > All settings` exposes the remaining camera/WIRIS, YOLO/RTSP, PX4/MAVLink, runtime, and GNSS/RTK settings; Apply delegates to the existing owner and Save persists the same settings schema at `workstation_data/stereo_distance_settings.json`. ROS2/uXRCE and still-image clock controls are intentionally absent from this surface.
- Workstation is a repository-level package in `Workstation/`, alongside `StereoWIRIS`; it launches through `Workstation/standalone_gui.py` and `Workstation/standalone_backend.py`. The Legacy implementation remains under `StereoWIRIS/wiris_stereo_distance/legacy_gui.py`, while `gui.py` remains a compatibility alias. The two UI runtimes no longer import or instantiate each other's panel owner.
- Workstation-owned runtime modules live in `Workstation/workstation_core/`; they import only `Workstation.workstation_core` and neutral `sasl_common` modules. This keeps Workstation's MAVLink-only telemetry and live RTCP/NTP frame timing independent from Legacy's optional middleware and still-image workflow.
- Workstation and Legacy StereoWIRIS are separate applications, not two skins
  over one runtime owner. Domain files are therefore not replaced with imports
  from the other app. Workstation decomposition stays inside `Workstation/`
  and `Workstation/workstation_core/`; only deliberately app-neutral contracts may live in
  `sasl_common`.
- `Workstation/workstation_core/gnss_protocol.py` owns Workstation UBX/RTCM framing,
  field-base serial probing, and WGS84 geodetic/ECEF parsing. It never aliases
  those values to PX4 NED, body FRD, or camera frames. The former
  `standalone_gui` symbols remain re-exported for compatibility.
- `Workstation/workstation_core/runtime_camera_status.py` owns pure live-stream status and
  timestamp formatting. RTCP/NTP UTC, source-clock seconds, and host-receive
  time remain distinct provenance domains; the Qt panel only renders the
  resulting text.
- `Workstation/telemetry_plots.py` owns Workstation-only attitude, generic
  telemetry, and RTK/alignment plot widgets. Plot inputs retain their source
  contracts—GPS local NED `[N,E,D]` and P1 optical RDF
  `[Right,Down,Forward]`—and the widgets do not perform frame conversion.
- `Workstation/workstation_core/mavlink_parameter_store.py` owns integer PARAM write/readback
  enforcement and REAL32-preserving calibration reads. It handles MAVLink wire
  types only and does not interpret coordinate values.
- The standalone copy currently preserves the shared measurement, camera, telemetry, calibration, and recording modules and their coordinate/schema contracts. Lifecycle sequencing is shared only through the UI-neutral `sasl_common/lifecycle.py`; Legacy and Workstation keep separate panel/resource owners.
- Workstation recordings, retryable spools, calibration captures, and alignment files live under `workstation_data/` (recordings under `workstation_data/recordings/`). Legacy continues to use `StereoWIRIS/stereo_distance_settings.json` and `StereoWIRIS/recordings/`; the two stores are not migrated or read through each other.
- Workstation intentionally omits the ROS2/uXRCE Pixhawk connection, namespace/topic discovery, and still-image/FTP/EXIF clock workflow. Legacy retains those existing workflows under `StereoWIRIS/`.
- `StereoWIRIS/main.py --ui workstation` is rejected because the applications
  are independent. Launch Workstation with `./run.sh` or
  `python -m Workstation`; Legacy keeps its canonical ROS/PX4 overlay
  validation and ROS setup.
- Workstation field flow is exposed in `Run > Experiment` in the operator's order: `0` QGC Pixhawk calibration (external), `1` connect/verify WIRIS, Pixhawk, and Base GNSS, `2` start Base Survey-in or NTRIP, `3` initial WIRIS time sync, `4` verify RTCM delivery and numeric Pixhawk `fix_type`, `5–7` physically align the baseline and Roll/Pitch/Yaw with repeated evidence sampling, `8` perform the final time-sync quality check/resync, `9` confirm YOLO/measurement settings, `10` measure, and `11` save. The underlying Setup/Run/Tasks pages remain available from each step.
- QGC Pixhawk sensor calibration is documented as an external prerequisite; Workstation does not duplicate that operation. Internal camera intrinsic/stereo calibration remains supported in `Tasks > Calibration` and should be completed before the field sequence.
- Workstation `Tasks > Calibration > Load intrinsics` opens a JSON file picker,
  validates positive image dimensions, finite 3×3 camera matrices, positive
  `fx/fy`, and finite distortion coefficients, then applies the selected
  profile. A `metadata.zoom_level` mismatch or unequal left/right calibration
  resolution requires explicit confirmation. `Load current zoom` retains the
  deterministic `intrinsics_zoom_##.json` lookup, while `Apply intrinsics`
  only reapplies the result already held in memory.
- RTK baseline evidence keeps the raw selected `P1→P2` GPS/RTK vector as local NED `[North, East, Down]`. When P1 attitude is available, Workstation also derives a separate `P1→P2 @ P1 body FRD` display/recording vector `[Forward, Right, Down]` using the inverse of the PX4 body-FRD→world-NED quaternion; this derived value is not camera optical RDF or public `baseline_center_frd`.
- Measurement Run control keeps Preview, Detection, Record, and Save session together; Current measurement quality mirrors the existing ray-gap, midpoint-range, method, attitude-timing, and Raw `baseline_center_frd` evidence.
- Recording shows an advisory experiment preflight for both camera streams, RTCP/NTP sync, detection, active stereo calibration, and the selected Pixhawk/GNSS or checkerboard geometry path. It reports missing evidence without changing the existing start/stop command behavior.
- GNSS readiness records numeric PX4/MAVLink `fix_type` for P1 and P2. The configurable `minimum_fix_type` defaults to `3` (3D fix); `RTK_FIXED=6` is shown as a fixed-sample ratio and is required only when the operator sets the threshold to `6` or enables the legacy `require_rtk_fixed` policy. A route can therefore be usable without claiming that every field condition is RTK_FIXED.
- GNSS/NTRIP field procedure (Workstation `./run.sh` and Legacy `StereoWIRIS/run.sh --ui legacy`): select `현장 Base Survey-in`, `NTRIP 보조 Fixed Base`, or `Network RTK 직접`. The recommended absolute-WGS84 path is `NTRIP 보조 Fixed Base`: connect the Base F9P/HG-C08 over USB, enter the current experiment's site/point and address, enter that experiment's caster/provider and mountpoint, select `Base GNSS` GGA, keep Fixed Base height as `WGS84 ellipsoid`, and route RTCM to `좌/우 Pixhawk`. The saved defaults are Survey-in `300 s` / `0.5 m`, target `both`, and profile reuse disabled; the Base USB path prefers the detected u-blox `/dev/serial/by-id/...` path and falls back to the operator-selected port.
- Site/provider data are per-session metadata. Do not reuse a prior site's fixed-coordinate profile or hardcode a caster/mountpoint; start each experiment with a new site/address and current service selection. Assisted mode uses the latest Base WGS84 geodetic sample for NTRIP GGA, validates repeated RTK Fixed Base samples, and records the coordinate/status. Rover RTK Fixed is displayed and retained as evidence but is not a normal measurement prerequisite unless the readiness threshold is explicitly raised to `6`; both Rover RTCM routes still must be active and healthy.
- GNSS coordinate contract: Base/rover positions are WGS84 latitude/longitude degrees with an explicit height datum; `CFG-TMODE3` receives WGS84 ECEF centimetres plus the `0.1 mm` high part. The resulting rover baseline remains local NED `[North, East, Down]`; no Base geodetic value is written into PX4 body-FRD, camera optical RDF, or stereo-center fields.
- Physical alignment sampling stores raw NED `[North, East, Down]`, a separate P1 body-FRD `[Forward, Right, Down]` projection, relative P2-to-P1 `[Roll, Pitch, Yaw]`, numeric fix types, and iteration quality. The operator changes the hardware physically; Workstation does not apply software R/P/Y or translation corrections. Existing ENU alignment profiles remain preserved and the new evidence is stored under a separate `physical_alignment` block.
- Experiment protocol method: purpose is to expose the ordered field checklist; inputs are StatusModel states plus existing calibration, time-sync, RTCM/fix, baseline, physical-alignment, and recording evidence; output is per-step `READY`, `WAIT`, `CHECK`, `SAMPLING`, `RUNNING`, or `EXTERNAL`. `READY` is software evidence only; hardware acceptance and long-duration validation remain outside this check.
- While recording is `ACTIVE` or `FINALIZING`, Preview stop, camera/Pixhawk mapping swaps, and Save session are disabled so the session protocol cannot be interrupted or saved prematurely.
- Workstation default screens keep the operator path compact: connect cameras → check timing/devices → measure → record. Discovery, low-level controls, and file review remain under `More tools`, `Advanced`, or `Diagnostics` without removing their command IDs.
- Workstation `Pixhawk` keeps the primary operator actions at the top in this order: `Connect` → `Find USB` → `Find network` → `Find host IP` → `Swap P1/P2`. The evidence groups then follow this order: `Connection` → `Attitude (R/P/Y)` → `P1/P2 GNSS receivers` → `P1→P2 baseline` → `Discovered endpoints` → `ETH MAV`.
- `P1 ETH MAV` and `P2 ETH MAV` use the corresponding configured USB control port and baud rate to start PX4 Ethernet MAVLink at runtime, then reconnect the UDP MAVLink stream. Confirm the link with Heartbeat and `ATTITUDE`/R/P/Y telemetry; USB control status and MAVLink telemetry status are displayed separately.
- Workstation `Advanced` opens directly on its tool tabs without duplicating the global CAM L/CAM R/P1/P2/REC/MEAS status summary. Use the dedicated camera, Pixhawk, measurement, and recording workspaces for those runtime states; each Advanced tool retains its own operation status.
- Workstation WIRIS SDK commands are sent from `Advanced > Camera control > WIRIS SDK command`. Select the left, right, or both-camera target and the protocol terminator, then use `Command responses` to read each camera's TCP reply or failure inline; the latest aggregate result remains in the status line and the existing Diagnostics log is retained. `Clear responses` clears only the current UI history and does not send a camera command.
- Workstation cards show only the value plus age/quality when useful. Frame and provenance details remain available on value tooltips and in Inspector/Diagnostics; the literal `source=` and `frame=` prefixes are not shown in the default metadata rows, and the underlying status/evidence fields are unchanged.
- Workstation status severity never changes a card, row, or status-label
  background. Neutral backgrounds remain fixed; only a small dot, short state
  text, icon, or thin border uses the status color. Valid sample-backed values
  show age separately (`● VALID · 38 ms`), and camera/Pixhawk/measurement
  samples older than the presentation freshness threshold are shown as
  `STALE` rather than remaining green.
- The obsolete minimum-bounding-box controls are not exposed or persisted; the remaining detection-gate controls are unchanged.
- The UI does not synthesize telemetry. Runtime values remain empty or UNKNOWN until a backend updates the shared state.
- Changes to the Legacy UI are not automatically propagated to the standalone Workstation copy; update and test both copies deliberately until the remaining migration is complete.
- Do not commit venvs, ROS build/install/log trees, runtime recordings, profiler output, downloaded models, or generated experiment runs. Audit the checkout with `python3 scripts/check_repository_policy.py`; `--allow-existing-debt` is only for reporting the historical tracked artifacts pending cleanup.
- Run the targeted tests first. The repository-wide test configuration in `pyproject.toml` excludes generated overlays, PX4 source, reports, and recordings from collection.
- Sealed qualification security tests require the immutable external fixture. Set `SASL_QUALIFICATION_FIXTURE=/path/to/exact-profile-freeze-...` to run them; without it they are reported as explicit skips rather than reading a developer-specific absolute path.

## Known Limitations and Workarounds

- Real WIRIS, Pixhawk, RTSP, YOLO, stereo matching, and calibration persistence are not connected yet in this software-only validation slice.
- Workstation still has a large compatibility facade in
  `Workstation/standalone_gui.py`; GNSS/Pixhawk sensor presentation and
  measurement/view-controller extraction remain in progress. Shared neutral
  domain contracts remain common dependencies, and long-duration hardware
  validation is pending. ROS2/uXRCE and still-image clock support are
  Legacy-only by design.
- Workstation settings and recordings are persisted under the ignored `workstation_data/` runtime root; Legacy settings and recordings remain under `StereoWIRIS/`.
- `ETH MAV` changes the running PX4 Ethernet MAVLink state only. Use the separate `ETH 저장` operation when the Ethernet MAVLink configuration must survive a Pixhawk reboot. A `CONNECTED` or Heartbeat state alone does not prove that attitude data is flowing; verify `ATTITUDE` and the R/P/Y values as well. Indoor GNSS may remain `NO DATA` while attitude telemetry is valid.
- Historical recordings and `perf.data` are still tracked repository debt. CI reports all 237 paths but temporarily permits that fixed cleanup debt; remove the escape hatch after a reviewed `git rm --cached` migration. New runtime output is ignored now.
- The sealed qualification fixture is intentionally not bundled because it is immutable experiment evidence. The normal software suite currently reports 98 skips when `SASL_QUALIFICATION_FIXTURE` is absent.
- Hardware/ROS integration remains platform-specific. ROS packages come from `/opt/ros/humble` and the reproducible repo bootstrap rather than pip.

## Change Log

- 2026-08-10: Moved the Workstation application, `workstation_core`, dedicated UI documentation, and dedicated tests under the repository-level `Workstation/` directory. Updated imports and launchers to `Workstation`, while preserving runtime settings and recordings under `workstation_data/`.
- 2026-08-10: Simplified the Workstation Pixhawk ETH MAV labels to `P1/P2 ETH MAV`; the USB control transport remains documented in the operation description and implementation.
- 2026-08-10: Reordered the Workstation `Pixhawk` screen so the five primary connection actions are first, followed by `Connection`, `Attitude (R/P/Y)`, `P1/P2 GNSS receivers`, `P1→P2 baseline`, `Discovered endpoints`, and `ETH MAV`. Documented the per-Pixhawk USB runtime Ethernet MAVLink start workflow and the Heartbeat/`ATTITUDE` verification rule.
- 2026-08-10: Fixed Workstation `Advanced > Camera control` initialization so
  the first WIRIS catalog item immediately populates the command, terminator,
  and description instead of leaving Send with an empty command. Corrected the
  owner-panel Send button binding so the asynchronous TCP worker starts and
  restores the actual command button after completion.
- 2026-08-10: Removed the duplicated CAM/Pixhawk/recording/measurement Status
  summary from the top of Workstation `Advanced`; its tool tabs and local
  operation statuses remain unchanged.
- 2026-08-10: Added an inline WIRIS SDK TCP response history to Workstation
  `Advanced > Camera control`, including per-camera replies, failures, command
  target/context, latest status, and a local clear action while retaining the
  existing Diagnostics log.
- 2026-08-10: Split Workstation WIRIS/time-sync workers, recording/session
  lifecycle, checkerboard calibration workflow, and camera-network/Pixhawk
  runtime orchestration into workstation-owned controllers/workers. Preserved
  compatibility delegates and coordinate/time contracts, and removed the
  StereoWIRIS-to-workstation runtime import.
- 2026-08-10: Moved GNSS/NTRIP/survey-in/RTCM orchestration, measurement and
  alignment presentation, and Pixhawk/GNSS sensor presentation out of the
  Workstation panel facade. The facade retains historical method delegates and
  UI assembly while controller modules own domain state transitions.
- 2026-08-10: Completed the recommended final Workstation split by moving
  camera stream/detection lifecycle, WIRIS command and optical-zoom control,
  and application shutdown cleanup into workstation-owned controllers. UI
  builders, settings binding, and compatibility delegates remain in the panel
  intentionally.
- 2026-08-10: Changed Workstation `Load intrinsics` to an explicit validated
  file-selection workflow, retained automatic per-zoom lookup as `Load current
  zoom`, and made `Apply intrinsics` a memory-only application action.
- 2026-08-10: Replaced severity-colored Workstation card, row, measurement,
  sensor-health, and protocol backgrounds with fixed neutral backgrounds.
  Status color is limited to compact dots/text/icons/borders, valid values show
  neutral age text, and stale sample-backed states are labeled explicitly.
- 2026-08-09: Declared the shared Python runtime and development dependencies, added bounded pytest/ruff configuration and CI, documented both intentional UI products, and added repository policy checks plus runtime-recording ignore/contract documentation. Historical tracked artifacts remain an explicitly reported cleanup debt.
- 2026-08-09: Reconciled operator telemetry schema evolution by accepting legacy public-FRD `raw_*` input aliases, preserving canonical `nominal_pose_*` fields, and emitting explicit stereo-center RDF `raw_*` compatibility aliases required by deployed residual selectors.
- 2026-08-09: Removed developer-home dependencies from sealed qualification tests, repaired current ROS/PX4 launcher fixtures and research-statistics schema reads, and restored the repository-wide baseline to 2,094 passed with 98 explicit external-fixture skips.
- 2026-08-09: Promoted Workstation to the repository-level `Workstation/` package alongside `StereoWIRIS`; its copied owner and shared measurement/saved-coordinate contracts remain unchanged.
- 2026-08-09: Separated Workstation runtime state into `workstation_data/`, kept Legacy storage under `StereoWIRIS/`, and moved the retryable lifecycle coordinator to neutral `sasl_common/lifecycle.py`.
- 2026-08-09: Hid repetitive `source=` provenance labels from Workstation main cards while retaining the source field and detailed provenance in Inspector/Diagnostics.
- 2026-08-09: Added a Workstation-wide settings surface for the remaining Legacy-only AppSettings, including nested GNSS/RTK operation settings, with owner-backed Apply/Save and no new settings store.
- 2026-08-09: Removed the unused minimum-bounding-box setting from the StereoWIRIS settings model, JSON schema, Legacy UI, Workstation UI, and evidence projection; existing image-pixel bbox coordinates remain unchanged.
- 2026-08-09: Reordered Workstation field actions around the experiment protocol, added the Time/Sync workspace and measurement quality/run-control surface, grouped Pixhawk/GNSS/Alignment and calibration actions, and added recording safety gates.
- 2026-08-09: Removed Workstation ROS2/uXRCE connection/discovery and still-image/FTP/EXIF TimeSync workflows; retained MAVLink, PC UTC, RTCP/NTP, and checkerboard calibration paths while leaving Legacy unchanged.
- 2026-08-09: Made the Workstation launcher boundary explicit: Workstation no longer bootstraps the ROS2/PX4 overlay or imports ROS2 bindings at startup; Legacy bootstrap and workflows remain unchanged.
- 2026-08-09: Added Workstation MAVLink endpoint/heartbeat/target identity to source-backed evidence and PX4 timeline rows without changing measurement frames, units, or time axes.
- 2026-08-09: Added received MAVLink source system/component, sequence, and message-id provenance to Workstation telemetry state, evidence, measurement snapshots, and session rows without changing measurement frames, units, or time axes.
- 2026-08-09: Added a mode-specific, advisory experiment preflight to Recording so operators can follow the camera → sync → detection → calibration → geometry protocol before capture.
- 2026-08-09: Removed the literal `source=` prefix from Inspector provenance display while retaining the underlying source metadata.
- 2026-08-09: Simplified the Workstation visual hierarchy and operator vocabulary: navigation now follows Setup/Run/Tasks, primary controls use short action labels, and frame/provenance details move from repeated metadata rows to tooltips and diagnostics without changing commands or coordinate contracts.
- 2026-08-09: Added an explicit P1→P2 body-FRD projection alongside the unchanged raw NED RTK baseline, with separate Alignment/Inspector/Diagnostics/preflight/recording fields and inverse-quaternion coordinate tests.
- 2026-05-04: Removed simulated mock state, live mock refreshes, fake target overlays, mock events, mock topics, and fake calibration values; the UI now starts in a disconnected state.
- 2026-05-04: Added Detection settings for selecting a YOLO `.pt` model path and reflecting the selected model in Live and Debug views.
- 2026-05-04: Added the standalone Qt UI prototype package with README-specified tabs, shared models, reusable widgets, and cross-tab state wiring.
