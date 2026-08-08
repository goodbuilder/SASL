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

This repository now contains a runnable desktop UI prototype for the stereo vision surveillance MVP described above. The `ref/` directory remains reference-only; the application code is self-contained under `uam_stereo_ui/`.

## Current Run Flow

Install the Qt binding and run the package module:

```bash
python3 -m pip install -r requirements.txt
python3 -m uam_stereo_ui
```

The app tries PyQt6 first and falls back to PySide6 if PyQt6 is unavailable.

## Inputs/Outputs and Interfaces

- Inputs are disconnected application state, editable settings controls, mapping swap buttons, sync source selection, video backend selection, YOLO `.pt` model selection, and the stereo parameter lock toggle.
- Outputs are the seven UI tabs: Live, Device, Calibration, Control, Events, Settings, and Debug.
- No hardware, ROS2 graph, RTSP stream, MAVLink endpoint, image asset, or external service is required for the prototype.

## Operational Rules

- Camera and Pixhawk mapping changes are applied in-memory and reflected across relevant tabs.
- Stereo parameter lock disables zoom, focus, and resolution controls in the Control tab.
- Time sync source changes update the Live tab sync mode.
- Video receiver backend changes update Live status and Debug diagnostics.
- YOLO `.pt` model selection updates Live status and Debug diagnostics, but does not run inference yet.
- Workstation `Tasks > All settings` exposes the remaining camera/WIRIS, YOLO/RTSP, PX4/ROS2, runtime, GNSS/RTK, and Clock/FTP settings; Apply delegates to the existing owner and Save persists the same settings schema at `workstation_data/stereo_distance_settings.json`.
- Workstation is a repository-level package in `workstation/`, alongside `StereoWIRIS`; it launches through `workstation/standalone_gui.py` and `workstation/standalone_backend.py`. The Legacy implementation remains under `StereoWIRIS/wiris_stereo_distance/legacy_gui.py`, while `gui.py` remains a compatibility alias. The two UI runtimes no longer import or instantiate each other's panel owner.
- The standalone copy currently preserves the shared measurement, camera, telemetry, calibration, and recording modules and their coordinate/schema contracts. Lifecycle sequencing is shared only through the UI-neutral `sasl_common/lifecycle.py`; Legacy and Workstation keep separate panel/resource owners.
- Workstation recordings, retryable spools, calibration captures, clock downloads, and alignment files live under `workstation_data/` (recordings under `workstation_data/recordings/`). Legacy continues to use `StereoWIRIS/stereo_distance_settings.json` and `StereoWIRIS/recordings/`; the two stores are not migrated or read through each other.
- Removing unused ROS2 Pixhawk and still-image clock workflows is a later, separately verified slice.
- Workstation field flow is ordered as `Devices > Cameras` (IP/control/RTSP readiness), `Devices > Time / Sync` (PC UTC, RTCP/NTP, still-image clock evidence), `Devices > Pixhawk`, `Devices > GNSS / RTK`, `Tasks > Calibration`, and `Processing > Measurement`.
- Measurement Run control keeps Preview, Detection, Record, and Save session together; Current measurement quality mirrors the existing ray-gap, midpoint-range, method, attitude-timing, and Raw `baseline_center_frd` evidence.
- While recording is `ACTIVE` or `FINALIZING`, Preview stop, camera/Pixhawk mapping swaps, and Save session are disabled so the session protocol cannot be interrupted or saved prematurely.
- Workstation main cards keep values compact; provenance remains available in the Inspector and Diagnostics views instead of being repeated as `source=` on every card.
- Inspector provenance remains available as a plain value without the literal `source=` prefix; the underlying status/evidence source fields are unchanged.
- The obsolete minimum-bounding-box controls are not exposed or persisted; the remaining detection-gate controls are unchanged.
- The UI does not synthesize telemetry. Runtime values remain empty or UNKNOWN until a backend updates the shared state.
- Changes to the Legacy UI are not automatically propagated to the standalone Workstation copy; update and test both copies deliberately until the remaining migration is complete.

## Known Limitations and Workarounds

- Real WIRIS, Pixhawk, RTSP, YOLO, stereo matching, ROS2, and calibration persistence are not connected yet.
- The standalone Workstation and Legacy GUI are source-copied runtime owners, but shared domain modules remain common dependencies; long-duration hardware validation and final removal of unused workflows are still pending.
- Workstation settings and recordings are persisted under the ignored `workstation_data/` runtime root; Legacy settings and recordings remain under `StereoWIRIS/`.
- If neither PyQt6 nor PySide6 is installed, the app cannot launch; install `requirements.txt` or install PySide6 manually.

## Change Log

- 2026-08-09: Promoted Workstation to the repository-level `workstation/` package alongside `StereoWIRIS`; its copied owner and shared measurement/saved-coordinate contracts remain unchanged.
- 2026-08-09: Separated Workstation runtime state into `workstation_data/`, kept Legacy storage under `StereoWIRIS/`, and moved the retryable lifecycle coordinator to neutral `sasl_common/lifecycle.py`.
- 2026-08-09: Hid repetitive `source=` provenance labels from Workstation main cards while retaining the source field and detailed provenance in Inspector/Diagnostics.
- 2026-08-09: Added a Workstation-wide settings surface for the remaining Legacy-only AppSettings, including nested GNSS/RTK operation settings, with owner-backed Apply/Save and no new settings store.
- 2026-08-09: Removed the unused minimum-bounding-box setting from the StereoWIRIS settings model, JSON schema, Legacy UI, Workstation UI, and evidence projection; existing image-pixel bbox coordinates remain unchanged.
- 2026-08-09: Reordered Workstation field actions around the experiment protocol, added the Time/Sync workspace and measurement quality/run-control surface, grouped Pixhawk/GNSS/Alignment and calibration actions, and added recording safety gates.
- 2026-08-09: Removed the literal `source=` prefix from Inspector provenance display while retaining the underlying source metadata.
- 2026-05-04: Removed simulated mock state, live mock refreshes, fake target overlays, mock events, mock topics, and fake calibration values; the UI now starts in a disconnected state.
- 2026-05-04: Added Detection settings for selecting a YOLO `.pt` model path and reflecting the selected model in Live and Debug views.
- 2026-05-04: Added the standalone Qt UI prototype package with README-specified tabs, shared models, reusable widgets, and cross-tab state wiring.
