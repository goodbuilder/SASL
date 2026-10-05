# StereoWIRIS Information Architecture

- Status: Implemented sessionless revision, 2026-08-26
- Depends on: 00_PRODUCT_VISION.md, 01_USER_WORKFLOW.md

## 1. Philosophy

Organize by engineering meaning and operator task. Persisted data reports what was captured; it
does not own whether a live Measurement command is allowed.

## 2. UI domains

1. Navigation
2. Workspace
3. Inspector
4. Toolbar
5. Diagnostics
6. Global Status

Every feature has one primary owner.

## 3. Navigation

```text
StereoWIRIS
├── Overview
├── Protocol
├── Devices
│   ├── Cameras
│   ├── Pixhawk
│   ├── Telemetry
│   ├── Base GNSS
│   └── Time / Sync
├── Data
├── Calibration
├── Alignment
├── Measurement
├── Advanced
├── Settings
└── Diagnostics
```

Left/Right camera and P1/P2 evidence are paired local views. Detection belongs to Cameras;
synchronization belongs to Time / Sync. `Data` owns fixed writer status, capture tools, data-folder
access, and a file-manager link to legacy recordings. It owns no Session lifecycle or readiness
gate.

## 4. Workspace ownership

- Measurement: transient run state, live inputs, method readiness, result, quality, history.
- Data: type-owned paths, writer queues/errors/drops, explicit HR/OZ and ULog tools.
- Alignment: app-lifetime pose candidates, frozen references, Apply/Revert/Reset.
- Calibration: reusable photo sets and K,D/R,t profiles, independent of Measurement.
- Advanced: Manual record and file/video utilities.
- Diagnostics: factual logs, raw states, communication, timing, performance.

## 5. Inspector

Contextual read-only/configuration projection. It does not contain large logs or camera preview.
For Data it shows writer/storage facts, not Session/package state.

## 6. Toolbar and commands

Commands request change; status confirms it. Configuration fields do not belong in the toolbar.
The Measurement request stays available when storage is DEGRADED or UNAVAILABLE.

## 7. Global status

Camera, Pixhawk, RTK, Detection, Measurement run, recording, storage degradation, and FPS are
compact projections only. `QUEUED` identifies camera-resource contention; it is not a save state.

## 8. Mapping examples

- Camera Preview → Workspace
- Connect Camera → Command
- Camera Connected → Status
- Measurement Start/Stop → Command
- Measurement `IDLE/QUEUED/STARTING/RUNNING/STOPPING/ERROR` → Status
- Fixed stream paths/drop counts → Data Workspace
- Manual record → Advanced command
- Capture HR/OZ/ULog → Data Capture
- Measurement result/quality → Measurement Workspace
- Raw MAVLink/logs → Diagnostics
- Calibration workflow/result → Calibration Workspace

## 9. Forbidden structures

- No Create/Open/Close/Finalize/Seal/Retry-Save Session UI.
- No automatic legacy Session catalog scan or recovery.
- No storage-health gate on Measurement admission.
- No duplicate Data/Recording/Review top-level navigation.
- No coordinate conversion in generic UI adapters.

## 10. Success criteria

- Operators can begin a Measurement after a long gap without closing old data state.
- Actual camera contention queues and resumes visibly.
- Data failures remain visible without disabling Measurement.
- Legacy folders remain untouched and file-manager-only.
- Alignment remains stable until Revert, Reset, or restart.
