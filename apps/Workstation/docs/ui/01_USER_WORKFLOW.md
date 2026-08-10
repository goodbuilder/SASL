# StereoWIRIS User Workflow

- Status: Draft
- Depends on: 00_PRODUCT_VISION.md

## 1. Purpose

Define how engineers and operators complete real engineering tasks. The UI shall be organized around workflows rather than implemented-feature lists.

## 2. Principle

StereoWIRIS shall not assume only one subsystem is used at a time. Typical work can require camera imagery, Pixhawk state, GNSS/RTK, measurement output, recording state, and diagnostics simultaneously.

## 3. Primary Workflows

1. Application startup and readiness verification
2. Hardware connection
3. Camera RTC and stream synchronization
4. Live camera observation
5. Measurement operation
6. Recording
7. Camera configuration
8. Calibration
9. PX4/attitude verification
10. GNSS/RTK verification
11. Detection configuration/operation
12. Diagnostics and fault investigation
13. Saved-session review and analysis

These are user jobs, not top-level tabs.

## 4. Startup and Readiness

Goal: determine whether the system is ready.

Persistent information candidates:
- camera state
- Pixhawk state
- GNSS/RTK
- detection
- recording
- measurement availability

Success: required devices for intended task are connected and providing expected data.

Field sequence for the Workstation implementation:
0. Complete Pixhawk Sensor Calibration in QGC outside Workstation.
1. Connect and verify WIRIS, Pixhawk P1/P2, and Base GNSS.
2. Start Base Survey-in or NTRIP.
3. Set/check WIRIS time synchronization.
4. Verify RTCM delivery to Left/Right Pixhawk and record numeric `fix_type`; do not assume every condition reaches `RTK_FIXED=6`.
5. Physically align the P1–P2 baseline.
6. Physically rotate the assembly to align Roll/Pitch/Yaw.
7. Repeat steps 5–6 with a new evidence sample.
8. Check final WIRIS time-sync quality and resync if required.
9. Confirm YOLO and measurement settings.
10. Start measurement.
11. Save the measured data.

Internal camera intrinsic/stereo calibration is a pre-experiment task. It remains
available in `Pre-run Calibration`, but it is not inserted into the numbered field
sequence after the final time-sync checkpoint.

The `Protocol` page mirrors this order and links to the corresponding Workstation page for
each action. QGC calibration is displayed as an external prerequisite and has no
in-app execution button.

### Calibration calculation execution

`Checkerboard capture validation` copies the currently displayed left/right images, fixes
their RTCP/NTP pair decision at capture time, then encodes PNG files and runs checkerboard
detection in the shared background calibration slot. Its purpose is to prevent image
encoding and OpenCV detection from blocking live controls. A stereo capture assumes both
frames contain RTCP/NTP camera timestamps within the configured pair limit; otherwise
neither side is admitted as a stereo pair. The output contract is unchanged OpenCV image
pixels with capture metadata recording the original RTCP/NTP timestamps and delta. PASS
means at least one available image was encoded and passed checkerboard detection (or was
explicitly retained because OpenCV was unavailable); FAIL leaves rejected images out of the
calibration set. This capture method does not use ground truth or produce NED/FRD values.

`Full checkerboard calibration` runs the existing OpenCV intrinsic calculation for both
cameras and, in checkerboard mode, the existing stereo extrinsic calculation. Its purpose is
to keep the Workstation responsive while processing an image set; the numerical method is
unchanged. Inputs are the selected checkerboard geometry and readable left/right images. In
checkerboard mode, left/right files must have matching capture IDs and at least three stereo
pairs. The output contract remains OpenCV optical RDF intrinsics and
`X_right = R_left_to_right * X_left + T_left_to_right_m`. Completion with a typed calibration
result is PASS; an OpenCV/input exception or invalid result is FAIL and leaves the prior
applied result unchanged.

`Random stereo-pair validation` repeats the same calculation on seeded subsets in the same
background calculation slot. It assumes a completed full result and at least three matching
pairs. PASS records each successful subset and reports intrinsic, baseline, and rotation
stability; partial subset failures remain visible in the result table and do not become
runtime calibration truth. Neither method uses ground truth or converts RDF values to NED,
FRD, or the public measurement frame.

## 5. Hardware Connection

Sequence:
1. select device
2. inspect configuration
3. modify if required
4. execute Connect
5. observe actual state
6. open Diagnostics on failure without losing context

Command and state must be separate: Connect ≠ Connected.

## 6. Live Camera Observation

Required visibility:
- connection/stream
- FPS
- selected camera identity
- recording
- detection
- diagnostics

Changing camera parameters should not unnecessarily hide live image.

## 7. Measurement Operation

Required visibility:
- relevant live imagery
- measurement result and unit
- validity/quality
- detection
- timing/synchronization when relevant
- RTK/baseline when relevant
- recording

Distinguish raw/corrected/configured/derived/invalid values.

## 8. Recording

Required visibility:
- recording state
- session/output path
- elapsed time when useful
- write/save errors
- relevant system state

Before capture, the Recording workspace shows an advisory experiment preflight. The
pass criteria are both camera streams live, RTCP/NTP pair synchronization verified,
detection running, active stereo intrinsics, and either P1/P2 MAVLink plus the GNSS/RTK
NED baseline or checkerboard left-to-right optical RDF geometry according to the selected
measurement path. GNSS readiness checks the configured numeric `minimum_fix_type` (default
3); `RTK_FIXED=6` and fixed duration are required only when explicitly selected. The same
preflight also requires a passing physical-alignment result and an explicit final WIRIS
time-sync checkpoint recorded after alignment. `READY`/`BLOCKED` is evidence only and does
not change the existing recording command.

State candidates: Idle, Starting, Recording, Stopping, Saved, Error.

Record command is not proof that data is being written.

## 9. Camera Configuration

Sequence:
1. select camera
2. inspect current values
3. edit
4. apply if required
5. confirm readback/applied state
6. observe effect

Where applicable distinguish current, pending, and applied values.

## 10. Calibration

Calibration is a multi-step engineering workflow. Typical sequence: select type, verify prerequisites, capture/load data, run calibration, inspect metrics, save, confirm active calibration.

## 11. PX4 / Attitude Verification

QGC Sensor Calibration is an external prerequisite. Workstation exposes the stored
calibration status read-only, plus connectivity, attitude, timestamp/age, source,
relative attitude, and relevant sensor state. Detailed raw telemetry belongs in
Diagnostics.

## 12. GNSS / RTK Verification

Expose receiver state, numeric `fix_type`, RTK label, satellites, eph/epv, RTCM route
delivery, baseline, source, timestamp/age, and the fixed-sample ratio. Numeric fix evidence
must remain visible even when the solution is not `RTK_FIXED`.
The selected antenna baseline remains `P1→P2` in local NED `[North, East, Down]`.
When P1 attitude is available, show the same vector separately in P1 body FRD
`[Forward, Right, Down]`; this is an attitude-derived operator aid and is not camera
optical RDF or `baseline_center_frd` measurement output.

## 13. Detection

Expose model, execution device, threshold, target class, runtime state, and target presence. Detection state should remain visible during measurement.

## 14. Diagnostics / Fault Investigation

Sequence:
1. observe abnormal state
2. keep current workspace
3. open diagnostics
4. inspect factual evidence
5. adjust/retry
6. confirm recovery

Prefer factual messages such as `RTSP port unreachable` over vague `camera unhealthy` unless rule-backed.

## 15. Saved Session Review

Open session → inspect metadata/samples/plots → compare sources → export/analyze.

## 16. Workflow Priority

Priority A: startup/readiness, live observation, measurement, recording, diagnostics.

Priority B: hardware connection, camera config, PX4, GNSS/RTK, detection config.

Priority C: calibration, saved-session analysis, advanced configuration.

## 17. Context Preservation Rules

1. Diagnostics shall not replace primary Workspace.
2. Property changes shall not unnecessarily hide affected data.
3. Recording state remains visible across navigation.
4. Critical connectivity remains visible during normal operation.
5. Command does not imply successful state.
6. Advanced workflows may replace Workspace but Global Status/Diagnostics remain available.

## 18. Representative Session

Launch → external QGC calibration → internal camera calibration → connect/verify WIRIS, Pixhawk, Base GNSS → Survey-in/NTRIP → initial WIRIS sync → RTCM/fix_type check → physical baseline alignment → physical R/P/Y alignment → repeat evidence → final sync check/resync → confirm YOLO/measurement settings → measure → save → review.

## 19. Failure-Oriented Example

If measurement becomes unavailable during recording, preserve recording state, show upstream abnormal states, open diagnostics, expose rejection reason, recover, continue.

## 20. Success Criteria

- readiness visible immediately
- video and relevant state visible together
- measurement and validity visible without searching
- recording state always determinable
- diagnostics accessible without losing active workspace
- common configuration can be changed while observing effect
- calibration has clear prerequisites/results/saved state

## 21. Summary

> task continuity + persistent system awareness + direct diagnostics
