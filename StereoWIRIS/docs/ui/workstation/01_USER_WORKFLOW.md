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
3. Camera clock and stream synchronization
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
1. Camera IP/control/RTSP readiness
2. WIRIS PC clock, RTCP/NTP, and still-image clock evidence
3. Pixhawk transport and P1/P2 role mapping
4. GNSS Base/Rover operation and NED baseline readiness
5. Calibration setup, capture/intrinsics, and validation
6. Measurement preflight, Preview, Detection, Record, stop, and Save session

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

Expose connectivity, attitude, timestamp/age, source, relative attitude, and relevant sensor state. Detailed raw telemetry belongs in Diagnostics.

## 12. GNSS / RTK Verification

Expose receiver state, fix/quality, satellites, eph/epv, baseline, RTK, source, timestamp/age.

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

Launch → camera readiness → time/sync evidence → connect Pixhawk → confirm GNSS/RTK → calibration → preview → enable detection → measurement quality check → start recording → monitor → diagnose if needed → stop → confirm saved session → review.

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
