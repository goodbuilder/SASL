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
2. Start Base Survey-in or NTRIP. For NTRIP, configure the provider's caster hostname,
   port, mountpoint, TLS setting, GGA policy, and credential environment-variable names in
   `Settings > GNSS`; new field settings use the NGII RTS2 Base-assisted profile.
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
each action. QGC remains the calibration executor. The Pixhawk page's `Pixhawk Parameters`
group provides a MAVLink PARAM readback of the QGC/PX4 result for each P1/P2, split into
Accelerometer, Gyroscope, Compass/Magnetometer, Level Horizon, and manual Board Orientation
tabs. The first four tabs expose a guarded `Reset to PX4 default` action that resets only
that calibration's owned parameters through the connected firmware's `param reset` command;
the action requires a disarmed vehicle and refreshes the readback after completion. Board
rotation and `SENS_BOARD_Z_OFF` remain manual parameters and intentionally have no calibration
reset button.

`CAL_MAG*_PRIO` is displayed with the PX4 semantic levels `Uninitialized (-1)`,
`Disabled (0)`, `Min (1)`, `Low (25)`, `Medium (50)`, `High (75)`, and `Max (100)`.
If a connected vehicle reports a non-standard raw value, Workstation preserves and shows
it as `Custom (raw)` rather than silently remapping it. MAG offsets, scales, soft-iron,
and board-rotation values remain PX4 sensor-axis parameters; Workstation does not convert
them to NED, ENU, or camera coordinates.

### Calibration calculation execution

`Checkerboard capture validation` copies the currently displayed left/right images, fixes
their RTCP/NTP pair decision at capture time, then encodes PNG files and runs checkerboard
detection in the shared background calibration slot. Its purpose is to prevent image
encoding and OpenCV detection from blocking live controls. A stereo capture assumes both
frames contain RTCP/NTP camera timestamps within the 100 ms pair limit; otherwise neither
side is admitted as a stereo pair. The output contract is unchanged OpenCV image
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

`R,t only` is the fixed-intrinsic stereo path. Its purpose is to estimate the physical
left/right rig transform from a large checkerboard without replacing the selected K,D.
Inputs are a selected intrinsics profile, matching synchronized left/right checkerboard
pairs, checkerboard dimensions, and square size in metres. It calls OpenCV stereo
calibration with `CALIB_FIX_INTRINSIC`. Output is a standalone stereo-extrinsics profile:
`R_left_to_right`, `T_left_to_right_m`, method, source zoom, source-intrinsics path/hash,
pair count, reprojection error, and the frame contract
`X_right = R_left_to_right X_left + T_left_to_right_m`. The derived right-camera origin in
left optical RDF is recorded separately as `C_right_in_left = -R^T T`; it must not be
treated as an alias of OpenCV T. PASS means at least three stereo pairs produced a finite,
proper rotation and metre translation. Input/OpenCV errors are FAIL and leave the previous
candidate unchanged.

Intrinsics and extrinsics are independently selectable. The general cross-zoom workflow is
`K,D` from the intended operating zoom B plus `R,t` estimated at any independently chosen
calibration zoom A. A and B are not fixed presets and may be any supported zoom levels; for
example, a 1x `R,t` profile can be paired with exact 8x `K,D`. A difference between A and B
is recorded as cross-zoom provenance and does not by itself reject `R,t`.

Exact operating `K,D` is a separate, fail-closed geometry gate. Each left/right profile
must match the observed operating zoom, native frame resolution, physical camera serial,
and stabilization state; zoom readback must also be present and fresh. Unknown or
mismatched evidence blocks optical-pose solving/application and other derived geometry
that claims this profile, while raw recording remains available. This distinction lets an
experiment retain useful evidence without silently treating requested zoom or a legacy
profile as runtime truth. Both intrinsics and extrinsics paths/hashes/source zooms and the
same-/cross-/unknown-zoom classification remain in provenance.

The Calibration workspace accepts an arbitrary supported visible zoom in `Selected zoom`,
can apply it to both cameras, and can query each camera with `GZVV`. Requested and read-back
values remain separate. A mismatch remains visible as evidence and makes exact operating
`K,D` inapplicable to derived geometry; it does not prohibit preserving raw camera,
telemetry, or session evidence.

### HR Capture, import, and HR→OZ bridge

The Recording workspace exposes one repeatable `HR Capture` action with an explicit
`Pre`, `Post`, `Checkpoint`, or `Unspecified` purpose. Every press sends `SIHR TRUE`,
triggers `CPTR` on the left and right cameras, and polls `ICPT` until each camera reports
completion. The request/result timestamps, left-minus-right request timing, zoom readback,
SDK responses, and completion state are appended to `<session>/hr_captures.jsonl`.
A capture failure is evidence and does not block the measurement session.

Capture initially leaves the photos in WIRIS internal storage. After retrieving them by an
external camera-storage workflow, the operator selects the capture event and explicitly
imports the left/right files. Workstation copies the exact original bytes into the session
without image re-encoding, verifies that they decode, and records byte count, SHA-256,
native resolution, camera side/serial, capture purpose, and import time. Imported HR is
therefore comparison/PnP evidence, not an automatic replacement for the operating-zoom
video image.

Any later HR-to-operating-zoom transfer is per physical camera and has the explicit optical
RDF contract `X_OZ = R_OZ_from_HR X_HR + T_OZ_from_HR`. A full-SE(3) bridge may transfer
rotation and translation; a rotation-only bridge records translation as unobservable and
must not transfer a stereo baseline. Bridge inputs, solver metrics, hashes, and pre/post
comparison remain analysis evidence rather than a hidden calibration truth.

### Optical pose initialization

The fourth local tab of Alignment is `Optical pose`. Its workflow is:

1. Capture a synchronized frozen native left/right reference burst. The exact operating
   per-camera `K,D` applicability decision and the input image IDs/hashes are frozen with it.
2. Annotate the actual horizon, a mountain-ridge polyline, or a Drone target point in native
   OpenCV pixels (`u` right, `v` down). The frozen reference viewport is fixed at 16:9 with
   a center crop (no side margins); this presentation transform does not alter saved
   coordinates, and a small red hover dot shows the native pixel under the pointer.
3. Select `Manual Horizon` or `Manual Mountain` to mark points directly. The corresponding
   `Auto Horizon Roll/Pitch` and `Auto Mountain PnP` buttons remain available after manual
   marking; they run the owner solver over the frozen evidence. Selecting Horizon disables
   the Yaw DOF because a horizon cannot observe Yaw. Use Mountain PnP or live target-Drone
   RTK with explicit camera/target line of sight for Yaw. Terrain PnP also requires a
   declared vertical datum; a skyline alone is not promoted to a 3-D correspondence or
   truth source.
4. Inspect immutable candidates, residual/uncertainty evidence, warnings, and observable
   degrees of freedom. Independent non-overlapping candidates may be combined.
5. Apply only selected DOF that the candidate actually optimized and observed, or revert to
   the previous immutable profile. The main KPI is right-minus-left optical boresight Yaw;
   low numerical residual never overrides missing applicability/observability evidence.

`Existing calibration` is a reference, not another Alignment solver. `Pre-run Calibration`
creates and applies the checkerboard `K,D + R,t` bundle; Alignment reads the active profile and
baseline as the prior/reference for optical-pose candidates and does not recalculate or overwrite
that calibration in this tab. If the selected calibration is not the intended operating profile,
the exact operating `K,D` applicability gate remains visible and derived geometry stays blocked.

`Combine candidates` is an explicit composition step. The operator checks at least two immutable
candidate IDs (for example, Horizon R/P plus Drone RTK Yaw); Alignment does not choose the latest
candidate and does not average poses. The owner accepts the request only when all sources share the
same explicit prior, the exact operating `K,D` profile, and non-overlapping optimized DOF. The
result is a new immutable candidate with source IDs, hashes, and quality evidence; it is not applied
automatically.

The method contract is: purpose = initialize only the observable optical-pose DOF; runtime inputs =
the frozen native image burst, the active exact operating `K,D/R,t` evidence, and the explicitly
provided horizon, mountain/GNSS, or Drone RTK observations; output = an immutable candidate in PX4
virtual-camera FRD to world NED; PASS = applicability, observability, prior, and method-specific
input checks pass; FAIL = no candidate is applied and the reason remains visible. Calibration-time
checkerboard results are reference evidence, not runtime ground truth.

Pose uses PX4 virtual camera FRD to world NED; image measurements and stereo calibration
remain OpenCV optical RDF. Workstation stores the candidate and selection history. It does
not label a horizon, Drone RTK sample, nominal profile, or later UAM trajectory as ground
truth.

When Terrain PnP is pressed, the owner asks for a JSON object such as
`{"left":{"object_points_ned_m":[[N,E,D],...]},"right":{"object_points_ned_m":[[N,E,D],...]}}`.
The points must correspond one-for-one, in order, to the ridge points clicked in each frozen
frame. Drone RTK Yaw similarly asks for per-side JSON such as
`{"left":{"camera_center_ned_m":[N,E,D],"target_point_ned_m":[N,E,D],"timestamp_alignment_verified":true}}`;
the right side may be supplied in the same object. A precomputed normalized ray and expected
LOS may be supplied instead. These files are hashed into the candidate and are not promoted
to live truth.

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

Each run also freezes a nominal K,D/R,t profile at recording start. If an optical-pose
selection becomes active later, the same immutable detection evidence is evaluated again
with the active profile. Pair rows carry `nominal_*` and `active_*` validity, range, output
frame, profile hash, selection ID, and effective pair boundary; an unprefixed legacy value
continues to mean the active calculation. Pixhawk-baseline rows retain this dual-profile
state as explicitly not-applicable rather than mixing GNSS geometry with checkerboard R,t.

The Recording workspace includes a PX4 ULog evidence group for P1, P2, and Target Drone.
Manual imports copy exact bytes into the session, validate ULog framing/size/SHA-256, and
record source role, ownership claim, runtime identity, and relative path. Automatic logger
capture is only allowed through an injected adapter that proves the same boot/connection,
correlated command receipt, post-command status, and remote log path; pre-existing or
ambiguous loggers are never stopped. UAM target ULog is normally `DEFERRED` until the
external file is supplied.

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
