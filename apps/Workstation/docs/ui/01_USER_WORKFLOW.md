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
13. Type-owned data review and later assembly

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
1. Select and explicitly Connect WIRIS, Pixhawk P1/P2/Drone, and Base GNSS. Pixhawk roles default
   to independent Telemetry Radio links, while Ethernet remains selectable. P1/P2/Drone must match
   MAVLink system IDs 1/2/3 and remain in `QUALIFYING` until HEARTBEAT+ATTITUDE stay continuously
   fresh for 10 seconds; only then may their transport status become `LIVE`. Ten-minute and 60-minute observations are
   separate field-stability evidence, not routine readiness delays.
   Before assignment, `Find USB` or `Find network` starts a fresh transient Discovery snapshot and
   displays only that command's new result. USB shows one row per physical endpoint and uses
   heartbeat sysid—not USB order—as the role evidence. Saved assignments remain intact until an
   explicit Assign action, and only explicit Connect opens the selected transport.
   For an Ethernet Drone, `Devices > Telemetry > Find Network` passively lists bounded UDP MAVLink
   sources. Only a HEARTBEAT reporting sysid 3 is selectable. `Use selected` fills the URL, after
   which the operator still presses Apply, Save and Connect. Apply/Save preserve the accepted form
   URL across live refresh; discovery never mutates host/PX4 networking or connects automatically.
2. Start Base Survey-in or NTRIP. For NTRIP, configure the provider's caster hostname,
   port, mountpoint, TLS setting, GGA policy, and credential environment-variable names in
   `Settings > GNSS`; new field settings use the NGII RTS2 Base-assisted profile.
3. Set/check WIRIS time synchronization for recorded NTP/clock evidence. Clock observation runs for
   both cameras in parallel, while STUS mutation is ordered Left exact-OK then Right. Clock success
   is a fresh/stable Left/Right relative RTCP delta at or below 100 ms, but it does not own live
   distance pairing or validity.
4. Verify RTCM delivery to each currently eligible Left/Right Pixhawk and record numeric `fix_type`;
   do not assume every condition reaches `RTK_FIXED=6`. A stalled side must show RTCM blocked while
   the other side continues independently.
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

The Pixhawk page also shows a read-only `Dr` geometry section below the baseline/yaw evidence.
`Dr from Baseline Center`, `Dr from P1`, and `Dr from P2` expose `F/R/D/Rg` in their explicitly
named local frames. `∠Dr-P1-P2` and `∠Dr-P2-P1` are separate unsigned triangle angles at P1/P2,
calculated from horizontal GNSS North/East components only. `Dr` means Drone Pixhawk; the `D`
axis in `F/R/D` remains Down. Missing or stale inputs stay visible as `WAITING`/`STALE` evidence
and do not change the existing connection or measurement workflow.

Pixhawk startup, Radio device return, transport changes, NIC assignment, USB Ethernet bootstrap,
role swap, reboot, and firmware completion never reconnect a board automatically. The operator
must press the named board's `Connect` action. Connect performs identity, GCS heartbeat,
process-local stream requests and RTCM routing only; parameter, shell, persistent Ethernet,
MAVFTP and firmware actions are manual.

Calibration PARAM inspection is operator-requested. Startup, heartbeat/status updates, reconnect,
and the first transition to `LIVE` do not start a snapshot. Until the operator presses the existing
side-specific Refresh command, the rows remain `PARAM readback 대기 중`. Reset/apply/save keeps its
explicit verification readback.

Network selection follows `Find network -> select physical NIC -> Assign Camera NIC/P1/P2 -> review
exact before/after roles, addresses, routes and active owners -> confirm/authenticate -> readback ->
device evidence`. Discovery remains read-only and lists every current wired NIC, including link-down
or address-less choices; one combined NIC and arbitrary multi-NIC splits use the same table.

Confirmed Assign applies only Workstation-owned role entries through NetworkManager, target first,
then removes only the moved role from the source NIC. Other roles and unrelated settings remain.
If affected Camera/Pixhawk owners were active, Workstation stops/reconnects only those on the
source/target NICs and saves after a new Camera frame or expected-sysid Pixhawk HB+ATT. Unrelated NICs
continue. Failure rolls back; Measurement/recording/save/finalization rejects before mutation.
Independent P1/P2 cards retain State/NIC/Local IPv4/Endpoint/Route and Connect still validates each
kernel route. Same-NIC topology has no warning/readiness gate; device liveness owns readiness.

P1/P2 Ethernet MAVLink state is evidence-driven: `ATTITUDE STALE` retains current heartbeat/other
traffic and permits only a bounded ATTITUDE request; `ETH MAVLINK RX STALLED` means all expected
system traffic has been silent for 10 seconds and blocks RTCM/commands without cycling the host
socket. Recovery is available only on the named stalled side. `Find USB` must have produced one
fresh unique heartbeat mapping (P1=sysid1, P2=sysid2); the operator confirms side/device/sysid and
the worker immediately revalidates it before one reboot command. Missing/ambiguous identity or an
active Measurement/save/finalization yields manual named-side power-cycle guidance. A sent reboot
is not success: Ethernet must enter `RECOVERING` and pass another 1-minute qualification.

The existing P1/P2 IMU table is also receive-evidence-driven. Initial Stock-profile command send is
not success: each `SCALED_IMU/2/3` row becomes `LIVE` only after a fresh expected-system packet.
Missing/stale streams use bounded side-specific retries while the Ethernet link is command-eligible;
`WAITING` and `STALE` remain visible and all-RX stall sends no retry.

The same Pixhawk IMU magnetometer view appends RM3100 `CAN1_MAG`/`CAN2_MAG` rows when stable
firmware emits the named `DEBUG_VECT` telemetry. These rows retain the `mavlink/DEBUG_VECT` source
and are not treated as `SCALED_IMU` streams.
When stable firmware does not emit those compatibility samples, the existing Pixhawk Shell owner
polls `listener sensor_mag` instances and `listener vehicle_magnetometer` so the same RM3100 rows
can be observed without flashing custom firmware. Shell rows retain their `px4-shell/sensor_mag`
source and diagnostic-rate freshness.

For P2 Ethernet, select the configured Ethernet transport, run `Find network` and apply a candidate
only when the host NIC/route needs changing, then press `Connect`. The normal connection path checks
the configured NIC, local address, route, peer endpoint, and expected P2 sysid Heartbeat/ATTITUDE.
It does not change PX4 board settings or run a separate topology workflow; use the explicit ETH
MAV/USB board-side operation when provisioning is required.

`CAL_MAG*_PRIO` is displayed with the PX4 semantic levels `Uninitialized (-1)`,
`Disabled (0)`, `Min (1)`, `Low (25)`, `Medium (50)`, `High (75)`, and `Max (100)`.
If a connected vehicle reports a non-standard raw value, Workstation preserves and shows
it as `Custom (raw)` rather than silently remapping it. MAG offsets, scales, soft-iron,
and board-rotation values remain PX4 sensor-axis parameters; Workstation does not convert
them to NED, ENU, or camera coordinates.

The visible `Devices > Pixhawk` page provides independent P1/P2 MAG Priority controls for the
logical roles `BOARD`, `CAN1`, `CAN2`, `GPS1_MAG`, and `GPS2_MAG`. `Apply & Verify` requires
Disarm, verifies the CAL_MAG PARAM readback, then waits for a post-apply
`vehicle_magnetometer.device_id` and fresh ATTITUDE. `ACTIVE VERIFIED` is side-specific; a P1
failure never changes P2. A live-Pixhawk Measurement is admitted only when each required side is
verified; methods without live Pixhawk inputs retain their existing readiness rules.

The same cards also show a read-only selection preview: the live `sensor_mag` instance-to-device
map, the priority candidate implied by the current combo values, and the actual PX4 selected
device. CAN2 priority remains editable even when its instance is outside the stock selector
range; the screen reports candidate versus actual instead of blocking the operator or inventing
an active-source result.

The Pixhawk `Magn` tables add read-only `RAW yaw°`, `Corrected yaw°`, and `PX4 yaw°` beside each
IMU/CAN row.
RAW is the signed `atan2(-y,x)` diagnostic in producer sensor axes with no calibration or tilt
correction. Corrected yaw applies the matching read-back `CAL_MAG` transformation and remains
`PARAM 대기` until the operator performs the manual parameter refresh. PX4 yaw is retained as
the fresh firmware `ATTITUDE.yaw` reference only when the row's exact
device ID matches the active `vehicle_magnetometer` device; standard `SCALED_IMU*` rows without
that identity remain `ID 확인 대기`; its `EKF2_MAG_DECL` declination correction is included in
the estimator value. The adjacent hidden `Δyaw°` remains `PX4−RAW`. The existing priority/active-device verification procedure
is now exposed on the same Pixhawk page, while the Advanced PX4 MAG controls remain a fallback.

### Calibration calculation execution

Measurement's Method result table follows one pair for up to one second, updating each
Method as its result arrives. Then it advances to the newest pair; calculation and saving
continue for every pair. Coordinate mode and row selection only change the view. The
selected row's detail reports matching Quality, all coordinate forms and R,t inputs.
Methods that previously participated remain visible when blocked. New runs clear old
values; after Stop, late results may still complete the last run. Existing representative
Method controls and primary history are independent of this read-only comparison.

Mountain/Cloud Solve uses capture-request GNSS Yaw90 with P1 Roll/Pitch. Readiness
`Mountain/Cloud 기준` reports missing reference inputs; Base R,t is not required for these
methods. Capture remains available without telemetry; Solve then requires recapture with
reference inputs. Candidate details show assumed Left and image-derived Right poses.
Reverse/telemetry changes affect future captures only. Cloud fills translation from pair
GNSS, and both actual measurement poses are recorded.

`Checkerboard capture validation` copies the currently displayed left/right images, fixes
their timing decision at capture time, then encodes PNG files and runs checkerboard
detection in the shared background calibration slot. Its purpose is to prevent image
encoding and OpenCV detection from blocking live controls. A stereo capture assumes both
Live/Auto frames contain same-host decoded-frame app-receive monotonic timestamps within the
configured pair limit; NTP/RTCP remains record-only and is not a fallback. Missing or over-limit
receive evidence makes checkerboard-valid sides K,D-only instead of admitting them to stereo R,t.
Verified exposure timestamps retain their higher-quality path, while FTP retains its separate
CPTR-dispatch proxy. The output contract is unchanged OpenCV image pixels with capture metadata
recording both app-receive timestamps, signed Right-minus-Left delta, original RTCP/NTP evidence,
and the allow/reject decision. Host receipt is `HOST_RECEIVE_PROXY`, never verified exposure. PASS
means at least one available image was encoded and passed checkerboard detection (or was
explicitly retained because OpenCV was unavailable); FAIL leaves rejected images out of the
calibration set. This capture method does not use ground truth or produce NED/FRD values.

Auto capture has an additional exact-source gate before this timing decision. The parser must mark
the AU non-delta, its NAL list must contain type 1 and exclude type 5, and the AU PTS must equal the
decoded-buffer PTS. Checkerboard detection carries the resulting current-generation per-camera
`FrameKey`, and capture pins exactly that ring slot. A missing or overwritten key waits for the next
candidate; Auto never selects a nearby timestamp. NAL1 remains non-IDR evidence, not an integrity
or random-access claim. Manual Live Pair and FTP/OZ retain their existing source paths.

The default Calibration owner is repository-relative `apps/Workstation/calibration/`; the setting
marker `calibration` selects that root, and each new run receives a collision-free timestamped
child. Existing data-root calibration bytes are historical input and are never moved or merged
automatically. The default Calibration `촬영` tab projects the existing Left/Right checkerboard overlays and a
source-backed capture summary. Per-side state distinguishes detection, found/not-found, missing
frame, stale evidence, and OpenCV unavailability. `이번 촬영` counts only checkerboard validation
jobs that actually started; Auto waiting/NAL/duplicate-pose ticks do not inflate it. `현재 캡처 세트`
independently scans the active root and reports Left/Right saved images, persisted stereo candidates,
and per-side K,D-only images. These are stored candidates before manual exclusion and solver pose
filtering, not proof that calibration quality or Apply readiness passed.

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

Operating `K,D` is a separate, fail-closed geometry gate for profile presence/identity and
native left/right frame resolution. Camera zoom readback, profile-to-live or left-to-right
zoom agreement, physical camera serial, and stabilization state remain evidence only;
missing, stale, future, or mismatched values neither block geometry nor appear as readiness
advisories. Raw recording remains available for every result. Both intrinsics and
extrinsics paths/hashes/source zooms and the same-/cross-/unknown-zoom classification remain
in provenance.

The Calibration workspace accepts an arbitrary supported visible zoom in `Selected zoom`,
can apply it to both cameras, and can query each camera with `GZVV`. Requested and read-back
values remain separate. Missing or mismatched zoom remains saved evidence but does not change
operating `K,D` applicability or prohibit raw camera, telemetry, Measurement, or derived geometry.

### Capture HR Still, import, and HR→OZ bridge

The Recording workspace exposes one repeatable `Capture HR Still` action with an explicit
`Pre`, `Post`, `Checkpoint`, or `Unspecified` purpose. Every press sends `SIHR TRUE`,
triggers `CPTR` on the left and right cameras, and polls `ICPT` until each camera reports
completion. The request/result timestamps, left-minus-right request timing, zoom readback,
SDK responses, and completion state are appended to `capture_events.jsonl`.
A capture failure is evidence and does not block Measurement.

Capture initially leaves the photos in WIRIS internal storage. After retrieving them by an
external camera-storage workflow, the operator selects the capture event and explicitly
imports the left/right files. Workstation copies the exact original bytes into the data root
without image re-encoding, verifies that they decode, and records byte count, SHA-256,
native resolution, camera side/serial, capture purpose, and import time. Imported HR is
therefore comparison/PnP evidence, not an automatic replacement for the operating-zoom
video image.

Any later HR-to-operating-zoom transfer is per physical camera and has the explicit optical
RDF contract `X_OZ = R_OZ_from_HR X_HR + T_OZ_from_HR`. A full-SE(3) bridge may transfer
rotation and translation; a rotation-only bridge records translation as unobservable and
must not transfer a stereo baseline. Bridge inputs, solver metrics, hashes, and pre/post
comparison remain analysis evidence rather than a hidden calibration truth.

### Alignment R,t method workflow

`Baseline Yaw90` is appended to the existing method tabs. Its live views replace the visible
frozen-reference/candidate regions only while selected, retaining their objects and operator state.
Observe P1→P2 azimuth plus ∠Dr–P1–P2 / ∠Dr–P2–P1 in the fixed GNSS table alongside the P1/P2
Roll/Pitch table and camera pair. Iterate yaw aiming and physical leveling; no 0° target column or
automatic completion gate is added. Center line is local display chrome, now removed from
Calibration. Reverse Apply/Save has moved from Measurement Settings; run transitions lock edits.
The original Telemetry midpoint table is retained as collapsed reference data in this tab.
Pixhawk's shared direction/alignment and Drone geometry displays remain available.

Select `Alignment` in Navigation. Common readiness and frozen-reference controls appear above six
promoted method tabs; there is no separate `Optical pose` Navigation item or outer `R,t methods`
wrapper. The workflow is:

1. Select `Capture Live Pair` to capture the newest decoded native left/right frames from the
   live RTSP streams, or `Import Pair` to choose local Left and Right images. Local Import asks no
   zoom question and emits no synthetic `zoom_index`; zoom remains provenance only. Neither path
   is raw RTP packet capture. The operating per-camera `K,D` applicability decision and the input
   image IDs/hashes are frozen with it. Missing/invalid K,D identity and native-raster mismatch
   remain blocking.
2. Annotate the actual horizon, a mountain-ridge polyline, or a Drone target point in native
   OpenCV pixels (`u` right, `v` down). The frozen reference viewport is fixed at 16:9 with
   a center crop (no side margins); this presentation transform does not alter saved
   coordinates, and a small red hover dot shows the native pixel under the pointer.
3. Select `Manual Horizon`, `Manual Mountain`, or `Manual Target` to mark points directly.
   `Manual Horizon` marks two horizon points for Roll/Pitch only. `Manual Mountain` marks
   same-order terrain correspondences for the GNSS + Mountain metric `R,t` solve (at least
   four pairs). Mountain translation requires finite capture-bound P1/P2 positions, a finite
   non-zero NED baseline, and `fix_type >= 3` on both sides. RTK Fixed is not required; ordinary
   3D Fix or RTK Float remains visibly `ESTIMATED`. `Manual Target` marks one Drone RTK target
   pixel per image for the
   single-target Drone RTK Yaw fallback; completing both sides also accumulates that Frozen
   pair with capture-time Drone/P1/P2 GNSS for JSON-free multi-view `R,t`.
   The corresponding
   `Auto Horizon Roll/Pitch` and `Auto Mountain PnP` buttons remain available after manual
   marking; they run the owner solver over the frozen evidence. Review the automatic overlay before
   `Use suggestion`. For Horizon and Mountain, `GOOD`/`LOW_CONFIDENCE` remains quality evidence
   and does not decide copy permission. Horizon copy requires exactly two current-frame finite
   in-frame endpoints per side separated by at least 25% of native width. Mountain copy requires at
   least four current-frame finite unique/distributed paired points after exclusions. Structurally
   blocked proposals remain overlay-only. Horizon candidates own only
   Roll/Pitch because a horizon cannot observe Yaw. Use Mountain PnP or live target-Drone
   RTK with explicit camera/target line of sight for Yaw. Terrain PnP also requires a
   declared vertical datum; a skyline alone is not promoted to a 3-D correspondence or
   truth source.
   The solver tabs are `Horizon`, `Mountain + GNSS`, `Terrain PnP`,
   `Drone Rotation`, `Drone Multi-view`, and `Combine`. Tab changes automatically
   select Horizon/Ridge/Target annotation mode without deleting points from other modes.
4. Inspect immutable candidates, residual/uncertainty evidence, warnings, observable
   degrees of freedom, and—when reviewing a Mountain + GNSS candidate—the numeric OpenCV
   relative `R_right_from_left_rdf`, `t_right_from_left_rdf_m`, and physical right-origin
   vector in the Candidate detail. Independent non-overlapping candidates may be combined.
5. Press `Apply candidate` to apply every DOF that the candidate owner optimized. Apply is
   blocked if any owned axis is not observable on both camera sides; there is no editable
   Candidate Apply-DOF mask. Use `Combine` when axes must come from different candidates, or
   revert to the previous immutable profile. The main KPI is right-minus-left optical boresight Yaw;
   low numerical residual never overrides missing applicability/observability evidence.

`Existing calibration` is a reference, not another Alignment solver. `Pre-run Calibration`
creates and applies the checkerboard `K,D + R,t` bundle; Alignment reads the active profile and
baseline as the prior/reference for optical-pose candidates and does not recalculate or overwrite
that calibration in this tab. Missing/invalid K,D identity or a native-frame resolution mismatch
keeps the operating `K,D` applicability gate visible and derived geometry blocked.

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

### Measurement method readiness and parallel evaluation

Measurement and Overview always show every catalog method category (ten, including the Cloud
far-field rotation + live GNSS hybrid). At child creation the latest
successful non-Pixhawk Draft and exact operating K,D are frozen as child-only inputs without
changing global Alignment Active. Horizon combines observed Roll/Pitch with pair-frozen Pixhawk
Yaw and GNSS Translation. Single-target Drone Rotation re-solves Yaw with pair-frozen Pixhawk
Roll/Pitch; full Drone Rotation adds pair-frozen GNSS Translation. Cloud far-field rotation
supplies Roll/Pitch/Yaw from sky/cloud correspondences and adds pair-frozen GNSS Translation; it
never observes translation itself. Mountain, Multi-view, Terrain, and Combined must already be
full 6-DOF. Readiness is per method, so an unavailable GNSS-dependent
method does not block an independent Checkerboard or full Multi-view/Terrain profile.

Every matched pair fans out to every catalog method from one immutable snapshot. Pixhawk attitude +
GNSS/RTK is always the representative result and is never replaced by a comparison method. Missing
input, solver failure, timeout, and queue overflow remain explicit per-method terminal outcomes.
The pair's attitude/GPS snapshots are selected before hybrid geometry is resolved, and the final
R,t plus those exact snapshot values are covered by `measurement_geometry_sha256`.
That selection retains the last valid process-local sample per side, symmetrically for P1/P2
attitude and P1/P2 GPS: a side that has already supplied one valid sample keeps supplying it while
its live delivery is stalled or disconnected, and a newer valid sample replaces it automatically.
Attitude received before the current Measurement is accepted. A side that never produced a valid
sample, a non-finite value, a GPS fix below the configured threshold, and a non-positive P1->P2
baseline stay hard blocks. Retained values are process-local and discarded on restart; reuse is
reported through the existing `reused_inputs` evidence and method advisories without any new
control or age display. A rig moved while its telemetry is unavailable cannot be detected, which is
the accepted operating limit of this behavior.
Where P1 and P2 cannot be connected at the same time, the retained GPS pair is additionally bound
to the correction frame it resolved in: the correction source identity and the Base coordinate
profile hash. Changing the Base coordinate, caster/mountpoint, or operation mode drops the retained
fixes for both sides and a sample observed before that change is not re-admitted, because a rover
fix carries its Base's asserted coordinate and two fixes only form one P1->P2 baseline while that
coordinate is identical. Swapping which board is plugged in does not change the frame and never
drops anything. The pair's capture span is reported as an advisory beyond the configured
left/right timestamp limit and is never blocked. A Base that physically moves while asserting the
same coordinate is not detectable in software and remains a field procedure rule.
When no method is ready, the matched detections/native references are retained and every
method stream records BLOCKED rather than discarding the pair or fabricating R,t.
Candidate details show left/right observability independently; Apply/Combine enables a common
axis only when both sides observed it.

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
- requested decoder mode and actual per-side GPU/CPU decoder, including Auto fallback reason
- per-side RTP packet input/suspected missing-packet rate, estimated loss percentage, H.264
  access-unit rate, and decoded Hz; these are worker-measured rolling values, not camera settings
- recording
- detection
- diagnostics

Changing camera parameters should not unnecessarily hide live image.

The field decoder workflow uses Auto by default: startup checks the `nvh264dec` factory, NVIDIA
management interface, and CUDA Driver API; then start Preview and confirm `GPU · nvh264dec` for both
sides in Camera Status, and use CAM tooltips/Diagnostics Performance for availability or fallback
detail. `CPU · avdec_h264 (selected)` is a valid explicit Software choice; `CPU fallback` is a WARN
that preserves Preview after an NVIDIA failure. YOLO CUDA/CPU and H.264 NVDEC/CPU are separate
resource statuses.

Normal launches permanently admit the current decoded frame even when NAL5/IDR or RTP integrity is
not proven. The operator sees `INPUT UNVERIFIED`; there is no strict/tolerant setting or process
environment switch. This changes no WIRIS encoder, RTSP, NIC, timing, calibration, or coordinate
setting.

## 7. Measurement Operation

The normal path is `Measurement > 측정 시작 → 측정 종료`. There is no parent folder, child,
Finalize, Seal, completeness, or Retry-Save command. Measurement is an observation interval:
Start freezes its generation/time and immediately enters `RUNNING`, including with zero available
sources. Camera, P1/P2, Base GNSS/NTRIP, Target and recording join or leave independently.

Manual record is stopped asynchronously and is not resumed. A competing camera owner makes only
the camera/recording sources wait; the observation run and available telemetry start immediately.
An actual failed camera does not reconnect automatically: the operator uses the existing side
reactivation action, and a newer-generation current frame rejoins the same run. Recovered video
uses a new no-overwrite MP4 segment. Manual/Auto STUS is clock work and may run during Measurement/YOLO/
recording without stopping or restarting their workers. Its logical clock-epoch transition blocks
only time-dependent cross-epoch results. Old Session folders and writer failures are never
consulted.

The readiness table remains an evidence projection. The compact method table lists every catalog
method category (ten); missing candidates/profiles remain `BLOCKED` with their required inputs. Clicking one
method navigates to read-only detail without merging PX4 body-FRD/world-NED, GNSS NED, candidate
virtual-FRD/world-NED, or OpenCV RDF contracts.

Required visibility:
- transient Measurement state and generation
- independent source state and join/leave reason
- relevant live imagery
- measurement result and unit
- validity/quality
- independent input-media provenance (`INPUT UNVERIFIED` when applicable)
- detection
- timing/synchronization when relevant
- RTK/baseline when relevant
- recording
- storage state/drop count as non-blocking diagnostics

Distinguish raw/corrected/configured/derived/invalid values.

## 8. Recording

Required visibility:
- Manual/automatic recording state
- resolved Measurement data root (default `<SASL_ROOT>/recordings`) and type-owned JSONL/CSV paths
- elapsed time when useful
- per-stream JSONL/CSV writer error/drop state
- relevant system state

Before capture, the Recording workspace shows an advisory experiment preflight. The
distance pass criteria are both camera streams live, effective pair timing within 100 ms,
detection running, active stereo intrinsics, and either P1/P2 MAVLink plus the GNSS/RTK
NED baseline or checkerboard left-to-right optical RDF geometry according to the selected
measurement path. GNSS readiness checks the configured numeric `minimum_fix_type` (default
3); `RTK_FIXED=6` and fixed duration are required only when explicitly selected. The same
preflight also requires a passing physical-alignment result and an explicit final WIRIS
time-sync checkpoint recorded after alignment. `READY`/`BLOCKED` is evidence only and does
not change the existing recording command.

Clock, Frame timing and Media remain separate. Clock VERIFIED uses robust Left/Right relative RTCP
observations at the configured 100 ms default. Actual frame pairing admits only same-epoch pairs at
or below 100 ms, but an occasional missing/outlier frame may be dropped and reported rather than
revoking Clock. Absolute SR-PC is still shown and recorded but is never a pass/fail gate.

Workstation live distance follows the timing preference frozen when Measurement starts. App receive
is the default. `RTCP/NTP preferred` selects pairs only with VERIFIED/current, finite, same-epoch
frame evidence inside the configured limit; any RTCP timing fault selects exact same-host app-
receive monotonic instead. The effective basis has a hard 100 ms cap. Fallback is explicit
provenance and does not by itself downgrade otherwise-valid geometry. `VALID` and
`INPUT UNVERIFIED` may still appear together. PX4 attitude/GNSS association remains host-monotonic.
Apply/Save during a run changes the next Measurement rather than mixing timing bases in one run.

Automatic Measurement MP4 records the overlay-free decoded NV12 branch through an isolated FFmpeg
`h264_nvenc` lossless writer. Its queue is bounded and oldest-drop; overload or recorder failure
does not stop Preview, YOLO, native evidence, or distance. There is no CPU encoder fallback and no
application bitrate, file-size, or duration cap. Workstation classifies the latest run only by file
presence: `SAVED` when both sides have a file, `PARTIAL` when only some files exist, and `FAILED`
when none exist. `SAVED` does not claim playback or integrity verification; later inspection uses
the OS file manager/player.

State candidates: Idle, Starting, Recording, Stopping, Saved, Partial, Failed, Degraded, Error.

Record command is not proof that data is being written.

Each run also freezes a nominal K,D/R,t profile at recording start. If an optical-pose
selection becomes active later, the same immutable detection evidence is evaluated again
with the active profile. Pair rows carry `nominal_*` and `active_*` validity, range, output
frame, profile hash, selection ID, and effective pair boundary; an unprefixed legacy value
continues to mean the active calculation. Pixhawk-baseline rows retain this dual-profile
state as explicitly not-applicable rather than mixing GNSS geometry with checkerboard R,t.

The Data Capture workspace includes a PX4 ULog group for P1, P2, and Target Drone.
Manual imports copy exact bytes into the data root using a creation-time no-replace filename,
validate ULog framing/size/SHA-256, and record source role, ownership claim, runtime identity, and
relative path. Automatic logger
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
`Devices > Pixhawk` keeps the separate P1/P2 ATTITUDE diagnostic. It is not used for the camera
installation angle.

`Alignment > Baseline Yaw90` displays the midpoint reference derived from selected P1/P2 and Drone
GNSS. Its origin is the P1/P2 GNSS antenna midpoint; Right is horizontal P1→P2, Down is NED Down,
and Forward is Right×Down. The acute horizontal installation angle is
`atan2(|Forward|,|Right|)` and `90° error=|90°−angle|`; 90° is the target. `CURRENT` requires every
application-receive age <=2.0 s and a maximum three-source receive-timestamp spread <=1000 ms.
`STALE` retains finite values with actual timing; `WAITING` shows no fabricated zero. Fix type
remains evidence rather than a gate.
This local-level antenna frame is not optical RDF, P1 body FRD, or camera/Measurement
`baseline_center_frd`. Moving this reference to Baseline Yaw90 does not change its recorded fields
or add a user-entered distance.

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

## 15. Data Review and Assembly

Open `Data > Status` to inspect fixed stream JSONL/CSV paths, writer state, last write, and separate
JSONL/CSV drops. Use the file manager or an external analysis tool for authoritative multi-run
assembly; JSONL remains the recovery source and CSV is a convenience mirror. Binary names carry
creation time and explicit UTC offset. Stopping Measurement also queues one derived
`latest_measurement.xlsx` for that most recent generation. Its Summary and source sheets preserve
the raw clocks and add readable KST timestamps; missing fields are `#N/A`.

Watch the fixed `Latest XLSX` status line. `READY` proves a validated workbook was published;
`WAITING/GENERATING/STALE/FAILED/CANCELLED` does not affect Measurement. Use `Update Latest XLSX`
when an automatic update was delayed or failed and `Open Latest XLSX` after publication. A new
Measurement or Manual record takes priority over export. The workbook is derived and replaceable;
fixed JSONL remains the source of truth. No Parquet or legacy `review.xlsx` is generated.

While Measurement or Manual Record is active, each primary Drone GNSS callback attempts one
`gnss.baseline.center` bulk row containing the live GNSS-derived CURRENT/STALE/WAITING result and
the exact selected P1/P2/Drone source evidence. `Data > Status` owns admitted/drop/failure truth.
The latest Measurement workbook adds `GnssBaselineCenter`; Manual-record rows remain JSONL-only.
No new recording control or Measurement gate is introduced.

Legacy `recordings/` trees are file-manager-only and are not scanned or recovered.

## 16. Workflow Priority

Priority A: startup/readiness, live observation, measurement, recording, diagnostics.

Priority B: hardware connection, camera config, PX4, GNSS/RTK, detection config.

Priority C: calibration, later data analysis, advanced configuration.

## 17. Context Preservation Rules

1. Diagnostics shall not replace primary Workspace.
2. Property changes shall not unnecessarily hide affected data.
3. Recording state remains visible across navigation.
4. Critical connectivity remains visible during normal operation.
5. Command does not imply successful state.
6. Advanced workflows may replace Workspace but Global Status/Diagnostics remain available.

## 18. Representative Run

Launch → external QGC calibration → internal camera calibration → verify Data root → connect/verify
WIRIS, Pixhawk, Base GNSS → Survey-in/NTRIP → WIRIS sync → RTCM/fix_type check → physical/optical
Alignment → confirm P1/P2 1-minute LIVE qualification → confirm YOLO/Measurement settings → measure → stop → immediately repeat or wait →
open/update the latest XLSX for a quick review or assemble authoritative fixed streams later.

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

### Target reference discovery

In Data → Status, select Drone, choose Find Network or Find USB, inspect the source and SYS/COMP, then Use selected and Connect target. Use selected edits only the draft. Save persists preferences separately. Candidate identity expires after30s and is revalidated at Connect. Manual endpoint input remains available; UAM stays DEFERRED without discovery.

Observe heartbeat and GPS/ATTITUDE freshness separately. No heartbeat, no GPS, invalid fix, stale data, busy resource and identity mismatch retain distinct feedback. Stop before changing an active connection/platform. Discovery cancellation and Stop complete before another Target operation starts. No startup auto-connect or automatic retry occurs.
