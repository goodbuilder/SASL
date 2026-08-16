# SASL StereoWIRIS Workstation

## Purpose

This package provides the operator Workstation for synchronized stereo-camera capture,
calibration, measurement, Pixhawk/GNSS evidence, and recording. It owns its runtime and UI
independently from the Legacy application.

## Current Run Flow

From the repository root, verify imports with `./apps/Workstation/run.sh --check`, then
start the UI with `./apps/Workstation/run.sh`. Use `--no-start` to open without camera
discovery or preview and `--start-pixhawk` to connect the configured Pixhawk endpoints
after launch.

For checkerboard calibration, select the calibration zoom, command both cameras to that
zoom, confirm the actual zoom readback, capture synchronized pairs, and calculate/save the
required intrinsics or stereo `R,t` profile. At measurement time, load the `K,D` profile for
the intended operating zoom and independently load the chosen `R,t` profile, then apply the
bundle.

`HR Capture` can be pressed repeatedly whenever the operator needs high-resolution evidence.
Each press enables visible-HR still storage, captures one photo on each camera, and waits
for both cameras to report completion. `OZ Capture` sits beside it in `Recording > Session`
and uses the same `Pre`/`Post`/`Checkpoint` purpose selector. It sends `SIVI TRUE`, `CPTR`,
and polls `ICPT` on both cameras for a camera-native visible/OZ still; the field camera
configuration is expected to provide the higher-resolution FHD evidence. Use `Pre` before
measurement and `Post` after measurement when this camera-local evidence is needed. Both
capture paths are independent of the RTSP/MP4 session video.

For network RTK, open `Settings > GNSS`, select either network-assisted fixed-base or
direct network RTK operation, and enter the caster hostname, port, mountpoint, TLS setting,
and credential environment-variable names supplied by the NTRIP provider. A newly created
settings file starts with the NGII RTS2 Base-assisted profile: `RTS2.ngii.go.kr:2101`,
mountpoint `VRS-RTCM32`, Base GNSS GGA, WGS84 ellipsoid height, and RTCM routing to both
left and right Pixhawk rovers. Other compatible casters remain configurable; existing
custom provider configurations are preserved, while retired legacy values are migrated to
the NGII profile when loaded.

Before starting the Workstation with the NGII profile, provide the credentials through the
environment. The username value is the RTS2-issued personal ID `SASL`; the password is read
from the environment and is never written to the settings JSON:

```bash
export NGII_RTS2_USERNAME='SASL'
export NGII_RTS2_PASSWORD='<RTS2 password>'
./apps/Workstation/run.sh
```

When a validated fixed Base coordinate is applied, the Workstation sends UBX-CFG-TMODE3 with
the fixed-position accuracy in the u-blox `fixedPosAcc` slot at payload offset 20. The
survey-in-only fields remain at offsets 24 (`svinMinDur`) and 28 (`svinAccLimit`); the ACK and
readback path compares the receiver's fixed-position accuracy against that same field.

To monitor NTRIP while it is running, open `Devices > GNSS`. The NTRIP panel separates the
caster connection from correction health: `CONNECTED_NO_DATA` means the caster accepted the
request but no stream bytes have arrived, `STREAMING` means bytes are arriving but no valid
RTCM frame has been observed yet, and `RTCM_ACTIVE` means fresh parsed RTCM is available.
`RTCM_STALE` or `STALE` means the last correction/transport age exceeded its configured limit;
`RECONNECTING` shows the existing retry loop, and `ERROR` shows the last failure. The panel
also shows chunk/byte counters, receive rate, last chunk age, RTCM message types, and reconnect
count. The compact `NTRIP` status-bar indicator keeps the same state visible from other pages.

For drone-side MAVLink, open `Devices > Telemetry`. The default endpoint listens on
`udpin:0.0.0.0:14552`; it can be changed to the serial or UDP/TCP endpoint used by the
drone Pixhawk. `Connect` starts a dedicated MAVLink link, receives the drone's
`GPS_RAW_INT`/`GPS2_RAW` position data, and shows the Left/Right Pixhawk positions beside
it. When `Forward Base RTCM` is enabled, fresh Base RTCM is forwarded to the connected
drone Pixhawk through MAVLink `GPS_RTCM_DATA` in real time.

The Telemetry panel also reports the live Left/Right GNSS baseline and computes the ideal
rectified-stereo disparity as `d = fx × B / Z`, using the active stereo calibration focal
length `fx`, the P1/P2 GNSS baseline `B`, and the automatically derived distance `Z` from
the P1/P2 baseline midpoint to the Drone GNSS position. There is no user-entered target
distance for this value. If one of the three GNSS positions or the active zoom focal length
is missing or stale, the panel reports a waiting/stale state instead of fabricating pixels.
This is a geometry reference for operator feedback, not image-observed disparity, and no
RTK-Fixed state is forced as a measurement gate; the GNSS checklist remains informational.

The `Alignment` workspace keeps the `Pixhawk`, `GNSS / RTK`, and `Alignment` operation
buttons directly below its title. The live evidence tables remain below these controls.

For optical-pose point selection, first press `Capture` in `Alignment > R,t methods`. The
`Auto-select Horizon` and `Auto-select Mountain` actions analyze only those
immutable native left/right frames with the exact frozen operating `K,D`. Review the dashed
overlay and quality strip, then press `Use suggestion` before running the existing Horizon or
Mountain solver. Automatic selection never applies `R,t` by itself. The manual Horizon,
Mountain, and Target modes remain available as a fallback.

The three manual actions have distinct roles: `Manual Horizon` marks two visible horizon
points and estimates Roll/Pitch only; `Manual Mountain` marks matching terrain features in
the same order on both images and supplies the paired observations for the GNSS + Mountain
metric `R,t` solve (at least four pairs); `Manual Target` marks one Drone RTK target pixel
per image for the single-target Drone RTK Yaw fallback. Full multi-view `R,t` refinement
uses the separately imported synchronized observations.

`Clear` removes the selected annotation mode from both frozen frames. `Import` loads a
left/right frozen pair from disk. The WIRIS alternatives are labeled `Capture (WIRIS FTP)`
and `Capture (WIRIS FTP OZ)`. The LRF status card is part of the R,t methods scroll content,
so it scrolls with the workflow instead of staying pinned below it.

The Alignment `Pose workflow` separates the Drone path into two reviewed stages. `Initialize from
Telemetry` uses fresh P1/P2/Drone GNSS to establish the local world-NED camera centers and uses
fresh left/right Pixhawk R/P/Y when available; it creates a candidate and never auto-applies it.
`Refine Drone RTK Multi-view R,t` imports a JSON set of synchronized per-camera rays (or native
pixel coordinates) and Drone RTK target positions, then optimizes full 6-DOF camera `R,t` for
both sides. The multi-view target positions must be in one local NED frame, with at least four
non-degenerate observations per side. The existing `Drone RTK Yaw` tab remains an advanced
single-target fallback and does not infer translation.

In Drone Target annotation mode, capturing another frozen burst keeps the previous target mark
visible for multi-view review until `Clear` is pressed. The mark remains bound
to its original frame identity; it is not silently reused as a pixel from the new frame, and new
points are blocked until the old mark is cleared.

When the session is saved, the static JSON also stores the frozen-frame artifact identities,
complete automatic proposals, accepted points, current annotation payload, and provenance.
Native pixels remain in the session frame artifacts; the append-only `run_events.jsonl` keeps
the full proposal, approval, and annotation-update audit events.

Run the Workstation regression suite from `apps/Workstation` with:

```bash
python3 -m pytest -q
```

## Inputs/Outputs and Interfaces

- Intrinsics profiles contain per-camera OpenCV `K,D` and their source zoom.
- Stereo-extrinsics profiles contain `R_left_to_right`, `T_left_to_right_m`, their source
  zoom, source-intrinsics provenance, pair count, and reprojection error.
- The transform contract is
  `X_right = R_left_to_right X_left + T_left_to_right_m` in OpenCV optical RDF frames.
- Calibration images are synchronized left/right checkerboard pairs under the configured
  calibration root. JSON profile paths are selected independently in the Calibration UI.
- HR Capture photos are created with `SIHR TRUE` + `CPTR`; `ICPT` is polled for completion.
  Images remain in each WIRIS camera's internal storage and are not FTP-downloaded.
- Each button press appends one JSON object to the current session's
  `hr_captures.jsonl`; the file contains evidence values, not image bytes.
- OZ Capture photos are created with `SIVI TRUE` + `CPTR`; `ICPT` is polled separately from
  HR and RTSP recording. Images remain in each WIRIS camera's internal storage and are not
  FTP-downloaded.
- Each OZ button press appends one JSON object to the current session's `oz_captures.jsonl`;
  `session_static.json` exposes the aggregate as `metadata.oz_photo_capture`, and
  `run_events.jsonl` records an `oz_capture_event`. These files contain evidence values, not
  image bytes.
- NTRIP inputs are a provider-independent caster hostname (without `http://` or `https://`),
  TCP port, mountpoint, optional TLS flag, GGA policy, and the names of environment variables
  holding the username/password. Credentials themselves are not saved in the settings file.
- The NGII RTS2 Base-assisted route is `NGII NTRIP → StereoDistance PC → USB/Serial → Base
  F9P` during Base initialization. After the fixed Base coordinate is confirmed, local RTCM
  follows `Base F9P → StereoDistance PC → Ethernet/MAVLink GPS_RTCM_DATA → Left/Right
  Pixhawk → Left/Right F9P`.
- UBX-CFG-TMODE3 fixed-base payload fields follow the u-blox wire order: `fixedPosAcc` at
  offset 20, `svinMinDur` at offset 24, and `svinAccLimit` at offset 28. Fixed-base commands
  populate only `fixedPosAcc`; survey-in commands populate the latter two fields.
- The supported field receiver arrangement is one Base F9P and one F9P-based receiver on each
  Pixhawk rover. Receiver firmware versions are observed on the devices at runtime and are
  not assumed by the settings profile.
- `Devices > Telemetry` is the active drone-Pixhawk link for position reception and optional
  Base RTCM forwarding. The existing target-drone artifact stream remains a separate,
  receive-only path.
- Frozen optical-pose frames are persisted as native PNG evidence and remain bound to
  `(frame_id, image_sha256, native_width, native_height)`. Automatic points are returned in
  native OpenCV `(u, v)` pixels and are kept separate from manual annotation payloads until
  explicit approval.
- Horizon suggestions are two endpoint candidates from a distortion-corrected gradient/Hough
  skyline line. Mountain suggestions use OpenCV SIFT, or ORB when SIFT is unavailable, followed
  by mutual ratio matching, RANSAC epipolar filtering, and spatially distributed point selection.
- Automatic suggestion evidence includes `detection_method`, `confidence`, `match_count`/
  `inlier_count`, `residual_px`, `eligible`, and `failure_reason`. Creation and approval are
  recorded as `optical_pose_auto_suggestion_created` and
  `optical_pose_auto_suggestion_accepted` events. The complete native point sets are retained
  in those event payloads, while `optical_pose_annotations_updated` records the committed
  annotation and `AUTO_ACCEPTED`/`MANUAL_EDITED` state.
- `session_static.json` stores the JSON-safe `metadata.optical_pose` projection. It references
  `run_setup.json`, `run_events.jsonl`, `native_frames.jsonl`, and
  `frames/optical-pose-burst`; it never embeds QImage/UI objects.

## Operational Rules

- Calibration zoom A and operating zoom B are arbitrary supported zoom levels. The system
  does not prescribe 1×, 8×, or any particular A→B combination.
- Cross-zoom application uses `K,D` from operating zoom B and `R,t` estimated at calibration
  zoom A.
- Requested zoom and camera `GZVV` readback are separate evidence. A mismatch is shown but
  does not automatically block applying a profile.
- The application records both profile paths, hashes, source zooms, selection mode, and
  `validation_gate=disabled` in measurement and recording provenance.
- Each HR Capture event records its sequence ID, timestamp, active zoom readback, left/right camera
  responses, completion state, and explicit `camera_internal`/`ftp_downloaded=false`
  storage contract, together with the selected `Pre`/`Post`/`Checkpoint` purpose.
- `OZ Capture` and `HR Capture` share the purpose selector but cannot run simultaneously. They
  do not start or stop RTSP/MP4 session recording.
- Each OZ Capture event records its sequence ID, timestamp, active zoom readback, left/right
  camera responses, completion state, selected purpose, and the same camera-local storage
  contract with `ftp_downloaded=false`.
- A caster host and mountpoint are required only when starting an NTRIP operation. The
  NGII RTS2 profile supplies them for new settings; an explicitly selected custom provider
  must supply its own values before starting.
- A caster HTTP response is not proof that corrections are usable. Check `RTCM_ACTIVE`, the
  last chunk/RTCM ages, and the P1/P2 RTCM distribution rows before starting a measurement.
- NTRIP monitoring reports color/state and retains the error in the evidence projection; it
  does not automatically stop an operation. In the default NGII RTS2 profile, GNSS readiness
  is informational and does not block measurement. The Base GNSS workspace displays these
  five checklist items without enforcing them: Base coordinate confirmed, Base RTCM output,
  Left Rover RTK Fixed, Right Rover RTK Fixed, and left/right Fixed stabilization. RTK Fixed
  (`fix_type=6`) is therefore visible evidence, not a measurement prerequisite.
- Automatic optical-pose actions are explicit proposals only. `Use suggestion` is the only
  action that copies automatic points into the existing annotation buffers, and the Horizon or
  GNSS + Mountain solver still requires its normal explicit button press afterward.
- `GOOD` and eligible suggestions may be approved. `NO_DATA`, `LOW_CONFIDENCE`, incomplete,
  duplicate, or near-collinear Mountain suggestions remain overlay-only and never replace
  existing manual points. Editing an accepted point changes its provenance display to
  `MANUAL_EDITED`.
- Saving or retrying a session refreshes the optical-pose persistence projection, so suggestions
  created after recording starts are not lost from recovery metadata.

## Known Limitations and Workarounds

The Workstation does not prove that a stereo rig's physical transform remains valid as the
lenses change zoom. Validate every intended A→B combination externally and retain that
evidence with the experiment. Missing external validation remains informational; only
malformed or non-computable calibration data is rejected by the application.

The Workstation records camera responses but cannot identify camera-generated file names or
include the still pixels without listing, downloading, or importing internal storage. This
applies to both HR and OZ captures; OZ currently has no Workstation file-import action, so
`oz_captures.jsonl` contains command/status evidence only. If either control endpoint is
missing or a capture times out, the failure is retained in the event file; the operator can
retry without changing measurement state.

NTRIP mountpoints are entered manually; the Workstation does not currently browse a
caster's source table. Provider-specific authentication methods beyond standard Basic
authentication are not supported.

If the status is `CONNECTED_NO_DATA` or `STREAMING`, verify the mountpoint and caster's
actual RTCM output. If it is `RTCM_STALE`/`STALE`, inspect the configured read/correction age,
the caster stream, and the downstream Base/Pixhawk route separately; a live TCP session can
exist while valid RTCM or downstream injection is unavailable.

The NGII profile does not identify the Base serial port or receiver firmware automatically in
the settings file. Use Base receiver discovery or confirm the connected F9P device and its
firmware before field operation.

Automatic Horizon detection can confuse clouds, trees, roofs, or multiple strong boundaries
with the sky/terrain edge; inspect its confidence, residual, and overlay before approval. It
uses a flat, unrefracted horizon model and observes Roll/Pitch only. Automatic Mountain matching
depends on shared visible texture and can be weak in fog, low-texture terrain, vegetation, or
repeated patterns; a failure or low-confidence result should be handled with manual clicks.
Mountain auto-selection does not create terrain heights or 3-D PnP correspondences and does not
replace the existing GNSS-baseline rotation solver. Terrain PnP still requires explicit 3-D
object points and an imported GeoTIFF with a declared vertical datum.

## Change Log

- 2026-08-17: Clarified that Telemetry ideal disparity derives `Z` automatically from the
  P1/P2 baseline midpoint to Drone GNSS and does not accept a user-entered target distance.
- 2026-08-17: Split optical-pose Drone alignment into a reviewed Telemetry initialization
  candidate and a separate Drone RTK multi-view full `R,t` refinement path; retained the
  single-target Drone RTK Yaw solver as an advanced fallback.
- 2026-08-17: Keep Drone Target marks visible across a new frozen burst for multi-view review;
  explicit `Clear` rebinds the editor to the new frame before new points are accepted.
- 2026-08-17: Shortened optical-pose annotation actions to `Capture`, `Clear`, and `Import`,
  documented the Horizon/Mountain/Target roles, and moved the LRF status card into the
  scrollable R,t workflow.
- 2026-08-17: Corrected UBX-CFG-TMODE3 fixed-base accuracy encoding and readback parsing to
  use the u-blox `fixedPosAcc` field at payload offset 20; added independent wire-layout
  regression coverage so a self-consistent but incorrectly shifted round trip cannot pass.
- 2026-08-14: Added separate visible/OZ `OZ Capture` beside `HR Capture`, using
  `SIVI TRUE + CPTR + ICPT`, Pre/Post/Checkpoint purpose evidence, and
  `oz_captures.jsonl`/`oz_capture_event` storage independent of RTSP session video.
- 2026-08-14: Persisted optical-pose automatic proposals and annotation review state in
  `session_static.json` and the session recovery metadata. Full native point sets now remain in
  the run-event audit log, and accepted/manual-edited annotations are recorded with their frame
  identity and provenance.
- 2026-08-14: Added explicit OpenCV Horizon/Mountain automatic point suggestions from frozen
  native frames, with separate overlays, confidence/inlier/residual evidence, audit events,
  approval-gated annotation copying, `AUTO_ACCEPTED`/`MANUAL_EDITED` provenance, and manual
  fallback. Documented that Mountain remains image-correspondence plus GNSS-baseline evidence;
  no map, GeoTIFF, or new learned model is used by auto-selection.
- 2026-08-14: Added the NGII RTS2 Base-assisted profile (`RTS2.ngii.go.kr:2101`,
  `VRS-RTCM32`, Base GGA, ellipsoid height, and left/right RTCM routing). Added the
  informational five-item GNSS checklist and documented that it does not gate measurement.
- 2026-08-14: Added continuous NTRIP transport/RTCM health monitoring with receive ages,
  counters, reconnect state, detailed GNSS status, and a global NTRIP indicator.

- 2026-08-13: Generalized cross-zoom calibration from the former 1×/8× example to
  arbitrary calibration and operating zoom levels; added arbitrary selected-zoom control
  and explicit same-/cross-/unknown-source selection provenance.
- 2026-08-13: Added repeatable manual left/right `HR Capture` with one evidence row per
  press in `hr_captures.jsonl`; images remain camera-local with no FTP transfer.
- 2026-08-13: Made NTRIP caster host, port, mountpoint, TLS, and credential
  environment-variable selection explicit in the GNSS settings surface.
- 2026-08-13: Moved the Pixhawk, GNSS / RTK, and Alignment operation tabs to the top of
  the Alignment workspace for immediate field access.
