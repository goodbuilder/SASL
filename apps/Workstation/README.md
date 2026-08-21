# SASL StereoWIRIS Workstation

## Purpose

This package provides the operator Workstation for synchronized stereo-camera capture,
calibration, measurement, Pixhawk/GNSS evidence, and recording. It owns its runtime and UI
independently from the Legacy application. An explicitly opened, timestamp-named session owns
multiple independent measurements; every measurement can be sealed as one portable evidence
folder, and closing the parent session seals the ordered child-manifest inventory. The top-level
`Session` workspace treats that parent as the daily-work lifecycle and separately projects current
Measurement evidence-collection readiness, valid-measurement readiness, and the factual inputs for
every geometry method. It shows blockers instead of repeating passing gates, keeps all eight method
categories visible, and expands factual values only for the selected method. High-rate RTCM
and camera-preview traffic is isolated from the Qt GUI thread: full-fidelity routing/evidence stays
on worker-owned data paths, while the UI consumes bounded latest-state snapshots. Every child
Measurement also owns an immutable method/fallback configuration boundary, including explicit
per-camera Roll/Pitch/Yaw and North/East/Down fill decisions. Finalization is fail-closed: the
session save must complete, measurement-scoped RTCM must be drained and semantically verified,
and every package artifact must pass containment and hash checks before a manifest can be sealed.

## Current Run Flow

From the repository root, run `./apps/Workstation/run.sh --check`, then start the UI with
`./apps/Workstation/run.sh`. The check validates declared dependency versions, Workstation
imports, the selected GStreamer/PyAV backend, and the approved default YOLO model's size and
SHA-256 against `model-manifest.json`; it exits nonzero on a missing or incompatible boundary.
Use `--no-start` to open without camera discovery or preview and `--start-pixhawk` to connect the
configured Pixhawk endpoints after launch.

Runtime state defaults to `.local/workstation_data` at the repository root. To keep it on another
volume, set an absolute data root before both checking and launching; relative overrides are
anchored to the repository root, not the shell's current directory:

```bash
export SASL_WORKSTATION_DATA_ROOT=/absolute/path/to/workstation_data
./apps/Workstation/run.sh --check
./apps/Workstation/run.sh
```

The settings JSON itself stores required asset paths relative to that data root. Selecting an
external intrinsics or YOLO file imports verified bytes into a content-addressed
`calibration/imported/` or `models/imported/` child before persistence. A calibration directory
outside the data root remains usable only for the current workflow until it is explicitly migrated;
Save rejects it instead of creating a machine-specific absolute dependency.

GStreamer is the default stream backend. Install the optional PyAV overlay only when the selected
settings profile uses PyAV:

```bash
python3 -m pip install -r apps/Workstation/requirements-pyav.txt
```

The normal workflow is explicit: after device/calibration/alignment preparation, open `Session`
immediately before the day's first result-bearing evidence and press `Open Session`, then press
`New Measurement`. Check the separate `Evidence collection` and `Valid measurement` projections
and resolve the blocker-only list before opening `Measurement`. The method table always includes
Pixhawk, Checkerboard, Horizon, Mountain, Drone Rotation, Drone Multi-view, Terrain, and Combined;
select one row to inspect its actual values and coordinate contract. Collect any `Pre` HR/OZ/alignment
evidence, run exactly one `Detection` interval, stop Detection, collect `Post` evidence, then return
to `Session` and press `Seal Measurement`. Stopping Detection still
publishes that measurement's typed Parquet generation and marks the session-level
`review.xlsx` pending; `Close Session` creates the integrated workbook once from all sealed
measurements. Detection stop still leaves a deliberate Post-capture
window. `Seal Measurement` waits for capture, video, and the real session-save worker, closes and
drains that measurement's RTCM evidence context, verifies its binary/index high-water, and only
then seals the current child revision as `SEALED_COMPLETE` or `SEALED_INCOMPLETE`. A save/drain or
semantic-integrity failure leaves the child `RECOVERABLE` without a new manifest. Press `New
Measurement` again to create the next isolated child (`m000002`, `m000003`, ...), or `Close
Session` to seal the ordered parent index. `Open Session Review` reviews children by the composite
session/measurement identity. A
revision adds late evidence to the same measurement and never grants another Detection interval.
Saved media/package review remains in the separate `Recording / Review` workspace.

Session lifecycle and readiness are intentionally independent. `OPEN` means the daily container
can own children; it does not claim the current child can calculate a valid range. The method-input
table exposes preferred/effective selection and fallback reason plus the values that actually feed
each method: live Pixhawk body-FRD-to-world-NED R/P/Y with age, P1→P2 GNSS NED baseline,
candidate virtual-camera-FRD/world-NED observable DOF, or OpenCV optical-RDF left-to-right `R,t`.
These coordinate contracts remain separate in the snapshot and UI.

For checkerboard calibration, select the calibration zoom, command both cameras to that
zoom, confirm the actual zoom readback, capture synchronized pairs, and calculate/save the
required intrinsics or stereo `R,t` profile. At measurement time, load the `K,D` profile for
the intended operating zoom and independently load the chosen `R,t` profile, then apply the
bundle.

At startup, automatic intrinsics loading first checks the exact persisted `intrinsics_path`. An
existing exact profile is applied only when it parses successfully and its
`metadata.zoom_level` equals the current operating zoom. A malformed, unknown-zoom, or mismatched
exact profile fails closed and is never replaced by another convenient file. Only when the exact
configured path is the calibration root's missing `intrinsics.json` alias may startup fall back to
that root's `intrinsics_zoom_XX.json`, under the same zoom check. A missing explicit external path
never falls through to unrelated storage bytes.

Measurement Settings does not select Intrinsics or a manual focal value. The active exact `K,D`
comes from `Calibration > Load & Apply`. Configure `YOLO input size` under
`Settings > YOLO / RTSP`; Apply updates the runtime owner and the new size is used from the next
Detection start. Confidence, IoU, and pair gates remain in Measurement Settings.

Calibration capture now saves a lossless, no-overlay native frame and records its native,
preview, and saved raster grids; a preview-resized image is not used as the calibration input.
Auto capture rejects a pair when either camera's checkerboard pose is effectively a duplicate of
an already accepted pose. Move the board across the image, change distance and roll, and introduce safe
yaw/pitch perspective instead of collecting many fronto-parallel repeats.

In `Calibration > Compute`, select a saved calibration session and review its matching Left/Right
files in `계산에 사용할 사진`. The table previews the session candidates until the operator fixes
independent Left and Right selections with the two file-selection buttons.

The `계산에 사용할 사진` table shows a separate local date/time for every Left and Right
photo, to millisecond precision, inside each information cell. Its tooltip shows the timestamp
source and evidence path. A manual/native or Auto image uses its side-specific RTCP/NTP frame
time; FTP/OZ files use the application capture-record time because the historical live-preview
RTCP value is not the downloaded still's exposure time. Missing or malformed legacy metadata
falls back to the explicitly labelled file `mtime`.

Calibration is split into `Capture` and `Compute` tabs. `Capture` contains live/HR/OZ/Auto
acquisition; `Compute` selects a saved photo session and owns pair review, K,D/R,t calculation,
FTP OZ sync, saving, and loading. Selecting a session updates the owner calibration root so
these actions all operate on the same saved dataset.

After calculation, review the calibration-quality panel before saving or applying the result.
It exposes the effective-pose count, 3×3 image coverage, principal-point offset, pose/perspective
diversity, and deterministic subset stability. `PASS` is directly usable, `WARNING` requires an
explicit operator confirmation, and `FAIL` cannot be saved or applied.

`Capture HR Still` can be pressed repeatedly whenever the operator needs high-resolution evidence.
Each press enables visible-HR still storage, captures one photo on each camera, and waits
for both cameras to report completion. `Capture OZ Still` sits beside it in `Recording > Session`
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

RTCM parsing, exact-byte evidence writes, Base forwarding, and Pixhawk injection run on the RTCM
data-plane worker. The GNSS UI is refreshed from a coalesced snapshot at most every 0.5 seconds;
the bandwidth row shows receive/persist/inject sequence high-water values, stale-drop counts,
ingress depth, and backpressure waits. Camera decode, native measurement frames, and detection are
independent of display cadence. Preview rendering is capped at 15 Hz per camera and keeps only the
latest pending UI frame when the GUI is temporarily busy.

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

For optical-pose point selection, open `Alignment` and use the common frozen-reference area above
the promoted method tabs, then press `Capture Live Pair`. The
`Auto-select Horizon` and `Auto-select Mountain` actions analyze only those
immutable native left/right frames with the exact frozen operating `K,D`. Review the dashed
overlay and quality strip, then press `Use suggestion` before running the existing Horizon or
Mountain solver. Automatic selection never applies `R,t` by itself. The manual Horizon,
Mountain, and Target modes remain available as a fallback.

Auto-selection runs in a background worker so the Workstation window remains responsive while
Horizon/Mountain image analysis is running. Both automatic-selection buttons are disabled during
the run, duplicate clicks and other optical-pose mutations are ignored until completion, and a
result is committed only if its frozen frame IDs and SHA-256 identities still match the current
burst. The status strip shows `RUNNING`, `LOW_CONFIDENCE`, or the failure reason.

The three manual actions have distinct roles: `Manual Horizon` marks two visible horizon
points and estimates Roll/Pitch only; `Manual Mountain` marks matching terrain features in
the same order on both images and supplies the paired observations for the GNSS + Mountain
metric `R,t` solve (at least four pairs); `Manual Target` marks one Drone RTK target pixel
per image for the single-target Drone RTK Yaw fallback. Full multi-view `R,t` refinement
uses the separately imported synchronized observations.

The Alignment solver surface exposes top-level `Horizon`, `Mountain + GNSS`, `Terrain PnP`,
`Drone Rotation`, `Drone Multi-view`, `References`, and `Combine` tabs below the common frozen
preview. On wide screens each camera preview is shown side by side and the common Candidate review
stays to the right of the selected method; narrow screens stack the previews and place Candidate
review below the tabs. Measurement no longer exposes the legacy two-value
Pixhawk/Checkerboard geometry selector. Its `Ready method` selector lists only methods whose
exact operating `K,D` and required geometry inputs are currently resolvable. A solved
candidate is eligible immediately only when its method contract can resolve a complete pose.
Horizon uses its observed Roll/Pitch plus pair-frozen Pixhawk Yaw and GNSS Translation. A
single-target Drone Rotation candidate re-solves Yaw against pair-frozen Pixhawk Roll/Pitch;
two-or-more-target Drone Rotation keeps its solved full R and adds pair-frozen GNSS Translation.
Mountain, Multi-view, Terrain, and Combined candidates must already be full 6-DOF and fail
closed instead of borrowing arbitrary live axes. Every accepted pair freezes the resulting
method instance, final R,t geometry hash, and per-axis source.

`Settings > Runtime / Calibration > Measurement fallback order` defines the default copied into
each new Measurement; later Settings edits do not rewrite an existing child. A manual Measurement
selection is preferred while ready. If it becomes unavailable, the first ready method in that
Measurement's frozen order is selected at the next pair boundary; a recovered preferred or
higher-ranked method must remain ready for one second before automatic return. When no method is
ready, detection/recording evidence may continue but no valid distance is accepted.

The Measurement `DOF fill for` selector exposes every currently available method. For each Left
and Right Roll/Pitch/Yaw/North/East/Down component, `OWN` means the method observes that value and
the control is disabled. A missing rotation component exposes a `PX4` toggle; a missing translation
component exposes a `GNSS` toggle. Safe fill toggles are enabled by default, can be disabled for the
current Measurement, and lock when Detection starts. Method-defining observations (Horizon
Roll/Pitch, Drone Rotation Yaw, and every DOF of full Mountain/Multi-view/Terrain/Combined methods)
cannot be replaced by a fallback merely to make an invalid candidate look ready.

Each Alignment method tab shows method-specific evidence in addition to the common Candidate DOF
control: Horizon shows model/RMS/sensitivity, Mountain shows pair/inlier/Sampson/baseline/lever-arm
metrics, Terrain shows GeoTIFF/2D↔3D/reprojection/rank data, Drone tabs show target/observation/sync
and 3D-spread data, References shows profile compatibility, and Combine shows source/axis conflict
status. References only registers reusable sources; Combine only composes them.

`Clear` removes the selected annotation mode from both frozen frames. `Import` loads a
left/right frozen pair from disk. The WIRIS alternatives are labeled `Capture HR Still (FTP)`
and `Capture OZ Still (FTP)`, while local files use `Import Pair`. Low-level WIRIS `GLRD` remains available through Advanced command
tools, but no LRF status card or refresh command is exposed in Alignment.

The bottom `Diagnostics > FTP` tab keeps the recent FTP lifecycle separate from the combined
Log tab. It shows HR/OZ camera-local capture, FTP download, checkerboard-save, OZ sync/delete,
local file paths, and failure messages, so a completed `Capture HR Still (FTP)` or `Capture OZ Still (FTP)` can
be verified without searching through unrelated camera and Pixhawk messages.
New settings enable `Enable FTP download` by default; the left/right WIRIS FTP credentials and
remote directory still need to match the camera configuration. Leaving `Remote dir` as `/` now
discovers the camera's `/ssd/*/img`, `/sdcard/*/img`, or `/flashd/*/img` session folders and
selects the channel-appropriate latest still; FTP connection setup retries once and defaults to
a 30-second per-camera timeout.
Raw FTP downloads are separated as `<timestamp>/calib_capture/HR/left|right` for HR and
`<timestamp>/calib_capture/OZ/left|right` for OZ; validated calibration PNGs remain in the
session root's `left/` and `right/` directories.
Before each FTP still capture, the worker snapshots the remote image list and accepts only a
file that appears or changes after `CPTR`; it never falls back to an older still when the camera
has not finished publishing the new file.

Every `Capture Live Pair` in the Alignment common frozen-reference area also records a `Drone RTK Capture
Observations` entry: the frozen left/right native frame identity plus whatever Left (P1),
Right (P2), and Drone GNSS is available from `panel.latest_gps` at that instant, saved as one
immutable, append-only observation. Missing or degraded GNSS never blocks the frame Capture
itself -- Horizon, Mountain, and Target annotation all keep working with no GNSS at all; the
observation is simply recorded as `INCOMPLETE` or `WARNING` evidence rather than raising an
error. This state describes the completeness/quality of the captured *evidence*, not the
accuracy of any later pose solve.

Frame and GNSS timestamps are each preserved from their own original source and are never
averaged or discarded. A `Left/Right frame -> Drone GNSS` delta is only computed when the
frame's `timestamp_reference` is confirmed `"app-receive"` (the same `time.monotonic()` host
clock every live GNSS sample also uses) -- otherwise the pair is `SYNC_UNVERIFIED` and no
delta number is fabricated. `Import` does not create a Drone RTK observation because the image's
capture-time GNSS is unknown. `Capture HR Still (FTP)` and `Capture OZ Still (FTP)` snapshot
Left/Right/Drone GNSS at the camera capture request and attach that evidence to the downloaded
Burst after FTP completes. The request timestamp is retained as the GNSS snapshot source; the
later download time is never substituted. Binding an annotated target pixel `(u, v)` to a specific
past observation, and feeding this evidence automatically into `Refine Drone RTK Multi-view R,t`,
are both later work; today the observation detail panel shows `Target: NOT ANNOTATED` for every
entry.

The `Drone RTK Multi-view R,t` tab imports a JSON set of synchronized per-camera rays (or native
pixel coordinates) and Drone RTK target positions, then optimizes full 6-DOF camera `R,t` for
both sides. The target positions must be in one local NED frame, with at least four
non-degenerate observations per side. The separate `Drone RTK Rotation` tab uses one target for
Yaw or two or more non-degenerate target/ray observations for full rotation; Measurement supplies
its missing Translation from live GNSS.

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

Run the repository portability gate from the repository root with:

```bash
python3 tools/check_portable_paths.py
```

## Inputs/Outputs and Interfaces

- The canonical runtime root is `.local/workstation_data/` at the repository root, or the path in
  `SASL_WORKSTATION_DATA_ROOT`. It owns `stereo_distance_settings.json`, `recordings/`,
  `calibration/`, `models/`, and runtime caches. The five persistent asset fields—intrinsics,
  detector weights, calibration root, validation root, and validation JSON—are normalized to
  data-root-relative POSIX paths. Known legacy values remain readable, but the next production Save
  refuses machine-external directories and never writes a checkout/user-home absolute path.
- New sessions live directly under `.local/workstation_data/recordings/<session-id>/`. The
  timestamp-based session ID is allocated only by explicit `Session > Open Session`; an app-run ID is
  recorded separately so one session can be recovered across application restarts. Existing
  `recordings/sessions/<session-id>/` trees remain read/recovery compatible but are never used for
  new writes; a duplicate ID across both layouts fails closed.
- Each measurement lives under
  `<session-id>/measurements/mNNNNNN/`; its spool, video, captures, recording generation,
  results, and revisions never share a writable path with a sibling measurement.
- Each child `revisions/rNNNN/package_manifest.json` uses schema
  `workstation-measurement-package-v1`. It inventories measurement-relative artifact paths, roles,
  producer/source identity, required/present/missing/failed state, byte size, SHA-256, media/schema
  information, and evidence completeness. `package_manifest.sha256` seals the canonical manifest;
  `latest_revision.json` points to the current revision and each later revision hashes its
  predecessor. Parent `measurement_session_manifest.json` uses schema
  `workstation-measurement-session-v1` and hashes every latest child manifest plus session-shared
  evidence.
- Each finalized measurement publishes one `data` link backed by
  `.workstation/publish/current`. It contains `measurement_pairs.parquet`,
  `camera_timeline.parquet`, `pixhawk_timeline.parquet`, `gnss_base_timeline.parquet`,
  `geometry_comparison.parquet`, and `context.json`. `measurement.json` records the atomic
  generation, verified schemas, row counts, sizes, and hashes. Default CSV, final raw-row JSONL,
  per-Pixhawk XLSX, and per-measurement XLSX files are not emitted.
- `recordings/<session-id>/review.xlsx` is the one operator-facing integrated workbook. It combines
  Session, Measurements, Results, Quality, Geometry, Devices, Captures, Warnings, and Files views
  across all finalized child measurements. It is derived and regenerable; workbook failure never
  invalidates verified Parquet evidence.
- Each evaluated measurement pair—valid, rejected, unresolved, or blocked—retains canonical
  input/result/readiness JSON and matching SHA-256 values in
  `data/measurement_pairs.parquet`. Stable typed columns support analysis, while canonical
  `row_json` preserves every dynamic source field.
  Its exact left/right decoded source pixels are stored as lossless, no-overlay native PNGs.
- `measurement-method-snapshot-v1` supplies the Session operator view without creating a second
  geometry owner. Runtime `method_capabilities` remains instance-driven, while the additive
  `method_catalog` always contains all eight stable method IDs. Every available capability carries
  readiness/reasons, component-source mapping, required inputs, and a read-only `input_values`
  projection. Missing candidates/profiles become blocked catalog rows and their values remain
  `null`; the projection
  never fabricates a candidate pose or merges PX4 FRD/NED, GNSS NED, candidate virtual-FRD/NED,
  and OpenCV RDF values.
- Continuous `video/raw_left.mp4` and `video/raw_right.mp4` are hashed only after their size and
  modification time stabilize. Bounded `ffprobe` metadata records codec, frame count, dimensions,
  FPS, duration, recording backend, and encoding provenance.
- The package copies external active model/profile inputs, the selected calibration source images and
  metadata reachable from those profiles, redacted effective settings, Git/software provenance,
  device evidence, application logs, ULogs, target/drone telemetry, and exact RTCM bytes when the
  corresponding source is active. ULogs and Target Drone files already sealed inside the
  measurement are registered in place by path/size/SHA rather than copied again under `assets/`.
- The default detector is `.local/workstation_data/models/yolov8m.pt`. Runtime model download is
  disabled: provision the approved bytes whose size and SHA-256 match `model-manifest.json` before
  launch. Relative custom model paths resolve under the canonical runtime root; enumerated legacy
  defaults remain readable without migration.
- Active RTCM evidence is append-only under `<measurement-root>/raw/gnss/`. Raw NTRIP chunks are
  stored in `ntrip_transport_chunk.rtcm`; parsed NTRIP and Base frames use
  `ntrip_rtcm_frame.rtcm` and `base_rtcm_frame.rtcm`. Each binary has a sibling `.rtcm.jsonl`
  index containing global `sequence`, per-source `source_sequence`, byte offset/size, SHA-256,
  producer receive UTC/monotonic nanoseconds, source, record kind, and RTCM message type. Recovery
  verifies contiguous offsets, every indexed hash, and the absence of unindexed binary tails
  before selecting the next sequence.

- Intrinsics profiles contain per-camera OpenCV `K,D`, source zoom, calibration-quality snapshot,
  and solver diagnostics. Diagnostics include effective view IDs, normalized checkerboard view
  geometry, per-view errors, OpenCV intrinsic/extrinsic standard deviations, deterministic subset
  membership, and subset estimates. The active profile path is the file actually loaded—not a
  reconstructed calibration-root path—and is retained for UI and package provenance.
- Stereo-extrinsics profiles contain `R_left_to_right`, `T_left_to_right_m`, their source
  zoom, source-intrinsics provenance, pair count, and reprojection error.
- The transform contract is
  `X_right = R_left_to_right X_left + T_left_to_right_m` in OpenCV optical RDF frames.
- Calibration images are synchronized left/right checkerboard pairs under the configured
  calibration root. By default manual/FTP runs use
  `.local/workstation_data/calibration/<timestamp>`, while Auto capture uses
  `.local/workstation_data/calibration/<timestamp>/calib_capture/AutoCapture`; JSON profile paths
  are selected independently in the Calibration UI.
- Each capture metadata record identifies the exact source frame, source/native dimensions,
  preview dimensions, saved dimensions, crop/scale contract, overlay state, and SHA-256. A live
  CameraPane capture fails closed when a fresh native frame cannot be matched. Preview fallback is
  disabled by default and exists only behind an explicit compatibility flag for lightweight tests.
- Calibration photo-time display reads `left_rtcp_frame_ntp_unix_s` and
  `right_rtcp_frame_ntp_unix_s` only when they are bound to a saved manual/native or Auto frame.
  It otherwise uses `capture_recorded_unix_ns`, then `capture_requested_unix_ns`, and finally the
  image file `mtime`; the UI always labels which source was used.
- `Capture HR Still` photos are created with `SIHR TRUE` + `CPTR`; `ICPT` is polled for completion.
  When WIRIS FTP is enabled, the worker lists the camera storage before capture, downloads only
  a new/changed post-CPTR image into the open session, and validates it during package finalization.
  If FTP is disabled or fails, the still remains camera-local until an exact HR file is imported.
- Each button press appends one JSON object to the current session's
  `hr_captures.jsonl`; downloaded/imported image bytes remain separate session artifacts.
- `Capture OZ Still` photos are created with `SIVI TRUE` + `CPTR`; `ICPT` is polled separately from
  HR and RTSP recording. With FTP enabled, fresh left/right originals are downloaded into the
  session; otherwise they remain camera-local.
- Each OZ button press appends one JSON object to the current session's `oz_captures.jsonl`;
  `session_static.json` exposes the aggregate as `metadata.oz_photo_capture`, and
  `run_events.jsonl` records an `oz_capture_event`. Event files contain evidence values while
  image bytes remain independently hashed artifacts.
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
- Measurement-method snapshots use stable IDs (`pixhawk_rtk_live`, `checkerboard_stereo`,
  `horizon_hybrid`, `mountain_gnss`, `drone_rtk_rotation_hybrid`,
  `drone_rtk_multiview`, `terrain_pnp`, and `combined_hybrid`). The settings JSON stores the
  ordered fallback list; legacy `pixhawk`, `checkerboard`, and `calibration` values migrate to
  the corresponding stable IDs.
- Every child package metadata record freezes `measurement_preferred_method`,
  `measurement_method_fallback_order`, and `measurement_method_fill_policy` under schema
  `measurement-method-config-v1`. The fill policy is keyed by method and then by
  `<left|right>.<roll_deg|pitch_deg|yaw_deg|north_m|east_m|down_m>`, with values `live_pixhawk`,
  `live_gnss`, or `none`. Restart recovery reloads this child-owned policy rather than current
  global Settings.
- Measurement rows record `measurement_method_id`, method instance ID, geometry SHA-256,
  `measurement_geometry_kind`, selection reason, and the left/right per-DOF component-source
  mapping in addition to the legacy triangulation source fields. `geometry_kind` distinguishes
  the math kernel (`relative_pose`/`live_pixhawk_mount`) from the operator-selected method.
- Visible settings ownership is split by lifecycle: Calibration owns the applied Intrinsics,
  `Settings > YOLO / RTSP` owns the canonical `yolo_input_size`, and Measurement Settings owns
  confidence, IoU, and pair-gate values. Legacy backend aliases remain readable for compatibility
  but are not duplicated as visible controls.

## Operational Rules

- `Capture Live Pair` selects the newest decoded native left/right frames from the live RTSP
  pipelines. It preserves RTP/RTCP timing metadata when available but does not capture raw RTP
  packets. Camera-native `CPTR` stills remain separate `Capture HR Still`/`Capture OZ Still`
  actions, with `(FTP)` appended only when the still is downloaded immediately.
- `Session > Open Session` opens only the daily parent. `New Measurement` must precede Pre capture or Detection.
  Detection can run once per measurement; stopping it moves only that child to `POST_CAPTURE`,
  and only explicit `Seal Measurement` seals it.
- Persisted lifecycle, derived readiness, and evidence completeness are separate. `OPEN` never
  implies `READY`; `Evidence collection` checks the open parent/current child, camera pair, and
  synchronization, while `Valid measurement` additionally requires at least one complete geometry
  method. The Session table shows blockers only; a blocked validity projection does not suppress
  raw/Pre evidence collection.
- Preferred, ready, and effective methods remain distinct. The Session table displays the actual
  source values and fallback reason only in selected-method detail. Missing method instances stay
  visible as blocked catalog rows, while each accepted or blocked pair persists its own immutable
  method/readiness snapshot at the pair boundary.
- A parent remains `OPEN` after a child is sealed. Another child may start only after the current
  child is sealed, and `Close Session` is allowed only when no mutable child or producer remains.
- Evidence completeness and measurement validity are independent. A session with no valid range
  can be `SEALED_COMPLETE` when every expected artifact explaining that outcome exists; a valid
  range can be `SEALED_INCOMPLETE` when a required original is missing.
- Required evidence is source-driven. A source activated at any point in the session remains an
  expectation even if it produces zero rows. A source never activated is `not_applicable`.
- Requesting HR/OZ makes both exact camera originals required. A timeout, one-sided download,
  invalid image, size/hash mismatch, or missing file is preserved and seals the package
  `SEALED_INCOMPLETE`; it never fabricates a successful pair.
- `Seal Measurement` does not discard a failed save. Its durable order is session-save completion,
  measurement-scoped RTCM close/drain acknowledgement, RTCM binary/index semantic verification,
  artifact inventory/readback, then manifest seal. Save, drain, manifest, write, or probe errors
  leave the package and spool `RECOVERABLE` and create no false-success manifest. A torn final spool
  record is quarantined while its valid prefix is retained; malformed interior rows remain a hard
  error. Retrying resumes the unfinished boundary instead of regenerating already manifested parent
  review bytes.
- Live recording spools and heterogeneous event ledgers remain JSONL because they must append and
  recover a valid prefix after interruption. Successful finalization converts tabular measurement,
  camera, Pixhawk, GNSS, and geometry rows to typed Parquet, verifies full readback, then deletes
  the temporary spool. CSV is available only as an explicit future/export boundary, not a default
  source of truth.
- Sealed revisions are immutable. Additional evidence requires `Create Revision`; a new manifest
  inventories the complete current folder and includes the preceding manifest hash. Adding an
  unlisted regular evidence file without a revision makes latest-package verification fail.
- Settings/provenance snapshots redact passwords, tokens, embedded URL credentials, and absolute
  machine paths. Multiline secret-bearing diff hunks are omitted fail-closed. Environment-variable
  names are retained, but credential values are never read into the package. Recovered
  session-relative and FTP-local artifact paths must remain beneath the active session root;
  package finalization never imports an arbitrary host path.
- Persistent settings path rules do not rewrite sealed evidence. Historical source paths may remain
  as provenance, but replay and preview must use package-relative artifact paths and verified hashes.
  `/dev`, `/opt`, bounded `/tmp`, and camera-side `/` remain explicit machine/device contracts rather
  than portable asset fields.
- Canonical capture/event rows are append-only evidence. Hiding a graph row changes only the view,
  corrupt JSONL lines remain represented by hashed synthetic records, and preview opens only after
  the artifact's recorded byte size and SHA-256 have been reverified.
- Full-rate RTCM envelopes enter a 4,096-item bounded worker queue. A full queue applies producer
  backpressure rather than silently discarding evidence. The separate Pixhawk delivery queue is
  latest-relevant and bounded: a replaced correction returns its exact pipeline sequence as an
  explicit stale drop, while the original received bytes remain in evidence.
- An RTCM correction older than two seconds at the injection boundary is not sent to Pixhawk. Its
  bytes, source timestamp, hash, sequence, targets, age, and `stale-before-injection` outcome remain
  observable. Evidence-write, recovery-index, and unconfirmed-delivery failures make an active
  measurement retryable/`RECOVERABLE`; they are never presented as successful delivery.
- Every RTCM envelope freezes its session root, measurement context token, and routing context when
  enqueued. Paired binary/index writes use explicit offsets, flush/readback checks, and rollback on
  failure; reconnect retires the prior subscriber generation. A measurement-specific drain ACK
  proves that all sequences through the acknowledged high-water have terminal evidence and delivery
  outcomes while leaving the global RTCM pipeline available for the next child measurement.
- Shutdown stops NTRIP and Base producers first, drains RTCM ingress and evidence, waits for every
  Pixhawk injection outcome (or records the exact unconfirmed sequences after the bounded timeout),
  and only then disconnects MAVLink/runtime workers and finalizes the session.
- Preview replacement applies only to resized UI frames. Native measurement/calibration frames,
  detection events, recording streams, command results, and RTCM evidence do not use the preview
  inbox and cannot be inferred from a skipped display frame.
- Changing `YOLO input size` does not resize an already-running detector. Stop Detection, Apply or
  Save the Settings value, and start Detection again; the next worker uses the updated size.

- Intrinsics are solved independently from the selected left and right image sets. Stereo `R,t`
  uses only timestamp pairs whose left and right observations both survived detection and
  near-duplicate-pose filtering; excluding one side does not silently exclude the other side's
  intrinsic observation. The configured calculation-pair limit is applied once, after this
  usable stereo intersection is formed.
- The default calibration-quality policy requires at least 12 distinct poses, at least 60% union
  extent on each image axis, checkerboard centers in at least five cells of a 3×3 grid, horizontal
  and vertical perspective variation, at least 20% scale variation, stable deterministic subset
  solves, OpenCV focal standard deviation no greater than 3%, principal-point standard deviation
  no greater than 2% of each raster axis, and no per-view reprojection error above 1 px. A
  centered raster warns when `cx` or `cy` is more than 5% of that axis away from
  center and fails above 10%; an out-of-frame principal point always fails.
- A quality `FAIL` blocks calibration-result save and apply. A `WARNING` is allowed only after the
  operator explicitly confirms the displayed reasons, and the override is recorded in validation
  metadata. Profiles without quality evidence remain load-compatible but are displayed as
  `UNVERIFIED`, never as `PASS`.
- Every intrinsics JSON load independently validates finite positive focal lengths, finite
  distortion values, positive raster dimensions, and in-frame `cx,cy`; a serialized `PASS` cannot
  override a malformed K/D raster contract.
- Automatic loading treats an existing persisted `intrinsics_path` as authoritative. Its
  `metadata.zoom_level` must exactly equal the operating zoom; parse failure, missing zoom metadata,
  or mismatch blocks automatic application without trying another profile. Storage-root fallback is
  permitted only for the calibration root's missing base alias, and the selected real path remains
  visible after loading. A failed quality/native-grid apply restores the preceding active calibration
  state. Saving a newly calculated profile persists its new per-zoom path for the next restart.
- Applying a newly calculated result also fails closed when its calibrated native raster differs
  from the current camera native raster. The grid contract must be resolved before the intrinsics
  are used for distance calculation.
- Camera-frame time, application capture-record time, and filesystem write time have different
  meanings and are never presented under one unlabeled timestamp. The Compute table reads time
  evidence directly from `capture_metadata/capture_<id>.json`; selection sidecars continue to
  contain paths and inclusion/exclusion decisions only.
- RTCP/NTP Unix time is never compared as if it were application monotonic time. Native-frame cache
  lookup uses the frame's application-receive monotonic stamp; an absent clock-domain reference
  remains `unknown`, and FTP/import capture without exposure-time evidence remains
  `SYNC_UNVERIFIED` rather than borrowing a live-preview timestamp.
- Calibration zoom A and operating zoom B are arbitrary supported zoom levels. The system
  does not prescribe 1×, 8×, or any particular A→B combination.
- Cross-zoom application uses `K,D` from operating zoom B and `R,t` estimated at calibration
  zoom A.
- Every measurement method requires the exact operating-zoom left/right `K,D` bundle and matching
  hashes. Cross-zoom `R,t` remains allowed, but it cannot waive that intrinsics gate. Methods that
  consume GNSS also require a synchronized, fresh, fixed-quality pair; stale, skewed, or one-sided
  data remains blocked evidence.
- Requested zoom and camera `GZVV` readback are separate evidence. A mismatch is shown but
  does not automatically block applying a profile.
- The application records both profile paths, hashes, source zooms, selection mode,
  `validation_gate=disabled` for the separate cross-zoom validation policy, and
  `quality_gate=calibration_quality_v1` with the available camera-calibration quality snapshot in
  measurement and recording provenance.
- Each `Capture HR Still` event records its sequence ID, timestamp, active zoom readback, left/right camera
  responses, completion state, camera-local/FTP/import storage state, and selected
  `Pre`/`Post`/`Checkpoint` purpose.
- `Capture OZ Still` and `Capture HR Still` share the purpose selector but cannot run simultaneously. They
  do not start or stop RTSP/MP4 session recording.
- Each `Capture OZ Still` event records its sequence ID, timestamp, active zoom readback, left/right
  camera responses, completion state, selected purpose, and per-side camera-local/FTP state.
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
- Auto-select Horizon/Mountain is CPU-heavy SIFT/gradient work but runs off the Qt UI thread;
  wait for the `RUNNING` status to clear before starting another optical-pose action. A stale
  result from an older frozen burst is rejected instead of being added to the current overlay.
- `GOOD` and eligible suggestions may be approved. `NO_DATA`, `LOW_CONFIDENCE`, incomplete,
  duplicate, or near-collinear Mountain suggestions remain overlay-only and never replace
  existing manual points. Editing an accepted point changes its provenance display to
  `MANUAL_EDITED`.
- Saving or retrying a session refreshes the optical-pose persistence projection, so suggestions
  created after recording starts are not lost from recovery metadata.
- Method readiness is independent. Static Checkerboard or full optical-pose candidates are not
  blocked because live GNSS/Pixhawk inputs needed by another method are missing. GNSS/Pixhawk
  gates are evaluated only when the selected method consumes those live components.
- The application supplies safe DOF-fill defaults; the operator does not have to define a default
  priority. Missing rotation axes default to pair-frozen fresh Pixhawk attitude and missing
  translation axes default to the synchronized GNSS/RTK baseline. Turning a toggle off uses
  `none`, makes that hybrid method unready, and never causes a different unrecorded source to be
  substituted.
- DOF fill policy is editable only while the child is `COLLECTING` and before its one Detection
  interval starts. Each change is written to child metadata and `measurement_method_fill_changed`
  run evidence. Detection freezes the policy; later changes require the next Measurement.
- Automatic fallback never changes an already-created MeasurementAttempt. A switch, recovery,
  or newer candidate becomes effective only at the next matched-pair boundary and is recorded as
  a `measurement_method_switched` event.
- Pair-dependent attitude/GPS inputs are selected before hybrid R,t is resolved. The same frozen
  snapshots are stored in MeasurementAttempt and included with final R,t in
  `measurement_geometry_sha256`; a UI-latest sample can never be substituted after the pair is
  frozen.
- If no method is ready, the matched detections are retained as a
  `matched_no_ready_measurement_method` blocked row with the complete readiness snapshot hash;
  no fake R,t or placeholder MeasurementAttempt is created.
- Candidate review keeps left/right observability separate. The common Apply/Combine axis is
  enabled only when both camera sides observed that axis; one-sided evidence remains visible in
  the detail text but cannot be promoted as a two-camera contribution.

## Known Limitations and Workarounds

The software integrity gates and simulated integration suite do not replace field qualification.
The WIRIS cameras/FTP control path, Base and rover F9P receivers, NGII live caster, three Pixhawk
links, MAVLink receiver-side acceptance, long-duration dual-video shutdown, and automatic ULog
collection still require bench and field soak verification with the intended hardware. A local
MAVLink send result is transport evidence, not proof that the remote receiver accepted and used the
correction.

The Session readiness and method-input panels are software projections of the latest owner
snapshots. They expose missing/stale/rejected evidence but do not independently prove camera timing,
physical alignment, GNSS accuracy, remote RTCM use, or calibration validity. Field operation must
still confirm the owning device/calibration evidence and retain the resulting Measurement package.
Blocked catalog rows indicate absent inputs, not that a solver was executed and failed.

Only the Session refinement in `docs/ui/12_UI_REFINEMENT_PLAN.md` is implemented. Alignment,
Measurement, Recording / Review, global docks, Overview, and Settings changes in that document are
planned work and must not be treated as current behavior.

The launcher's `--check` intentionally rejects an installed environment outside
`requirements/runtime.txt`. On a host with NumPy 1.25+ or pyqtgraph 0.14+, create/use a compatible
environment (the supported constraints currently select NumPy 1.23.5–1.24.x and pyqtgraph 0.13.x)
instead of bypassing the check. PyAV is optional and must be installed from
`requirements-pyav.txt` only when that backend is selected.

A clean checkout does not download model weights. Copy the approved `yolov8m.pt` into
`.local/workstation_data/models/` (or the selected data root's `models/` directory) and require
`run.sh --check` to pass before operation. Historical `apps/Workstation/calibration/` data is
preserved and ignored by Git; migrate it only through an operator-approved copy-and-hash process.

`.local/workstation_data` remains Git-ignored even though its internal settings paths are portable.
To continue on another PC, transfer that directory or an approved checksum inventory separately;
Git pull alone does not provide models, calibration images, recordings, credentials, or device-local
settings. Large external calibration directories are deliberately not copied implicitly by Save.

A calibration image directory is not itself an intrinsics profile. If startup reports a missing
`intrinsics_zoom_XX.json`, calculate and quality-check K,D at that actual camera zoom or explicitly
select an existing profile whose metadata declares the same zoom. Disabling `auto_load_intrinsics`
only suppresses startup loading; it does not make measurement geometry ready. Never rename a
different-zoom profile to satisfy the filename convention.

`review.xlsx` intentionally contains operator-sized summaries rather than every high-rate camera,
Pixhawk, or GNSS sample. Use the typed Parquet files for complete analysis and canonical
`row_json` replay. Parquet readers must preserve signed `int64` nanosecond fields; the workbook
renders nanosecond timestamps and hashes as text to avoid Excel's numeric precision limit. If
workbook generation fails, reopen/regenerate it from the verified Parquet generation.

The Workstation does not prove that a stereo rig's physical transform remains valid as the
lenses change zoom. Validate every intended A→B combination externally and retain that
evidence with the experiment. Missing external cross-zoom validation remains informational and
is separate from the camera-calibration quality gate.

The calibration-quality thresholds are conservative initial defaults and are not a substitute
for field qualification with the actual cameras, lens state, checkerboard, working distance, and
temperature range. The principal-point check assumes a centered native raster; configure a
documented ROI-specific expectation before accepting a deliberately off-center sensor crop. Old
profiles without view geometry or subset diagnostics remain `UNVERIFIED`, so retain their source
images and recalculate or recapture before relying on them for precision measurement.

Historical FTP HR/OZ calibration metadata does not preserve the downloaded still's exact exposure
timestamp. For those photos the table therefore shows the labelled application capture-record time,
not the contemporaneous live-preview RTCP/NTP value. Legacy external images with no capture metadata
show their labelled file `mtime`; neither fallback should be interpreted as camera exposure time.

Solved optical-pose candidates remain session-memory objects unless their extrinsics profile is
explicitly saved. After restart, live Pixhawk/RTK and loaded Checkerboard/saved profiles can become
ready again, but unsaved Horizon/Mountain/Drone/Terrain candidates must be solved again. Hybrid
methods also become unavailable whenever the required live Pixhawk attitude or GNSS baseline is
stale or unusable.

Per-DOF fill currently permits only frame-compatible live sources: Pixhawk for virtual-FRD
rotation axes and GNSS/RTK for NED translation. Checkerboard and saved relative R,t profiles remain
atomic geometry and cannot be used as arbitrary absolute NED axis fillers. Full coupled
Mountain/Multi-view/Terrain/Combined candidates still fail closed when any method-defining DOF is
unobservable; use the explicit Combined solver when cross-candidate axis composition is required.

Automatic HR/OZ retrieval requires working FTP credentials and a camera directory that can be
listed. The freshness guard never substitutes an older still when a new post-CPTR file does not
appear. HR supports exact manual file import; OZ currently relies on automatic FTP retrieval.
When either control/FTP endpoint is missing or a capture times out, the event and any successful
side are retained, but the requested capture source keeps the sealed package incomplete.

Package manifests use SHA-256 chaining for integrity and change detection, not operator identity
or legal non-repudiation. There is no electronic signature, organization PKI integration, bundled
source tree, virtual environment, or container image. Reproduction therefore requires a compatible
Workstation checkout identified by the recorded Git/software provenance.

Video package verification waits for stable bytes and runs bounded `ffprobe`; full-stream decode is
available to the verification helper but is not enabled for every normal `Seal Measurement` because it can be
expensive on long recordings. Validate long-duration dual-camera writer shutdown, camera FTP, and
automatic ULog collection with the actual field hardware before treating those boundaries as
field-qualified.

Automatic restart recovery resumes an open hierarchical session only when exactly one such session
is discoverable. Its interrupted active measurement is marked `RECOVERABLE`; if several open
sessions exist, Workstation refuses to guess and reports that explicit recovery is required.

NTRIP mountpoints are entered manually; the Workstation does not currently browse a
caster's source table. Provider-specific authentication methods beyond standard Basic
authentication are not supported.

If the status is `CONNECTED_NO_DATA` or `STREAMING`, verify the mountpoint and caster's
actual RTCM output. If it is `RTCM_STALE`/`STALE`, inspect the configured read/correction age,
the caster stream, and the downstream Base/Pixhawk route separately; a live TCP session can
exist while valid RTCM or downstream injection is unavailable.

The 4,096-envelope ingress capacity, 96-request Pixhawk injection queue, 2-second stale-injection
limit, 0.5-second UI snapshot interval, and 15 Hz preview cap are currently code-level operational
limits rather than Settings controls. Slow evidence storage intentionally propagates backpressure;
if the field input rate cannot be sustained, stop the measurement and preserve the reported
integrity failure instead of increasing queues without a load test. Unit and simulated end-to-end
tests verify byte/order/hash preservation and bounded UI updates, but long-duration dual-camera +
live-caster + three-Pixhawk operation still requires a field soak test before qualification.

The NGII profile does not identify the Base serial port or receiver firmware automatically in
the settings file. Use Base receiver discovery or confirm the connected F9P device and its
firmware before field operation.

Automatic Horizon detection can confuse clouds, trees, roofs, or multiple strong boundaries
with the sky/terrain edge; inspect its confidence, residual, and overlay before approval. Only
a physical/geometric horizon compatible with the selected `horizon_model` is a valid input --
a mountain ridge, building skyline, or tree line is not; use the Mountain method for a ridge.
Automatic Mountain matching depends on shared visible texture and can be weak in fog,
low-texture terrain, vegetation, or repeated patterns; a failure or low-confidence result
should be handled with manual clicks. Mountain auto-selection does not create terrain heights
or 3-D PnP correspondences and does not replace the GNSS-baseline rotation solver. Terrain PnP
still requires explicit 3-D object points and an imported GeoTIFF with a declared vertical
datum.

Atmospheric refraction is not modeled anywhere in Horizon or Mountain. `spherical_geometric`
Horizon candidates only correct the unrefracted, purely geometric Earth-curvature dip; no
refraction coefficient is applied or estimated. The GNSS-antenna-to-optical-center lever arm is
not automatically estimated anywhere in this codebase; without an explicit calibrated offset it
remains `UNMODELED` (zero-vector proxy) and is a real systematic uncertainty in the Mountain
metric baseline, not a solved unknown.

## Optical-Pose Horizon and Mountain Methods

Both Horizon and Mountain are geometry-from-evidence methods, not PnP: neither consumes a known
3-D landmark or terrain height, and neither performs a 3-D-to-2-D reprojection. Metrics named
`reprojection_error_px` therefore only ever appear for genuine PnP methods (Terrain PnP); Horizon
and Mountain instead report `*_rms_normalized`/`*_rms_px_equivalent` (Horizon, a line/model
fit-quality metric) and `sampson_rms_normalized`/`sampson_rms_px_equivalent` (Mountain, a
normalized Sampson epipolar-coplanarity metric). Neither metric is a statistical pose
uncertainty.

**Horizon.** A fitted image horizon line constrains only the projective direction of gravity in
the camera frame, i.e. Roll and Pitch; Yaw and the optical-center position are never observable
from a horizon and are always copied unchanged from the prior/anchor pose. The prior's expected
gravity direction is used only to resolve the `g` vs `-g` projective sign ambiguity of the fitted
line -- it is never the source of the recovered Roll/Pitch value, and changing the prior Yaw (or
any DOF that does not flip the sign) never changes the recovered Roll/Pitch. Two horizon models
are supported and always recorded on the candidate as `horizon_model`: `flat_local` (`g.r=0`,
the historical default) and `spherical_geometric` (`g.r=sin(dip)`, where `dip` is the
unrefracted spherical-Earth horizon dip from `geometric_horizon_dip_rad(camera_height_m)`).
`spherical_geometric` requires an explicit `camera_height_above_reference_surface_m` -- GNSS
ellipsoid altitude is never automatically substituted for this value, because the two are not
the same physical quantity in general. For a manual two-endpoint annotation, the interpolated
samples between the two clicked endpoints are not independent observations, so a small
`rms_normalized` must never be read as proof of small pose uncertainty; instead the candidate
reports `*_roll_sensitivity_1px_deg`/`*_pitch_sensitivity_1px_deg`, the worst-case Roll/Pitch
change from an independent +/-1px perturbation of each endpoint coordinate one axis at a time.
Endpoints closer together than 25% of the image width are rejected outright, since a short
endpoint baseline amplifies click noise into a large angular error.

**Mountain.** Same-order left/right frozen `(u,v)` clicks (or accepted auto-suggestions) supply
paired image bearings; capture-time GNSS supplies a metric baseline; the selected Left absolute
pose supplies the world-frame anchor. The *direction* of that baseline conditions a two-stage
robust epipolar-rotation fit (`(u,v)` correspondences), while the baseline's *metric length* only
scales the resulting translation -- the fitted rotation and its normalized Sampson residual are
provably invariant to the baseline's metric length. Correspondences are classified inlier/outlier
by a normalized Sampson threshold after a soft-L1 seed solve, and the candidate is only
`apply_eligible` when the final fit has full rotation rank, a bounded Jacobian condition number,
and a sufficient inlier count/ratio; GNSS translation is only `apply_eligible` when both Left and
Right report `RTK_FIXED`. Mountain never measures the Left absolute pose -- it remains the
prior/anchor -- so a candidate's `left_anchor_pose_sha256` input hash changes whenever the
selected Left anchor changes, even for identical pixels and GNSS baseline; UI review must treat
such a candidate as anchor-dependent, not as an independent absolute-pose measurement. The GNSS
antenna is not assumed to be the camera optical center: without an explicit
`lever_arm_baseline_correction_left_rdf_m` override the candidate's `lever_arm_status` is
`UNMODELED` and the raw antenna baseline is used as an optical-baseline proxy; this module never
estimates that correction automatically.

Frozen Mountain evidence resolves its GNSS baseline from the Drone RTK Capture Observation bound
to that exact frozen burst (or from an explicit payload override), never from whatever GNSS is
live on the panel at solve time -- a later live-GPS change cannot silently retarget an already
frozen Mountain image.

## Change Log

- 2026-08-21: Made persistent Workstation asset paths checkout-independent. New defaults and saved
  settings use `SASL_WORKSTATION_DATA_ROOT`-relative paths; external intrinsics and model files are
  atomically copied into content-addressed data-root storage with SHA-256 verification; external
  calibration directories and traversal/symlink escapes fail closed. Added runtime validation,
  tracked-path audit tooling/CI, repo-root-derived Simulation defaults, and an `AGENTS.md` path
  portability gate while preserving historical evidence paths as non-replay provenance.

- 2026-08-21: Fixed restart auto-load of explicitly selected intrinsics. Startup now prefers the
  exact persisted `intrinsics_path`, preserves that real source path in the UI/provenance, requires
  matching `metadata.zoom_level`, rejects malformed/unknown/mismatched exact files without fallback,
  rejects missing external paths without substitution, and consults generated storage profiles only
  for a missing calibration-root alias. New calibration saves replace the persisted exact path, and
  a rejected apply restores the preceding active calibration state.

- 2026-08-21: Refined Session presentation around operator decisions. Readiness now lists blockers
  only, one valid next action is promoted, and a compact eight-row method catalog keeps Mountain,
  Drone RTK, Terrain, and Combined visible before candidates exist. Full values, instance/hash,
  blocker, required inputs, and coordinate contract move to selected-method detail; runtime method
  selection remains instance-driven and unchanged. Added a plan-only refinement document for the
  remaining workspaces.

- 2026-08-21: Promoted the daily parent lifecycle to a top-level `Session` workspace. It owns Open,
  New Measurement, Seal Measurement, and Close; separates evidence-collection readiness from valid
  measurement readiness; lists ordered child state; and observes preferred/effective method,
  fallback reason, live PX4 R/P/Y, GNSS NED baseline, candidate pose, or checkerboard `R,t` without
  collapsing their coordinate contracts. Live acquisition remains in `Measurement`, while saved
  package/media inspection is now labelled `Recording / Review`.

- 2026-08-21: Closed the critical save/seal integrity gap. The save controller now owns a valid Qt
  worker thread, preserves failed spools for retry, and blocks measurement manifests until session
  save and the measurement-scoped RTCM drain/semantic-verification barrier both succeed.

- 2026-08-21: Hardened RTCM lifetime and persistence. Enqueue freezes the measurement context;
  binary/index pairs use checked offsets, fsync/readback and rollback; injection rechecks age at
  send time; reconnect retires the old subscriber; and terminal delivery outcomes plus drain
  high-water are inventoried before seal.

- 2026-08-21: Hardened evidence containment and append-only behavior. Recovery rejects paths outside
  the session root, secret-bearing multiline diffs are omitted fail-closed, late evidence can satisfy
  a required role without erasing failure history, graph hiding no longer deletes canonical rows,
  corrupt JSONL remains hashed evidence, and preview revalidates artifact bytes.

- 2026-08-21: Separated Unix/RTCP and monotonic clock domains and tightened geometry/calibration
  gates. All methods require exact operating-zoom K/D provenance; live GNSS consumers require fresh,
  synchronized fixed data; FTP/import time remains unverified without exposure evidence; ICPT,
  horizon, fixed-base, survey-in, GGA, and numerical rotation paths now fail closed.

- 2026-08-21: Restored truthful UI/runtime closure. Navigation and diagnostics preserve their
  workspaces, no-sample/stale device states cannot display success, settings Apply persists the
  visible payload, run setup locks only after evidence exists, and synchronous capture results are
  no longer overwritten by pending labels.

- 2026-08-21: Centralized settings, recordings, calibration, models, and caches under
  `.local/workstation_data` with `SASL_WORKSTATION_DATA_ROOT` override, preserved known legacy/custom
  paths without destructive migration, added dependency/backend/model validation to
  `run.sh --check`, pinned a compatible pyqtgraph line, and made PyAV an explicit optional overlay. This
  supersedes the 2026-08-20 default calibration-root location while leaving its data intact.

- 2026-08-21: Simplified Measurement UI ownership. Removed Intrinsics source, manual focal, and
  YOLO input-size controls from Measurement Settings; moved canonical YOLO input size to
  `Settings > YOLO / RTSP`; and moved the Session Details entry point below Measurement history
  while preserving its `Recording / Sessions` navigation target.

- 2026-08-21: Added per-Measurement DOF fill controls for every available method. Method-observed
  Left/Right R/P/Y/N/E/D values render as disabled `OWN`; compatible missing rotations and
  translations expose `PX4`/`GNSS` toggles with safe defaults. Policies lock at Detection, persist
  in child metadata and run evidence, survive restart independently of Settings, and fail closed
  when a required fill is disabled or a method-defining DOF is unobservable.

- 2026-08-21: Renamed capture controls by evidence source: live decoded-stream pairs now use
  `Capture Live Pair`, camera-native stills use `Capture HR Still`/`Capture OZ Still`, immediate
  downloads append `(FTP)`, and disk-loaded pairs use `Import Pair`. Command IDs and stored capture
  schemas remain unchanged; the live action retains RTP/RTCP metadata but is not raw RTP capture.

- 2026-08-21: Added per-photo Left/Right timestamps to the calibration Compute file table and
  provenance tooltips. Native/manual and Auto photos use side-specific RTCP/NTP frame time; FTP/OZ
  and legacy files use explicitly labelled application-record/request or file-mtime fallbacks.
  Metadata file changes now invalidate the table cache, and the compatibility pair picker uses the
  same per-side timestamp renderer.

- 2026-08-21: Hardened checkerboard calibration against unstable `cx,cy` results. Calibration now
  uses matched lossless native frames, rejects repeated Auto-capture poses, deduplicates detected
  observations, solves left/right intrinsics from independent selections, and forms stereo pairs
  only from observations valid on both sides. Added `calibrateCameraExtended` diagnostics,
  deterministic subset-stability checks, pose/coverage/principal-point quality evidence, UI
  `PASS`/`WARNING`/`FAIL` review, save/apply gates, native-grid compatibility checks, and JSON and
  measurement-evidence persistence of the quality snapshot.

- 2026-08-21: Moved full-rate NTRIP/Base RTCM parsing, exact-byte evidence, Base forwarding, and
  Pixhawk injection accounting off the Qt GUI thread. Added bounded/backpressured ingress,
  producer timestamps, reconnect/recovery-safe sequence high-water, binary/index hash validation,
  explicit stale/drop/unconfirmed outcomes, and drain-before-MAVLink shutdown. Added a latest-only
  15 Hz preview inbox while keeping native measurement, detection, and recording paths independent.

- 2026-08-21: Simplified recording persistence. New sessions now live at
  `recordings/<session-id>/`; legacy `recordings/sessions/` trees remain read/recovery-only. Live
  JSONL spools finalize into five verified typed Parquet datasets with lossless canonical
  `row_json`, and one session-level `review.xlsx` replaces default CSV and per-device/per-measurement
  workbooks. Added single-pointer `data` publication, non-fatal report regeneration, direct/legacy
  collision guards, in-place registration of already sealed ULog/Target Drone artifacts, and
  explicit appendable-ledger-only revision snapshots.

- 2026-08-21: Split the explicit parent session from individual measurement packages. `Start
  Session` now opens a timestamp session, `New Measurement` creates isolated `mNNNNNN` output
  roots, `Seal Measurement` seals one child, and `Close Session` hashes the ordered child manifests
  and session-shared evidence. Added nested catalog/review keys, nested spool recovery, idempotent
  child/parent seal retry, exact measurement IDs in recording rows, and single-open-session restart
  recovery.

- 2026-08-21: Added portable, revisioned measurement evidence packages with explicit
  `Start Session → Detection → Post Capture → Finalize`, active-source completeness,
  COMPLETE/INCOMPLETE and RECOVERABLE states, SHA-256 manifest chains, relative-path review,
  calibration/model/settings/software provenance, stable-video probing, canonical per-pair
  inputs/results, exact native frames, HR/OZ validation, RTCM/Drone raw evidence, spool-tail
  recovery, legacy-session review, and Measurement/Session Details controls.

- 2026-08-21: Removed the optional `Initialize from Telemetry` optical-pose initializer and its
  command/backend delegate. Drone RTK Multi-view and the other independent measurement methods
  remain available.

- 2026-08-21: Removed the redundant outer `R,t methods`/inner solver-tab nesting. Alignment now
  keeps common readiness and adaptive frozen previews above seven promoted method tabs, preserves
  annotations while auto-switching mode by tab, and keeps Candidate review beside the method area
  on wide screens or below it on narrow screens.

- 2026-08-21: Removed the Alignment LRF status/refresh UI while retaining low-level GLRD in the
  Advanced WIRIS command catalog; verified Horizon/Mountain Auto-Select remains available through
  the background suggestion worker and explicit Use suggestion flow.

- 2026-08-21: Split Optical Pose into method-specific tabs; added ready-only Measurement method
  selection, per-axis Pixhawk/GNSS completion for partial candidates, Settings-owned fallback
  priority, pair-boundary automatic fallback/recovery, and method/profile provenance in recording
  evidence.
- 2026-08-21: Hardened ready-method resolution so pair attitude/GPS snapshots are frozen before
  method-specific R,t solving, final geometry hashes include the exact live values, interrupted
  recovery timers restart, no-ready pairs remain blocked evidence, and full coupled solvers no
  longer receive generic per-axis live substitutions.

- 2026-08-20: Added a dedicated `Diagnostics > FTP` tab with a bounded FTP lifecycle log for
  HR/OZ capture, download, calibration save, OZ sync/delete, local paths, and failures.
- 2026-08-20: Changed the calibration pair picker default from a fixed recent-count window to
  pairs saved after the current Workstation launch; `전체 사진 보기` remains the explicit
  way to review legacy pairs.
- 2026-08-20: Moved the default calibration root to `apps/Workstation/calibration` and created
  one timestamped subfolder per Workstation calibration run for FTP HR/OZ and checkerboard files.
- 2026-08-20: Isolated Auto capture output under
  `calibration/<timestamp>/calib_capture/AutoCapture/left|right`
  so automatically collected pairs cannot mix with manual or FTP calibration runs.
- 2026-08-20: Split Calibration into Capture/Compute tabs and added a saved-session selector
  that routes pair review, calculation, save, and load/apply through one calibration root.
- 2026-08-20: Added pre/post-CPTR FTP freshness validation and polling; stale remote stills are
  rejected instead of being saved as the current HR/OZ calibration image.
- 2026-08-20: Enabled WIRIS FTP download by default for new settings and migrated the current
  Workstation settings value to enabled.
- 2026-08-20: Made FTP capture discover dynamic WIRIS session image folders from `/`, prefer
  `highRes`/`visible` files for HR/OZ, retry delayed FTP startup, and preserve `ftp_error` detail.

- 2026-08-20: Moved Horizon/Mountain auto-selection off the Qt UI thread, disabled duplicate
  optical-pose actions while analysis runs, and added frozen-frame identity validation before
  committing a background result. This prevents the `Main.py is not responding` symptom caused
  by synchronous SIFT/gradient analysis and preserves stale-result safety.

- 2026-08-17: Hardened optical-pose Mountain and Horizon evidence/geometry. Mountain now
  resolves its GNSS baseline from the frozen burst's own Drone RTK Capture Observation (never
  from live panel GNSS), fits relative rotation with a baseline-scale-invariant normalized
  Sampson residual and a robust soft-L1-seed/inlier-refit/rank-and-condition-gated solve, records
  `left_anchor_pose_sha256` and an explicit `lever_arm_status` (`UNMODELED` unless a calibrated
  correction is supplied), and no longer reports a `reprojection_error_px` metric it never
  computed. Horizon gained an explicit `flat_local`/`spherical_geometric` model selection (the
  spherical model requires an explicit `camera_height_above_reference_surface_m` and is never
  fed GNSS altitude automatically), documented that its expected-gravity prior only resolves a
  sign ambiguity, and added `*_roll_sensitivity_1px_deg`/`*_pitch_sensitivity_1px_deg` so a
  small line-fit RMS from a two-point manual click is never read as a pose-uncertainty proof.
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
