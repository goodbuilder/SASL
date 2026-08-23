# SASL StereoWIRIS Workstation

## Purpose

This package provides the operator Workstation for synchronized stereo-camera capture,
calibration, measurement, Pixhawk/GNSS evidence, and recording. It owns its runtime and UI
independently from the Legacy application. Calibration capture, calculation, load, save, and apply
can run before a parent session exists, using the configured reusable calibration root. Explicit
`Create Session Folder` has no device, synchronization, calibration, method, or Alignment
readiness gate; it allocates the timestamp-named container. The top-level `Sessions`
workspace combines `Current`, `Evidence`, and `Review` without changing backend ownership.
`Current` shows the folder lifecycle and ordered children, `Evidence` projects the active package
and capture/ULog evidence, and `Review` opens saved session/measurement artifacts.
Calibration performed after opening and all
Alignment evidence are Session-shared, while an active pre-session profile is snapshotted into each
Measurement package that uses it. Every measurement is automatically saved and sealed as one
portable evidence folder, and closing the parent session seals the ordered child-manifest inventory.
Session-shared Calibration and Alignment selections are versioned as immutable content-addressed
files. Compatibility aliases may move, but every Measurement copies the exact selected bytes and
SHA-256 into its own input evidence before those bytes can affect a valid result.
Measurement readiness, geometry-method inputs, Detection, and retry-save controls remain in the
`Measurement` workspace; stored artifacts remain in `Sessions > Review`. High-rate RTCM
and camera-preview traffic is isolated from the Qt GUI thread: full-fidelity routing/evidence stays
on worker-owned data paths, while the UI consumes bounded latest-state snapshots. Hidden camera
panes retain only the latest implicitly shared `QImage` and do not perform QPixmap conversion or
smooth scaling; calibration overlays remain explicitly throttled. Every child
Measurement also owns an immutable all-method input boundary, including explicit per-camera
Roll/Pitch/Yaw and North/East/Down fill decisions. Finalization is fail-closed: the
session save must complete, measurement-scoped RTCM must be drained and semantically verified,
and every package artifact must pass containment and hash checks before a manifest can be sealed.
The derived session `review.xlsx` keeps the common operator sheets and adds one stable worksheet
for each of the eight Measurement geometry methods, populated from that method's authoritative
per-pair outcome stream.

Runtime fault isolation keeps one callback, device, camera child, or evidence writer failure from
silently taking down unrelated links. Qt callback exceptions are logged and trigger a best-effort
session flush; bounded device/worker queues expose timeout, retry, partial-success, and recoverable
storage states instead of blocking the GUI indefinitely.

## Current Run Flow

From the repository root, run `./apps/Workstation/run.sh --check`, then start the UI with
`./apps/Workstation/run.sh`. The check validates declared dependency versions, Workstation
imports, the selected GStreamer/PyAV backend, and the approved default YOLO model's size and
SHA-256 against `model-manifest.json`; it exits nonzero on a missing or incompatible boundary.
Use `--no-start` to open without camera discovery or preview and `--start-pixhawk` to connect the
configured Pixhawk endpoints after launch.

Each manual `Connect cameras` request performs one WIRIS discovery, activation, and RTSP-frame
verification attempt. A failed request does not retry the same camera IP in the background; press
`Connect cameras` again to start a fresh discovery after correcting power, Ethernet mode, or IP.
This initial-connect rule does not disable the default GStreamer backend's post-connect recovery
for an interrupted live RTSP stream.
If a camera child process itself exits unexpectedly, its owner reports
`스트림 프로세스 크래시 복구 중`, restarts it with bounded backoff, and stops after three
consecutive failed recoveries with an explicit manual-restart alarm.

The field dual-Pixhawk profile uses Workstation `10.0.0.10/24`, P1/Left
`10.0.0.2:14550` with MAVLink system ID `1`, and P2/Right `10.0.0.3:14551` with system ID `2`.
Save each board's Ethernet profile over USB once, reboot it, then verify both heartbeats with USB
removed. The exact parameters, SD-card files, recovery sequence, and field checks are recorded in
[`Pixhawk.md`](Pixhawk.md).

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

Keep the persisted WIRIS visible-stream encoder profile at bitrate `0`, QP `17`, GOP `30`,
and IDR interval `1`. Bitrate `0` selects the camera's constant-quality mode so QP is active;
any nonzero bitrate disables QP control and can make the Live RTSP image visibly blocky. Whenever
the Workstation enables or reactivates Ethernet/RTSP mode, it sends that profile to each selected
camera and verifies all four SDK readbacks before probing the stream. Edit the profile under
`Settings > YOLO / RTSP > WIRIS visible encoder profile`.

`Sessions > Current > Create Session Folder` creates
`recordings/YYYYMMDD_HHMMSS_nanoseconds/` using the local date and creation time. It directly calls
the Session-folder backend and does not inspect cameras, Pixhawks, Drone Telemetry, time
synchronization, Calibration, Methods, or Alignment; it
may be used before those inputs are ready. After creation, verify that `State` reads
`Ready for saving` and that `Folder name`, `Created`, and `Storage path` identify the intended
directory. The action is unavailable only while an existing parent is open or another lifecycle
operation prevents safe ownership of new outputs.

Calibration can be completed before or after `Create Session Folder`. Before a Session exists, capture,
calculation, load, save, and apply use the configured reusable calibration root. After a Session is
open, new calibration runs and loaded-profile copies are stored as Session-shared assets. Alignment
capture remains Session-owned. Approved K,D/R,t bytes are published under content-addressed
`profiles/` paths; `intrinsics.json` and per-zoom filenames remain compatibility aliases rather than
Measurement evidence identities. A `Pre` HR/OZ capture lazily creates a Measurement draft; otherwise
`측정 시작` creates the next child immediately before Detection. Child creation is lifecycle-only
and does not require geometry readiness, so Pre and blocked evidence retain the correct owner. Before Detection produces a
valid result, apply the intended operating-zoom `K,D` and an eligible stereo `R,t`/Alignment source,
connect and verify the required devices, wait for the intended camera synchronization state, and
check the separate
`Evidence collection` and `Valid measurement` projections in `Measurement`, and resolve its
blockers. Every matched pair is evaluated by Pixhawk, Checkerboard, Horizon, Mountain, Drone
Rotation, Drone Multi-view, Terrain, and Combined against one shared immutable snapshot. The
representative result is always `Pixhawk attitude + GPS/RTK`; another method is never promoted when
that primary method is blocked or fails. Overview shows all eight methods as read-only cards with
READY/ACTIVE/BLOCKED/FAILED/TIMEOUT/REUSED state, the first blocker plus `+N more`, and per-Measurement
VALID/BLOCKED/FAILED counters. Clicking a card opens Measurement details without changing a method.
Press `측정 시작` once and
`측정 종료` when that Measurement is complete. Measurement stop automatically
publishes that Measurement's typed Parquet generation, drains its producers, and requests sealing;
the `Retry Measurement Save` action is reserved for recoverable failures. The session-level
`review.xlsx` remains pending until `Close Session` creates the integrated workbook once from all
sealed measurements. Review `M_Pixhawk_RTK`, `M_Checkerboard`, `M_Horizon`, `M_Mountain_GNSS`,
`M_Drone_RTK_Rotation`, `M_Drone_RTK_Multiview`, `M_Terrain_PnP`, and `M_Combined` for the
method-specific projections; a header-only method tab means that Measurement contains no evaluated
pair for the method. Automatic finalization waits for capture, video, all pending eight-method
barriers, and the real session-save worker, closes and
drains that measurement's RTCM evidence context, verifies its binary/index high-water, and only
then seals the current child revision as `SEALED_COMPLETE` or `SEALED_INCOMPLETE`. A save/drain or
semantic-integrity failure leaves the child `RECOVERABLE` without a new manifest. Press `측정 시작`
again after sealing to create the next isolated child (`m000002`, `m000003`, ...), or return to
`Sessions > Current > Close Session`. Close automatically saves and seals an empty or Post-Capture current
Measurement before sealing the ordered parent index. If Detection is still running, stop it first;
save or integrity failure leaves the Session open and recoverable. `Review Stored Files` switches
to `Sessions > Review`, which reviews children by the composite session/measurement identity. A
revision adds late evidence to the same measurement and never grants another Detection interval.
`Protocol` owns Preflight; target Drone/UAM reference connectivity remains in
`Sessions > Current`, and manual recording remains an advanced fallback under `Advanced > Files`.

Session folder state and Measurement readiness are intentionally independent. `OPEN` means only
that the displayed directory owns subsequent outputs; it does not claim that devices, calibration,
or a valid geometry method are ready, and opening it never waives downstream checks. The Session
workspace never polls device or geometry-method readiness. Per-method readiness, blockers, frozen
source values, and their PX4 FRD/NED, GNSS NED, candidate virtual-FRD/NED, or OpenCV RDF contracts
remain in Measurement/Alignment projections and persisted pair evidence.

For checkerboard calibration, select the calibration zoom, command both cameras to that
zoom, confirm the actual zoom readback, capture synchronized pairs, and calculate/save the
required intrinsics or stereo `R,t` profile. At measurement time, load the `K,D` profile for
the intended operating zoom and independently load the chosen `R,t` profile, then apply the
bundle. This primary workflow uses independent K,D and R,t candidates. The Advanced validation
action `Apply result` is narrower: in Checkerboard mode it applies that full result's own embedded
K,D and R,t as one exact bundle, and never reuses an older selected R,t path with the fresh K,D.

Calculate, Load, and Save create or preserve calibration candidates; they do not replace the
matrices, profile path, hash, or zoom identity currently active in Measurement. Only a successful
Apply promotes candidate `K,D`/`R,t` into the active bundle after the calibration-quality and
native-raster gates. Checkerboard and Optical-Pose candidates remain visible for review. Apply is
still the only global Active promotion, but each new Measurement independently freezes the latest
successful Draft as a comparison input without changing that Active state. The frozen hybrid keeps
its per-axis candidate/Pixhawk/GNSS source contract; later K,D or Draft changes affect only the next
Measurement. Optical-Pose
Apply additionally requires one existing active K,D matrix/path/content-hash identity; a nominal
runtime fallback is not an active calibration for this transition. The complete next profile and
composed intrinsics are prepared before owner mutation, and a persistence failure restores the
preceding active fields, selection history, and Measurement cache.

The Measurement comparison table is an analysis-only surface. Unapplied candidates may be
evaluated there only when their captured K,D profile hash still matches the active K,D identity;
stale candidates produce a blocked analysis row and no coordinates. Persisted comparison rows use
`evidence_namespace=analysis/geometry_comparison`, `result_class=analysis_only`, and
`not_measurement_result=true`, with candidate/profile/K,D identities. They are never a primary
MeasurementAttempt or an official distance result.

Blocked-method semantics are explicit across canonical recording and derived review. A present
`measurement_method_id` key is authoritative even when its value is empty. A legacy
`measurement_geometry_kind=unresolved` row with no method remains excluded from every `M_*` sheet;
new BLOCKED/TIMEOUT/FAILED per-method outcomes stay in their own method sheet. Only a row whose
method key is truly absent under the legacy paired-frame v1/v2 schemas may migrate the bounded
legacy aliases `pixhawk`, `checkerboard`, or `calibration`.

Calibration selection distinguishes an absent sidecar from an invalid existing sidecar. Absence
selects the normal automatic policy. Existing JSON must use the supported v1/v2 schema and valid
field types, portable paths/pair identities, and manual-selection semantics; malformed JSON,
unknown schema, unrecoverable paths, or semantic errors block calculation before a worker is
created and never fall back to all photos.

Each calculation command now freezes one `workstation-calibration-input-snapshot-v1` before the
worker starts. The visible Photo set root, independent left/right files, selected stereo pairs,
includes/excludes, automatic/manual policy, pair limit, sidecar identity, zoom-metadata references,
and exact file size/SHA-256 values define its hash. The worker derives every path from this snapshot,
reverifies bytes immediately before OpenCV reads them, and echoes the same snapshot/hash into full
calibration diagnostics or R,t candidate metadata.

At startup, automatic intrinsics loading first checks the exact persisted `intrinsics_path`. An
existing exact profile is applied only when it parses successfully and its
`metadata.zoom_level` equals the current operating zoom. A malformed, unknown-zoom, or mismatched
exact profile fails closed and is never replaced by another convenient file. Only when the exact
configured path is the calibration root's missing `intrinsics.json` alias may startup fall back to
that root's `intrinsics_zoom_XX.json`, under the same zoom check. A missing explicit external path
never falls through to unrelated storage bytes.

Measurement Settings does not select Intrinsics or a manual focal value. The active exact `K,D`
comes from `Calibration > Intrinsics > Profiles`. Apply the compatible stereo bundle under
`Calibration > Extrinsics > Profiles`. Configure `YOLO input size` under
`Settings > YOLO / RTSP`; Apply updates the runtime owner and the new size is used from the next
Detection start. Confidence, IoU, and pair gates remain in Measurement Settings.

Select detector weights under `Settings > YOLO / RTSP > YOLO runtime` with `파일 선택…` beside
`Weights path`. Selecting a file outside `SASL_WORKSTATION_DATA_ROOT` atomically imports and
SHA-256-verifies it under `models/imported/<sha256>/`, then fills the portable relative path.
Cancellation or import failure preserves the previous selection and reports the failure in the
same settings group. `Apply` uses the selected model from the next Detection start; `Save` also
persists that relative path for later Workstation runs. The picker accepts `.pt`, `.onnx`,
`.engine`, and `.torchscript` files.

Workstation uses the single `field_measurement_v1` policy. When no calibrated profile is active,
fresh equal left/right zoom readbacks plus known native rasters automatically generate a
`NOMINAL_UNVERIFIED` pinhole profile from the 1280x720 WIRIS zoom focal table. A structurally valid
calibrated profile whose zoom differs from the fresh readback also yields to that nominal profile.
The generated K,D uses centered principal points and zero distortion, records its formula/hash, and
always retains `precision_qualified=false`. Malformed K,D, stale/unknown/disagreeing zoom, and
native-raster mismatch remain hard failures.

Field measurement blocks only missing/ambiguous correspondence, missing or malformed K,D/R,t,
same-Measurement telemetry absence for live geometry, non-finite/frame-invalid inputs, impossible
two-ray geometry, lifecycle conflicts, persistence integrity failures, unapplied geometry
candidates, and missing/mismatched candidate-to-active K,D identity. RTCP/NTP availability, pair
delta, vertical delta, telemetry age, GNSS fix/accuracy/readiness, and nominal K,D remain visible
advisories while a finite result is recorded as `ESTIMATED`.

Calibration capture now saves a lossless, no-overlay native frame and records its native,
preview, and saved raster grids; a preview-resized image is not used as the calibration input.
Auto capture rejects a pair when either camera's checkerboard pose is effectively a duplicate of
an already accepted pose. Move the board across the image, change distance and roll, and introduce safe
yaw/pitch perspective instead of collecting many fronto-parallel repeats.

In Calibration's shared input area, select a saved calibration session and review its matching Left/Right
files in `계산에 사용할 사진`. The table previews the session candidates until the operator fixes
independent Left and Right selections with the two file-selection buttons. The selected Photo set
is sent as both the owner calibration root and calculation/validation root in the same setup
transaction, so the worker scans the exact dataset shown in the table.

Manual selections and outlier exclusions are stored beside that Photo set in
`calibration_pair_selection.json` schema v2. Its `root` is `.` and image fields use
`left/<name>` or `right/<name>` POSIX paths. Legacy checkout-absolute paths are rebased only when
their suffix can be proven to remain under the current Photo set. If any selected or excluded path
cannot be recovered, the UI reports the path error and calculation blocks instead of silently
falling back to every image.

The `계산에 사용할 사진` table shows a separate local date/time for every Left and Right
photo, to millisecond precision, inside each information cell. Its tooltip shows the timestamp
source and evidence path. A manual/native or Auto image uses its side-specific RTCP/NTP frame
time; FTP/OZ files use the application capture-record time because the historical live-preview
RTCP value is not the downloaded still's exposure time. Missing or malformed legacy metadata
falls back to the explicitly labelled file `mtime`.

Calibration is split into `Intrinsics` and `Extrinsics`. Each has `Capture`, `Compute`, and
`Profiles` tabs. Zoom, camera preview, saved photo-set selection, and exact left/right file rows are
shared. Intrinsics independently solves and quality-checks each camera's K,D. Extrinsics consumes
the synchronized rows plus the selected left/right K,D and solves left-to-right R,t in OpenCV
optical RDF. The final Apply action exists only under `Extrinsics > Profiles` and applies the
verified K,D+R,t bundle.

When either workflow selects `Capture`, the shared Live/FTP camera pair is followed immediately by
one capture action bar. Preview, live/stereo pair capture, automatic capture, HR/OZ FTP capture,
and FTP sync use the existing command owners. Auto settings and per-camera command/response logs
are collapsed by default. Switching to Compute or Profiles hides the preview/capture surface so
calculation controls use the available workspace height. Intrinsics and Extrinsics share the same
workflow index, and their Capture hint rows have equal height, so switching calibration kind does
not move the preview or capture bar.

`OZ 촬영` stores repeated OZ stills on both cameras without downloading them. `FTP 자동 전송`
then discovers every OZ still accumulated since the previous sync, pairs Left/Right files by
trigger order, and downloads them automatically. It is not a camera-storage browser and does not
offer manual remote-file selection. HR/OZ workers synchronize the two host-side `CPTR` dispatches
and record per-camera monotonic/Unix dispatch timestamps under one capture ID. Calibration accepts
the downloaded stills as a pair only when that explicit dispatch contract exists and its delta is
within the configured pair gate; the camera exposure timestamps remain unknown.

The Calibration Live and FTP preview pairs always divide their available width equally. Both
camera surfaces therefore have the same displayed 16:9 size; moving the splitter handle restores
the equal-width layout instead of leaving one camera visually larger than the other.

After calculation, review the calibration-quality panel before saving or applying the result.
It exposes the effective-pose count, 3×3 image coverage, principal-point offset, pose/perspective
diversity, and deterministic subset stability. `PASS` is directly usable, `WARNING` requires an
explicit operator confirmation before Apply, and `FAIL` cannot be applied. `WARNING`/`FAIL` may be
saved only as non-applicable attempt evidence; doing so never promotes active calibration state.

`Capture HR Still` can be pressed repeatedly whenever the operator needs high-resolution evidence.
Each press enables visible-HR still storage, captures one photo on each camera, and waits
for both cameras to report completion. `Capture OZ Still` sits beside it in `Sessions > Evidence`
and uses the same `Pre`/`Post`/`Checkpoint` purpose selector. It sends `SIVI TRUE`, `CPTR`,
and polls `ICPT` on both cameras for a camera-native visible/OZ still; the field camera
configuration is expected to provide the higher-resolution FHD evidence. Use `Pre` before
measurement and `Post` after measurement when this camera-local evidence is needed. Both
capture paths are independent of the RTSP/MP4 session video.

For network RTK, open `Settings > GNSS`, select either network-assisted fixed-base or
direct network RTK operation, and confirm the caster hostname, port, mountpoint, TLS setting,
GGA policy, and direct Username/Password fields. A newly created settings file starts with the
NGII RTS2 Base-assisted profile: `RTS2.ngii.go.kr:2101`, mountpoint `VRS-RTCM32`, Base GNSS
GGA, WGS84 ellipsoid height, RTCM routing to both left and right Pixhawk rovers, Username
`SASL`, and masked session Password `ngii`. The runtime owner receives the same credential
defaults during launch, so `Start NTRIP` can use them even if Settings was never opened.
Editing either field overrides its default; after editing the Settings workspace, press `Apply`
or `Save` before starting NTRIP so the masked Password reaches the runtime owner. Other compatible
casters remain configurable; existing custom provider configurations are preserved, while retired
legacy values are migrated to the NGII profile when loaded.

The legacy environment-variable fields remain an optional fallback. They contain variable
*names*, not credential values. To use that path, clear the direct Password, press `Apply`, and
export the configured variables before launch:

```bash
export NGII_RTS2_USERNAME='SASL'
export NGII_RTS2_PASSWORD='<RTS2 password>'
./apps/Workstation/run.sh
```

The active Password is never written to `stereo_distance_settings.json` or measurement evidence.
The deployment default itself is nevertheless plaintext in
`workstation_core/gnss_operation.py`; it is an operational convenience, not secret storage.

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

The Base GNSS `RTCM distribution` table reports correction delivery independently for `P1`,
`P2`, and Telemetry (`T`). Each row shows frames, MAVLink packets, bytes, failures, and the
age of the last successful delivery. A `T` row with increasing counters is evidence that NTRIP
RTCM is being forwarded through the connected Drone Telemetry MAVLink endpoint; zero counters
or failures must not be interpreted as active delivery.

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
per image for the single-target Drone RTK Yaw fallback. Once both target pixels are present,
the Workstation also binds that pair to the same Frozen burst's Drone/P1/P2 GNSS snapshot and
accumulates it for JSON-free multi-view `R,t` refinement.

In Alignment's frozen Left/Right camera previews, wheel up/down zooms around the native pixel under
the cursor from 1× to 8×. Zoom changes presentation only: annotation coordinates and frozen source
pixels remain unchanged, and returning to 1× recenters the full image.

The Alignment solver surface exposes top-level `Horizon`, `Mountain + GNSS`, `Terrain PnP`,
`Drone Rotation`, `Drone Multi-view`, `References`, and `Combine` tabs below the common frozen
preview. On wide screens each camera preview is shown side by side and the common Candidate review
stays to the right of the selected method; narrow screens stack the previews and place Candidate
review below the tabs. Measurement no longer chooses one geometry method or promotes an automatic
fallback. At Measurement creation, the latest successful Draft for each non-Pixhawk method is
approved only as that child's `Frozen Comparison Input`; this does not mutate the global Alignment
Active selection. The exact operating K,D and every available Draft are copied under
`inputs/methods/<method_id>/frozen_input.json`. A Draft created or changed later applies only to the
next Measurement, and a missing Draft produces an explicit BLOCKED outcome for every pair.

Horizon uses its frozen observed Roll/Pitch plus pair-frozen Pixhawk Yaw and GNSS Translation. A
single-target Drone Rotation Draft re-solves Yaw against pair-frozen Pixhawk Roll/Pitch;
two-or-more-target Drone Rotation keeps its solved full R and adds pair-frozen GNSS Translation.
Mountain, Multi-view, Terrain, and Combined Drafts must already be full 6-DOF and fail closed
instead of borrowing arbitrary live axes. All eight method attempts for a pair share the same
left/right detections, telemetry snapshots, native-frame file identities, and capture-sync basis.

The Measurement `DOF fill for` selector exposes every currently available method. For each Left
and Right Roll/Pitch/Yaw/North/East/Down component, `OWN` means the method observes that value and
the control is disabled. A missing rotation component exposes a `PX4` toggle; a missing translation
component exposes a `GNSS` toggle. Safe fill toggles are enabled by default, can be disabled for the
current Measurement, and lock when Detection starts. Method-defining observations (Horizon
Roll/Pitch, Drone Rotation Yaw, and every DOF of full Mountain/Multi-view/Terrain/Combined methods)
cannot be replaced by a fallback merely to make an invalid candidate look ready.

Runtime evaluation uses one dedicated persistent process for
`pixhawk_rtk_live` and seven independent persistent comparison processes. OpenCV/NumPy/BLAS child
thread counts are capped at one. Comparison timeouts start only after that method worker reports
actual execution start: Checkerboard 3 s, Horizon 1 s, Mountain 3 s, Drone Rotation 3 s, Drone
Multi-view 5 s, Terrain PnP 10 s, and Combined 3 s. A timeout or crash terminates and recreates only
that method process. The primary Pixhawk result may appear immediately, but permanent pair commit
waits for all eight terminal outcomes and writes them in canonical method order. At most 20 pair
snapshots may be pending; the next pair is still persisted as `BACKPRESSURE_BLOCKED`. Queue payloads
contain paths/hashes and immutable numeric metadata, never QImage or raw pixel bytes; native PNGs
are stored before enqueue.

Each Alignment method tab shows method-specific evidence alongside the read-only Candidate
observability summary: Horizon shows model/RMS/sensitivity, Mountain shows pair/inlier/Sampson/baseline/lever-arm
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
later download time is never substituted. When both frozen images have exactly one Drone Target,
the annotation is bound to the capture observation with matching burst/frame identities. The
Observation History `Target` column shows `READY` only when Drone/P1/P2 positions are RTK_FIXED,
the frame/Drone GNSS clock domain is verified, the delta is within the configured 350 ms bound,
and the exact frozen operating `K,D` is geometry-applicable. Incomplete or unverified bindings are
retained as `BLOCKED` audit evidence and are never passed to the solver.

The `Drone RTK Multi-view R,t` tab consumes the Session's accumulated READY target pairs directly;
its normal UI path does not open or require a JSON file. Capture and annotate at least four Frozen
pairs in `Drone Rotation`, move the Drone through varied three-dimensional positions, then press
`Solve Drone RTK Multi-view R,t`. All samples must share the same exact operating
`K,D` profile. The first READY sample's P1 GNSS is the local NED origin; every captured Drone GNSS
position is converted to `(north, east, down)` metres in that frame. P1 is initialized at the
origin and P2 from its captured P2-minus-P1 GNSS baseline, while the solver optimizes full 6-DOF
camera `R,t` for both sides. Full-rank geometry is mandatory; repeated or poorly distributed
Drone positions fail closed. GNSS antenna positions initialize camera centers, but optical lever
arms remain unmodeled. Explicit `drone_rtk_multiview` payloads and JSON paths remain backend
compatibility inputs, not the default operator workflow. The separate `Drone RTK Rotation` tab
uses one target for Yaw or two or more non-degenerate explicit target/ray observations for full
rotation; Measurement supplies its missing Translation from live GNSS.

In Drone Target annotation mode, capturing another frozen burst automatically clears a completed
target pair after it has been accumulated, so the new pair is ready for the next left/right click.
An incomplete or not-yet-recorded mark stays visible and remains bound to its original frame
identity; it is never silently reused as a pixel from the new frame, and `Clear` is required before
discarding that unfinished operator work.

When the session is saved, the static JSON also stores the frozen-frame artifact identities,
complete automatic proposals, accepted points, current annotation payload, Drone RTK capture
history, accumulated multi-view samples, and provenance.
Native pixels remain in the session frame artifacts; the append-only `run_events.jsonl` keeps
the full proposal, approval, annotation-update, and multi-view sample-revision audit events.

Run the Workstation regression suite from `apps/Workstation` with:

```bash
python3 -m pytest -q
```

Run the repository portability gate from the repository root with:

```bash
python3 tools/check_portable_paths.py
```

## Inputs/Outputs and Interfaces

- Dual-Pixhawk Ethernet input is received on `udpin:0.0.0.0:14550` for P1/Left and
  `udpin:0.0.0.0:14551` for P2/Right. Custom CAN magnetometer telemetry uses MAVLink message
  `SASL_CAN_MAG` (ID `180`) and distinguishes CAN1/CAN2 with its `can_port` field. The subscriber
  requests that stream at 20 Hz and, if the initial interval request is lost, retries only
  `SASL_CAN_MAG` after two seconds, up to three total attempts; the first dedicated sample stops
  further retries.
- Each Pixhawk exposes both `GPS_RAW_INT` (GPS1) and `GPS2_RAW` (GPS2). The Pixhawk workspace
  shows all four receiver rows, including MSL/ellipsoid altitude, HDOP/VDOP, H/V accuracy, fix,
  satellites, and age. Its bottom P1/P2 antenna selectors independently choose the receiver used
  by the P1→P2 GNSS baseline, NTRIP GGA, rover readiness/status, Drone reference calculation,
  Measurement calculation, and frozen GNSS contract; the default and persisted mapping is
  `P1 GPS1 → P2 GPS1`. No automatic averaging or best-fix substitution occurs.
- The approved patched v6X custom firmware is
  `firmware/px4_fmu-v6x_sasl_can_mag/px4_fmu-v6x_default.px4`, with SHA-256
  `3ef5736aabdaec9047cf97b0cd5eefa2cfc074672c3ae43776c3f0e12d2349ce`. It retains the custom
  CAN1/CAN2-to-MAVLink path and guards MAVLink signing status lookup against null instance slots.
- Measurement result evidence schema v2 adds `policy_id=field_measurement_v1`,
  `validity=VALID|ESTIMATED|INVALID`, `hard_block_reasons`, and `advisory_reasons`. Paired Parquet
  rows expose the same decision as `measurement_policy_id`, `measurement_validity`,
  `measurement_hard_block_reasons`, and `measurement_advisory_reasons`. The compatibility
  `measurement_valid` field remains true for both computed `VALID` and `ESTIMATED` results.

- The canonical runtime root is `.local/workstation_data/` at the repository root, or the path in
  `SASL_WORKSTATION_DATA_ROOT`. It owns `stereo_distance_settings.json`, `recordings/`,
  `calibration/`, `models/`, and runtime caches. The five persistent asset fields—intrinsics,
  detector weights, calibration root, validation root, and validation JSON—are normalized to
  data-root-relative POSIX paths. Known legacy values remain readable, but the next production Save
  refuses machine-external directories and never writes a checkout/user-home absolute path.
- New sessions live directly under `.local/workstation_data/recordings/<session-id>/`. The
  timestamp-based session ID is allocated only by explicit `Sessions > Current > Create Session Folder`; an
  app-run ID is recorded separately so one session can be recovered across application restarts.
  `measurement_session_snapshot` supplies the visible folder basename, absolute `session_dir`,
  local open date/time metadata, lifecycle state, and exact recovery error. Existing
  `recordings/sessions/<session-id>/` trees remain read/recovery compatible but are never used for
  new writes; a duplicate ID across both layouts fails closed. Opening this parent has no readiness
  input interface and consumes no device, synchronization, calibration, method, or Alignment state.
- Pre-session Calibration uses the configured reusable calibration root, normally
  `.local/workstation_data/calibration/`. Session-shared Calibration runs live under
  `<session-id>/calibration/<run-time>/`, including synchronized left/right photos, computed
  intrinsic JSON files, and immutable copies of any operator-loaded intrinsic JSON. Approved
  intrinsics and extrinsics are additionally published under
  `profiles/intrinsics/<kind>-<sha256>.json` and
  `profiles/extrinsics/<kind>-<sha256>.json`; mutable compatibility aliases are never used as the
  child evidence identity. Calibration run
  caches are keyed by parent Session ID, so a second Session in the same application process cannot
  reuse the first Session's writable path. The exact active pre-session profile file is copied into
  a child package's `inputs/calibration/active/` evidence when that Measurement starts.
  Alignment captures/profiles and their native-frame/run-event evidence remain outside
  `measurements/` at the same Session root; applying or reverting Alignment atomically updates
  `alignment/active_alignment.json` as a pointer/projection. Every applied or reverted selection is
  retained under `alignment/selections/alignment-<sha256>.json`, and the active immutable selection
  is copied into each consuming child under `inputs/alignment/active/`.
- Each measurement lives under
  `<session-id>/measurements/mNNNNNN/`; its spool, video, captures, recording generation,
  results, and revisions never share a writable path with a sibling measurement.
  Measurement-start method inputs are copied independently to
  `inputs/methods/<method_id>/frozen_input.json` for all eight canonical IDs.
  Artifact registration resolves real paths against that child root and rejects absolute escapes,
  `..` traversal, and symlink resolution into a sibling or parent-owned path.
- Each child `revisions/rNNNN/package_manifest.json` uses schema
  `workstation-measurement-package-v1`. It inventories measurement-relative artifact paths, roles,
  producer/source identity, required/present/missing/failed state, byte size, SHA-256, media/schema
  information, and evidence completeness. `package_manifest.sha256` seals the canonical manifest;
  `latest_revision.json` points to the current revision and each later revision hashes its
  predecessor. Parent `measurement_session_manifest.json` uses schema
  `workstation-measurement-session-v1` and hashes every latest child manifest plus session-shared
  evidence. Schema version 2 additionally indexes each child's order/current recording generation,
  authoritative JSONL/context fragment hashes, and native binary artifact inventory without
  concatenating a second parent-level authoritative stream; version 1 remains readable.
- Each finalized measurement publishes one `data` link backed by
  `.workstation/publish/current`. Canonical `data/raw/*.jsonl` and
  `data/context/{geometry_contexts,time_sync_epochs}.jsonl` are the authoritative structured
  records. `measurement_pairs.parquet`, `camera_timeline.parquet`,
  `pixhawk_timeline.parquet`, `gnss_base_timeline.parquet`, and
  `geometry_comparison.parquet` are verified derived analysis views. `context.json` and
  `measurement.json` record the atomic generation, authority/derived-from contract, schemas, row
  counts, sizes, and hashes. Default CSV, per-Pixhawk XLSX, and per-measurement XLSX files are not
  emitted.
- `data/raw/method_evaluation_bundles.jsonl` is the atomic recovery/commit authority for one
  snapshot's eight terminal outcomes. Finalization validates each bundle hash and materializes one
  independently verified stream at `data/raw/method_results/<method_id>.jsonl` for every canonical
  method, including empty streams. `data/raw/stereo_measurements.jsonl` remains the representative
  Pixhawk attitude + GNSS/RTK projection and is never replaced by a comparison result.
- `recordings/<session-id>/review.xlsx` is the one operator-facing integrated workbook. It combines
  Session, Measurements, Results, Quality, Geometry, Devices, Captures, Warnings, and Files views
  across all finalized child measurements. It also creates one stable `M_*` worksheet for each of
  the eight canonical Measurement methods. Each method sheet reads that method's outcome JSONL and
  combines its terminal status/result summary with relevant Geometry and pair-quality fields;
  methods with no evaluated pair are header-only. A legacy blocked result whose
  `measurement_method_id` is empty remains in `Results` and is
  not inherited into any `M_*` sheet from the Measurement summary. It is derived and regenerable;
  workbook failure never invalidates verified authoritative JSONL/native evidence.
- Each evaluated measurement pair—valid, rejected, unresolved, or blocked—retains its primary
  canonical input/result/readiness JSON in `data/raw/stereo_measurements.jsonl` plus all eight
  self-contained method attempts/results in the method bundle/result streams. `terminal_record_id`
  is stable across finalization retry,
  and `geometry_context_id` plus side-specific time-sync epoch IDs resolve into the child context
  registries. Derived Parquet stable typed columns support analysis, while `row_json` preserves
  every dynamic source field.
  Its exact left/right decoded source pixels are stored as lossless, no-overlay native PNGs.
  PNG encoding, fsync, and delayed decode/dimension verification run on a bounded daemon writer;
  the GUI records a deterministic pending path and finalization waits until every queued request
  has published a complete, missing, or failed terminal result.
- `measurement-method-snapshot-v1` supplies the Measurement operator view without creating a second
  geometry owner. Runtime `method_capabilities` remains instance-driven, while the additive
  `method_catalog` always contains all eight stable method IDs. Every available capability carries
  readiness/reasons, component-source mapping, required inputs, and a read-only `input_values`
  projection. Missing candidates/profiles become blocked catalog rows and their values remain
  `null`; the projection
  never fabricates a candidate pose or merges PX4 FRD/NED, GNSS NED, candidate virtual-FRD/NED,
  and OpenCV RDF values.
- Continuous `video/raw_left.mp4` and `video/raw_right.mp4` are hashed only after their size and
  modification time stabilize. Bounded `ffprobe` metadata records codec, frame count, dimensions,
  FPS, duration, recording backend, and encoding provenance. Probe and optional full-stream decode
  run on a dedicated verification thread while the UI/status timers continue; sealing waits for
  the worker's terminal descriptors.
- The package copies active calibration/Alignment profile inputs, the selected calibration source
  images and metadata reachable from those profiles, redacted effective settings, Git/software provenance,
  device evidence, application logs, ULogs, target/drone telemetry, and exact RTCM bytes when the
  corresponding source is active. ULogs and Target Drone files already sealed inside the
  measurement are registered in place by path/size/SHA rather than copied again under `assets/`.
- YOLO detector weights remain runtime-owned under the Workstation data root. Session and
  Measurement packages do not copy the model into `inputs/models/`, do not register an
  `active_yolo_model` artifact, and do not make model-file presence part of evidence completeness.
  The selected portable model path can still appear in the redacted effective-settings snapshot.
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
  membership, and subset estimates. Candidate and active profile paths are independent. The active
  path identifies the bytes matching the active matrices and is retained for Measurement/package
  provenance; Calculate/Load/Save updates only the candidate path until Apply succeeds.
- Stereo-extrinsics profiles contain `R_left_to_right`, `T_left_to_right_m`, their source
  zoom, source-intrinsics provenance, pair count, and reprojection error.
- The transform contract is
  `X_right = R_left_to_right X_left + T_left_to_right_m` in OpenCV optical RDF frames.
- Calibration images are synchronized left/right checkerboard pairs under the active calibration
  root. Before `Create Session Folder`, the configured reusable root is used directly. While a Session is
  open, manual/FTP runs use `<session-id>/calibration/<timestamp>`, while Auto capture uses that
  run's `calib_capture/AutoCapture` child. JSON profile paths are selected independently in the
  Calibration UI. `calibration_pair_selection.json` v2 stores only Photo-set-relative paths;
  unresolved legacy paths block calculation and are never replaced by an all-photo fallback.
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
  The left/right `CPTR` dispatches share a capture ID and barrier; their host monotonic delta is the
  explicit FTP pairing basis. It is classified as `HOST_DISPATCH_PROXY`: it may permit pair
  persistence but always has `precision_qualified=false` and
  `actual_exposure_verified=false`. It is not an exposure timestamp. RTCP-bound live-frame evidence
  is `CAMERA_TIMESTAMP_VERIFIED`; only explicitly verified per-camera exposure fields can become
  `EXPOSURE_TIMESTAMP_VERIFIED`. If FTP is disabled or fails, the
  still remains camera-local until an exact HR file is imported.
- Each button press appends one JSON object to the current session's
  `hr_captures.jsonl`; downloaded/imported image bytes remain separate session artifacts.
- `Capture OZ Still` photos are created with `SIVI TRUE` + `CPTR`; `ICPT` is polled separately from
  HR and RTSP recording. With FTP enabled, fresh left/right originals are downloaded into the
  session and retain the same capture-ID/dispatch pairing evidence; otherwise they remain
  camera-local.
- Each OZ button press appends one JSON object to the current session's `oz_captures.jsonl`;
  `session_static.json` exposes the aggregate as `metadata.oz_photo_capture`, and
  `run_events.jsonl` records an `oz_capture_event`. Event files contain evidence values while
  image bytes remain independently hashed artifacts.
- NTRIP inputs are a provider-independent caster hostname (without `http://` or `https://`),
  TCP port, mountpoint, optional TLS flag, GGA policy, direct Username, masked session Password,
  and optional legacy environment-variable names. An untouched launch supplies direct defaults
  `SASL`/`ngii`; operator edits replace them. The active Password is not saved in the settings
  file or evidence, although the deployment default is plaintext in the source tree.
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
  `run_events.jsonl`, `native_frames.jsonl`, and
  `frames/optical-pose-burst`; it never embeds QImage/UI objects.
- Measurement-method snapshots use stable IDs (`pixhawk_rtk_live`, `checkerboard_stereo`,
  `horizon_hybrid`, `mountain_gnss`, `drone_rtk_rotation_hybrid`,
  `drone_rtk_multiview`, `terrain_pnp`, and `combined_hybrid`). Legacy preferred/fallback settings
  remain readable for old packages, but new all-method execution does not use them to replace the
  fixed Pixhawk/RTK representative result.
- Every child package metadata record freezes `measurement_method_fill_policy` and
  `measurement_frozen_comparison_inputs` under schema `measurement-method-config-v1`. The fill
  policy is keyed by method and then by
  `<left|right>.<roll_deg|pitch_deg|yaw_deg|north_m|east_m|down_m>`, with values `live_pixhawk`,
  `live_gnss`, or `none`. Restart recovery reloads this child-owned policy rather than current
  global Settings.
- Measurement rows record `measurement_method_id`, method instance ID, geometry SHA-256,
  `measurement_geometry_kind`, selection reason, and the left/right per-DOF component-source
  mapping in addition to the legacy triangulation source fields. `geometry_kind` distinguishes
  the math kernel (`relative_pose`/`live_pixhawk_mount`) from the independently evaluated method.
- Visible settings ownership is split by lifecycle: Calibration owns the applied Intrinsics,
  `Settings > YOLO / RTSP` owns the canonical `yolo_input_size`, and Measurement Settings owns
  confidence, IoU, and pair-gate values. Legacy backend aliases remain readable for compatibility
  but are not duplicated as visible controls.
- `stereo_distance_settings.json` owns `wiris_visible_encoder_bitrate`,
  `wiris_visible_encoder_qp`, `wiris_visible_encoder_gop`, and
  `wiris_visible_encoder_idr`. These map to WIRIS `SEBR/SEQZ/SEGS/SEIR VISIBLE`; the independent
  readbacks use `GEBR/GEQZ/GEGS/GEIR VISIBLE`. The operational profile keeps bitrate at `0` so
  the configured QP remains effective.

## Operational Rules

- Never issue `mavlink stop` as part of USB Ethernet recovery or persistent startup. On affected
  PX4 v1.17 custom firmware, stopping and restarting a live instance can leave a sparse instance
  slot that the signing callback dereferences and hard-fault task `mavlink_rcv_if2`. Do not issue a
  second `mavlink start` either: `MAV_2_CONFIG=1000` is the sole owner of the Ethernet instance,
  while `extras.txt` contains only `ifup eth0` and the board-specific `ifconfig`. Persist all
  `MAV_2_*` values, reboot, and verify one broadcast stream per board. Simultaneous matching
  unicast and broadcast packets immediately after boot, before any host MAVLink packet is sent,
  indicate competing instances. Keep QGC closed while Workstation owns UDP ports `14550` and
  `14551`.
- While Workstation performs the automatic PX4 calibration-parameter readback after a side's first
  heartbeat, that side's normal ATTITUDE/GNSS receive loop is intentionally reserved. Its status
  shows `PARAM SYNC` with detail `PX4 파라미터 조회 중 (정상)` instead of falsely reporting
  `STALE` or `DISCONNECTED`. Nonmatching ATTITUDE/GPS messages consumed by the blocking PARAM or
  shell receive path are deferred and replayed through the normal handler after the reservation;
  they are not discarded. P1/P2 remain independent, and identical receive errors are reported at
  most once per second per side.
- Pixhawk connect and disconnect operations are mutually exclusive. Their P1/P2 buttons are
  disabled while either worker owns the subscriber; rapid repeated clicks cannot start an
  overlapping socket transition or publish a connected label for a closed transport.
- Base GNSS serial uses a one-second write timeout and a 64-item latest-preserving output queue.
  A blocked USB/radio writer emits `failed`, closes the port, and lets lifecycle shutdown continue
  instead of keeping the Base status falsely connected. PC-IP elevation through `pkexec` has a
  15-second authorization timeout and returns an actionable `sudo ip addr add ...` fallback.
- A dual-camera WIRIS command treats a broken synchronization barrier as partial failure: the
  healthy camera continues without waiting for or inheriting the peer's error. Interactive zoom
  contention is reported as control-channel busy and retried up to five times; it is not labeled
  as a camera failure. TCP command responses are accumulated across fragments until terminator,
  EOF, bounded idle, or the request deadline.
- Treat GPS1/GPS2 as independent receiver solutions. `RTK_FLOAT` alone does not prove vertical
  agreement. Before a measurement, inspect all four rows and choose the physically corresponding
  P1/P2 antennas at the bottom of the Pixhawk workspace. The baseline uses `GPS_RAW_INT.alt`
  (MSL metres), not `GPS_RTK` internal moving-base vector fields; `GPS_RAW_INT.eph/epv` are HDOP/
  VDOP factors, while `h_acc/v_acc` are the metre accuracy fields displayed in the receiver table.
  A selected GPS2 source with no current sample remains unavailable and never falls back to GPS1;
  changing source also resets its cached Measurement fix and RTK-Fixed duration.
- WIRIS encoder-profile application is fail-closed. An SDK error, malformed readback, or value
  mismatch marks that camera `encoder-profile-failed` and prevents the activation workflow from
  reporting the stream check as successful. Changing encoder settings may pause RTSP temporarily.
- `Capture Live Pair` selects the newest decoded native left/right frames from the live RTSP
  pipelines. It preserves RTP/RTCP timing metadata when available but does not capture raw RTP
  packets. Camera-native `CPTR` stills remain separate `Capture HR Still`/`Capture OZ Still`
  actions, with `(FTP)` appended only when the still is downloaded immediately.
- FTP calibration pairing never borrows the current RTSP preview clock. Both cameras must report
  the same capture ID, the synchronized-CPTR-dispatch contract, and per-camera dispatch timestamps;
  the absolute dispatch delta must satisfy the configured pair gate. These host timestamps prove
  trigger proximity only. Metadata continues to mark the camera exposure time as unknown.
- `Sessions > Current > Create Session Folder` has no device, synchronization, Calibration,
  Methods, or Alignment readiness gate. It only allocates the parent and is constrained by lifecycle
  conflicts such as an already-open parent, active measurement/recording work, or an unfinished
  save retry. A successful click must show the exact folder basename, local creation time, and
  absolute path. A backend `False` return or exception remains visible as a Session-folder error
  across status polling instead of appearing as a no-op. Calibration capture/load/save is available
  before opening and uses the reusable calibration root; Alignment capture remains Session-owned.
  `Measurement > 측정 시작` atomically creates the next child and starts Detection. It is constrained
  only by parent/child lifecycle and producer ownership; missing Alignment, exact K,D, GNSS,
  synchronization, or a ready method does not block child creation. Those conditions are displayed
  and enforced when a pair is promoted to a valid MeasurementAttempt, while blocked raw evidence
  remains in that child. Detection can run once per Measurement; `측정 종료` automatically saves,
  verifies, and seals that child. `Retry Measurement Save` is the explicit recovery path. A `Pre`
  HR/OZ capture before start may lazily create the same draft child; start reuses it instead of
  allocating another ID.
- Persisted lifecycle, derived readiness, and evidence completeness are separate. Session
  `Ready for saving` means only that its folder is open. In Measurement, `Evidence collection`
  checks the open parent/current child, camera pair, and synchronization, while `Valid measurement`
  additionally requires at least one complete geometry method. A blocked validity projection does
  not suppress raw/Pre evidence collection.
- The Measurement view displays all eight frozen/live source values independently. Missing method
  instances remain visible as blocked catalog rows, while each pair persists one atomic bundle with
  method ID, instance ID, component sources, geometry hash, status, warnings, and worker timing for
  every method. `review.xlsx` method sheets read those streams directly; a method-less legacy
  blocked pair remains only in the common Results view.
- A parent remains `OPEN` after a child is sealed. Another child may start only after the current
  child is sealed. `Close Session` automatically persists and seals a lifecycle-only empty child
  or a completed Post-Capture child before closing the parent. An active Detection interval must be
  stopped first, and any save, drain, or integrity failure leaves the parent open rather than
  publishing a false-success Session manifest.
- Evidence completeness and measurement validity are independent. A session with no valid range
  can be `SEALED_COMPLETE` when every expected artifact explaining that outcome exists; a valid
  range can be `SEALED_INCOMPLETE` when a required original is missing.
- Required evidence is source-driven. A source activated at any point in the session remains an
  expectation even if it produces zero rows. A source never activated is `not_applicable`.
- Calibration quality and evidence preservation are separate. `WARNING` and `FAIL` calculation
  attempts are saved under `attempts/` without changing active aliases or settings. `WARNING`
  requires confirmation before Apply and `FAIL` cannot be applied. Any applied PASS/confirmed
  WARNING profile is first published to an immutable content-addressed path.
- Candidate and active calibration identities are separate state. Calculate, Load, or Save may
  update candidate paths and immutable candidate bytes, but only Apply may atomically update
  `active_intrinsics`, the active K,D path/hash/zoom, and active R,t identity. A new Measurement
  snapshots exact operating K,D plus the latest successful Draft for each comparison method without
  altering global Active. Missing Draft or creation-time K,D mismatch becomes that method's BLOCKED
  evidence rather than a reason to substitute another method.
- Session is the field-work grouping boundary; Measurement is the atomic writable, retry, and seal
  boundary; Revision is append-only late evidence for the same Measurement and never grants a
  second Detection interval.
- Requesting HR/OZ makes both exact camera originals required. A timeout, one-sided download,
  invalid image, size/hash mismatch, or missing file is preserved and seals the package
  `SEALED_INCOMPLETE`; it never fabricates a successful pair.
- Automatic Measurement finalization does not discard a failed save. Its durable order is session-save completion,
  measurement-scoped RTCM close/drain acknowledgement, RTCM binary/index semantic verification,
  artifact inventory/readback, then manifest seal. Save, drain, manifest, write, or probe errors
  leave the package and spool `RECOVERABLE` and create no false-success manifest. A torn final spool
  record is quarantined while its valid prefix is retained; malformed interior rows remain a hard
  error. Retrying resumes the unfinished boundary instead of regenerating already manifested parent
  review bytes.
- Live recording spools and heterogeneous event ledgers remain JSONL because they must append and
  recover a valid prefix after interruption. Successful finalization first writes and rereads
  canonical JSONL, then creates typed Parquet as a derived view, validates the published generation,
  and only then deletes the temporary spool. CSV is available only as an explicit future/export
  boundary, not a default source of truth.
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
  a torn final JSONL write is quarantined while a malformed middle line is a hard failure, and
  preview opens only after the artifact's recorded byte size and SHA-256 have been reverified.
- Spool and run-evidence `OSError` failures stop recording, retain the working spool, and expose a
  retryable/`RECOVERABLE` session error without escaping a GUI timer callback. Run-event JSONL
  appends flush to the kernel immediately but batch `fsync` at session/manifest/shutdown barriers;
  native PNG and video verification each have their own asynchronous durability barrier.
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
- NTRIP, RTCM pipeline, and Base GNSS worker results that update status labels, logs, tables, or
  evidence projections are always queued onto the Qt GUI thread. Direct cross-thread connections
  are limited to documented lock/queue adapters that never access widgets.
- Shutdown stops NTRIP and Base producers first, drains RTCM ingress and evidence, waits for every
  Pixhawk injection outcome (or records the exact unconfirmed sequences after the bounded timeout),
  and only then disconnects MAVLink/runtime workers and finalizes the session. Window close remains
  non-blocking and retryable during the cooperative phase. A five-second process-wide close
  deadline then flushes evidence, terminates any remaining camera child, requests interruption on
  outstanding `QThread`s, and bounds the final forced wait to one additional second.
- Preview replacement applies only to resized UI frames. Native measurement/calibration frames,
  detection events, recording streams, command results, and RTCM evidence do not use the preview
  inbox and cannot be inferred from a skipped display frame.
- Raw camera frames update the latest calibration snapshot without broadcasting a duplicate
  calibration-preview signal. Hidden legacy/advanced preview widgets do not render those frames;
  checkerboard overlays remain available through the throttled overlay path or explicit Refresh.
- The normal GStreamer H.264 path repeats SPS/PPS at every IDR (`h264parse config-interval=-1`)
  and keeps compressed H.264 queues non-leaky and bounded to 250 ms so access units/IDRs are not
  discarded before decode. The post-decoder queue is `leaky=downstream` with eight buffers, so a
  stalled reader drops stale decoded frames without back-pressuring depay/decoder. The recording
  tee branch is independently `leaky=downstream` with a five-second time bound, preventing disk
  stalls from growing memory without limit. `decoded_hz` uses a five-second rolling window and
  returns to zero after a genuine gap.
- Linux camera child processes arm a parent-death signal. A Workstation crash or Force Quit
  therefore terminates its decode children instead of leaving CPU-consuming orphan workers for
  the next launch; normal shutdown still uses the cooperative stop/drain path. Unexpected child
  death is supervised separately from RTSP reconnect and receives bounded automatic restart.
- Keep `wiris_visible_encoder_bitrate=0` for both WIRIS cameras. Confirm camera readback as
  `GEBR VISIBLE=0` and `GEQZ VISIBLE=17` after RTSP enable/reactivation; a nonzero bitrate makes
  QP ineffective and is not the approved Workstation Live-preview profile.
- Changing `YOLO input size` does not resize an already-running detector. Stop Detection, Apply or
  Save the Settings value, and start Detection again; the next worker uses the updated size.
- Changing `Weights path` also does not hot-swap an already-running detector. Use `파일 선택…`
  before Detection, confirm the verified relative path, then Apply or Save. An external model must
  complete the content-addressed import before it can become the persisted model dependency.

- Intrinsics are solved independently from the selected left and right image sets. Stereo `R,t`
  uses only timestamp pairs whose left and right observations both survived detection and
  near-duplicate-pose filtering; excluding one side does not silently exclude the other side's
  intrinsic observation. The configured calculation-pair limit is applied once, after this
  usable stereo intersection is formed.
- The visible Photo set is the calculation root. Setup sends the same value to the owner validation
  root before Run. Sidecar v2 paths are root-relative; a legacy path is accepted only after bounded
  rebase beneath the current root. Any unresolved selected or excluded path blocks calculation and
  is shown to the operator—never silently replaced by the complete directory.
- The default calibration-quality policy requires at least 12 distinct poses, at least 60% union
  extent on each image axis, checkerboard centers in at least five cells of a 3×3 grid, horizontal
  and vertical perspective variation, at least 20% scale variation, stable deterministic subset
  solves, OpenCV focal standard deviation no greater than 3%, principal-point standard deviation
  no greater than 2% of each raster axis, and no per-view reprojection error above 1 px. A
  centered raster warns when `cx` or `cy` is more than 5% of that axis away from
  center and fails above 10%; an out-of-frame principal point always fails.
- A quality `FAIL` is saved only as non-applicable attempt evidence and blocks Apply. A `WARNING`
  attempt is also saved without promotion and requires explicit operator confirmation before
  Apply; the override is recorded in validation metadata. Applied profiles are published to an
  immutable content-addressed path. Profiles without quality evidence remain load-compatible but
  are displayed as `UNVERIFIED`, never as `PASS`.
- Every intrinsics JSON load independently validates finite positive focal lengths, finite
  distortion values, positive raster dimensions, and in-frame `cx,cy`; a serialized `PASS` cannot
  override a malformed K/D raster contract.
- Nominal intrinsics fallback is an automatic low-confidence runtime path, not a calibration result.
  It requires fresh equal zoom readbacks and known native rasters, labels every consuming
  Measurement `NOMINAL_UNVERIFIED`, assumes zero distortion, and never reports precision
  qualification or camera-calibration PASS.
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
  remains `unknown`. An FTP pair with the same capture ID and command round, successful storage,
  download, and decode on both sides may use the operator-approved
  `command_round_assumed_sync` basis for R,t; it records `pair_delta_s=null` and an explicit warning
  that simultaneous exposure was assumed, not measured. Other FTP/import evidence remains
  `SYNC_UNVERIFIED` rather than borrowing a live-preview timestamp.
- Calibration zoom A and operating zoom B are arbitrary supported zoom levels. The system
  does not prescribe 1×, 8×, or any particular A→B combination.
- Cross-zoom application uses `K,D` from operating zoom B and `R,t` estimated at calibration
  zoom A.
- Every measurement method requires the exact operating-zoom left/right `K,D` bundle and matching
  hashes. Cross-zoom `R,t` remains allowed, but it cannot waive that intrinsics gate. Methods that
  consume GNSS use the latest valid sample independently for P1 and P2. A new side replaces only
  that side when it has finite position and `fix_type >= 3`; an invalid/sub-3D update leaves the
  previous valid value intact. Values may be reused across Measurements in the same app run and
  reset on app restart. Age, side timestamps, pair delta, fix type, and reuse flags remain evidence;
  `RTK_FIXED` (`fix_type=6`) is a warning/quality fact, not a hard measurement gate. The method is
  BLOCKED only while either side has never supplied a valid 3D position or the resulting baseline
  is structurally unusable.
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
- An untouched NTRIP UI uses Username `SASL` and masked Password `ngii`. The runtime owner is
  initialized before Settings is opened. A Password changed or cleared in `Settings > GNSS`
  takes effect only after `Apply` or `Save`; clearing it enables the keyring/legacy-environment
  fallback instead of silently restoring the deployment default in that running session.
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
- Method readiness is independent. A frozen static Checkerboard or full Optical-Pose Draft is not
  blocked because live GNSS/Pixhawk inputs needed by another method are missing. GNSS/Pixhawk gates
  are evaluated only for methods that consume those live components; frozen Draft identity and K,D
  identity remain separate mandatory gates.
- The application supplies safe DOF-fill defaults; the operator does not have to define a default
  priority. Missing rotation axes default to pair-frozen Pixhawk attitude and missing translation
  axes default to the latest valid per-side GNSS/RTK baseline. Turning a toggle off uses
  `none`, makes that hybrid method unready, and never causes a different unrecorded source to be
  substituted.
- DOF fill policy is editable only while the child is `COLLECTING` and before its one Detection
  interval starts. Each change is written to child metadata and `measurement_method_fill_changed`
  run evidence. Detection freezes the policy; later changes require the next Measurement.
- A newer Draft or profile never changes an existing Measurement. It becomes eligible only when the
  next Measurement freezes its inputs; every already-enqueued pair keeps the same snapshot hash.
- Pair-dependent attitude/GPS inputs are selected before hybrid R,t is resolved. The same frozen
  snapshots are stored in MeasurementAttempt and included with final R,t in
  `measurement_geometry_sha256`; a UI-latest sample can never be substituted after the pair is
  frozen.
- If no method is ready, the matched detections and native frame references are still retained and
  all eight method streams receive explicit BLOCKED outcomes. No fake R,t is created and the primary
  result remains blocked instead of falling back to a comparison method.
- Candidate review keeps left/right observability separate. The common Apply/Combine axis is
  enabled only when both camera sides observed that axis; one-sided evidence remains visible in
  the detail text but cannot be promoted as a two-camera contribution.

## Known Limitations and Workarounds

The seven comparison processes deliberately run concurrently while Pixhawk/RTK has its own worker.
On a CPU- or storage-constrained workstation, simultaneous OpenCV startup, native-PNG writes, and
Terrain/Multi-view solves can increase evaluation duration, trigger method-local timeouts, or fill
the 20-pair queue even though queue wait itself is not counted against timeout. Diagnose this from
`queue_wait_ms`, `evaluation_duration_ms`, per-method TIMEOUT counts, process restart events, and
`BACKPRESSURE_BLOCKED` outcomes. First reduce Detection/pair production or remove unrelated CPU/I/O
load; do not raise BLAS/OpenCV child thread counts. If repeatable contention remains on qualified
hardware, treat a bounded-concurrency or sequential diagnostic mode as a reviewed architecture
change and rerun lifecycle, timeout, and throughput tests before deployment. Never hide the symptom
by silently dropping a method or increasing the queue beyond 20.

The corrected 2026-08-24 bench check used Ethernet subscriber endpoints while USB supplied board
power. After removing the manual start from both `extras.txt` files and rebooting, P1/P2 each sent
one broadcast stream. A 30-second real Workstation subscriber run received both heartbeats, both
CAN1/CAN2 `SASL_CAN_MAG` streams, P1 ATTITUDE/GPS counts `1761/294`, and P2 counts `1738/290`, with
no connection/status errors. This supersedes the earlier provisional 45-second warm-state result,
but it is not a USB-removed or long-duration field soak. P1 runs the patched custom v6X artifact;
P2 still runs the earlier custom firmware and may retain the latent null-slot defect, so P2 should
receive the same patched artifact during a maintenance window.
The current captured receiver state also showed P1 GPS1/P2 GPS1 MSL altitudes near `108.30/100.29 m`
(`~8.01 m` difference) while P2 GPS1/GPS2 differed by `~8.83 m`. This is receiver-solution or
physical antenna/configuration evidence, not a Workstation datum conversion; verify antenna mapping,
correction delivery, and RTK Fixed convergence before accepting the baseline.

The software integrity gates and simulated integration suite do not replace field qualification.
The WIRIS cameras/FTP control path, Base and rover F9P receivers, NGII live caster, three Pixhawk
links, MAVLink receiver-side acceptance, long-duration dual-video shutdown, and automatic ULog
collection still require bench and field soak verification with the intended hardware. A local
MAVLink send result is transport evidence, not proof that the remote receiver accepted and used the
correction.

Camera JSONL rows describe GUI-delivered frame observations, while MP4 may be encoded pass-through
or decoded-frame recording. No exact 1:1 frame index is currently proven;
`mp4_frame_index_contract=not_available` is the authoritative limitation. Use the exact native PNG
bound to a terminal pair when pixel-level replay is required.

Nominal intrinsics fallback uses an ideal centered pinhole and the WIRIS 1280x720 zoom-index focal
table scaled independently to the current native width and height. It does not model per-camera
principal-point offset, lens distortion, crop, temperature, focus, or unit-to-unit variation. Use
it to keep estimated geometry workflows available, not as precision or calibration evidence.

The operational bitrate `0`, QP `17`, GOP `30`, IDR `1` profile uses constant-quality mode to avoid
the visible macroblocking observed with bitrate `1`. Constant-quality mode can increase bandwidth,
especially during fast full-frame motion, so the current dual-camera 720p stream still requires
field qualification for sustained FPS and network stability.

The five-second decoded-rate window stabilizes the operator value; it is not a substitute for a
throughput test. If the value remains low after warm-up, inspect the status/log for the active
GStreamer decoder and test the selected RTSP transport independently. UDP can lose RTP packets on
the camera link, while TCP can expose camera-side RTSP service resets or startup codec-header gaps;
changing transport alone is not an approved performance fix. Restart the Workstation after changing
the decoder, transport, or pipeline code so existing child processes do not retain the prior setup.

The Session folder panel is a software projection of `measurement_session_snapshot`. Its displayed
basename, creation time, and path identify the selected storage owner, but do not prove that every
later producer successfully wrote an artifact. Confirm producer-specific results in Measurement and
`Sessions > Evidence / Review`, and retain the sealed manifests. `Create Session Folder` intentionally
performs no readiness verification, so `Ready for saving`/`OPEN` must never be treated as proof that
remote devices are the intended physical endpoints or that a valid Measurement can begin.
`Close Session` is a one-action workflow only after Detection has stopped. It may remain in
`VERIFYING` while writers, RTCM evidence, persistence, and hashes drain; a failure intentionally
leaves the child `RECOVERABLE` and the parent open for `Retry Measurement Save`.

Content-addressed `profiles/` and `alignment/selections/` files are the durable identities used by
new Measurements. The older `intrinsics.json`, `intrinsics_zoom_XX.json`, extrinsics aliases, and
`alignment/active_alignment.json` remain compatibility/current-selection projections and may be
updated. External tools must use the immutable path/hash recorded in the child manifest rather than
assuming an alias still represents an earlier Measurement.

The unified Sessions workspace is implemented as Current, Evidence, and Review tabs. Alignment,
Measurement, global docks, Overview, and Settings remain separate owner-backed surfaces.

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
Pixhawk, or GNSS sample. Use typed Parquet for high-rate analysis, canonical `row_json` replay, and
`data/raw/method_results/*.jsonl` for complete per-method outcomes. Parquet readers must preserve
signed `int64` nanosecond fields; the workbook
renders nanosecond timestamps and hashes as text to avoid Excel's numeric precision limit. If
workbook generation fails, reopen/regenerate it from the verified Parquet generation.

The eight `M_*` worksheets are filtered derived views, not evidence authority. Their rows come from
the separate `data/raw/method_results/<method_id>.jsonl` streams and include BLOCKED/FAILED/TIMEOUT
as well as valid results. They do not replace the atomic bundle, `stereo_measurements.jsonl`,
`geometry_contexts.jsonl`, frozen input files, Alignment selections, or `run_events.jsonl`.
Method-less legacy blocked rows appear only in the common Results view.

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
New captures add synchronized host-side `CPTR` dispatch evidence for pairing, but this still does
not prove simultaneous camera exposure and requires qualification on the intended WIRIS hardware.
When both downloaded/decoded sides share one capture ID and command round, Measurement may use the
operator-approved simultaneous-exposure assumption for R,t; the stored warning and null measured
delta must remain visible in analysis.

Solved optical-pose Drafts remain session-memory objects unless their extrinsics profile is
explicitly saved. A Measurement opened before restart can freeze the latest successful Draft even
when it is not global Active; after restart, an unsaved Horizon/Mountain/Drone/Terrain Draft must be
solved again. Hybrid methods remain BLOCKED until their required live Pixhawk attitude exists and
both GNSS sides have supplied at least one valid 3D position; old valid GNSS may be reused and is
labelled `REUSED`.

Per-DOF fill currently permits only frame-compatible live sources: Pixhawk for virtual-FRD
rotation axes and GNSS/RTK for NED translation. Checkerboard and saved relative R,t profiles remain
atomic geometry and cannot be used as arbitrary absolute NED axis fillers. Full coupled
Mountain/Multi-view/Terrain/Combined candidates still fail closed when any method-defining DOF is
unobservable. Alignment Candidate review has no editable Apply-DOF checkboxes: `Apply candidate`
always promotes the candidate's complete owner-declared `optimized_dof` mask, and rejects the
candidate if any owned axis is not observable on both camera sides. Legacy callers may include a
`dof` payload only when it exactly equals that owner mask; partial overrides fail closed. Use the
explicit Combined solver when axes must come from different candidates; its per-source axis table
remains the sole editable candidate-axis composition surface.

Automatic HR/OZ retrieval requires working FTP credentials and a camera directory that can be
listed. The freshness guard never substitutes an older still when a new post-CPTR file does not
appear. HR supports exact manual file import; OZ currently relies on automatic FTP retrieval.
When either control/FTP endpoint is missing or a capture times out, the event and any successful
side are retained, but the requested capture source keeps the sealed package incomplete.

Package manifests use SHA-256 chaining for integrity and change detection, not operator identity
or legal non-repudiation. There is no electronic signature, organization PKI integration, bundled
source tree, virtual environment, or container image. Reproduction therefore requires a compatible
Workstation checkout identified by the recorded Git/software provenance.

Video package verification waits for stable bytes and runs bounded `ffprobe`; optional full-stream
decode runs off the GUI thread but finalization still waits for it and each side retains a
300-second decode timeout. It is not enabled for every automatic Measurement finalization because
long recordings remain expensive. The queue/supervisor/write-timeout regressions use mock or local
process boundaries; validate sustained RSS, blocked removable storage, USB serial timeout, dual
camera recovery, camera FTP, and automatic ULog collection with actual field hardware before
treating those boundaries as field-qualified.

Automatic restart recovery resumes an open hierarchical session only when exactly one such session
is discoverable. Its interrupted active measurement is marked `RECOVERABLE`; if several open
sessions exist, Workstation refuses to guess and reports that explicit recovery is required.

NTRIP mountpoints are entered manually; the Workstation does not currently browse a
caster's source table. Provider-specific authentication methods beyond standard Basic
authentication are not supported.

The built-in NTRIP Password default is plaintext source configuration and must not be treated as
a protected secret. Anyone who can read the checkout can read it, and the non-empty session
default takes priority over a keyring or legacy environment credential. For a different account,
replace Username/Password in the UI and press `Apply`, or clear Password to select the stored/env
fallback. Rotate the provider credential and change the deployment constant when the shared
default is no longer appropriate.

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
The two camera decoder child processes may still consume substantial CPU while Preview is active;
this is expected producer-side work and is separate from GUI responsiveness. Stop Preview when
video is not required. If a build predating 2026-08-23 was force-quit, terminate its orphaned
`multiprocessing.spawn` camera children once before starting the updated Workstation.

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

- 2026-08-24: Hardened Workstation runtime isolation and shutdown across Qt, Pixhawk/GNSS, WIRIS,
  camera children, Measurement evaluators, and evidence storage. Added a non-aborting Qt exception
  hook with emergency flush, Base serial write timeout/bounded latest queue, lossless deferred
  MAVLink command telemetry, receive-error throttling, mutually exclusive connect/disconnect,
  asynchronous RTCM-worker replacement, partial-success WIRIS barriers, bounded fragmented TCP
  reads, and a five-second shutdown deadline. GStreamer now uses an eight-frame leaky decoded queue
  plus a five-second bounded recording branch, and camera child SIGKILL receives supervised restart.
  Measurement spawn/join/restart, native PNG persistence, and video verification moved off the GUI
  thread; worker geometry is reused without GUI recalculation. Disk errors become recoverable
  recording failures with batched run-event fsync. Malformed detection payloads, zero quaternions,
  and malformed status values fail closed. Full Workstation suite: `1100 passed, 1 skipped`.

- 2026-08-24: Prevented false Pixhawk `STALE`/`DISCONNECTED` UI status during the automatic
  calibration-parameter refresh triggered by the first heartbeat. The status projection now reuses
  the subscriber's exclusive-command state, reports only the reserved P1/P2 side as `PARAM SYNC`,
  and returns to the normal ATTITUDE-age-based `LIVE` state after command completion.

- 2026-08-24: Removed editable Candidate Apply-DOF checkboxes. Candidate Apply now derives its
  immutable mask from `optimized_dof`, requires every owned axis to be observable on both camera
  sides, and applies the complete mask automatically. This prevents full 6-DOF Multi-view `t`
  from being silently dropped by the former rotation-only UI default. Combine retains its explicit
  non-overlapping per-source axis selection.

- 2026-08-24: Propagated the operator-selected P1/P2 GPS1/GPS2 source through NTRIP GGA, GNSS
  readiness/status, Drone reference geometry, Measurement package activation, and frozen GNSS
  contract provenance. Source changes reset cached fix/fixed-duration state. Consolidated
  `latest_rtk_baseline` ownership in `MeasurementViewController`; Alignment now renders the
  canonical estimate without replacing it. Full suite: `1068 passed, 1 skipped`.

- 2026-08-24: Added JSON-free Drone RTK Multi-view collection. Each completed left/right target
  pair is bound to the matching Frozen burst's immutable Drone/P1/P2 GNSS and synchronization
  evidence, persisted with READY/BLOCKED state, and accumulated per Session. The Alignment
  Multi-view action now consumes at least four READY same-K,D samples directly, builds a stable
  first-P1-origin local NED input, and solves full camera `R,t`; explicit payload/JSON input remains
  backend-compatible. Added automatic editor rollover after a completed sample plus controller,
  UI-routing, persistence, failure, and full-R,t regression coverage.

- 2026-08-24: Hardened the live H.264 decode path. GStreamer now repeats SPS/PPS at IDR boundaries,
  avoids one-buffer leaky queues before decode while retaining bounded latency, and reports decoded
  rate through a five-second rolling estimator. Added pipeline-parse, recovery, and rolling-rate
  regression coverage; the full Workstation suite passes `1058 passed, 1 skipped`.

- 2026-08-24: Replaced single-method candidate/fallback execution with one immutable-pair,
  eight-method Measurement barrier. Pixhawk attitude + GNSS/RTK is the fixed representative;
  seven comparison solvers run in isolated persistent processes with method-local timeouts, a
  20-pair bounded queue, native-frame-first byte-free payloads, and crash-local restart. Added
  Measurement-start `inputs/methods/<method_id>` copies, latest-valid independent P1/P2 3D-fix
  reuse, command-round FTP sync assumption evidence, atomic bundle recovery, eight authoritative
  method-result JSONL streams, per-method workbook rows, and read-only Overview state/counters.

- 2026-08-24: Classified calibration timing evidence independently from pair persistence. FTP CPTR
  timing is now `HOST_DISPATCH_PROXY` and can never precision-qualify or verify exposure; camera
  frame timing and explicit exposure timing use separate classes and propagate into input snapshots.

- 2026-08-24: Added immutable calibration input identity. Visible Photo set requests now produce a
  hashed file/pair/selection snapshot that is the worker's sole input authority; workers reverify
  bytes and echo the same identity into candidate provenance, preventing hidden-widget root drift.

- 2026-08-24: Made calibration selection fail closed. A typed sidecar loader now distinguishes
  ABSENT/VALID/INVALID, accepts only supported v1/v2 contracts, and blocks malformed, unknown, or
  semantically invalid metadata before worker creation while retaining portable v2 exclusions.

- 2026-08-24: Replaced truthiness-based Measurement method fallback with an explicit legacy
  migration boundary. Empty/current blocked rows remain method-less from canonical JSONL through
  `MeasurementReview` and XLSX; only genuinely missing v1/v2 keys may migrate known aliases.

- 2026-08-24: Isolated candidate geometry comparison from official Measurement evidence. Matching
  unapplied candidates retain an explicitly analysis-only namespace plus candidate/active K,D
  hashes and quality; stale candidates are not triangulated, and geometry-comparison Parquet v2
  exposes the same non-measurement contract.

- 2026-08-24: Fixed full-checkerboard Apply freshness. The normal Extrinsics action continues to
  compose independently selected K,D and R,t, while Advanced `Apply result` binds to that result's
  own R,t and publishes an immutable path derived from the exact profile being applied, preventing
  a stale selected R,t path/hash from being mixed with fresh K,D.

- 2026-08-24: Made Optical-Pose Apply an atomic active-geometry transition. Apply now validates the
  exact active K,D matrix/path/hash and Session persistence owner, prepares the complete R,t bundle
  before mutation, and restores the prior active identity and Measurement cache on persistence
  failure; nominal fallback K,D cannot be promoted as active calibration by this action.

- 2026-08-24: Separated calibration candidate and active identities. Calculate/Load/Save no longer
  replace active K,D provenance; quality/native-grid Apply is the sole promotion boundary.
  Unapplied Checkerboard/Optical-Pose candidates and stale K,D hashes now block Measurement
  readiness while the currently applied hybrid candidate retains pair-frozen component sources.
  Unified the visible Photo set with the worker calculation root, introduced portable
  `calibration_pair_selection.json` v2 with fail-closed legacy recovery, added synchronized-CPTR
  FTP pairing evidence without claiming an exposure timestamp, and stopped method-less blocked
  results from inheriting the Measurement summary method in `review.xlsx`.

- 2026-08-24: Added deployment NTRIP UI/runtime defaults `SASL`/`ngii`. Direct Start NTRIP now
  receives them even when Settings is untouched; operator edits still override them, while the
  masked Password remains excluded from settings/evidence JSON. Documented that the convenience
  default is plaintext source configuration rather than secure credential storage.

- 2026-08-24: Removed background same-IP retries from the initial manual `Connect cameras` flow.
  Each click now performs one discovery/activation/frame-verification attempt and reports failure;
  GStreamer post-connect RTSP recovery remains unchanged.

- 2026-08-24: Corrected dual-Pixhawk Ethernet persistence to one `MAV_2_CONFIG`-owned instance per
  board. Removed manual `mavlink start/stop` from persistent, recovery, and stream-setup paths;
  `extras.txt` now owns only interface setup, USB recovery saves files/parameters and reboots, and
  MAVLink shell access explicitly releases its exclusive flag. A 30-second real dual subscriber
  check confirmed both heartbeats and CAN1/CAN2 streams; the full suite passed `1054` tests.

- 2026-08-24: Added source-aware dual-GNSS inspection and selection. Pixhawk now displays P1/P2
  GPS1/GPS2 together, persists independent P1/P2 calculation-source choices, freezes the selected
  receiver source into Measurement GPS snapshots, and decodes MAVLink HDOP/VDOP separately from
  true horizontal/vertical accuracy fields.

- 2026-08-24: Patched the PX4 signing-status null-slot hard fault and added bounded
  `SASL_CAN_MAG` request recovery. Rebuilt the v6X custom firmware at SHA-256
  `3ef5736aabdaec9047cf97b0cd5eefa2cfc074672c3ae43776c3f0e12d2349ce`.

- 2026-08-24: Added one stable `review.xlsx` worksheet for each canonical Measurement geometry
  method. Method sheets combine actual terminal pair results, matching pair-quality fields, and
  Geometry summaries; unused methods remain header-only, while canonical JSONL/profile evidence
  remains authoritative.

- 2026-08-23: Renamed Calibration's `FTP 접근` action to `FTP 자동 전송` so the UI states
  that accumulated OZ stills are discovered, paired, and downloaded automatically rather than
  opened for manual remote-file selection.

- 2026-08-23: Removed duplicate raw calibration-preview fan-out and per-frame camera header
  rebuilding, skipped QPixmap conversion/scaling for hidden CameraPane surfaces, and retained
  latest calibration frames through implicit-sharing snapshots. Added Linux parent-death signals
  so camera decode children cannot survive a crashed or force-quit Workstation. Live dual-camera
  verification reduced the main thread from about 69% to 22% CPU and camera QThreads from about
  81% to 6% each while preserving frame, overlay, recording, and native-calibration contracts.

- 2026-08-23: Queued every GNSS/NTRIP/RTCM worker callback that mutates the UI onto the Qt GUI
  thread, preventing `QTextBlock`/`QTextCursor` cross-thread access and native `libQt5Gui` crashes.
  Pixhawk shutdown now remains pending until asynchronous disconnect and RTCM workers finish instead
  of destroying a running `QThread`.

- 2026-08-23: Made canonical JSONL and native bytes authoritative, retained Parquet/XLSX as
  derived views, added stable terminal IDs, static geometry contexts, observed time-sync epochs,
  parent/child generation hash catalogs, JSONL-first readers with legacy Parquet fallback, atomic
  WIRIS FTP byte sealing, and strict torn-tail recovery for direct ledgers. Coordinate transforms,
  measurement methods, RTK/NTRIP behavior, and the lack of exact MP4 frame mapping are unchanged.

- 2026-08-22: Removed the separate `New Measurement` control from the normal Measurement workflow.
  `측정 시작` now creates the next `mNNNNNN` child when necessary and then starts Detection and
  automatic recording; `측정 종료` stops Detection and retains the existing automatic save/seal
  path. A prepared Pre-evidence child is reused, a `Pre` HR/OZ request can lazily create that draft,
  and creation/save conflicts fail closed without starting another producer.

- 2026-08-22: Stopped copying YOLO detector weights into Session or Measurement evidence packages.
  Runtime model selection remains under the Workstation data root and in redacted settings, while
  package completeness no longer expects an `active_yolo_model` artifact.

- 2026-08-22: Removed the unused Run setup workflow end to end. New sessions no longer expose
  Run setup controls or create, require, or register `run_setup.json`; append-only evidence uses
  the session ID directly. Existing packages containing the legacy file remain readable because
  it is ignored. Target Drone/UAM connectivity remains available as `Target reference`.

- 2026-08-22: Added a visible `파일 선택…` action beside
  `Settings > YOLO / RTSP > Weights path`. External YOLO models are copied into
  content-addressed Workstation storage with SHA-256 verification, successful selection fills the
  portable relative path, and cancellation/import failure leaves the previous model unchanged.

- 2026-08-22: Moved Calibration capture actions into one shared bar directly below the Live/FTP
  camera pair. Removed duplicated Intrinsics/Extrinsics capture buttons, added a compact L/R/pair
  status summary, collapsed Auto settings and command logs by default, and hid the capture surface
  on Compute/Profiles without changing capture commands or evidence. Equalized Capture layout
  heights and synchronized workflow selection so kind switching does not shift downstream groups.

- 2026-08-22: Replaced measurement-quality blocking with the single `field_measurement_v1` policy.
  Correspondence identity, coordinate/raster contracts, required K,D/R,t, finite geometry,
  lifecycle, and persistence integrity remain hard. Timing, synchronization, telemetry age,
  GNSS quality/readiness, nominal K,D, and optical Alignment K,D provenance mismatches now produce
  stored `ESTIMATED` results with explicit advisory reasons. Nominal K,D selection is automatic.

- 2026-08-22: Split Calibration into `Intrinsics` and `Extrinsics`, each with
  `Capture / Compute / Profiles`. Kept zoom and photo selection shared, isolated K,D and R,t
  commands/results, and made Extrinsics show and apply the explicit left/right K,D + R,t bundle
  under the OpenCV optical RDF left-to-right contract.

- 2026-08-22: Removed unreachable legacy Session readiness/method/history renderers, removed the
  hidden Start/Close Session controls from Measurement, and reduced `Advanced > Files` to manual
  recording, retry-save, and video playback. Session lifecycle remains in `Sessions > Current` and
  all ordinary saved artifact review remains in `Sessions > Review`.

- 2026-08-22: Unified the former Session and Recording / Review navigation surfaces as
  `Sessions > Current / Evidence / Review`. Moved Run setup and Preflight to Protocol, moved the
  manual-recording fallback to `Advanced > Files`, preserved legacy context aliases, and kept all
  measurement/session schemas and coordinate contracts unchanged.

- 2026-08-22: Added the initial nominal intrinsics fallback. Missing calibrated K,D can be replaced
  automatically by a zoom-index/native-raster ideal pinhole profile with
  centered principal point, zero-distortion assumption, immutable hash/provenance, and
  `NOMINAL_UNVERIFIED`/`precision_qualified=false` evidence. Calibrated profiles remain preferred;
  malformed, stale, unknown, or left/right-mismatched boundaries still fail closed.

- 2026-08-22: Made `Close Session` automatically persist and seal an empty or completed current
  Measurement before sealing the parent Session. Active Detection still requires an explicit stop,
  and save/drain/integrity failures remain fail-closed and retryable instead of publishing a parent
  manifest.

- 2026-08-22: Made Measurement the atomic writable/seal boundary while retaining Session as the
  field-work grouping boundary. Removed the Alignment-before-New-Measurement hard gate, preserved
  blocked evidence when geometry is unavailable, saved quality WARNING/FAIL calibration attempts
  without runtime promotion, published applied Calibration and Alignment selections as immutable
  content-addressed files, copied exact Alignment bytes/hash into consuming children, and retained
  child-root containment plus Parquet/RTCM/manifest integrity barriers.

- 2026-08-22: Reduced the top-level Session workspace to its storage-container responsibility.
  `Create Session Folder` now directly creates `recordings/YYYYMMDD_HHMMSS_nanoseconds/` without
  applying Run Setup or polling device/method readiness, and displays state, exact folder name,
  local creation time, absolute storage path, and persistent action errors. Moved New Measurement
  and retry-save controls to Measurement, removed Session readiness/method/history panels and the
  ambiguous red default border, and added real pointer hit-testing plus filesystem-backed creation
  regression coverage.
- 2026-08-22: Removed the `Open Session` readiness Gate and its compatibility helper. Session open
  now depends only on parent/save lifecycle safety, while device, synchronization, Calibration,
  Methods, and Alignment readiness remain downstream. Enabled pre-session Calibration against the
  reusable calibration root and retained the exact saved/loaded profile path for Measurement
  evidence; an applied checkerboard profile remains eligible after Session Alignment reset.
- 2026-08-22: Documented bitrate `0` as the required WIRIS visible-stream operating profile so
  QP `17` remains active, including the camera readback checks and constant-quality bandwidth risk.
- 2026-08-22: Locked Calibration Live/FTP preview pairs to equal left/right widths so both
  16:9 camera surfaces render at the same displayed size after resize or splitter movement.

- 2026-08-22: Closed Session-workflow ownership gaps: the actual file-picker `Load Intrinsics`
  now preserves an immutable Session copy, Calibration run caches are isolated by parent Session,
  applied/reverted Alignment is atomically published as Session-shared JSON, a new Session clears
  prior Alignment state, and New Measurement controls expose and enforce their readiness
  prerequisites independently from parent Session creation.

- 2026-08-22: Promoted Session above Calibration and made it the timestamped experiment container
  for Session-shared calibration photos/results, loaded intrinsic copies, alignment evidence, and
  repeated Measurement children. Session open allocates that parent without a readiness gate;
  Detection stop automatically saves and seals each Measurement.

- 2026-08-22: Enabled cursor-centered plain-wheel zoom for both Alignment frozen camera previews
  while preserving native annotation coordinates, the 1× recenter behavior, and Ctrl+wheel as the
  default interaction for annotation views outside Alignment.

- 2026-08-22: Added the Telemetry route to the Base GNSS RTCM distribution table and standardized
  its compact route label as `T`, alongside `P1` and `P2` delivery counters.

- 2026-08-22: Persisted the field-tested WIRIS visible encoder profile (`1/17/30/1`) in
  `AppSettings`. RTSP enable/reactivation now reapplies `SEBR/SEQZ/SEGS/SEIR VISIBLE`, independently
  reads back all four values, and fails closed on SDK errors or mismatches.

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
