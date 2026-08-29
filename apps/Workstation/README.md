# SASL StereoWIRIS Workstation

## Purpose

This package provides the operator Workstation for synchronized stereo-camera capture,
calibration, Measurement, Pixhawk/GNSS evidence, and recording. It owns its runtime and UI
independently from the Legacy application.

As of 2026-08-29, Measurement is an observation interval rather than a package-completion
transaction. There is no Create, Open, Close, Finalize, Seal, recovery, completeness, source-
readiness, media-integrity, or storage gate before a Measurement can start. Pressing `측정 시작`
freezes a new run identity/time and immediately enters `RUNNING`, including with zero available
sources. Camera, P1/P2, Base GNSS/NTRIP, Target and recording join or leave independently.
Persisted data never owns admission. An old open or recoverable Session directory is inert
historical data and cannot disable `측정 시작`.
After an abnormal process exit, restart likewise leaves the incomplete run as inert append-only
history and creates a new runtime generation only when the operator presses `측정 시작` again.

Structured records append to fixed, type-owned files beneath one relative
`measurement_data_root`. Videos, still images, native PNGs, and ULogs use their own creation-time
filename with local UTC offset and never replace an existing file. Each row retains its UTC,
monotonic, and available source-clock evidence, so an operator can collect or join records later.
Storage failures and bounded-queue drops are visible in `Data > Status`, but do not stop camera,
Detection, GNSS/RTCM routing, target telemetry, or the next Measurement. Result/event queues are
reserved ahead of bulk camera/PX4/GNSS traffic.

If an essential stream (`events`, `measurements`, or `method_evaluations`) cannot write, the
background owner retains FIFO records in a 256-record/8 MiB reserve and retries with bounded
backoff. Video and high-rate telemetry are never accumulated without bound; their drops remain
visible and the observation continues.

The top-level `Data` workspace exposes writer health, fixed paths, capture tools, the current data
folder, and a file-manager link to untouched legacy `recordings/`. It does not scan, recover, or
mutate legacy Session catalogs. Alignment is app-lifetime state: Apply persists an append-only
Alignment event, and the active Alignment remains available across Measurements until Revert,
`Reset Alignment`, or application restart.

Runtime fault isolation keeps one callback, device, camera child, or evidence writer failure from
silently taking down unrelated links. High-rate camera, PX4, GNSS, RTCM, and target traffic stays
on bounded worker-owned paths while the Qt UI consumes latest-state snapshots.

Calibration now presents a StereoWIRIS-style primary workflow while retaining the full Workstation
calibration implementation. The default surface exposes only `촬영`, `계산·적용`, and
`프로파일`; file selection, photo-set roots, per-view diagnostics, subset evidence, checkerboard
setup, and raw K,D/R,t details remain available under the collapsed `고급 캘리브레이션` area.

## Current Run Flow

From the repository root, install the Ubuntu manifest and create the one supported venv before
running the app:

```bash
sudo apt-get update
grep -Ev '^[[:space:]]*(#|$)' requirements/ubuntu-22.04-operator-runtime.txt | xargs sudo apt-get install
tools/check_operator_runtime.sh --deployment
if [[ -d .venv ]]; then mv .venv ".venv.backup-$(date +%Y%m%d-%H%M%S)"; fi
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements.txt
install -Dm0644 "$APPROVED_WORKSTATION_MODEL" .local/workstation_data/models/yolov8m.pt
tools/check_operator_runtime.sh --execution
./apps/Workstation/run.sh --check
./apps/Workstation/run.sh
```

The Workstation/Legacy launchers and common runtime doctor start from a closed operator
environment: user-site packages and ambient Python, loader, GI and GStreamer plugin/system/scanner
paths are cleared before the canonical repository paths are added. Deployment checks can bootstrap
with `/usr/bin/python3 -S` before the venv or third-party `packaging` module exists; version parsing
is loaded only by the execution check after the interpreter gate.
The Ubuntu manifest includes both `python3-gi` and `python3-cairo`, which keeps the system
PyGObject dependency graph complete when exposed through the canonical venv. Set
`APPROVED_WORKSTATION_MODEL` to the approved external file before the `install` command; `--check`
verifies its bytes against `model-manifest.json`.

The launcher accepts only repository `.venv/bin/python`; app-local, Simulation, system Python,
manual `dist-packages` `.pth`, and arbitrary `PYTHON_BIN` fallback are unsupported. `--check`
validates the interpreter, declared dependency versions, Workstation imports/settings/model,
Gst/GstRtp typelibs, required pipeline element find/make, and the selected decoder. It prints
`PASS/WARN/FAIL`, selected/effective decoder, missing elements, Ubuntu packages, and recovery
commands before returning the shared `0/1/2` exit contract. Both launchers force
`PYTHONNOUSERSITE=1`, use only explicit repository/common Python paths, and remove implicit
`~/.local/bin`, `~/.local/lib`, and ambient `PYTHONPATH` dependency. A package visible only through
the user site therefore fails the doctor instead of making this machine appear reproducible.
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

For the normal calibration flow, open `Calibration`, use `촬영` to confirm the actual zoom and
collect diverse checkerboard views, use `계산·적용` to calculate and save K,D (and R,t when
needed), then use `프로파일` to load and apply the verified bundle. Open `고급 캘리브레이션`
only when selecting exact photo files, reviewing quality reasons, changing board/setup parameters,
or inspecting persisted profile evidence.

The field dual-Pixhawk profile uses Workstation `10.0.0.10/24`, P1/Left
`10.0.0.2:14550` with MAVLink system ID `1`, and P2/Right `10.0.0.3:14551` with system ID `2`.
Save each board's Ethernet profile over USB once, reboot it, then verify both heartbeats with USB
removed. The exact parameters, SD-card files, recovery sequence, and field checks are recorded in
[`Pixhawk.md`](Pixhawk.md).

The current field wiring isolates camera and Pixhawk Layer-2 traffic on separate NICs. Cameras use
`enx588694fce07e` with `10.0.0.11/32` and preferred-source host routes to `.231/.232`; P1/P2 use
the replacement RTL8156B `enx0023a40a2e39` with `10.0.0.10/24` and `.2/.3` routes. Neither device
profile supplies a default route. Do not move camera and Pixhawk traffic back onto one broadcast
domain: that topology reproduced a persistent dual-PX4 MAVLink service stall under RTSP load.
Both Workstation and Legacy defaults now preserve this split; a clean or reset settings file no
longer proposes the Pixhawk NIC's `.10/24` address for the camera interface.

At runtime Workstation identifies itself on both Ethernet links as MAVLink GCS `255/190`. Connect
immediately seeds each configured peer with one GCS heartbeat to break PX4's pre-identity broadcast
deadlock, then sends at most one heartbeat per side per second. A new or rebooted
P1/P2 link remains `QUALIFYING` until expected-system HEARTBEAT and ATTITUDE stay continuously fresh
for 10 minutes. `ATTITUDE STALE` means other MAVLink traffic is still arriving; `ETH MAVLINK RX
STALLED` means all expected-system traffic has been silent for 10 seconds. Only `LIVE` passes the
Pixhawk readiness gate.

Pixhawk Connect opens transport on its worker, but status, completion, failure, cleanup and receive-
timer work that touches Qt is queued to the GUI owner. Progress cannot restyle topology buttons
before completion starts the 20 ms receive timer. Dynamic button state keeps authoritative text and
the `role` property without synchronous QStyle unpolish/polish. Successful RTCM progress is logged
at most once per side every five seconds; exact frame/packet/byte/failure counters and errors remain
unthrottled in their owning status surfaces.

Camera Preview uses the persisted H.264 decoder mode. The supported field default is `Auto`: when
the NVIDIA driver and GStreamer `nvh264dec` element are both usable, Auto requests NVDEC first.
`PLAYING` and decoder status text remain candidate evidence; only a decoded frame carrying the
pane's current stream generation makes that side `GPU · nvh264dec`. A classified NVDEC launch or
decoder-health failure records a side-specific software override and reason. Preview-only fallback
restarts only that side inside its worker and becomes `CPU fallback · avdec_h264` only after its
software-decoded frame arrives. RTSP/source, parser, mux, and filesink errors never authorize CPU
fallback.

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
.venv/bin/python -m pip install -r requirements/optional-pyav.txt
```

Persisted WIRIS visible-stream encoder fields are manual-reference values only. Normal startup,
Auto-Connect, RTSP Enable and Reactivate never send `SEBR/SEQZ/SEGS/SEIR`. They read `GETH`, send
`SETH TRUE` only when Ethernet mode is actually disabled, and then probe the stream. Do not infer
the physical bitrate/QP/GOP/IDR from the saved JSON; use explicit readback, and change an encoder
field only through a separately approved manual command path.

Choose `Settings > Storage > Data folder` before field work. The value must be relative to
`SASL_WORKSTATION_DATA_ROOT`; the default is `measurement_data`. `Data > Status` shows every fixed
file, writer state, dropped-row count, last write, and error. `Retry Storage` retries failed streams,
but it is never a prerequisite for Measurement.

Calibration can be completed at any time against the reusable calibration root. Alignment can
also be captured and applied before or between Measurements. The active Alignment is reused until
`Revert active pose`, `Reset Alignment`, or application restart; starting or ending a Measurement
does not clear it.

For an ordinary run:

1. Connect the intended WIRIS, P1/P2, GNSS/RTCM, and optional Target Drone inputs.
2. Apply the intended operating-zoom `K,D` and stereo `R,t`/Alignment source, then inspect
   readiness. Missing geometry may make a result BLOCKED, but it does not prevent data collection.
3. Press `측정 시작`. If Manual record is active, Workstation stops it first and queues this start.
   If a still capture, camera command, time sync, discovery, or RTSP activation owns the cameras,
   the transient run shows `QUEUED` and starts automatically when that owner releases them.
4. Press `측정 종료` when finished. Camera/Detection/video owners stop and the run returns to
   `IDLE`; no save, hash, manifest, RTCM drain, review workbook, or previous data state can delay the
   next start.
5. Repeat whenever needed, including after long gaps. Join the type-owned records later using their
   per-row timestamps and transient `runtime_generation` where present.

Every matched pair is still evaluated independently by Pixhawk, Checkerboard, Horizon, Mountain,
Drone Rotation, Drone Multi-view, Terrain, and Combined against the frozen inputs for that run.
The representative result remains `Pixhawk attitude + GPS/RTK`; another method is not promoted
when the primary method is blocked or fails. Result and method events append immediately to their
fixed JSONL streams. No automatic Parquet or `review.xlsx` is produced.

Manual recording remains under `Advanced > Files`. It uses the same fixed structured streams and
timestamped binary paths. Starting Measurement stops Manual record and does not resume it after the
Measurement. Explicit HR/OZ capture, ULog import, and Alignment actions remain available from
`Data > Capture` even outside a Measurement and always append their own event records.
Once application shutdown begins, a new Manual record request is rejected. If any recording-start
step raises before commit, Workstation restores the exact preceding session flags, row owners,
timestamps, video paths, and UI state, and releases only resources allocated by that failed attempt.

For checkerboard calibration, select the calibration zoom, command both cameras to that
zoom, confirm the actual zoom readback, and capture checkerboard observations. A capture over
`Max Live pair Δt` is retained per camera for independent `K,D` but is never passed to stereo
`R,t`; timing-eligible captures remain stereo candidates. Calculate and save the required
intrinsics or stereo `R,t` profile. At measurement time, load the `K,D` profile for
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
current-run telemetry absence for live geometry, non-finite/frame-invalid inputs, impossible
two-ray geometry, an already-running camera interval, unapplied geometry
candidates, and missing/mismatched candidate-to-active K,D identity. RTCP/NTP availability, pair
delta, vertical delta, telemetry age, GNSS fix/accuracy/readiness, and nominal K,D remain visible
advisories while a finite result is recorded as `ESTIMATED`.

Calibration capture now saves a lossless, no-overlay native frame and records its native,
preview, and saved raster grids; a preview-resized image is not used as the calibration input.
Auto capture rejects a pair when either camera's checkerboard pose is effectively a duplicate of
an already accepted pose. Move the board across the image, change distance and roll, and introduce safe
yaw/pitch perspective instead of collecting many fronto-parallel repeats.

In Calibration's shared input area, select a saved calibration photo set and review its matching Left/Right
files in `계산에 사용할 사진`. The table previews the photo-set candidates until the operator fixes
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
for both cameras to report completion. `Capture OZ Still` sits beside it in `Data > Capture`
and uses the same `Pre`/`Post`/`Checkpoint` purpose selector. It sends `SIVI TRUE`, `CPTR`,
and polls `ICPT` on both cameras for a camera-native visible/OZ still; the field camera
configuration is expected to provide the higher-resolution FHD evidence. Use `Pre` before
measurement and `Post` after measurement when this camera-local evidence is needed. Both
capture paths are independent of timestamped RTSP/MP4 recording.

For network RTK, open `Settings > GNSS`, select either network-assisted fixed-base or
direct network RTK operation, and confirm the caster hostname, port, mountpoint, TLS setting,
GGA policy, and direct Username/Password fields. A newly created settings file starts with the
NGII RTS2 Base-assisted profile: `RTS2.ngii.go.kr:2101`, mountpoint `VRS-RTCM32`, Base GNSS
GGA, WGS84 ellipsoid height, RTCM routing to both left and right Pixhawk rovers, and Username
`SASL`. The masked Password starts empty. Credential resolution uses the current UI Password
first, then the OS keyring, then configured legacy environment variables. After editing the
Settings Password, press `Apply` or `Save` before starting NTRIP so that explicit value reaches the
runtime owner. Other compatible casters remain configurable; existing custom provider
configurations are preserved, while retired legacy values are migrated to the NGII profile when
loaded.

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
discovers the camera's `/ssd/*/img`, `/sdcard/*/img`, or `/flashd/*/img` image directories and
selects the channel-appropriate latest still; FTP connection setup retries once and defaults to
a 30-second per-camera timeout.
Raw FTP downloads are separated as `<timestamp>/calib_capture/HR/left|right` for HR and
`<timestamp>/calib_capture/OZ/left|right` for OZ; validated calibration PNGs remain in the
selected photo set's `left/` and `right/` directories.
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

The `Drone RTK Multi-view R,t` tab consumes READY target pairs accumulated since the last
`Reset Alignment` directly;
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

Alignment updates append frozen-frame artifact identities, complete automatic proposals, accepted
points, current annotation payload, Drone RTK capture history, accumulated multi-view samples, and
provenance to `alignment.jsonl`/event streams. Native pixels remain timestamped data-root artifacts.

Run the Workstation regression suite from `apps/Workstation` with:

```bash
python3 -m pytest -q
```

Run the repository portability gate from the repository root with:

```bash
python3 tools/check_portable_paths.py
```

## Inputs/Outputs and Interfaces

- The simple Calibration controls dispatch the existing `calibration.*` command IDs; they do not
  implement a second solver or persistence path. `calibration.run_intrinsic`,
  `calibration.run_extrinsics`, Save/Load, and `calibration.apply_bundle` therefore retain the same
  frozen photo-set, quality, zoom, native-raster, immutable-profile, and rollback contracts as the
  advanced controls.

- `detection.toggle` is an always-available operator request shared by the Measurement button,
  menu, and toolbar. Presentation state never disables one surface independently of another. A
  duplicate active run is rejected, but an active manual writer, busy camera, missing RTSP URL,
  identical Left/Right stream, missing device, storage error, or old finalization cannot reject the
  observation interval. Those conditions affect only their source branch and method results.
- `measurement_run_snapshot()` exposes additive `sources` entries with
  `PENDING/WAITING_RESOURCE/LIVE/UNAVAILABLE/LEFT`, join/leave clocks, and factual detail. Every
  persisted transition carries `runtime_generation` so late callbacks cannot mutate or masquerade
  as the next run.

- Dual-Pixhawk Ethernet input is received on `udpin:0.0.0.0:14550` for P1/Left and
  `udpin:0.0.0.0:14551` for P2/Right, with expected system IDs `1` and `2`. Operational connections
  explicitly use GCS source `255/190`. The Stock default profile requests only consumer-backed
  standard messages: ATTITUDE, SYS_STATUS, GPS_RAW_INT/GPS2_RAW, GPS_RTK/GPS2_RTK,
  SCALED_IMU/2/3, DEBUG_VECT, RAW_IMU, LOCAL_POSITION_NED, GLOBAL_POSITION_INT, VFR_HUD, and
  BATTERY_STATUS. It does not request or retry private `SASL_CAN_MAG`; an unsolicited custom-firmware
  sample remains decodable for compatibility. Each `udpin` endpoint is exclusively owned: connection fails with an explicit
  port-in-use error if another Workstation, StereoWIRIS process, QGC session, or leaked worker has
  already bound it, and the live pymavlink socket disables later address reuse. If heartbeat stays
  live while `ATTITUDE` is absent or older than two seconds, Workstation re-requests the 30 Hz
  `ATTITUDE` interval up to three times, waits ten seconds, and repeats the bounded recovery cycle;
  a fresh `ATTITUDE` sample immediately clears that outage cycle. When all expected-system RX is
  silent for 10 seconds, stream/parameter commands and RTCM are fail-closed instead of cycling the
  host socket.
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
  `validity=VALID|ESTIMATED|INVALID`, `hard_block_reasons`, and `advisory_reasons`. The fixed
  `measurements.jsonl` stream preserves the same decision and the compatibility
  `measurement_valid` field remains true for both computed `VALID` and `ESTIMATED` results.

- The canonical runtime root is `.local/workstation_data/` at the repository root, or the path in
  `SASL_WORKSTATION_DATA_ROOT`. It owns `stereo_distance_settings.json`, `measurement_data/`,
  `calibration/`, `models/`, historical `recordings/`, and runtime caches. Persistent asset and data
  folder settings are normalized to data-root-relative POSIX paths.
- New Measurement data uses the relative `measurement_data_root` setting, default
  `measurement_data`. Hand-edited absolute or escaping values fall back to that safe default and
  report `DEGRADED`; they do not reject Measurement.
- Structured files have stable type ownership and append across application restarts:

  | Stream | Fixed relative file |
  | --- | --- |
  | run/capture events | `events.jsonl`, `capture_events.jsonl` |
  | primary pair results | `measurements.jsonl` |
  | eight-method bundles/outcomes | `method_evaluations.jsonl` |
  | camera observations | `camera_left.jsonl`, `camera_right.jsonl` |
  | rig and target PX4 | `px4_left.jsonl`, `px4_right.jsonl`, `px4_drone.jsonl` |
  | Base GNSS | `gnss_base.jsonl` |
  | analysis comparisons | `geometry_comparisons.jsonl` |
  | Alignment changes | `alignment.jsonl` |
  | binary inventory/logs | `artifacts.jsonl`, `native_frames.jsonl`, `application_log.jsonl` |

- The writer removes Session membership fields, adds host UTC and monotonic record timestamps, and
  preserves producer coordinate/source-clock fields. A torn final JSONL fragment is left as one bad
  historical line; the next valid append begins on a new line. Files are not rotated automatically.
- Priority streams (`events`, results, method evaluations, captures, Alignment, artifacts, native
  frames) use a reserved bounded queue. Bulk camera/PX4/GNSS rows use a separate bounded queue whose
  oldest queued item may be dropped under overload. Each stream reports its own state/error/drop
  count and can be retried independently.
- Essential write failures retain FIFO `events`, `measurements`, and `method_evaluations` within
  the 256-record/8 MiB reserve. Automatic retry uses per-stream 1–30 second backoff; `Retry Storage`
  requests an immediate retry. Reserve overflow rejects the newest row and increments its drop
  count without blocking a producer or Measurement.
- MP4, WIRIS still downloads, exact HR imports, native PNGs, and ULogs use
  `YYYYMMDD_HHMMSS_nanoseconds+offset_<label>.<ext>`. Collision suffixes are added and publication
  refuses replacement, so yesterday's `..._left.mp4` cannot be erased by a later capture.
  A recovered camera adds a side-specific segment with the same run generation and a new segment
  index rather than reopening an earlier file.
- Exact RTCM correction bytes append only while Measurement or Manual record is active under
  `raw/gnss/{ntrip_transport_chunk,ntrip_rtcm_frame,base_rtcm_frame}.rtcm`, with a fixed sibling
  offset/hash JSONL index. RTCM evidence failure is diagnostic; injection/routing and Measurement
  admission remain independent.
- Calibration continues to use its reusable calibration root and content-addressed profiles.
  Calibration photo-set “sessions” are calibration-domain groupings only; they are not Measurement
  Session gates. Alignment events append to the data root and the active in-memory pose spans runs
  until Revert, Reset, or restart.
- Existing `recordings/<id>/` and `recordings/sessions/<id>/` trees are never scanned or recovered
  by the new workflow. `Data > Status > Open Legacy Recordings Folder` exposes them read-only through
  the file manager. Old manifests, Parquet, and `review.xlsx` remain untouched but no new instance is
  generated automatically.
- `measurement-method-snapshot-v1` still supplies the live Measurement view. Runtime
  `method_capabilities` remains instance-driven while `method_catalog` lists all eight method IDs;
  missing inputs become explicit BLOCKED outcomes rather than invented pose data.
- YOLO detector weights remain runtime-owned under the Workstation data root and are not copied into
  Measurement data.
- The default detector is `.local/workstation_data/models/yolov8m.pt`. Runtime model download is
  disabled: provision the approved bytes whose size and SHA-256 match `model-manifest.json` before
  launch. Relative custom model paths resolve under the canonical runtime root; enumerated legacy
  defaults remain readable without migration.
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
- Calibration images are per-camera checkerboard observations grouped by capture ID under the
  active calibration root. Timing-eligible matching observations form stereo pairs; an over-limit
  capture remains in the independent left/right `K,D` inputs with `stereo_pair_saved=false` and
  `intrinsics_only_sides`, and is excluded from `R,t`. Manual, FTP, and Auto capture all use the
  configured reusable calibration root; Measurement start/stop never changes that owner. JSON
  profile paths are selected independently in the Calibration UI.
  `calibration_pair_selection.json` v2 stores only Photo-set-relative paths;
  unresolved legacy paths block calculation and are never replaced by an all-photo fallback.
- Each capture metadata record identifies the exact source frame, source/native dimensions,
  preview dimensions, saved dimensions, crop/scale contract, overlay state, SHA-256, timing gate,
  `stereo_pair_saved`, and `intrinsics_only_sides`. A live
  CameraPane capture fails closed when a fresh native frame cannot be matched. Preview fallback is
  disabled by default and exists only behind an explicit compatibility flag for lightweight tests.
- Calibration photo-time display reads `left_rtcp_frame_ntp_unix_s` and
  `right_rtcp_frame_ntp_unix_s` only when they are bound to a saved manual/native or Auto frame.
  It otherwise uses `capture_recorded_unix_ns`, then `capture_requested_unix_ns`, and finally the
  image file `mtime`; the UI always labels which source was used.
- `Capture HR Still` photos are created with `SIHR TRUE` + `CPTR`; `ICPT` is polled for completion.
  When WIRIS FTP is enabled, the worker lists the camera storage before capture, downloads only
  a new/changed post-CPTR image into the data root and validates the downloaded bytes immediately.
  The left/right `CPTR` dispatches share a capture ID and barrier; their host monotonic delta is the
  explicit FTP pairing basis. It is classified as `HOST_DISPATCH_PROXY`: it may permit pair
  persistence but always has `precision_qualified=false` and
  `actual_exposure_verified=false`. It is not an exposure timestamp. RTCP-bound live-frame evidence
  is `CAMERA_TIMESTAMP_VERIFIED`; only explicitly verified per-camera exposure fields can become
  `EXPOSURE_TIMESTAMP_VERIFIED`. If FTP is disabled or fails, the
  still remains camera-local until an exact HR file is imported.
- Each button press appends one JSON object to `capture_events.jsonl`; downloaded/imported image
  bytes remain separate timestamped artifacts.
- `Capture OZ Still` photos are created with `SIVI TRUE` + `CPTR`; `ICPT` is polled separately from
  HR and RTSP recording. With FTP enabled, fresh left/right originals are downloaded into the
  data root and retain the same capture-ID/dispatch pairing evidence; otherwise they remain
  camera-local.
- Each OZ button press appends an `oz_capture_event` object to `capture_events.jsonl`. Event rows
  contain evidence values while
  image bytes remain independently hashed artifacts.
- NTRIP inputs are a provider-independent caster hostname (without `http://` or `https://`),
  TCP port, mountpoint, optional TLS flag, GGA policy, direct Username, masked runtime Password,
  and optional legacy environment-variable names. Username may have a non-secret deployment
  default, but Password starts empty and must come from the current UI, OS keyring, or legacy
  environment fallback. It is never stored in settings, source defaults, logs, or evidence.
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
- `alignment.jsonl` stores JSON-safe Alignment update/reset events and references timestamped native
  frame paths; it never embeds QImage/UI objects.
- Measurement-method snapshots use stable IDs (`pixhawk_rtk_live`, `checkerboard_stereo`,
  `horizon_hybrid`, `mountain_gnss`, `drone_rtk_rotation_hybrid`,
  `drone_rtk_multiview`, `terrain_pnp`, and `combined_hybrid`). Legacy preferred/fallback settings
  remain readable for old packages, but new all-method execution does not use them to replace the
  fixed Pixhawk/RTK representative result.
- Every transient run freezes `measurement_method_fill_policy` and
  `measurement_frozen_comparison_inputs` under schema `measurement-method-config-v1`; the values
  are attached to append-only result/method records rather than a package. The fill policy is keyed by method and then by
  `<left|right>.<roll_deg|pitch_deg|yaw_deg|north_m|east_m|down_m>`, with values `live_pixhawk`,
  `live_gnss`, or `none`. Application restart begins a new transient run and does not recover an
  unfinished package.
- Measurement rows record `measurement_method_id`, method instance ID, geometry SHA-256,
  `measurement_geometry_kind`, selection reason, and the left/right per-DOF component-source
  mapping in addition to the legacy triangulation source fields. `geometry_kind` distinguishes
  the math kernel (`relative_pose`/`live_pixhawk_mount`) from the independently evaluated method.
- Visible settings ownership is split by domain: Calibration owns the applied Intrinsics,
  `Settings > YOLO / RTSP` owns the canonical `yolo_input_size`, and Measurement Settings owns
  confidence, IoU, and pair-gate values. Legacy backend aliases remain readable for compatibility
  but are not duplicated as visible controls.
- `stereo_distance_settings.json` owns `wiris_visible_encoder_bitrate`,
  `wiris_visible_encoder_qp`, `wiris_visible_encoder_gop`, and
  `wiris_visible_encoder_idr`. These map to WIRIS `SEBR/SEQZ/SEGS/SEIR VISIBLE`; the independent
  readbacks use `GEBR/GEQZ/GEGS/GEIR VISIBLE`. The operational profile keeps bitrate at `0` so
  the configured QP remains effective.

## Operational Rules

- Treat the three simple Calibration tabs as an ordered workflow, not as a quality bypass.
  `FAIL` continues to block Apply, `WARNING` still requires explicit review/confirmation in
  `고급 캘리브레이션`, and a zoom or native-raster mismatch remains a hard failure. The
  advanced area is presentation-only disclosure; collapsing it never disables its underlying
  command state or changes the active calibration.

- Camera Preview defaults to RTSP TCP with Auto decoder. Auto uses `nvh264dec` only when its
  factory can be created and `nvidia-smi` confirms a working driver; otherwise it reports WARN and
  starts with mandatory `avdec_h264`. A direct NVDEC element/health failure, or a 30-second decoded
  first-frame/stall timeout while RTP input remains current, may authorize one software fallback.
  RTSP/source/parser and MP4 mux/filesink failures retain their own error class and never trigger a
  decoder change. Explicit `NVIDIA` has no software fallback.
- Decoder fallback ownership is per side. Preview-only fallback restarts only the failed side and
  leaves the healthy side's decoder/worker running. During MP4 recording, the failed side finalizes
  and stops; its software override applies on the next side start because reopening the same MP4
  path is forbidden. The healthy side's recording worker is not restarted. A terminal child exit
  is not treated as a restartable crash.
- Read decoder state in `Devices > Cameras` first. `Decoder LEFT/RIGHT` is the primary actual
  pipeline evidence; compact CAM L/R tooltips include the same actual/fallback state, while
  `Diagnostics > Performance` shows GPU model/driver, NVDEC availability, requested/effective
  candidates, and per-side detail. `CUDA` shown for YOLO inference is a separate resource decision
  and does not prove that H.264 decode uses NVDEC.
- A camera QThread/process launch is only a source-level `STARTING` request. Workstation sets a
  side LIVE only
  after a decoded frame carrying that pane's current stream generation reaches the GUI boundary.
  The Measurement run is already `RUNNING`; after 35 seconds a side without an ACK is stopped and
  marked `UNAVAILABLE`. Zero, one, or two verified sides never roll back the observation interval.
- MP4 readiness is independent from Measurement readiness. The recording projection reports
  `video=STARTING` until each allocated side-specific path exists and grows beyond its allocation
  size, then reports provisional `RUNNING`; one failed side is `PARTIAL`, and no growing side is
  `FAILED`. A recovered side writes a new no-overwrite segment. Online post-run state is presence-only:
  `SAVED` means both sides produced a file, `PARTIAL` means only some files exist, and `FAILED`
  means none exist. These labels do not claim playback or integrity verification.
  Video failure never turns a frame-ready Measurement back from `RUNNING`, but Workstation does not
  claim video recording is active and preserves the per-side failure evidence.
- Before a real soak, run the strict synthetic queue/evidence/cleanup gate from the SASL root:

  ```bash
  PYTHONPATH=apps:common/python .venv/bin/python -m Workstation.qualify_camera_backpressure \
    --duration 600 --gui-stall 2
  ```

  Its exit status is nonzero for exact-frame loss, insufficient durable pairs, missing ACKs,
  writer/child/shared-memory cleanup failures, queue/ring violations, Stop latency, or configured
  RSS growth/high-water/slope violations. It does not instantiate the real Qt preview and therefore
  does not replace the dual-camera GUI/hardware qualification.

- Audit persisted real native evidence independently from the producer implementation with:

  ```bash
  PYTHONPATH=apps:common/python .venv/bin/python -m Workstation.qualify_native_evidence \
    --data-root .local/workstation_data/measurement_data \
    --runtime-generation run-<exact-runtime-id>
  ```

  The verifier imports no Workstation ring/writer/store code. It checks run-scoped FrameKey and
  descriptor canonical bytes, PNG CRC/IDAT/RGB8 structure, file size/SHA/path, origin request and
  pair ACK. Nonzero exit means the runtime cannot be claimed as exact native evidence.

- Never issue `mavlink stop` as part of USB Ethernet recovery or persistent startup. On affected
  PX4 v1.17 custom firmware, stopping and restarting a live instance can leave a sparse instance
  slot that the signing callback dereferences and hard-fault task `mavlink_rcv_if2`. Do not issue a
  second `mavlink start` either: `MAV_2_CONFIG=1000` is the sole owner of the Ethernet instance,
  while `extras.txt` contains only `ifup eth0` and the board-specific `ifconfig`. Persist all
  `MAV_2_*` values, reboot, and verify one broadcast stream per board. Simultaneous matching
  unicast and broadcast packets immediately after boot, before any host MAVLink packet is sent,
  indicate competing instances. Keep QGC closed while Workstation owns UDP ports `14550` and
  `14551`.
- Treat each Workstation `udpin` port as single-owner state. Do not run Workstation and StereoWIRIS
  against the same `14550`/`14551` listeners simultaneously. A second owner now fails at connect
  instead of silently sharing pymavlink's reusable socket; close the other application or leaked
  worker and retry. If heartbeat remains current but `ATTITUDE` becomes stale, leave the connection
  open while the bounded interval re-request runs and inspect its recovery/failure log before
  power-cycling the Pixhawk.
- Workstation seeds each fixed P1/P2 peer on the same exclusive bound `udpin` socket and sends GCS
  heartbeat `255/190` immediately, before any inbound packet. This avoids the startup deadlock in
  which PX4 waits for a known GCS peer while Workstation waits for the first packet. Only an
  expected-sysid `HEARTBEAT` verifies identity; other expected-sysid telemetry remains transport
  evidence but cannot unlock commands. No expected heartbeat within 10 seconds is shown as
  `MAVLINK INITIAL RX TIMEOUT`/`NO HB`, with peer and listener age.
- Treat Pixhawk Connect progress as transport setup, not as permission to synchronously repaint
  Connect/Disconnect roles. Worker callbacks are queued to the GUI thread, the receive timer must
  run on its QObject owner, and button text remains the primary state cue. A live status callback
  must not call QStyle `unpolish()`/`polish()`.
- Treat `ETH MAVLINK RX STALLED` as a remote PX4-side service failure, not as proof that the NIC,
  ping, or host UDP socket is down. Workstation keeps only the 1 Hz GCS recovery probe, blocks new
  RTCM/stream/parameter traffic to that side, and leaves the other side running. Re-pressing Connect
  does not close/reopen the host socket as a claimed recovery.
- The stall recovery command is enabled only for the named stalled side. Run `Find USB` immediately
  before recovery; Workstation requires one unique USB heartbeat mapping (P1=sysid 1, P2=sysid 2),
  no older than 30 seconds, asks the operator to confirm the side/device/sysid, and revalidates it in
  the one-shot worker before sending reboot. Missing, stale, duplicate, or changed identity; active
  Measurement/recording/save/finalization; or another PX4 command fails closed. USB command send is
  not success: Ethernet must enter `RECOVERING` and complete a fresh 10-minute qualification.
- USB Ethernet profile persistence and runtime stall recovery are separate operations. The verified
  recovery command writes no PX4 parameter/config/file and never flashes firmware. Automatic reboot
  is not supported.
- Workstation does not start the automatic PX4 calibration-parameter readback at the first packet
  or heartbeat. It waits until that side completes the continuous 10-minute qualification and
  reaches `LIVE`; manual refresh remains available. During a parameter read, that side's normal
  ATTITUDE/GNSS receive loop is intentionally reserved. Its status
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
  actions, with `(FTP)` appended only when the still is downloaded immediately. Manual, Auto,
  and FTP captures over `Max Live pair Δt` still save every side whose checkerboard validation
  succeeds, mark those files `INTRINSICS ONLY`, and exclude them from stereo `R,t`. Auto capture
  consumes each new frame identity once and counts only timing-eligible two-sided captures as
  valid stereo pairs. Calibration photo-set manifest `pair_count` uses the same R,t-eligibility
  rule; K,D-only files remain visible in the selected Photo set without inflating that count.
- FTP calibration pairing never borrows the current RTSP preview clock. Both cameras must report
  the same capture ID, the synchronized-CPTR-dispatch contract, and per-camera dispatch timestamps;
  the absolute dispatch delta must satisfy the configured pair gate. These host timestamps prove
  trigger proximity only. Metadata continues to mark the camera exposure time as unknown. An FTP
  capture outside the dispatch gate is retained for per-camera `K,D` only.
- `측정 시작` is gated only by live command/resource facts: valid distinct Left/Right RTSP inputs,
  no already-running transient Measurement, and availability of the camera resource. It never
  reads old Session state, legacy manifests, storage completeness, writer health, or save-retry
  flags.
- Camera-resource contention produces `QUEUED`, not failure. Manual record is stopped first;
  still capture, Alignment/Calibration capture, WIRIS commands/time/zoom/image format, camera
  discovery, RTSP check, activation, and reactivation are polled until they release the cameras.
  Manual record is not resumed automatically.
- A run transitions `IDLE → STARTING → RUNNING → STOPPING → IDLE`. `측정 종료` releases live
  owners and requests writer flush without waiting for save, hash, probe, RTCM drain, package, or
  workbook completion. A new run can start immediately after the actual camera workers stop.
- Structured data is recorded only while Measurement or Manual record is active. Explicit capture,
  import, and Alignment commands always append their own event even outside those intervals.
- Storage is best-effort. A setup/open/write/serialization/fsync failure drops the affected record
  or pauses only that stream and is reported in `Data`; it never stops the recorder or raises through
  a Qt callback into Measurement admission. `Retry Storage` clears failed-stream state.
- Native PNG completion is joined to its append-only recording row by the unique async request ID,
  independently of the 1024-pair lookup cache. A stale completion for an older request cannot
  overwrite a newer pending request for the same pair. The disk-I/O barrier is acknowledged when
  the writer finishes even if row projection must be deferred, so a later row-construction failure
  cannot hold the next run open. Drone RTK Multi-view `BLOCKED` samples stay in audit evidence
  but are not considered collected; only `READY` samples clear the target editor when the next
  Frozen burst arrives.
- The Measurement view displays all eight frozen/live source values independently. Missing method
  instances remain visible as blocked catalog rows, while each pair persists one atomic bundle with
  method ID, instance ID, component sources, geometry hash, status, warnings, and worker timing for
  every method. No automatic workbook or Parquet materialization follows run stop.
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
- Requesting HR/OZ does not create a parent or draft. Success, timeout, one-sided download, invalid
  image, and import failure each append an explicit capture event. Exact downloaded/imported bytes
  use timestamp names and are never overwritten.
- Settings and event projections redact passwords, tokens, and embedded URL credentials. Persistent
  data paths remain relative to `SASL_WORKSTATION_DATA_ROOT`; `/dev`, `/opt`, bounded `/tmp`, and
  camera-side `/` remain explicit machine/device interfaces.
- Canonical capture/event rows are append-only evidence. Hiding a graph row changes only the view,
  and a malformed historical line is not rewritten. The next record always appends after a newline.
- Full-rate RTCM envelopes enter a 4,096-item bounded worker queue. A full queue applies producer
  backpressure within the RTCM worker rather than the GUI. The separate Pixhawk delivery queue is
  latest-relevant and bounded: a replaced correction returns its exact pipeline sequence as an
  explicit stale drop, while the original received bytes remain in evidence.
- An invalid RTCM CRC never authorizes the parser to discard the corrupt candidate's declared
  length. A false length may enclose a later valid frame, so CRC failure drops only the false `0xD3`
  sync byte and rescans the remaining bytes. Only independently CRC-valid RTCM frames enter routing.
- An RTCM correction older than two seconds at the injection boundary is not sent to Pixhawk. Its
  bytes, source timestamp, hash, sequence, targets, age, and `stale-before-injection` outcome remain
  observable. Evidence-write, recovery-index, and unconfirmed-delivery failures remain diagnostics;
  they are never presented as successful delivery and never create a save-retry gate.
- Every RTCM envelope freezes its data-root context and routing context when enqueued. Paired
  binary/index writes use explicit offsets, flush/readback checks, and rollback on failure; reconnect
  retires the prior subscriber generation. Routing remains available after an evidence failure.
- NTRIP, RTCM pipeline, and Base GNSS worker results that update status labels, logs, tables, or
  evidence projections are always queued onto the Qt GUI thread. Direct cross-thread connections
  are limited to documented lock/queue adapters that never access widgets.
- Shutdown stops NTRIP and Base producers first, drains RTCM ingress, waits for every
  Pixhawk injection outcome (or records the exact unconfirmed sequences after the bounded timeout),
  and only then disconnects MAVLink/runtime workers. Window close remains non-blocking and retryable
  during the cooperative phase. Each camera keeps its full 12-second cooperative-stop window,
  including the nine-second GStreamer MP4 EOS wait, even when an earlier shutdown step delayed the
  camera stop request. Session-save and both HR/OZ capture threads are explicit shutdown gates.
  Camera success or synthesized failure finalization evidence, native-ring lease cleanup, and a
  thread/process/socket/shared-memory resource report must be accepted before settings are flushed;
  `DataStorage` closes last. After the cooperative deadline, escalation requests terminate/kill at
  the camera's existing 12/20-second thresholds and waits at most one second per forced pass, but a
  surviving resource keeps the window close pending instead of being reported as clean success.
- Native camera rings use owner-identifying `sasl_ws_native_<pid>_<token>` names. Normal Quit unlinks
  them explicitly, multiprocessing tracker EOF reclaims them after a typical `SIGKILL`, and the next
  startup provides a strict fallback for same-UID rings whose embedded owner PID is dead. Startup
  never sweeps arbitrary `psm_*` objects or rings belonging to a live process.
- Preview replacement applies only to resized UI frames. Native measurement/calibration frames,
  detection events, recording streams, command results, and RTCM evidence do not use the preview
  inbox and cannot be inferred from a skipped display frame.
- Detection pairing expires buffered observations after one monotonic second, clears a side
  immediately on an explicit no-detection update, and rejects a delayed older stream generation.
  Once a left/right pair is accepted, both exact `FrameKey` slots are leased before the result is
  exposed to the GUI; a replaced/unconsumed result releases those leases. The native writer accepts
  that single-owner reservation, verifies each incoming `frame_descriptor_sha256` against the pinned
  descriptor before writing either side, and fails the whole pair without nearest-frame substitution
  when any compact/component key or descriptor hash disagrees.
  Frozen detection metadata retains every descriptor-hash input, including app-receive
  `timestamp_s`, so a later audit can recompute the digest rather than only compare copied values.
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
- Each GStreamer side starts fail-closed and, after any RTP sequence omission, resumes only when
  the normalized Annex-B access unit contains an H.264 IDR slice (NAL type 5) and the decoder
  returns that exact PTS. A parser `non-delta` flag or a later dependent decoded frame is not a
  recovery boundary. The encoded recording tee remains blocked until the parsed IDR; native
  evidence, Preview, and YOLO remain blocked until that IDR's exact decoded PTS is observed. After
  the 30-second integrity deadline, the side becomes explicitly unavailable and does not
  auto-reconnect; the healthy peer remains live. Correct the sender so it emits IDRs, then reconnect
  manually. Workstation does not change GOP/IDR or any other camera encoder setting automatically.
- Linux camera child processes arm a parent-death signal. A Workstation crash or Force Quit
  therefore terminates its decode children instead of leaving CPU-consuming orphan workers for
  the next launch; normal shutdown still uses the cooperative stop/drain path. Unexpected child
  death is supervised separately from RTSP reconnect and receives bounded automatic restart.
- Keep `wiris_visible_encoder_bitrate=0` for both WIRIS cameras. The 2026-08-27 field readback was
  bitrate/QP/GOP/IDR `0/1/30/1`; only IDR `0 -> 1` was operator-approved, and Workstation does not
  apply that profile automatically. A nonzero bitrate makes QP ineffective. Always use live SDK
  readback rather than the saved JSON as the hardware oracle.
- Changing `YOLO input size` does not resize an already-running detector. Stop Detection, Apply or
  Save the Settings value, and start Detection again; the next worker uses the updated size.
- Changing `Weights path` also does not hot-swap an already-running detector. Use `파일 선택…`
  before Detection, confirm the verified relative path, then Apply or Save. An external model must
  complete the content-addressed import before it can become the persisted model dependency.

- Intrinsics are solved independently from the selected left and right image sets. Stereo `R,t`
  uses only timestamp pairs whose left and right observations both survived detection and
  near-duplicate-pose filtering and whose capture metadata does not explicitly reject stereo
  persistence. Excluding a pair for timing or excluding one side does not silently exclude either
  surviving intrinsic observation. A manual selection cannot promote an explicit timing rejection
  back into `R,t`. The configured calculation-pair limit is applied once, after this usable stereo
  intersection is formed.
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
- A dual-camera `Set camera clock` operation sends Left STUS first and sends Right only after Left
  returns an exact `OK` ACK. Only STUS mutation responses may wait up to 15 seconds; generic SDK and
  HIWS retain their configured timeout. One action permits at most two rounds within 90 seconds and
  uses fresh replacement-generation SR feedback for the second round. During recording the
  operation is disabled and the controller also rejects direct requests. After RTSP reconnect and
  fresh-SR warmup reaches `VERIFIED`, Workstation
  continues a 1 Hz health check; it immediately changes to `FAILED` on a current-frame/SR loss or
  regression, RTCP/NTP pair delta over 50 ms, SR-PC offset over 100 ms, SR age over the configured
  target (7 s by default), or NTP/RTP clock-rate error over 0.5%. The limit stays configurable; an
  older saved 10 s value is not silently changed, so Apply 7.0 s before the approved field
  qualification and retain that effective value in its evidence. Stored per-frame SR age is advanced
  using the current monotonic and wall-clock observations, so two frozen streams cannot remain green.
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
  do not start or stop RTSP/MP4 recording.
- Each `Capture OZ Still` event records its sequence ID, timestamp, active zoom readback, left/right
  camera responses, completion state, selected purpose, and per-side camera-local/FTP state.
- A caster host and mountpoint are required only when starting an NTRIP operation. The
  NGII RTS2 profile supplies them for new settings; an explicitly selected custom provider
  must supply its own values before starting.
- An untouched NTRIP UI uses Username `SASL` and an empty masked Password. An explicit runtime edit
  has highest priority; otherwise a saved keyring credential and then configured legacy environment
  credentials are resolved. A session password is bound to the host/mountpoint/username visible when
  `Apply` or `Save` is pressed; changing that account cannot silently reuse the old session value.
  A newly typed password is saved to the keyring only after a caster `200` response. A
  `HTTP/1.1 401 Unauthorized` fails immediately as `HTTP_AUTH_FAILED` without retrying the same
  credential three times; verify the displayed credential source and account instead of assuming a
  transport or RTCM fault.
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
- Applying, reverting, or resetting Alignment appends its persistence projection independently of
  Measurement start/stop.
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

### NVIDIA NVDEC unavailable after `s2idle` resume — observed 2026-08-29

On the qualified RTX 4050 laptop, an active dual-`nvh264dec` Preview did not survive one lid-close
`s2idle` cycle under Linux `6.8.0-138` and the NVIDIA `595.84` open kernel module. The NVIDIA
systemd hooks were installed, enabled and successful: `nvidia-suspend.service` completed before
sleep at 12:56, and `nvidia-resume.service` completed after wake at 14:00. This was therefore not a
missing-hook or missing-package failure.

Immediately after resume, the kernel reported `Xid 31` GPU MMU `FAULT_PDE` virtual-read faults for
both independent camera decoder workers (`pid=60967/60969`). A subsequent driver `modprobe` channel
faulted as well. UVM then reported global fatal error `0x60`, and `Xid 154` changed the required
recovery action to `Node Reboot Required`. The strongest current cause inference is a
suspend/resume video-memory or CUDA/UVM page-table restore failure, likely a driver/platform timing
boundary triggered while two NVDEC contexts were active. Xid 31 can also represent an application
or hardware defect, so one resume-correlated event does not prove the ultimate NVIDIA-driver root
cause; recurrence after a cold boot or outside suspend requires separate NVIDIA/hardware analysis.
[NVIDIA's Xid catalog](https://docs.nvidia.com/deploy/xid-errors/analyzing-xid-catalog.html) defines
Xid 31 as a GPU memory page fault and Xid 154 as the recovery action selected for the accompanying
error.

The host selects `[s2idle]` even though `deep` is available. NVIDIA `595.84` documentation warns
that some `s2idle` systems have a resume timing issue and lists `deep` as the interim workaround;
changing the system sleep mode still requires an independent laptop/driver qualification rather
than an application default.
[NVIDIA Linux power-management documentation](https://download.nvidia.com/XFree86/Linux-x86_64/595.84/README/powermanagement.html)
also explains why active video-memory allocations must be preserved across suspend.

The post-fault symptoms are intentionally explicit:

- `nvidia-smi` can still identify the GPU through the management interface, but reports
  `GPU Recovery Action: Reboot`; this is not proof that CUDA/NVDEC can create a context.
- Loading GStreamer's installed `nvcodec` plugin fails CUDA initialization with `0x64`
  (`CUDA_ERROR_NO_DEVICE=100`). The plugin consequently registers no usable `nvh264dec` factory,
  and `gst-inspect-1.0 nvh264dec` reports `No such element or plugin`.
- Camera `Auto` records `gst_parse_error: no element "nvh264dec"` per side and correctly falls back
  to `avdec_h264`. The observed UI showed both sides as `CPU fallback`, about `5.9 Hz`, with one side
  becoming `STALE` under the extra CPU load.
- App or Preview restart alone cannot recover this fatal driver state. The current Auto worker also
  keeps its proven CPU override for that generation instead of repeatedly probing a failed GPU.
- This failure is independent of WIRIS GOP/RTP/IDR behavior; the camera streams merely kept active
  GPU contexts present when the suspend boundary was crossed.

Immediate recovery is a full OS reboot, followed by a fresh Preview start so `Auto` can request
`nvh264dec` again. Until a suspend/resume strategy is independently qualified, stop Preview before
closing the lid or suspending the host. A future application mitigation may consume systemd-logind
`PrepareForSleep`, stop GPU workers before suspend, and create new worker/CUDA contexts only after
resume; that behavior is not implemented today. Useful read-only diagnostics are:

```bash
journalctl -k -b | rg 'NVRM: Xid|global fatal error'
nvidia-smi -q | rg 'Driver Version|GPU Recovery Action'
GST_DEBUG_NO_COLOR=1 GST_DEBUG='nvcodec*:7,cuda*:7' gst-inspect-1.0 nvh264dec
cat /sys/power/mem_sleep
```

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

The 2026-08-25 check used Stock Stable PX4 v1.17 on both boards. Direct PX4 shell readback showed
one Ethernet UDP instance per board (`14550` for system ID 1 and `14551` for system ID 2),
`MAV_2_CONFIG=1000`, and an `extras.txt` containing only board-specific `ifup`/`ifconfig` lines.
Independent four-second listeners received P1/P2 heartbeat counts `5/5`, ATTITUDE counts
`169/167`, GPS1 counts `29/28`, and GPS2 counts `28/28`. This verifies current standard MAVLink
production and supersedes the earlier statement that P2 still required the patched custom image.
Stock firmware does not emit private `SASL_CAN_MAG`, and this short observation is not a
USB-removed or long-duration field soak; per-CAN identity display therefore remains unavailable
and sustained link qualification is still required.
The later real USB `AUTOPILOT_VERSION` and parameter readback supersede older file/boot artifacts:
both boards run PX4 1.17.0 with system IDs 1/2, intended `MAV_2_*` Ethernet ports and 2,000,000 rate.
Repository firmware bundles remain provenance only, not installation evidence.

The observed reboot-only recovery, alive host network/USB evidence, valid RTCM frames, and repeatable
roughly 5.5-minute all-RX silence match the still-open PX4 issue
[#25956](https://github.com/PX4/PX4-Autopilot/issues/25956), which describes a NuttX UDP
`MavlinkReceiver`/IOB exhaustion deadlock under high traffic. This remains a high-confidence cause
inference, not an onboard stack-trace proof. The Workstation changes mitigate load, detect/isolate
the failure, and provide a guarded recovery; they do not patch PX4 firmware.

Desktop accessibility remains supported and must not be disabled as a Workstation workaround. On
the 2026-08-26 qualification host, an already-running Orca process and AT-SPI destination queue were
severely saturated; the pre-fix Pixhawk Connect path then blocked inside synchronous QStyle polish
before its receive timer started. Workstation removes that synchronous critical-path call and
throttles successful RTCM progress text. Diagnose an independently runaway screen reader as a
system resource fault rather than turning off accessibility or treating it as PX4 packet loss.
The earlier captured receiver state also showed P1 GPS1/P2 GPS1 MSL altitudes near `108.30/100.29 m`
(`~8.01 m` difference) while P2 GPS1/GPS2 differed by `~8.83 m`. This is receiver-solution or
physical antenna/configuration evidence, not a Workstation datum conversion; verify antenna mapping,
correction delivery, and RTK Fixed convergence before accepting the baseline.

The software integrity gates and simulated integration suite do not replace field qualification.
The WIRIS cameras/FTP control path, Base and rover F9P receivers, NGII live caster, three Pixhawk
links, MAVLink receiver-side acceptance, long-duration dual-video shutdown, and automatic ULog
collection still require bench and field soak verification with the intended hardware. A local
MAVLink send result is transport evidence, not proof that the remote receiver accepted and used the
correction.

A clean shutdown resource report proves only the Workstation-owned thread/process/socket/shared-
memory postconditions that the application can inspect. Workstation's `SAVED` label proves only
that the expected MP4 paths exist; it deliberately sets unreviewed output to
`integrity_validation_status=NOT_RUN` and does not claim `recording_usable`. Review playback or
integrity later with OS/offline tools. This never upgrades `source_media_integrity_verified` and
cannot repair information already lost in the camera/RTP stream. OS `SIGKILL`, power loss, or a
machine reset bypasses cooperative close, so an existing MP4 may still be incomplete.

Synthetic NVIDIA qualification on the current RTX 4050/driver 595.84 decoded 1920x1080@30 H.264
through `nvh264dec` with CUDA-memory output. An unconstrained fast-loop stress drove the dedicated
decoder engine to 78–100%; this is deliberately not a normal realtime workload, not whole-GPU
utilization, and not an operating ceiling. Real Left/Right WIRIS RTSP decoder utilization,
first-frame latency, sustained frame rate, drops, thermal behavior, and fallback recovery remain
the field acceptance evidence.

The decoder truth/failure-isolation tests use fake GStreamer boundaries and spawned local workers.
They prove current-generation gating, error routing, per-side override, terminal-exit suppression,
and UI projection in software; they do not prove that a physical WIRIS/NVDEC failure will recover
within the field timing limits. Keep that item open until dual-camera hardware qualification.

Camera JSONL rows describe GUI-delivered frame observations, while MP4 may be encoded pass-through
or decoded-frame recording. No exact 1:1 frame index is currently proven;
`mp4_frame_index_contract=not_available` is the authoritative limitation. Use the exact native PNG
bound to a terminal pair when pixel-level replay is required.

Nominal intrinsics fallback uses an ideal centered pinhole and the WIRIS 1280x720 zoom-index focal
table scaled independently to the current native width and height. It does not model per-camera
principal-point offset, lens distortion, crop, temperature, focus, or unit-to-unit variation. Use
it to keep estimated geometry workflows available, not as precision or calibration evidence.

The simple Calibration surface intentionally omits exact photo selection and the full reason-code
table. When automatic selection is unsuitable, a quality result is `WARNING`/`FAIL`, or an operator
must approve a warning, expand `고급 캘리브레이션` and complete the existing detailed workflow;
the simple surface does not provide an override.

Historical encoder-profile experiments are not an automatic operating contract. Constant-quality
mode can increase bandwidth, especially during fast full-frame motion, and saved reference values
may differ from physical readback. The current dual-camera 720p stream requires pre-run readback and
field qualification; RTSP connection actions deliberately do not reconcile or mutate those fields.

The five-second decoded-rate window stabilizes the operator value; it is not a substitute for a
throughput test. If the value remains low after warm-up, inspect the status/log for the active
GStreamer decoder and test the selected RTSP transport independently. UDP can lose RTP packets on
the camera link, while TCP can expose camera-side RTSP service resets or startup codec-header gaps;
changing transport alone is not an approved performance fix. Restart the Workstation after changing
the decoder, transport, or pipeline code so existing child processes do not retain the prior setup.

A sender that never emits H.264 IDR NAL units has no software-recoverable random-access boundary.
Workstation can prevent its dependent or concealment frames from reaching Preview, YOLO, native
evidence, and the encoded recording tee, but it cannot reconstruct missing reference pictures or
turn a non-IDR intra picture into an independently decodable frame. Such a side remains unavailable
after the bounded integrity wait; inspect the camera's encoder readback and correct it outside the
automatic RTSP connection path.

On 2026-08-27, an operator-approved exact `GEIR VISIBLE=1` readback on both FW 2.18.72 cameras still
produced zero type-5 NAL units in 15-second elementary captures: Left had 206 AUs with nine NAL1
intra AUs, Right had 112 AUs with three NAL1 intra AUs, and `ffprobe` marked every frame
`key_frame=0`. Sequential SDK reboots were later completed. Left rebooted with IDR reset to 0;
Right rebooted with Ethernet mode off and profile `0/17/16/0`, then was restored to the approved
`GETH=TRUE`, `0/1/30/1` state. On both models, fresh streams and an additional `SEIR VISIBLE 1`
issued while RTSP was active still produced zero NAL5. That sender output remains unrecoverable by
this software gate and now requires Workswell firmware/vendor correction. Zero IDR also blocks the current-generation frame evidence needed
for STUS round two, so camera output, clock convergence and portable MP4 remain one shared CRITICAL
hardware blocker.

The type-owned append model deliberately provides no transactional “all evidence for one
Measurement” package. A power loss or queue overload can leave some streams present and others
missing; inspect `Data > Status`, `dropped_records`, and per-stream errors before later analysis.
This is the accepted tradeoff for never losing a measurement opportunity to an unfinished save.

The always-enabled Measurement command is request availability, not evidence readiness. It may
still reject invalid/identical RTSP URLs or an already-running transient run, and it queues while an
actual camera owner is busy. Inspect the transient Run detail rather than looking for a Session or
save-recovery state.

Content-addressed Calibration profiles remain durable identities. Alignment selection is now
app-lifetime state backed by append-only `alignment.jsonl`; it is not automatically restored after
restart. Save any reusable extrinsics profile separately when it must survive restart, and use
`Reset Alignment` when the current in-memory state should be cleared without deleting stored data.

`Data` has only `Status` and `Capture` tabs. Legacy Session folders are available through
`Open Legacy Recordings Folder` in the file manager; Workstation does not catalog, verify, recover,
or convert them automatically.

The launcher's `--check` intentionally rejects an interpreter outside the repository `.venv` and
an installed environment outside `requirements/runtime.txt`. On a host with NumPy 1.25+ or
pyqtgraph 0.14+, back up and recreate the canonical venv with the supported constraints (currently
NumPy 1.23.5–1.24.x, OpenCV 4.8–4.11, and pyqtgraph 0.13.x) instead of bypassing the check. PyAV is optional and must
be installed from root `requirements/optional-pyav.txt` only when that backend is selected.
The manifest also fixes setuptools to 77–79: Torch 2.13 requires at least 77 while the supported
Ubuntu/ROS system-site `colcon-core` requires a version below 80.

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

Older/imported calibration metadata without timing-decision fields remains a
`LEGACY_UNVERIFIED` stereo candidate for compatibility; this is not proof of synchronized exposure.
An explicit `stereo_pair_saved=false` or `pair_persistence_allowed=false` always wins and keeps the
matching images K,D-only. Malformed capture metadata is not silently promoted and the existing
zoom/evidence gate blocks calculation until it is repaired.

Workstation no longer generates `review.xlsx` or typed Parquet automatically. Later analysis must
read the fixed JSONL streams, interpret append/replace operations by `_record_index`, retain signed
nanosecond timestamps, and join records by time and transient generation where appropriate. Any
spreadsheet or Parquet conversion is an explicit downstream task and is not an evidence authority.

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

Solved optical-pose Drafts remain application-memory objects unless their extrinsics profile is
explicitly saved. A Measurement started before restart can freeze the latest successful Draft even
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
side are retained; no completeness state or package gate is created.

There is no automatic manifest hash chain, electronic signature, organization PKI integration,
bundled source tree, virtual environment, or container image for the new append streams. Record
hashes remain useful for local artifact identification, not legal non-repudiation.

Video files are closed by their camera writer without putting full validation inside the bounded
EOS/child-stop budget. A background package worker consumes the immutable recording backend and live
integrity evidence, hashes the stable bytes and runs duration-aware bounded MP4/H.264 validation;
until it passes, the recording is not usable. Measurement stop and the next run do not wait for the
full probe. Validate sustained RSS, blocked removable storage,
USB serial timeout, dual-camera recovery, camera FTP, and automatic ULog collection with actual
field hardware before treating those boundaries as field-qualified.

There is no Measurement restart recovery. An interrupted run ends with the process; existing bytes
remain in the fixed streams, and a new app run starts from transient `IDLE` without inspecting old
Session directories.

NTRIP mountpoints are entered manually; the Workstation does not currently browse a
caster's source table. Provider-specific authentication methods beyond standard Basic
authentication are not supported.

There is no built-in NTRIP Password. A session Password has highest priority and is never written
to settings/evidence; the OS keyring is the only persistent password store. On a host without a
usable keyring backend, enter the Password again for each application run.

If the status is `CONNECTED_NO_DATA` or `STREAMING`, verify the mountpoint and caster's
actual RTCM output. If it is `RTCM_STALE`/`STALE`, inspect the configured read/correction age,
the caster stream, and the downstream Base/Pixhawk route separately; a live TCP session can
exist while valid RTCM or downstream injection is unavailable.

The 4,096-envelope RTCM ingress capacity, 96-request Pixhawk injection queue, 2-second
stale-injection limit, 0.5-second UI snapshot interval, 4,096-row bulk data queue, 256-row priority
data queue, and 15 Hz preview cap are currently code-level operational limits rather than Settings
controls. Bulk overload drops older queued bulk rows and increments `dropped_records`; priority
results/events have a separate reservation but can also drop if their own queue is exhausted. Unit
and simulated end-to-end tests verify bounded behavior, but long-duration dual-camera +
live-caster + three-Pixhawk operation still requires a field soak test before qualification.
For this Ethernet MAVLink mitigation specifically, software tests are insufficient for a resolved
claim. Required hardware evidence is three independent cold boots with at least 10 minutes each,
one dual-P1/P2 60-minute full-load soak with RTCM and normal Workstation load, symmetric one-side
isolation, and verified-USB recovery with another 10-minute post-reboot qualification. Until all
of those pass, do not describe the field issue as resolved or update `Pixhawk.md` with a remedy.
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

- 2026-08-29: Documented the observed NVIDIA `595.84`/Linux 6.8 `s2idle` resume failure with two
  active NVDEC workers: paired Xid 31 MMU faults, UVM fatal error, Xid 154 reboot requirement,
  GStreamer CUDA-init failure, truthful CPU fallback symptoms, reboot-only recovery, and the
  current stop-Preview-before-suspend operational workaround.

- 2026-08-29: Made Measurement observation-first. Start immediately opens a run with zero or more
  sources; camera, Pixhawk, Base GNSS/NTRIP, Target and recording join independently. Added manual
  camera rejoin with non-overwriting MP4 segments, bounded FIFO retry for essential structured
  evidence, presence-only `SAVED/PARTIAL/FAILED`, and removed artifact/integrity persistence from
  current- and next-run admission.

- 2026-08-27: Reproduced the operator runtime in a separate official Ubuntu 22.04 KVM guest and
  made the external model provisioning step explicit. Added the missing system `python3-cairo`
  dependency so the fresh canonical venv also passes `pip check`.

- 2026-08-27: Added the implementation-independent native evidence qualifier and retained
  descriptor `timestamp_s` in canonical detection snapshots. A real 88-pair/176-PNG audit verified
  487,531,968 bytes with zero failures; K,D/zoom/R,t/correspondence/telemetry/STUS BLOCKED states
  were also verified not to disable global Measurement Start.

- 2026-08-27: Updated both application camera-interface defaults to isolated `10.0.0.11/32` so a
  settings reset cannot recreate the Pixhawk NIC's `.10/24` subnet on the camera NIC. Legacy
  automatic startup/receive/Preview/restart/recording captured zero SDK control packets.

- 2026-08-27: Closed the 12-item repository portability residual and reproduced both launchers from
  a fresh same-host checkout/new venv under isolated HOME. Added the explicit setuptools 77–79
  intersection discovered by that recreation. A separate clean Ubuntu OS/VM remains required.

- 2026-08-27: Normalized PX4 `NaN`/infinite optional telemetry to JSON `null` with exact
  `nonfinite_source_fields` evidence, preventing Measurement-start callback traceback storms while
  retaining strict storage validation. Full Pixhawk disconnect now stops its GUI-owned receive
  timer on the GUI thread before worker transport closure, eliminating the shutdown
  `QObject::killTimer` warning.

- 2026-08-27: Made GStreamer camera startup and RTP-loss recovery fail closed on a parsed H.264
  IDR NAL plus its exact decoded PTS. Non-IDR/non-delta AUs cannot reopen the encoded recording
  tee, and decoder concealment cannot reopen Preview, YOLO, or native evidence. Zero-IDR senders
  terminate once as unavailable without an automatic reconnect storm, and a post-LIVE failure
  clears only that side while preserving the healthy peer. No encoder setting is mutated.

- 2026-08-27: Made recording start transactional, blocked new Manual record admission during
  shutdown, and routed direct panel `deleteLater()` through the normal bounded lifecycle so an
  active interface-worker `QThread` cannot be destroyed with its parent. RTSP reactivation, eth0
  probe timeout, and legacy recording-barrier retries now use panel-owned generation-guarded timers.

- 2026-08-27: Removed a Pixhawk Connect GUI-freeze path exposed by a saturated desktop
  accessibility service. MAVLink worker/status/completion callbacks are explicitly queued to the
  GUI owner, progress cannot repolish topology buttons before completion, receive-timer start is
  owner-thread guarded, and dynamic roles no longer force synchronous QStyle unpolish/polish.
  Successful RTCM progress logs are side-throttled to one per five seconds while exact counters and
  errors remain unchanged.

- 2026-08-27: Hardened RTCM resynchronization after a corrupt length/CRC candidate so a valid frame
  enclosed later in the same byte stream is recovered instead of discarded with the false frame.

- 2026-08-27: Made native-frame shared memory recoverable across abnormal termination. Rings retain
  shared tracker ownership and carry a Workstation/PID name; startup reclaims only strict same-UID
  dead-owner entries. A real active-Measurement `SIGKILL`/restart left no child/socket/ring, did not
  adopt the old run, kept `측정 시작` enabled, and created a distinct new run ID.

- 2026-08-27: Closed the launcher/runtime environment against ambient Python, ROS loader, GI and
  GStreamer plugin/system/scanner paths, and made the deployment doctor importable before the
  canonical venv's third-party requirement parser exists.

- 2026-08-26: Made both launchers disable Python user-site packages and ambient Python/library
  paths. The runtime doctor now validates requirement version ranges as well as presence, and the
  OpenCV range is capped below 4.12 to preserve the calibrated NumPy 1.24 stack.

- 2026-08-26: Aligned application close with the camera's 12-second cooperative-stop and
  nine-second MP4 EOS windows. Session-save and OZ capture now gate close, per-side finalization
  evidence must enqueue before the data writer closes, released retired native rings are swept,
  and forced cleanup reports verified thread/process/socket/shared-memory survivors instead of
  returning unconditional success. Settings flush precedes the final DataStorage close barrier.

- 2026-08-26: Made decoder activity current-frame evidence instead of a `PLAYING`/status inference.
  Added RTSP/source, recording-output, decoder-health and generic-pipeline error classes; only direct
  NVDEC health authorizes Auto CPU fallback. Runtime overrides/reasons are per side, preview fallback
  leaves the healthy worker running, recording fallback stops only the failed side without reopening
  its MP4, and intentional terminal child exits are no longer respawned as crashes.

- 2026-08-26: Closed the exact-frame GUI-delay window. Detection pairing now expires stale and
  no-detection state, rejects stream-generation rollback, and reserves both selected native ring
  slots before GUI polling. Native evidence validates redundant `FrameKey` fields and the incoming
  descriptor hash against the pinned descriptor before either PNG is written; tampering, writer
  failure, result replacement, stop, and reconnect paths release their leases fail-closed.

- 2026-08-26: Serialized dual-camera STUS as Left exact-OK ACK then Right, retained the recording
  interlock, and replaced one-shot RTCP/NTP verification with continuous 1 Hz health reevaluation.
  Current frame/SR age, SR counter regression, 50 ms pair delta, 100 ms SR-PC offset, 7 s default SR
  age, and 0.5% clock-rate limits now fail the visible sync state instead of leaving stale green.

- 2026-08-26: Fixed dual-Pixhawk Ethernet startup by seeding the named peer on each exclusive
  `udpin` socket and sending GCS `255/190` heartbeat before inbound identity. Identity now requires
  the expected-sysid HEARTBEAT, missing identity has a bounded 10-second `NO HB` state, and automatic
  bulk PARAM refresh waits for LIVE qualification. A real current-boot dual-Pixhawk run crossed the
  prior 5.5-minute failure interval and reached LIVE at 610 seconds.

- 2026-08-26: Rearmed the per-side Stock MAVLink stream profile after a physical/remote PX4 reboot
  that leaves the Workstation UDP socket open. Two increasing `ATTITUDE.time_boot_ms` samples at
  more than 2 seconds behind the prior epoch confirm a reboot; regressions up to 2 seconds and uint32
  wrap do not. This also covers a rapid reboot before the prior boot clock reaches 10 seconds.
  The same host-listener generation is retained, old-boot identity/qualification is invalidated,
  and the next expected-sysid HEARTBEAT sends one side-locked profile burst for only that Pixhawk.
  Delayed prior-boot ATTITUDE is not published downstream. No reboot, shell, parameter, or persistent
  `MAV_2_*` setup command is introduced by this recovery path.

- 2026-08-26: Bound session NTRIP credentials to one caster/mountpoint/username, delayed keyring
  persistence until HTTP 200, made 401 an immediate `HTTP_AUTH_FAILED`, and exposed the selected
  credential source without recording the secret.

- 2026-08-26: Made camera backpressure qualification fail closed on missing exact frames, writer
  ACK/close failure, child/shared-memory leaks, queue/ring violations, Stop latency, and RSS
  growth/high-water/slope thresholds. The report now preserves its threshold/check matrix and raw
  RSS samples; this synthetic harness remains separate from the required real Qt/dual-camera soak.

- 2026-08-26: Replaced camera-thread launch success with a current-generation first-decoded-frame
  readiness gate. Added bounded dual/partial/no-frame rollback and independent per-side MP4
  `STARTING/RUNNING/PARTIAL/FAILED` evidence based on positive file growth.

- 2026-08-26: Removed automatic WIRIS encoder-profile writes from startup, Auto-Connect, Enable and
  Reactivate. These paths now send `SETH TRUE` only after a `GETH=FALSE` readback; stored encoder
  values remain manual-reference data and cannot silently mutate a connected camera.

- 2026-08-26: Made Auto the approved GPU-first Camera Preview policy and exposed requested versus
  actual H.264 decode truth without a new dashboard. Existing Camera `Decoder LEFT/RIGHT` now
  distinguishes GPU NVDEC, explicitly selected CPU, and CPU fallback; CAM status detail/tooltip
  carries fallback evidence; Diagnostics Performance reports NVIDIA identity, NVDEC availability,
  selection candidates, and per-side actual decoder. Worker count, queues, stream rate/quality,
  recording paths, and explicit NVIDIA fail-closed behavior are unchanged.

- 2026-08-26: Implemented the software side of P1/P2 Ethernet MAVLink stall mitigation without
  changing PX4 firmware, parameters, or SD-card configuration. Added explicit GCS `255/190` and
  bounded 1 Hz heartbeat, a consumer-backed Stock stream profile, per-side HB/ATT/ANY liveness,
  10-minute qualification, truthful ATTITUDE-stale versus all-RX-stalled status, no false host
  reconnect, RTCM send-boundary fail-close with separate `blocked_stall` evidence, and
  operator-confirmed unique-sysid USB reboot with immediate worker revalidation. Hardware
  3-cycle/60-minute/isolation/recovery acceptance remains pending; `Pixhawk.md` is intentionally
  unchanged. Root-venv verification passed 275 focused tests and the full Workstation suite
  (`1224 passed, 1 skipped`). Those 12 historical portability findings were later resolved or
  correctly classified by the 2026-08-27 R10 audit; the current policy reports PASS.

- 2026-08-26: Removed the operational Measurement Session/package lifecycle. Measurement now uses
  transient `IDLE/QUEUED/STARTING/RUNNING/STOPPING/ERROR` state and never consults prior Session or
  storage integrity to admit a run. Added one relative `measurement_data_root`, fixed type-owned
  JSONL/RTCM files, reserved result/event queues, per-stream failure/drop status and retry,
  creation-time collision-free names for MP4/images/native PNG/ULog, app-lifetime Alignment with an
  explicit Reset event, and best-effort target telemetry in `px4_drone.jsonl`. Replaced Sessions
  navigation with `Data > Status/Capture`; legacy recordings are file-manager-only. Automatic save,
  seal, recovery, Parquet, and `review.xlsx` generation are detached from the new workflow.

- 2026-08-26: Made the repository root `.venv` created with `--system-site-packages` the only
  supported Workstation interpreter. Added the read-only Ubuntu/GStreamer doctor and strict
  `--check`, removed system Python fallback, separated optional PyAV, pinned camera children to
  `sys.executable`, made Auto's software degradation explicit, and made NVIDIA mode fail closed.

- 2026-08-25: Unified Legacy/Workstation recorder ownership, single-owner PX4 MAVLink setup,
  exact duplicate suppression, secure NTRIP credentials, TCP+Auto `nvh264dec` runtime fallback,
  and latest-only preview behavior. Removed the plaintext NTRIP password default.

- 2026-08-25: Prevented intermittent NTRIP authentication from using an untouched masked
  deployment Password ahead of a saved keyring or configured environment credential; only an
  explicit operator edit now has session-level priority. Made each MAVLink `udpin` endpoint
  exclusive so another application or leaked worker produces an actionable port-in-use failure,
  and added bounded `ATTITUDE` interval recovery while heartbeat remains live. Stock Stable
  P1/P2 shell readback showed one Ethernet instance per board with clean `extras.txt`; independent
  four-second captures received ATTITUDE `169/167`, GPS1 `29/28`, and GPS2 `28/28`. Runtime check
  passed; full Workstation suite: `1157 passed, 1 skipped`.

- 2026-08-24: Replaced the default Calibration presentation with a StereoWIRIS-style
  `촬영 → 계산·적용 → 프로파일` workflow and moved the existing session/file selection,
  setup, diagnostics, quality evidence, and K,D/R,t details under a collapsed
  `고급 캘리브레이션` disclosure. All simple actions route to the existing Workstation
  controllers, and quality-gated duplicate Apply controls share the same enabled state. Full
  Workstation suite: `1137 passed, 1 skipped`.

- 2026-08-24: Removed all presentation-level disabling of the Measurement start/stop command.
  The Measurement button and its shared QAction now remain consistent and always clickable across
  lifecycle, saving, and manual-recording states, while backend RTSP, writer, single-interval, and
  package-integrity checks continue to reject unsafe transitions with evidence. Full Workstation
  suite: `1136 passed, 1 skipped`.

- 2026-08-24: Preserved timing-rejected manual, Auto, and FTP checkerboard captures as independent
  per-camera K,D observations. Capture metadata and UI now identify `INTRINSICS ONLY` files, Auto
  retains each new pose without increasing its valid-stereo counter, and both full calibration and
  `R,t only` filter explicit timing rejections after automatic or manual selection. Legacy photo
  sets without timing-decision fields remain compatible as `LEGACY_UNVERIFIED`. Full Workstation
  suite: `1134 passed, 1 skipped`.

- 2026-08-24: Closed the Workstation global-review regressions in worker deferred start, structured
  persistence-failure propagation, Target Drone/ULog/terrain finalization, crash/shutdown flush,
  RTK failure classification, Drone Multi-view editor rollover, and asynchronous native-frame row
  completion. Quaternion transforms now normalize each attitude once, repeated Pixhawk/YOLO error
  reporting shares one message-aware throttle, and PARAM_VALUE polling shares one receive loop.
  Full Workstation suite: `1120 passed, 1 skipped`.

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

- 2026-08-24: Added deployment NTRIP UI/runtime fields. The 2026-08-25 hardening removed the
  historical plaintext password convenience default; operator session/keyring/env credentials
  remain excluded from settings/evidence JSON.

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
