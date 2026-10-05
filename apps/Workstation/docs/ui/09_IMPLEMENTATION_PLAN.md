# StereoWIRIS Workstation UI Implementation Plan

> Superseded for Measurement persistence/navigation on 2026-08-26. The active sessionless plan and
> contracts are in `docs/changes/20260825-long-interval-measurement-workflow/`. Historical Session,
> package, finalization, recovery, and workbook items below describe the retired implementation and
> must not be used as current operator behavior.

- Status: Implemented — GO WITH CONDITIONS for the software migration slice; hardware and long-duration finalization remain open
- Depends on: 00–08 design documents
- Primary constraint: preserve functionality, keep legacy UI available, and isolate the Workstation owner while removing unused Workstation workflows

Implementation gate (2026-08-08): the launcher selector, shared close coordinator, thin
backend adapter, source-backed status model, Workstation shell, and copied standalone owner
are implemented on `audit/20260808-audit-prompt`. The legacy UI remains the default and the
Workstation uses its own copied `standalone_gui.py`/`standalone_backend.py` owner. See
[`11_DESIGN_AUDIT_RESULT.md`](11_DESIGN_AUDIT_RESULT.md) for the implementation evidence
and remaining validation conditions.

### 2026-09-02 GNSS baseline-center F/R/D/Rg slice

Telemetry now derives one immutable, runtime-only antenna-midpoint local-level FRD result from
selected P1/P2 and Drone `GpsPositionState`. A pure transform owns explicit NED input,
horizontal-P1→P2 Right, NED Down, Right×Down Forward, metres, Rg, 2.0 s age, operator-selected
1000 ms receive-spread, fix provenance and WAITING/CURRENT/STALE state. The Drone Telemetry
controller publishes the mapping and existing ideal disparity consumes the same Rg/readiness.
One fixed non-selectable table below `Live position` renders it in place. No command, setting,
persistence/schema, device/protocol, worker, queue, Measurement, Recording or canonical camera
frame changes are included.

### 2026-09-02 GNSS baseline-center recording slice

The follow-up persistence slice adds one fixed best-effort bulk stream and no worker/queue. After
the existing raw primary Drone GPS row, Sensor presentation calls a public Recording-owner method;
while recording, it snapshots selected P1/P2 plus exact Drone, reuses the pure estimator, builds a
<=4 KiB self-contained CURRENT/STALE/WAITING row, and non-blockingly appends
`gnss.baseline.center`. MeasurementDataStore remains sole writer and Data Status owns failure/drop
truth. Latest XLSX adds exact-generation `GnssBaselineCenter`; raw PX4 streams, canonical
Measurement fields, legacy Parquet, settings, commands and device behavior remain unchanged.

### 2026-09-02 guided field-topology slice

The Pixhawk workspace now exposes one fixed, fail-closed preset for P1 D30 Radio, P2 Ethernet, and
Drone DU Radio. A pure immutable state machine and atomic journal separate operation truth from Qt;
a single controller owns command enablement, confirmation, commit-last settings, restart recovery,
cancel, and guided rollback. One bounded worker owns serial/MAVLink/MAVFTP operations, revalidates
unique sysid/isolation/disarmed evidence immediately before writes, performs exact SiK NETID
readback, captures/restores the P2 PX4 parameters and SD files byte-for-byte, and verifies P2 UDP
identity plus ATTITUDE. Lifecycle shutdown cancels and waits for that worker.

Implementation verification is sliced into pure transition/parser/journal tests, worker boundary
tests, controller persistence/restart/rollback tests, fixed-geometry command/UI tests, affected
Workstation regressions, full pytest, launcher preflight, and an offscreen render inspection. Real
radio/Pixhawk mutation and three-board cold-boot qualification remain operator-run hardware gates;
the automated suite must not claim them.

### 2026-08-29 Pixhawk network assignment slice

Pixhawk `Find network` uses a bounded read-only worker to inventory physical wired links, global
IPv4 addresses and side-specific kernel `dev/src` route evidence. The compact workspace table lets
the operator select one NIC/address candidate; independent P1/P2 cards show each assigned
State/NIC/Local IPv4/Endpoint/Route. Assign persists the last side selection with additive settings
fields. Connect preflights each disconnected side independently; it never
mutates host or PX4 networking. Existing custom endpoint/USB paths, expected sysid, 10-second
qualification, stalled-side RTCM gating and verified-USB recovery remain authoritative.
Same/different and Camera-shared NIC choices are accepted without a UI topology warning; field
stability remains a separate hardware evidence boundary.

### 2026-08-26 Ethernet MAVLink stability slice

The 2026-08-29 network-role amendment adds one bounded host-role coordinator above existing Camera
and Pixhawk public owner contracts. It owns explicit confirmation, stable physical identity,
role-set union/delta, exact app-owned NetworkManager provenance, target-first transaction/readback,
compensating rollback and commit-last persistence. Existing Camera stream and MAVLink subscriber
owners remain unchanged. Verification covers one/many NICs, shared roles, authorization/readback
failure, active affected-owner reconnect, unrelated-NIC isolation and mapping-lock rejection.

The Workstation software now owns explicit GCS `255/190` identity and bounded 1 Hz heartbeat,
consumer-backed Stock stream requests, per-side HB/ATT/ANY liveness with a 600-second qualification,
truthful ATTITUDE-only versus all-RX stall status, no false host reconnect, RTCM enqueue/send-lock
fail-close with separate `blocked_stall` evidence, and operator-confirmed verified-USB side reboot.
PX4 firmware, `MAV_2_RATE`, parameters, and SD-card configuration are unchanged by this slice.

The P1 IMU delivery amendment tracks actual `SCALED_IMU/2/3` receive timestamps, retries only
missing/stale standard streams with the existing bounded attempt/cooldown policy and side send lock,
and projects `WAITING/LIVE/STALE/PARTIAL` into the existing table. It adds no worker, automatic USB
command, persistent PX4 write, coordinate conversion, or new screen.

Calibration PARAM snapshots are manual-only. Completing the 10-second `LIVE` qualification sends
no PARAM request and causes no unrequested post-command qualification reset. Existing P1/P2 full
refresh, Advanced MAG/attitude refresh, and reset/apply/save verification remain on the original
side-specific command worker.

### 2026-08-31 Pixhawk Radio-default transport slice

P1, P2 and active Drone now share one role-configured MAVLink owner with fixed sysids 1/2/3.
Settings migrate once to Radio selection while preserving Ethernet configuration and leaving Radio
devices unassigned. Per-board Radio/Ethernet controls, endpoint ownership locks and explicit
Connect/Disconnect actions replace implicit startup/recovery connection paths.

Radio uses critical-only demand during qualification and deterministic 30/15-second optional
restoration. `RADIO_STATUS` is diagnostic-only. Connect no longer launches persistent MAV_2 setup;
that setup and all parameter/shell/reboot work are explicit commands.

Offset-aware MAVFTP supports upload, download, cancellation, restart and verified resume using
remote prefix readback, remote size/CRC32 and local size/SHA-256/CRC32. Firmware may be staged over
Radio, while actual flash uses a hash-pinned official PX4 uploader, verified direct USB identity,
bootloader board matching, global serialization and postflight sysid verification. No telemetry
transport reconnects after flashing.

The 2026-09-01 Discovery follow-up gives USB/network search one replace-only transient snapshot,
one row per physical USB serial endpoint, identity-enriched role presentation, and revision-driven
selection/feedback reset. The existing workers, explicit Assign/Connect boundary, saved settings,
device ownership, and CPU/GPU topology remain unchanged.

Software completion does not close the field condition. Required hardware evidence remains: actual
firmware/sysid/parameter inventory, three independent cold boots of at least 10 minutes, one
dual-link 60-minute RTCM/full-load soak, symmetric one-side isolation, verified-USB recovery, and a
new 10-minute post-reboot qualification. `Pixhawk.md` must remain unchanged until all gates pass.
Automated software evidence currently includes 275 focused passes and a full Workstation result of
`1224 passed, 1 skipped`. The repository-wide portability command still fails on 12 pre-existing,
out-of-scope paths; no file in this ETH change packet is reported.

### 2026-08-26 NVIDIA decoder status slice

Camera Preview keeps the existing one-worker-per-side pipeline and Auto fallback lifecycle. The
persisted operating mode is Auto (NVDEC first, one bounded software fallback). Requested/effective
selection, actual per-side decoder, fallback reason, and static NVIDIA identity are projected into
the existing Camera status, CAM tooltips, and Diagnostics Performance; no new global badge,
worker, queue, rate, affinity, clock, power, or quality policy is introduced. Synthetic NVDEC is
automated-real evidence; actual dual-WIRIS RTSP remains a hardware/manual acceptance condition.

### 2026-09-03 RTP/H.264 input metrics UI slice

The existing GStreamer RTP sequence and H.264 parser observations now emit five-second rolling
rates to the existing Workstation stats payload: received RTP packets/s, sequence-gap missing
packets/s, estimated loss percentage, parser access-units/s, and decoded Hz. Camera workspace
details and Diagnostics Performance render these per-side measurements in fixed-height text slots;
missing or non-GStreamer metrics render as `--`. This slice is observational only and does not
change camera encoder settings, IDR policy, RTSP transport, queue policy, decoder selection, or
measurement gates.

### 2026-08-29 tolerant Measurement and decoded-NVENC recording slice

Camera admission is permanently tolerant: current decoded frames feed Preview, YOLO, native
evidence, and official Measurement while `INPUT UNVERIFIED` remains a separate visible/persisted
dimension. Existing timing, calibration, correspondence, and geometry gates are unchanged and may
still produce `VALID`. Automatic MP4 uses a bounded oldest-drop decoded-NV12 branch and isolated
FFmpeg `h264_nvenc` lossless writer. Recorder failure cannot stop YOLO/distance; no CPU encoder
fallback, bitrate cap, file-size cap, duration cap, or warning burn-in is introduced. Left-camera
hardware and synthetic pixel-exact NVENC evidence are complete; dual-camera field distance remains
a hardware condition.

### 2026-08-29 observation-first admission slice

Measurement now opens its transient generation/time boundary immediately with zero or more sources.
Camera, Pixhawk, Base GNSS/NTRIP, Target and recording publish independent source transitions;
manual camera recovery rejoins with a new MP4 segment. Essential structured records have a bounded
retry reserve, while media integrity and artifact persistence never gate the current or next run.
No Session/package, catalog, revalidation UI or new media worker is introduced.

### 2026-08-29 Mountain suggestion quality-separation slice

The existing `Use suggestion` control now treats Mountain detector confidence as quality evidence,
not copy permission. A shared pure validator and the controller enforce current frozen identity,
paired exclusions, native bounds, equal count, minimum-four, uniqueness and distribution before
copying filtered points. The UI shows `COPY READY/BLOCKED` in the existing status strip. At that
slice Horizon was unchanged. No direct/automatic Solve, Apply, layout, worker, device or schema
migration was introduced.

### 2026-08-29 Horizon suggestion quality-separation slice

Horizon now uses the same quality-versus-copy split as Mountain. Detector state/confidence remains
visible and persisted, while shared pure/UI/controller validation owns current identity/dimensions,
exactly two finite in-frame endpoints per side and 25%-width separation. The existing group shows
`COPY READY/BLOCKED`; explicit Horizon Solve/Apply, layout, worker and device behavior are unchanged.

## 1. Migration Strategy

> New UI, copied owner, shared domain modules; Legacy retained as an independent runtime.

Lifecycle sequencing is neutral in `sasl_common/lifecycle.py`. Workstation settings and
recordings are rooted at `.local/workstation_data/`, while Legacy keeps `apps/StereoWIRIS/` storage;
the two application data stores are not read through each other.

Legacy and Workstation code paths should coexist, but one operational UI per application run is the safe default until backend ownership is audited.

## 2. Branch Strategy

Recommended branch: `feature/ui-workstation-refactor`. Keep main stable.

## 3. Runtime Selection

- Runtime:
- `python main.py --ui legacy`
- `python main.py --ui workstation`

Default remains legacy. `run.sh` forwards the same selector and documents the mode.

## 4. Candidate Structure

`apps/StereoWIRIS/wiris_stereo_distance/`
- `legacy_gui.py` (Legacy panel implementation; `gui.py` is its compatibility alias)
저장소 루트 `apps/Workstation/`
- `standalone_gui.py` (Workstation copy of the panel module)
- `main_window.py`
- `standalone_backend.py` (Workstation copy of the adapter)
- `evidence.py`
- `command_registry.py`
- `status_model.py`
- `navigation.py`
- `components.py`
- `inspector.py`
- `workspaces.py`
- `runtime_paths.py` (Workstation data/settings/recordings ownership)

`sasl_common/`
- `lifecycle.py` (UI-neutral ordered retry coordinator)

Do not create empty abstractions merely to match a diagram.

## 5. Phase 1 Scope

Implemented:
- WorkstationMainWindow
- Menu Bar
- Compact Toolbar
- Navigation Tree
- Central Workspace Host
- Bottom Diagnostics Dock
- Global Status Bar
- source-backed System/Camera/Pixhawk/GNSS/Log/Measurement/Performance diagnostics
- explicit Overview, Camera, and Measurement workspaces
- explicit Time / Sync workspace for PC UTC, WIRIS RTC, and RTCP/NTP stream evidence
- decoder-independent parallel RTCP observation, ordered Left-then-Right STUS mutation,
  Manual/Auto scheduling, logical clock epochs, and separate Clock/Frame/Media truth; SR-PC is
  diagnostic-only and the relative default bound is 100 ms
- Settings-preferred Workstation live-distance pairing with app receive as default and mandatory
  fallback; healthy VERIFIED/current same-epoch RTCP may select pairs, the effective basis has a
  100 ms cap, and fallback provenance alone cannot degrade distance
- Calibration Live/Auto pairing on decoded-frame app-receive monotonic with its existing editable
  gate; persisted `HOST_RECEIVE_PROXY` may admit a pair but never verifies exposure or precision,
  NTP remains record-only, and FTP retains its separate CPTR-dispatch proxy
- Calibration Auto-only NAL1/non-delta AU classification with exact AU-to-decoded PTS propagation,
  current-generation native FrameKey pin/read, bounded provenance cache, and no timestamp-near
  fallback; other camera consumers and manual/FTP/OZ capture remain unchanged
- Measurement upstream evidence: left/right frame proxies, detection, synchronization,
  GNSS/RTK baseline, recording, and FPS
- source-backed Alignment/Pixhawk/GNSS attitude, receiver, and NED baseline tables
- source-backed Calibration profile/intrinsics/stereo R/T/validation projection
- source-backed Recording/Review/Export session, artifact, finalization, and sample-count projection
- source-backed Camera Detection configuration and target/bbox-pixel projection
- protocol-ordered camera readiness actions, Pixhawk/GNSS/Alignment local action tabs, and Calibration setup/capture/validation tabs
- Measurement Run control and source-backed Current measurement quality projection
- recording safety gates for preview interruption, camera/Pixhawk mapping swaps, and premature session save
- editable Measurement settings and Time / Sync limits with Apply and Save routed through the existing owner
- editable Calibration setup (checkerboard geometry, zoom, validation root/mode/seed) with Apply, scan, and Save routed through the existing owner
- Alignment source scan, baseline sampling, profile save/load, and Pixhawk-side swap commands routed through the existing owner
- session catalog, artifact selection, CSV preview, folder/artifact open, and manifest error visibility without rewriting saved rows
- read-only, context-specific Inspector summaries for PX4/GNSS/Detection/Recording/Calibration,
  with paired Camera and Pixhawk status plus selected-value evidence
- phase-one direct operation commands for camera discovery/RTSP/control, Pixhawk discovery/MAVLink,
  GNSS Base serial/Survey-in/NTRIP, and calibration capture/run/save/load/apply/validation
- Advanced owner workspace now directly exposes live checkerboard preview/zoom, raw WIRIS SDK, PX4 MAG/attitude/Shell, MAVLink field-watch, baseline field browser, and session media/artifact review. The Workstation does not expose ROS2/uXRCE or still-image/FTP/EXIF clock workflows; those remain Legacy-only. The existing panel remains the technical resource owner; video opens through the system-associated player and workbook generation remains the existing session-save writer.

The Inspector owns no engineering settings store: it exposes runtime evidence,
context-specific read-only details, and the public measurement contract. Camera RTSP and
other operational controls remain in their Workstation workspace.

Core workspaces: Overview, Camera, Measurement, subject to audit feasibility.

## 6. Non-Goals

No full panel-method decomposition, no top-level-tab removal, no PyQt5→PyQt6 migration, no backend rewrite, no complete Inspector/calibration/diagnostics redesign, no plugin system.

## 7. WorkstationMainWindow Responsibility

Own shell/layout/navigation/presentation only. It shall not own measurement algorithms, camera communication, PX4 parsing, GNSS calculations, recording implementation, or calibration math.

## 8. Workspace Host

Use stable internal workspace IDs and controlled switching without recreating the shell. QStackedWidget is an acceptable Phase 1 candidate.

## 9. Legacy Fallback

Incomplete contexts may remain in Legacy UI or use safe adapters. Do not duplicate backend logic merely to eliminate legacy dependence early.

## 10. Command Reuse

For each command the copied `StandalonePanelBackend` calls the public handler on the copied
Workstation panel directly. The Workstation does not invoke Legacy `QPushButton.click()` and
does not create a second camera, measurement, Pixhawk, GNSS, calibration, or recording owner
inside one process. `CommandRegistry` is the single QAction path for menus, toolbar, shortcuts,
and contextual commands.

## 11. Status Model

`BackendStatusAdapter` projects only existing panel evidence into one `StatusModel`:
CameraPane status/frame age, MAVLink transport plus attitude samples, GNSS operation
state and receiver/baseline evidence, camera RTCP/NTP synchronization state, inference
stats, MeasurementAttempt/result fields, session spool/save state, and decoded FPS. Every
displayed status retains source and clock metadata; command intent is not treated as
success evidence.

The clock reducer uses a robust three-SR Left/Right relative window. One isolated observation or
unmatched frame does not create a false long-run failure. Frame pairing separately rejects
cross-epoch or over-100-ms candidates while preserving explicit drop/rejection evidence.

That Clock/Frame reducer supplies only the public eligibility snapshot used when the frozen
Measurement preference is `RTCP/NTP preferred`. The distance coordinator selects healthy RTCP or
exact app fallback once, then freezes requested/effective basis, delta and reason through every
method and saved row. App-default runs remain independent of Clock/STUS.

## 12. Diagnostics

System, Camera, Pixhawk, GNSS, Log, Measurement, and Performance tabs are implemented from
existing status/log/attempt/telemetry evidence. Structured tables use `QTableView` with
`QStandardItemModel`; phase-one camera/Pixhawk/GNSS operations use guarded command facades,
while advanced workflow-specific actions use typed owner facades and read-only values come from
`EvidenceAdapter`. The copied owner widget is retained only as an advanced technical fallback,
not as the normal Workstation entry point. The original Legacy widget is not imported by the
Workstation launch path.

## 13. Settings

Reuse existing engineering settings. UI-only preferences such as geometry/dock state/last workspace may be stored separately.

## 14. Threading

Preserve existing worker/thread architecture and Qt thread affinity. Do not multiply polling or consume one-consumer queues from two UIs.

## 15. Two-Day Candidate Plan

Day 1: branch, workstation package, shell/menu/toolbar/docks/workspace host, navigation,
overview, basic status, and launcher/legacy preservation.

Day 2: camera/measurement evidence reuse, recording/save command facade, diagnostics,
offscreen smoke, 1366/1600/1920 layout checks, and regression tests.

The implementation is still conditional for field operation: real RTSP, Pixhawk MAVLink,
GNSS/RTK, video-writer finalization, and long-running close behavior require hardware/manual
validation. Recording commands remain thin calls into the existing session owner.

## 16. Test Strategy

- static/import
- UI smoke
- functional regression
- Workstation shell/navigation/status/command/close contract tests
- hardware-dependent manual validation where required

Critical workflows: startup, camera, preview, measurement, recording, PX4, GNSS/RTK, settings, shutdown.

## 17. Performance / Shutdown

`LifecycleCoordinator` preserves ordered settings save, auxiliary worker stop, GNSS network/base
shutdown, camera/Pixhawk/runtime worker stop, and session-save completion. A pending step is
retried by the close event without starting a second backend owner. Real timeout/throughput
validation remains a field condition.

## 18. Migration Phases

Phase 1: shell + critical workspaces/status/diagnostics — implemented with a copied standalone backend facade.
Phase 2: deepen command postconditions and PX4/GNSS/Detection/Sync/Alignment source
adapters; the software-side settings, alignment operations, advanced owner controls, and
session review slice is implemented, while hardware postconditions remain open.
Phase 3: keep media review on the stable session manifest/evidence contract. Workstation
previews CSVs, opens binary/XLSX/Parquet/video artifacts unchanged, and invokes the existing
session-save writer for workbook generation; no second decoder or writer is introduced.
Phase 4: completed the Workstation-only removal of unused ROS2/uXRCE and still-image/FTP/EXIF workflows after parity tests; retain those workflows in Legacy until the separate Legacy retirement decision.
Phase 5: remove legacy only after full parity and operator validation.

## 19. Phase 1 Completion

Phase 1 is marked **GO WITH CONDITIONS** only after demonstrating one
operational UI per process, shared backend ownership, safe close/finalization, source-backed
status, explicit frame/unit labels, and regression coverage for legacy launch and core
measurement/recording paths.

- legacy launches
- workstation launches
- shell/navigation/status/diagnostics work
- camera/measurement/recording core workflows work if included by audit
- settings/measurement/coordinate/recording semantics unchanged
- no blocking regression

Current evidence: launcher parsing, fake-backend shell/status/command/close smoke, lifecycle
retry test, source projection tests, advanced-owner facade tests, selected legacy UI regressions,
**203 targeted tests**, full **627-test** suite, and compile/import checks pass. Measurement upstream evidence,
coordinate labels, source projection, settings delegation, alignment operations, session CSV
review, command-failure propagation, and Qt teardown are covered by the tests. The remaining
conditions are hardware/manual validation rather than a software full-suite failure.

## 20. Governing Principle

> Preserve the instrument while replacing the control panel.
