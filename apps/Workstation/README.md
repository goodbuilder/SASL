# SASL StereoWIRIS Workstation

Current code ownership and feature boundaries are documented in [Architecture](docs/ARCHITECTURE.md).
The four former god-module entrypoints are composition/compatibility boundaries; screens, panel
features, recording services and measurement services have explicit owners and contracts.

The [remaining-module follow-up](docs/changes/20261010-remaining-module-decomposition/status-audit.md)
has completed its fixed software decomposition scope: the 15 targets have explicit responsibility,
state and resource owners, and the generated MAVLink dialect has reproducible repository inputs.
The final isolated regression passed **2,696 tests with 1 unsupported-NVENC skip**, zero failures
and zero warnings. Its 649 Python source hashes and the delegate stub were checked after execution;
see the [verification ledger](docs/changes/20261010-remaining-module-decomposition/verification.md).

Calibration, Optical Pose and Pixhawk screens have component-owned widgets and selection state.
Capture/profile application, device administration, GNSS correction/evidence, and camera
recovery/rejoin/Stop have explicit runtime owners. Session finalization streams canonical rows
through temporary disk indexes; recording review uses bounded asynchronous queries. Existing
operator commands, confirmation gates, source identities, coordinate contracts and saved schemas
remain governing contracts. Hardware operation and memory qualification limits are separate from
software regression completion.

A separate 180-second synthetic media run passed its 22 unchanged checks. Its parent-process RSS
was 36.508 → 45.965 MiB, peak 50.922 MiB; this is not whole-process-tree or GPU qualification.
Storage scaling measurements and the preserved earlier 20-second failures are recorded with
source artifacts in the linked verification entry.

Current Measurement lifecycle, method-worker recovery, and evidence ownership are specified in
[Measurement runtime contract](docs/MEASUREMENT_RUNTIME_CONTRACT.md). Follow
[the operator workflow](workflow.md) for operation and [the recording schema](docs/RECORDING_SCHEMA.md)
for saved fields. The dated Change Log describes historical changes, not current requirements.
The [2026-10-09 RCA correction plan and verification](docs/changes/20261009-rca-reliability-remediation/change-brief.md)
records the before/after behavior and measured effects.

### 2026-09-08: Target geographic coordinates in Measurement records

Measurement results additionally store `target_latitude_deg`, `target_longitude_deg`,
`target_altitude_ellipsoid_m`, and `target_altitude_msl_m` in the result records and
workbook coordinate/method/flat sheets. Existing Forward/Right/Down/Range retain their
baseline-center camera frame. Target position uses the frozen left optical RDF point,
the method's world rotation and P1 GNSS origin, with coincident Pixhawk/optical centers
and the existing GNSS-antenna-as-camera-origin approximation. Camera mounting rotation
is retained. Relative-only profiles without a world anchor have no target coordinates.
Missing ellipsoid height leaves geographic fields empty; missing target geoid height
leaves MSL empty. These missing outputs do not block existing distance calculation.
Target fields are calculated positions, not direct GNSS target measurements.
Journal rebuilds retain the original values; `reproduce_target_coordinates_exact()` in
`workstation_core/measurement_reproduce.py` independently reruns the workbook's frozen
geometry and geolocation inputs. See [the coordinate schema](docs/RECORDING_SCHEMA.md).

During Measurement, `Result` is followed by `Target 위치 · WGS84`: the same latest
accepted result's target latitude/longitude (nine decimal places) and ellipsoid/MSL
altitudes (three decimal places, metres). The line below shows the age of that result.
Unavailable fields show `--`; a new result with missing coordinates clears the prior
coordinates. Clicking a value opens its geographic unit/frame detail in Inspector.
The display uses the immutable history result and does not read newer GNSS/attitude
or perform coordinate conversion during screen refresh.

## Purpose

This package provides the operator Workstation for synchronized stereo-camera capture,
calibration, Measurement, Pixhawk/GNSS evidence, and recording. It owns its runtime and UI
independently from the Legacy application.

`Alignment > Baseline Yaw90` gathers live left/right previews, the display-only `Center line`,
P1/P2 Roll/Pitch, and a permanently visible GNSS table: P1→P2 baseline azimuth (North=0°,
clockwise), ∠Dr–P1–P2, and ∠Dr–P2–P1 (horizontal internal angles). Adjust yaw and physically
level each rig repeatedly while observing these values; there is no level-target column or
automatic alignment verdict. Azimuth remains available without Drone or ATTITUDE. Missing,
stale and producer-reported frozen inputs retain their individual status.

After restarting Workstation, use the mouse wheel over either Baseline Yaw90 live
image to zoom its display independently from 1× to 8× around the cursor. Wheel up
zooms in; wheel down zooms out. Returning to 1× restores the centered full image.
New frames retain the selected display zoom. The `Center line` follows the native
image midpoint, remains one display pixel wide, and disappears when that midpoint
is outside the zoomed view. This is display-only zoom; camera optical zoom, native
source pixels and measurement coordinates remain unchanged.

Open installation alignment from `Alignment > Baseline Yaw90`; the Measurement
shortcut button was removed on 2026-09-08.

The tab reuses the existing baseline-yaw method and camera streams. Reverse uses its original
setting key: Apply changes the current setting; Save persists it. Changes are locked during
Measurement startup, running and stopping. Measurement navigation does not select a method or
start a run. The collapsed midpoint reference retains GNSS local-level F/R/D/Rg and the original
midpoint line angle/90° deviation; it is not camera `baseline_center_frd`. Near-target convergence
does not establish parallel camera yaw. Existing measurements and recordings are unchanged.

`Devices > Pixhawk` retains its common ATTITUDE, baseline and Drone geometry displays.
The installation table has moved from Telemetry, Reverse from Measurement Settings, and Center
line from Calibration. Calibration Live/FTP, zoom, capture and profiles remain available.
Other Alignment tabs retain their frozen frames, annotations and candidates across a Yaw90 visit.
Leaving Yaw90 clears only its Inspector detail selection so an inactive snapshot cannot remain
labelled CURRENT. Window-height and Diagnostics-dock changes refit the live pair from the actual
viewport size; queued layout updates coalesce and stop while another Method is shown.
If the display snapshot query fails or returns malformed rows, the tables and Inspector keep the
last confirmed display values, show `조회 오류`, and advance their ages until the next successful
read. Reverse edits pause while this state is unverified; the existing draft remains. This is
display history, not a substitute for current Measurement evidence. It never clears or rewrites
the last-valid P1/P2 attitude/GNSS inputs used for continued distance calculation after disconnect.

Calibration's default `촬영` surface shows the existing Left/Right checkerboard overlays together
with source-backed board state, current-process inspection outcomes, and persisted capture-set
candidate counts. Those counters explain capture evidence; they never replace the K,D/R,t quality
and Apply gates.

The same Pixhawk page also shows `Dr from Baseline Center`, `Dr from P1`, and `Dr from P2`
as `F/R/D/Rg` metre values. `Dr` means Drone Pixhawk; the three rows explicitly identify the
GNSS midpoint local-level FRD, P1 body-FRD, and P2 body-FRD frames. It separately reports the
horizontal internal angles `∠Dr-P1-P2` and `∠Dr-P2-P1` in degrees. These are latest-state,
read-only aids: the angles use only horizontal North/East data, while missing/stale GNSS or
ATTITUDE remains visible as `WAITING`/`STALE` and no derived geometry is persisted.

The same `Devices > Pixhawk` page shows `RAW yaw°`, `Corrected yaw°`, and `PX4 yaw°` beside each visible
IMU/CAN magnetometer row. RAW is the signed `atan2(-y,x)` diagnostic in the producer sensor axes
with no calibration, sensor rotation, board rotation, or tilt correction. Corrected yaw applies
the manually read-back `CAL_MAG` transformation and shows `PARAM 대기` until that readback exists.
`PX4 yaw°` is the current firmware `ATTITUDE.yaw` reference with PX4 estimator processing and
`EKF2_MAG_DECL` declination correction, only when the row's exact device ID matches the active
`vehicle_magnetometer` device and ATTITUDE is fresh; unmatched or identity-free rows show an
explicit waiting/non-selected state. The hidden `Δyaw°` remains `PX4−RAW` wrapped to
`[-180°,180°)`. The same page also exposes independent P1/P2 `MAG Priority /
active selection` controls for `BOARD`, `CAN1`, `CAN2`, `GPS1_MAG`, and `GPS2_MAG`. `Apply & Verify`
requires Disarm, verifies the CAL_MAG PARAM readback, then waits for a post-apply
`vehicle_magnetometer.device_id` and fresh ATTITUDE. `ACTIVE VERIFIED` confirms the requested
selection; an unverified required side is recorded as advisory evidence and does not block
Measurement Start. The Advanced PX4 MAG controls remain as a fallback.

Measurement remains an observation interval rather than a package-completion transaction. There is
still no storage/completeness gate before a Measurement can start. For a selected method that
consumes live Pixhawk evidence the independent per-side MAG active-selection check above is
advisory: an unverified side is logged as a warning and no longer blocks `측정 시작`. Pressing
`측정 시작` freezes a new run identity/time and records the current MAG selection snapshot
(verified or not) as provenance; camera, Base GNSS/NTRIP, Target and recording continue to follow
their existing method-specific lifecycle.
After an abnormal process exit, restart likewise leaves the incomplete run as inert append-only
history and creates a new runtime generation only when the operator presses `측정 시작` again.

Each Measurement owns `recordings/<start stamp>/` by default. A blank
`measurement_data_root` selects that default; a custom value stays relative to
`SASL_WORKSTATION_DATA_ROOT`. The first Start click freezes one KST token,
`YYYYMMDD_HHMMSS_nnnnnnnnn+0900`. Every Measurement output uses that token, including
`<stamp>_measurement.xlsx`, `raw/<stamp>_<stream>.jsonl` and its same-stem CSV, MP4
segments, PNGs, imported still images, RTCM bytes/indexes, and ULogs/metadata. A collision
allocates one suffix for the entire run. Source capture/receive clocks remain separately
recorded. Late results keep the original run's directory and stamp.

Structured originals are stored as JSONL plus CSV, with complete canonical payloads in
CSV `record_json`; there is no extra ZIP backup. The durable Measurement journal includes
frozen inputs, detections, pair/method decisions, recorded R,t and outcomes, so `Data >
Rebuild Latest XLSX` can reconstruct the latest Measurement after restart. `Retry Storage`
repairs missing CSV rows from JSONL, including overflowed retry rows and stopped-run RTCM
indexes. Failures and bounded-queue drops remain visible without blocking Measurement.
Existing histories and outside-Measurement capture/manual-record paths stay intact.

The workbook opens with `00_요약`, `01_거리비교`, `02_좌표결과`, `03_진단`, `04_Rt`, and `05_그래프`.
The dedicated R,t table and every Method tab use the same frozen records; all 12 R,t
columns remain expanded on Method tabs. Source/evidence tables follow, while exact and
internal analysis tables are normally hidden. Distances/coordinates/t display three decimals,
R nine, angles two and elapsed seconds three, without rounding the stored workbook numbers.
Reader sheets use `—` for missing values; compatibility evidence tables retain `N/A`.
The summary includes method counts, distance statistics and a link to `05_그래프`. This sheet
contains twelve editable Excel charts: camera Forward/Right/Down/Range, horizontal
Forward/Right/Down/Range, and target latitude/longitude/ellipsoid height/MSL height. Each chart
compares Methods against elapsed seconds from Measurement start; Method colours and dash
styles stay consistent across charts. Latitude/longitude use degrees; distances and heights
use metres. Horizontal outputs retain each Method's midpoint and the pair-frozen P1 level
axes; geographic heights retain their separate datums. Only finite `VALID`/`ESTIMATED` values
are plotted. Missing/failed values create gaps, one result appears as a point, and an empty
field shows `데이터 없음` without a numeric axis. Chart-only sampling is capped at 2,000 points
per Method per field and retains bucket extremes and gaps; tables retain all rows using
numbered continuation sheets at Excel's limit. After restarting the app, use `Data > Open
Latest Measurement XLSX` for new exports or `Rebuild Latest XLSX` to add the charts to a saved
run. Rebuilding preserves recorded values; unavailable older coordinate fields stay empty.
The existing exact-reproduction sheets and camera-RDF/stereo-center-FRD contracts remain intact.
Noto Sans CJK KR is the report font; another machine may substitute an installed CJK font.

The background writer updates the same run workbook atomically at checkpoints, Stop and late
completion. `Open Latest Measurement XLSX` includes the new run directories; rebuild is a
background operation disabled during active recording or pending calculations. Different-rate
sensor streams keep separate rows/sheets, without an invented timestamp join. Raw storage
health and incomplete reconstruction are reported independently of calculation status.

If an essential stream (`events`, `measurements`, `method_evaluations`, or `measurement.journal`) cannot write, the
background owner retains FIFO records in a 256-record/8 MiB reserve and retries with bounded
backoff. Video and high-rate telemetry are never accumulated without bound; their drops remain
visible and the observation continues.

The top-level `Data` workspace exposes writer health, timestamped paths, capture tools, the current data
folder, and a separate file-manager link to historical data-root `recordings/`. It does not scan,
recover, migrate, or mutate legacy Session catalogs. Alignment is app-lifetime state: Apply persists an append-only
Alignment event, and the active Alignment remains available across Measurements until Revert,
`Reset Alignment`, or application restart.

Runtime fault isolation keeps one callback, device, camera child, or evidence writer failure from
silently taking down unrelated links. High-rate camera, PX4, GNSS, RTCM, and target traffic stays
on bounded worker-owned paths while the Qt UI consumes latest-state snapshots.

Each board transport is `Radio`, `Ethernet`, or `USB`. Radio and USB are both serial links that
open one exclusive device path and share the same Connect preflight; they differ in bandwidth
policy: P1/P2 Radio uses a fixed minimal stream profile; Drone Radio retains its tiered
stream policy, and USB carries the stock stream set of a direct cable. USB connects through the board's existing bootstrap endpoint, so the saved Radio
endpoint survives a transport switch. The application never pretends to
remove a radio, plug in Ethernet, or power-cycle a board; it waits for actual sysid, carrier,
readback, and cross-endpoint evidence before enabling the next action.

`Alignment > Baseline Yaw90` exposes a collapsed read-only GNSS midpoint reference whenever
selected P1/P2 and Drone GNSS positions are available. Its origin is the P1/P2 GNSS antenna
midpoint; Right is horizontal P1→P2, Down is NED Down, and Forward is Right×Down. The horizontal
baseline↔Drone angle is `atan2(|Forward|,|Right|)` and the displayed installation error is
`|90°−angle|`. This GNSS-derived local-level frame is explicitly separate from the optical-center/camera-axis
`baseline_center_frd` Measurement contract.
While Measurement or Manual Record is active, every primary Drone GNSS callback also attempts one
self-contained best-effort row in `gnss_baseline_center.jsonl`; raw P1/P2/Drone files remain
unchanged.

The Telemetry workspace provides bounded read-only `Find Network` discovery for an Ethernet
Drone Pixhawk. It passively observes UDP MAVLink ports, shows NIC/source IP/listen port/system ID,
and permits selection only when a HEARTBEAT identifies `sysid=3`. `Use selected` fills the existing
MAVLink URL; Apply, Save and Connect remain separate explicit commands. Apply/Save preserve the
accepted URL instead of allowing an older runtime snapshot to replace the edit.

Workstation live-distance pairing follows `Measurement > Pair gates > Time basis`. The default is
exact host-monotonic app receipt. `RTCP/NTP preferred` actually selects a pair only while Clock is
VERIFIED and both decoded frames carry finite, current, same-epoch evidence inside the configured
limit; every timing fault falls back to exact `app_receive_monotonic_ns`. The selected basis is
hard-capped at 100 ms. Fallback provenance is visible and stored but does not by itself downgrade
otherwise-valid geometry.

Camera media admission is permanently tolerant. Decoded frames continue to Preview, YOLO, native
evidence, and official Measurement even when RTP loss or the WIRIS IDR defect prevents compressed-
source integrity verification. Existing timing, calibration, correspondence, and geometry gates
remain authoritative, so a distance may be `VALID` while the camera and result independently show
`INPUT UNVERIFIED`. This behavior has no setting, environment switch, or strict-mode toggle.

Calibration now presents a StereoWIRIS-style primary workflow while retaining the full Workstation
calibration implementation. The default surface exposes only `촬영`, `계산·적용`, and
`프로파일`; file selection, photo-set roots, per-view diagnostics, subset evidence, checkerboard
setup, and raw K,D/R,t details remain available under the collapsed `고급 캘리브레이션` area.
Calibration Live/Auto pair admission uses the same decoded-frame host-monotonic app-receive clock,
while retaining NTP/RTCP as separately labelled record-only evidence. This is a host-receipt proxy,
not proof of simultaneous camera exposure. FTP still pairing retains its separate CPTR-dispatch proxy.
Auto capture additionally admits only parser-observed NAL type 1, non-delta frames and binds the
checkerboard observation to the same decoded native `FrameKey`; it does not treat NAL1 as IDR.
Each successful Auto Start begins a new visible `이번 촬영` counter run without deleting or
renumbering the persisted capture set.

Applied operating K,D is admitted by immutable profile identity and exact native left/right raster.
Live zoom readback presence, age, and value, left/right zoom agreement, physical camera serial,
and stabilization state remain saved provenance only; they never block or warn in Measurement
readiness. This does not bypass Calibration quality/Apply rules or native-raster matching.

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

No media-integrity environment variable is required. A normal `run.sh` launch always uses the
tolerant decoded-frame contract. The historical non-IDR trial switch is no longer a product
behavior input.

For Calibration Auto capture, leave Preview running and move the checkerboard only after a saved
pair or a duplicate-pose message. Auto waits for a positive NAL1/non-delta candidate on each side,
then saves only the exact native FrameKeys used by checkerboard detection. An expired ring key is
not replaced by a nearby timestamp; the next live candidate is the retry.
After Stop, wait for any in-flight capture worker to finish before pressing Start again. A successful
restart resets `검사`, `Stereo 저장`, `K,D 전용`, and `실패` under `이번 촬영` to zero; the files and
the separate cumulative `촬영 세트` counts remain.

The field workflow is deliberately start-first: press `측정 시작` at the observation boundary,
then recover unavailable inputs while the run remains active. Busy camera resources join after
release. A failed camera rejoins only after the operator uses its existing Reactivate action and a
new-generation frame is observed. Recovered video always uses a new no-overwrite MP4 segment.
If Measurement requests explicit `NVIDIA` while `nvh264dec` is unavailable, the observation still
opens but its camera/recording sources are marked unavailable. Any already-live Preview remains
visible. Stop that observation, select `Auto` or `Software`, and retry; Workstation does not silently
override an explicit decoder choice.

For camera installation, open `Alignment > Baseline Yaw90` and observe the azimuth and two
GNSS vertex angles alongside live previews and P1/P2 Roll/Pitch. Iterate yaw aiming and physical
leveling. The midpoint reference is optional detail; no GNSS angle or CURRENT state is a camera
alignment acceptance verdict.

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

Before physical baseline/Roll/Pitch/Yaw alignment, open `Devices > Pixhawk` and inspect the raw
`Attitude (R/P/Y)` and `P1→P2 baseline` groups. Directly below them, confirm the separate
`Baseline−P1` and `Baseline−P2` signed values and top-view arrows. `+` means the P1→P2
bearing is clockwise/right of that Pixhawk's Forward direction; `-` means
counter-clockwise/left. `CURRENT/PARTIAL/STALE/WAITING` describes input freshness only and is not
an alignment pass/fail verdict. In the following `Drone Pixhawk geometry` group, read the three
`Dr from ...` `F/R/D/Rg` rows and the two separately named horizontal angles. The vector direction
is reference→Dr; `D` in the vector header means Down, and `Dr` in the point labels means Drone.

For a LAN Drone, open `Devices > Telemetry`, press `Find Network`, select the `DRONE VERIFIED`
row, and press `Use selected`. Review the URL, then press `Apply`, `Save`, and `Connect`. LIVE
requires the expected system-3 heartbeat and telemetry; discovery alone is not connection success.

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

During capture, `보드 L/R` reports current checkerboard detection and corner count, `이번 촬영`
reports checkerboard validation jobs that actually started, and `현재 캡처 세트` reports the files
recoverable from the active root. `Stereo 저장 후보` is before manual exclusion and solver pose
filtering. Before `K,D 계산`, select one Photo set, keep each camera's input at one native raster,
and enter the board's exact inner-corner dimensions. A mixed raster or wrong pattern fails closed;
a finite K,D result still requires the quality panel to pass before Apply.

The field dual-Pixhawk profile uses Workstation `10.0.0.10/24`, P1/Left
`10.0.0.2:14550` with MAVLink system ID `1`, and P2/Right `10.0.0.3:14551` with system ID `2`.
Save each board's Ethernet profile over USB once, reboot it, then verify both heartbeats with USB
removed. The exact parameters, SD-card files, recovery sequence, and field checks are recorded in
[`Pixhawk.md`](Pixhawk.md).

One field-qualified wiring reference isolates camera and Pixhawk Layer-2 traffic on separate NICs. Cameras use
`enx588694fce07e` with `10.0.0.11/32` and preferred-source host routes to `.231/.232`; P1/P2 use
the replacement RTL8156B `enx0023a40a2e39` with `10.0.0.10/24` and `.2/.3` routes. Neither device
profile supplies a default route. This remains the field-qualified topology because a shared
camera/Pixhawk broadcast domain reproduced a persistent dual-PX4 MAVLink service stall under RTSP
load. The operator may nevertheless use one combined NIC or any multi-NIC split; that selectable
layout is not a claim of sustained shared-NIC field stability.

`Find network` is always read-only and inventories every current physical wired NIC, stable
identity, current IPv4/carrier, and P1/P2 route evidence. Camera uses `Assign Camera NIC`; Pixhawk
uses `Assign P1` and `Assign P2`. An Assign request shows the exact current/proposed roles,
addresses/routes, and active devices that will restart. Only after operator confirmation and OS
authorization does Workstation change NetworkManager. It changes only exact Workstation-owned role
entries, never adds a default route, and permits `Camera,P1,P2` on one NIC or arbitrary splits.

Pixhawk `Find USB` and `Find network` each start a new transient Discovery snapshot. Prior rows,
selection, and candidate feedback are cleared, and completion shows only the newly requested USB or
network result; saved Radio endpoints, network assignments, settings, and live connections are not
cleared. USB aliases are deduplicated to one row per physical serial endpoint. A heartbeat enriches
that row with the observed role (`P1=sysid 1`, `P2=sysid 2`, `Drone=sysid 3`) instead of creating a
second MAVLink row. Discovery remains read-only and never assigns or connects a board.

For the guided field topology, open `Devices > Pixhawk`, select
`P1 Radio · P2 Ethernet · Drone Radio`, and press `Check`. Keep every board disarmed and disconnect
all Workstation Pixhawk links first. Follow only the enabled row action: isolate Drone while P1/P2
air radios are off, confirm the exact Drone NETID25→26 write, fully power-cycle the Drone air radio,
verify Drone sysid3 only on DU, remove the P2 air radio, and verify/provision P2
`10.0.0.3:14551` over its unique direct USB when required. `Save topology` commits settings only
after both hardware postconditions; then use the existing P1/P2/Drone `Connect` buttons. No link is
opened automatically.

If setup is interrupted, restart shows `PARTIAL`. Run `Check` again, then use the current Resume
action or `Guided rollback`. A saved device change is never hidden by Cancel. A malformed operation
record is archived before a new read-only Check. P2 Ethernet failure after completion disconnects
only P2; DU remains owned by Drone and there is no automatic Radio fallback.

After P1, P2 and Drone positions appear in `Devices > Telemetry`, use `Alignment > Baseline Yaw90`
for installation and its collapsed midpoint reference for the former GNSS table. `CURRENT` requires each
source age to be at most 2.0 s and the three application-receive timestamps to span at most 1000 ms.
`STALE` retains the finite signed values and angle with actual age/skew and reason; `WAITING` uses
`--` for missing/invalid geometry. Fix labels remain evidence and do not require RTK Fixed. The existing
ideal-disparity reference consumes the same Rg/readiness and needs no additional operator action.
During Measurement or Manual Record, the same primary Drone callback—not the UI refresh—writes one
CURRENT/STALE/WAITING evidence row. Check `Data > Status > gnss.baseline.center` for admitted,
dropped, or failed writes. The timestamped latest Measurement workbook includes the latest generation
in a `GnssBaselineCenter` sheet.

The target role is applied and read back before the moved role is removed from its old NIC. If
Camera/Pixhawk is already healthy, Workstation stops only owners on the source/target NICs,
saves the applied host role, and leaves Pixhawk disconnected until an explicit `Connect`.
Other NICs continue. NetworkManager apply failures restore the previous OS snapshots where possible
and report any incomplete rollback. `APPLIED · SAVE FAILED` means the host change succeeded but
publishing or saving its settings failed: review the error, then retry `Save` before reconnecting.
It does not mean the OS change was rolled back. The operation uses the address/endpoint settings
captured when accepted, and stays busy until its worker has exited. An active Measurement,
recording, save or finalization rejects the change before any stop or host mutation.

At runtime Workstation identifies itself on both Ethernet links as MAVLink GCS `255/190`. Connect
immediately seeds each configured peer with one GCS heartbeat to break PX4's pre-identity broadcast
deadlock, then sends at most one heartbeat per side per second. A new or rebooted
P1/P2 link remains `QUALIFYING` until expected-system HEARTBEAT and ATTITUDE stay continuously fresh
for 60 seconds. `ATTITUDE STALE` means other MAVLink traffic is still arriving; `ETH MAVLINK RX
STALLED` means all expected-system traffic has been silent for 10 seconds. Only `LIVE` passes the
Pixhawk readiness gate. This one-minute product gate permits routine attitude-dependent operation;
the separate 10-minute comparison/cold-boot runs and 60-minute integrated soak remain the evidence
for long-duration field stability.

Pixhawk Connect opens transport on its worker, but status, completion, failure, cleanup and receive-
timer work that touches Qt is queued to the GUI owner. Progress cannot restyle topology buttons
before completion starts the 20 ms receive timer. Dynamic button state keeps authoritative text and
the `role` property without synchronous QStyle unpolish/polish. Successful RTCM progress is logged
at most once per side every five seconds; exact frame/packet/byte/failure counters and errors remain
unthrottled in their owning status surfaces.

Camera Preview uses the persisted H.264 decoder mode. The supported field default is `Auto`: when
the NVIDIA driver, CUDA Driver API (`cuInit(0)`), and GStreamer `nvh264dec` element are all usable,
Auto requests NVDEC first.
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
`calibration/imported/` or `models/imported/` child before persistence. The default Calibration
photo/output owner is the repository-relative `apps/Workstation/calibration/`; generated K,D saved
there is hash-verified into the data-root `calibration/imported/` store before its restart dependency
is persisted. Any other calibration directory outside these two owned roots remains current-run
only and Save rejects it instead of creating a machine-specific absolute dependency.

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

Leave `Settings > Storage > Data folder` blank to use `<SASL_ROOT>/recordings`. A non-empty custom
value must be relative to `SASL_WORKSTATION_DATA_ROOT`. When that environment variable is explicitly
set and the field remains blank, the default becomes
`<SASL_WORKSTATION_DATA_ROOT>/recordings`. `Data > Status` shows the resolved folder, every fixed
file, writer state, dropped-row count, last write, and error. `Retry Storage` retries failed streams,
but it is never a prerequisite for Measurement. `Update Latest XLSX` regenerates the most recent
Measurement view and `Open Latest XLSX` opens only a successfully published app-owned workbook.
`WAITING`, `GENERATING`, `STALE`, `FAILED`, or `CANCELLED` remains local Data status and never owns
Measurement admission.

Calibration can be completed at any time against the reusable
`apps/Workstation/calibration/` root. New runs allocate new timestamped children and do not merge,
move, or delete historical `.local/workstation_data/calibration/` bytes. Alignment can
also be captured and applied before or between Measurements. The active Alignment is reused until
`Revert active pose`, `Reset Alignment`, or application restart; starting or ending a Measurement
does not clear it.

For an ordinary run:

1. Connect the intended WIRIS, P1/P2, GNSS/RTCM, and optional Target Drone inputs.
2. Apply the intended operating-zoom `K,D` and stereo `R,t`/Alignment source, then inspect
   readiness. Missing geometry may make a result BLOCKED, but it does not prevent data collection.
   Leave timing at the default `App receive`, or select `RTCP/NTP preferred` in Measurement
   Settings. A change made during a running Measurement is saved for the next Measurement.
3. Press `측정 시작`. If Manual record is active, Workstation stops it first and queues this start.
   If a still capture, camera command, time sync, discovery, or RTSP activation owns the cameras,
   the transient run shows `QUEUED` and starts automatically when that owner releases them.
4. Press `측정 종료` when finished. Camera/Detection/video owners stop and the run returns to
   `IDLE`; no save, hash, manifest, RTCM drain, workbook, or previous data state can delay the next
   start. After that boundary, Workstation queues a background timestamped latest Measurement XLSX rebuild.
5. Repeat whenever needed, including after long gaps. For a quick one-run review, wait for
   `Latest XLSX · READY` and use `Open Latest XLSX`; for authoritative or multi-run analysis, join
   the type-owned records using their per-row timestamps and transient generation identity.

Each matched pair submits the ready subset of the ten registered methods against the frozen
inputs for that run. Unready methods remain explicit not-attempted workbook entries.
The representative result remains `Pixhawk attitude + GPS/RTK`; another method is not promoted
when the primary method is blocked or fails. While that Measurement is still `RUNNING`, a terminal
`pixhawk_rtk_live` `VALID` or `ESTIMATED` outcome is priority-enqueued to `measurements.jsonl`
before the table, history, plot, or distance banner is updated. The primary worker starts and
publishes first; comparison workers start after a terminal primary outcome. There is no compute
deadline after a start acknowledgement. A worker that never acknowledges start has a 30-second
startup guard. A successful primary does not wait for comparison completion.
If that primary enqueue is rejected, it keeps independent background retry ownership even after the
submitted-method bundle is saved. The UI remains `EVIDENCE DEGRADED` until admission. If Stop or the
next run wins first, the retry switches to a compact `late_primary_measurement_outcome` record in
`measurements.jsonl` and never projects into the new run's table, counters, history, plot, or banner.
Primary `BLOCKED`, `FAILED`, `TIMEOUT`, and `BACKPRESSURE_BLOCKED` outcomes remain only in
`method_evaluations.jsonl`. Here "immediate" means bounded asynchronous priority enqueue, not a
synchronous write or `fsync`. No automatic Parquet or legacy Session `review.xlsx` is produced;
each run's XLSX is a derived projection updated at checkpoints, Stop and late completion.

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
`R,t`; Live/Auto compares signed Right-minus-Left decoded-frame app-receive monotonic time and
applies the gate to its absolute value. NTP/RTCP does not substitute for missing app-receive
evidence. Timing-eligible captures remain stereo candidates. Calculate and save the required
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
new BLOCKED/FAILED/BACKPRESSURE_BLOCKED per-method outcomes stay in their own method sheet. Only a row whose
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

When settings are saved, a persisted `intrinsics_path` that no longer names a file is cleared and
logged instead of making settings save fail. This lets normal application shutdown complete while
keeping a stale or non-reopenable asset path out of persistent settings. Startup still applies the
strict profile-selection rules above.

That startup rule chooses which profile may be applied; it is separate from runtime K,D readiness.
After a profile is active, later missing/stale/mismatched zoom, serial, or stabilization evidence
does not replace it with another profile and does not block geometry. Profile identity and native
left/right frame dimensions remain authoritative.

Measurement Settings does not select Intrinsics or a manual focal value. The active selected `K,D`
comes from `Calibration > Intrinsics > Profiles`. Apply the compatible stereo bundle under
`Calibration > Extrinsics > Profiles`. Configure `YOLO input size` under
`Settings > YOLO / RTSP`; Apply updates the runtime owner and the new size is used from the next
Detection start. Confidence, IoU, and pair gates remain in Measurement Settings.

Select detector weights under `Settings > YOLO / RTSP > YOLO runtime` with `파일 선택…` beside
`Weights path`. Selecting a file outside `SASL_WORKSTATION_DATA_ROOT` atomically imports and
SHA-256-verifies it under `models/imported/<sha256>/`, then fills the portable relative path.
Cancellation or import failure preserves the previous selection and reports the failure in the
same settings group. `측정 시작` applies the currently displayed weights path to the runtime owner
before Detection starts, so an older applied or persisted model cannot replace the current
selection. If that handoff is rejected, Measurement does not start with the stale model. `Apply`
still applies the complete Settings form, while `Save` explicitly persists the relative path.
The existing normal-shutdown whole-settings save behavior is unchanged. The picker accepts `.pt`,
`.onnx`, `.engine`, and `.torchscript` files.

Workstation uses the single `field_measurement_v1` policy. When no calibrated profile is active,
fresh equal left/right zoom readbacks plus known native rasters automatically generate a
`NOMINAL_UNVERIFIED` pinhole profile from the 1280x720 WIRIS zoom focal table. The generated K,D
uses centered principal points and zero distortion, records its formula/hash, and always retains
`precision_qualified=false`. A selected calibrated profile no longer yields to nominal K,D because
live zoom/serial/stabilization evidence is missing, stale, or different. Malformed/missing K,D,
invalid profile identity, and unknown/mismatched native raster remain hard failures. Zoom evidence
can still prevent constructing nominal K,D when no calibrated profile is available because the
nominal focal table needs a definite common zoom.

Field measurement blocks only missing/ambiguous correspondence, the effective pair delta over
the configured limit or 100 ms hard cap, failure of both preferred RTCP timing and app fallback,
missing or malformed K,D/R,t,
current-run telemetry absence for live geometry, non-finite/frame-invalid inputs, impossible
two-ray geometry, an already-running camera interval, unapplied geometry
candidates, and missing/mismatched candidate-to-active K,D identity. An RTCP timing fault alone is
not a result advisory: it activates app-receive timing provenance. Vertical delta, telemetry age,
GNSS fix/accuracy/readiness, and nominal K,D retain their existing advisory behavior.

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

`Stop NTRIP` rejects queued connection/status events from that session. A new Start waits
for the previous network thread to exit; late events or counters from the previous connection
cannot reactivate it or validate the new session's password. Normal reconnects within one
active session retain the captured credentials. RTCM evidence already admitted to the data
plane still follows its existing drain and recording rules.

For `SYSTEM_CENTER` GGA, both selected P1/P2 GNSS positions must have arrived within the last
10.0 seconds and their application-receive timestamps may differ by at most 3000 ms. Exact boundary
values are accepted. These relaxed limits apply only to the NTRIP center-position hint; Measurement
and P1→P2 baseline readiness retain their stricter freshness and synchronization gates.

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
Mountain solver. Horizon/Mountain `GOOD`/`LOW_CONFIDENCE` is quality evidence rather than copy
permission: `Use suggestion` is available whenever current-frame native points remain structurally
usable. Horizon requires exactly two in-frame endpoints per side separated by at least 25% of image
width; Mountain requires paired unique/distributed points after exclusions. Automatic selection
never applies `R,t` by itself. The manual Horizon, Mountain, and Target modes remain available.

Auto-selection runs in a background worker so the Workstation window remains responsive while
Horizon/Mountain image analysis is running. Both automatic-selection buttons are disabled during
the run, duplicate clicks and other optical-pose mutations are ignored until completion, and a
result is committed only if its frozen frame IDs and SHA-256 identities still match the current
burst. The status strip shows `RUNNING`, `LOW_CONFIDENCE`, or the failure reason.
Closing the window waits for the analysis thread to exit. If a stop attempt times out,
the window keeps waiting and retains the worker so it can finish cleanup.

`Auto-select Mountain` biases SIFT/ORB matches toward the ridge using a per-frame sky/ground
classifier, and it also measures whether that classifier found a separable ground class at all:
ground must be at least 2× more textured than sky and, unless overwhelmingly so (≥6×), at least
12% of full scale darker. When neither frame has a separable ground class (an all-sky or all-cloud
pair) the action still runs: it skips the ridge bias, matches the far-field sky/cloud texture
unfiltered, applies the usual inlier/distribution/confidence gates, and adds the warning
`no separable ground/ridge in either frame; correspondences are far-field sky/cloud texture, valid
for rotation only when both frames are simultaneous`. The suggestion metadata records the filter
tier that produced the points (`ridge_boundary`, `ground_only`, `unfiltered`, or
`unfiltered_all_sky`) and the per-side `sky_ground_separability` metrics so a proposal can be
audited offline.

`Auto-select Cloud` (same tab) is the dedicated far-field path: it runs the Mountain matcher with no
ridge bias at all (tier `cloud_far_field`), and its points are meant for `Solve Cloud rotation`, which
reuses the Mountain rotation solver but produces a **rotation-only** candidate
(`cloud-far-field-rotation`): translation is never apply-eligible, and the rotation is apply-eligible
only when the frozen left/right frames are simultaneous within 0.350 s on the app-receive clock
(`frame_pair_simultaneity` = `VERIFIED`/`EXCEEDED`/`UNVERIFIED` in the candidate provenance). The
tenth measurement method, `Cloud far-field rotation + live GNSS` (`cloud_gnss`), consumes that
candidate and fills N/E/D from live GNSS, like the Drone RTK Rotation hybrid. Auto Mountain remains
usable without a ridge; Cloud is the explicit choice when the operator knows the scene is sky. Thresholds were calibrated
against all-sky synthetic frames (texture ratio 1.05–1.11, brightness gap ≤ 0.17) and real ridge
photos (ratio 3.4–9.5, gap ≥ 0.30); see `tests/test_optical_pose_mountain_all_sky.py`.

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
`Drone Rotation`, `Drone Multi-view`, and `Combine` tabs below the common frozen
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
instead of borrowing arbitrary live axes. All submitted method attempts for a pair share the same
left/right detections, telemetry snapshots, native-frame file identities, and capture-sync basis.

The Measurement `DOF fill for` selector exposes every currently available method. For each Left
and Right Roll/Pitch/Yaw/North/East/Down component, `OWN` means the method observes that value and
the control is disabled. A missing rotation component exposes a `PX4` toggle; a missing translation
component exposes a `GNSS` toggle. Safe fill toggles are enabled by default, can be disabled for the
current Measurement, and lock when Detection starts. Method-defining observations (Horizon
Roll/Pitch, Drone Rotation Yaw, and every DOF of full Mountain/Multi-view/Terrain/Combined methods)
cannot be replaced by a fallback merely to make an invalid candidate look ready.

Runtime evaluation uses one persistent process per registered method: the primary
`pixhawk_rtk_live` process and nine comparison processes, started lazily after a primary terminal
outcome. OpenCV/NumPy/BLAS child thread counts remain capped at one. Every method has an independent
20-pair pending limit; only the saturated method returns `BACKPRESSURE_BLOCKED`. Slow calculations
continue across Measurement Stop and subsequent runs. A crashed process is replaced; an active
calculation has no automatic timeout. `TIMEOUT` is retained only when reading historical records.
A `VALID` or `ESTIMATED` primary outcome received while its run is `RUNNING` is priority-enqueued
before its table/history/plot/banner projection. Submitted terminal outcomes form a canonical
method-order bundle; that bundle is independent of the already-enqueued primary
row. A bundle is committed and counted only after its enqueue is admitted. A rejected active-row
append remains pending and retries the same logical row; if Stop wins before retry, that retry is
labelled `late=true`. Row admission carries an owner-epoch/index lease, so clear, rebind, close, or
numeric-index reuse cannot redirect an old retry into a new row. An unavailable owner is detached
and the record is retried through the direct priority writer. Primary and bundle retry owners each
use a 1-record-per-tick budget plus 1,024-record/8 MiB bounds; overflow and shutdown-budget drops
are explicit counters/log/status evidence, never silent growth. Queue payloads contain paths/hashes
and immutable numeric metadata, never QImage or raw pixel bytes; native PNGs are stored before enqueue.

Each Alignment method tab shows method-specific evidence alongside the read-only Candidate
observability summary: Horizon shows model/RMS/sensitivity, Mountain shows pair/inlier/Sampson/baseline/lever-arm
metrics, Terrain shows GeoTIFF/2D↔3D/reprojection/rank data, Drone tabs show target/observation/sync
and 3D-spread data, and Combine shows source/axis conflict status.

Alignment's `R,t · 실시간` table automatically refreshes `Attitude + GNSS` and `GNSS Yaw90`
at the existing visible-page status cadence. No Capture, Use suggestion, calibration, or
registration is required to view these transforms. `References` and its manual registration
buttons were removed on 2026-09-08. The live table shows the 3×3 rotation, translation in
OpenCV optical RDF metres, state and oldest input age. `X_right = R * X_left + t` is the
same canonical direction used by Measurement; t is not the P1→P2 NED baseline vector.
GNSS Yaw90 uses each Pixhawk's Roll/Pitch and the shared baseline-perpendicular yaw,
including the existing Reverse setting. The selected GPS1/GPS2 receivers are respected.
Finite old values remain visible as STALE; missing inputs show WAITING, and query errors
retain the last readout as ERROR while its age advances. These live rows never register
or apply a candidate, write parameters, warm Measurement caches, or change a run's inputs.
Solved candidates and Combine retain their independent selection. In Baseline Yaw90 setup,
the live table follows the installation surface so the first-viewport controls stay in place.

`Clear` removes the selected annotation mode from both frozen frames. Local `Import Pair` asks only
for the Left and Right image paths and dispatches them without a zoom dialog or synthetic
`zoom_index`; missing zoom remains provenance only. The WIRIS alternatives are labeled
`Capture HR Still (FTP)` and `Capture OZ Still (FTP)`. Low-level WIRIS `GLRD` remains available
through Advanced command tools, but no LRF status card or refresh command is exposed in Alignment.

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

Drone network discovery listens passively on bounded UDP MAVLink ports 14550–14559 for one scan.
Its candidate rows are transient and read-only. The selected URL keeps the existing
`udpin:0.0.0.0:<port>` interface and is persisted only by Telemetry `Save`.

- Operating-intrinsics snapshots retain selected profile/hash, native dimensions, zoom evidence,
  camera serial, and stabilization provenance. Admission consumes only profile presence/identity
  and native left/right raster; zoom/serial/stabilization fields remain record-only. The verified
  `intrinsics_zoom_08_20260831.json` profile is 1280x720 K,D-only input and requires a separately
  selected R,t for methods that use stereo extrinsics.
- `alignment.pose.import_burst` from local `Import Pair` carries only
  `sources.left.path`/`sources.right.path`. It does not request, infer, or serialize a zoom index.
  Startup auto-load zoom selection and no-active-profile nominal K,D construction remain separate
  workflows with their existing zoom requirements.

- The simple Calibration controls dispatch the existing `calibration.*` command IDs; they do not
  implement a second solver or persistence path. `calibration.run_intrinsic`,
  `calibration.run_extrinsics`, Save/Load, and `calibration.apply_bundle` therefore retain the same
  frozen photo-set, quality, zoom, native-raster, immutable-profile, and rollback contracts as the
  advanced controls.
- `workstation_calibration_capture_snapshot()` exposes additive
  `workstation-calibration-capture-status-v1` evidence: per-side detection state/corners/age,
  current-process request and outcome counts, current-root Left/Right/K,D-only/stereo-candidate
  inventory, minimum image setting, and the latest controller status. It is read-only and is not a
  persisted calibration profile or solver result.

- `detection.toggle` is an always-available operator request shared by the Measurement button,
  menu, and toolbar. Presentation state never disables one surface independently of another. A
  duplicate active run is rejected, but an active manual writer, busy camera, missing RTSP URL,
  identical Left/Right stream, missing device, storage error, or old finalization cannot reject the
  observation interval. Those conditions affect only their source branch and method results.
- `measurement_run_snapshot()` exposes additive `sources` entries with
  `PENDING/WAITING_RESOURCE/LIVE/UNAVAILABLE/LEFT`, join/leave clocks, and factual detail. Every
  persisted transition carries `runtime_generation` so late callbacks cannot mutate or masquerade
  as the next run.
- `AppSettings.yolo_gate_time_basis` stores `app_receive_monotonic` or `camera_rtcp`. Missing and
  invalid values use app receive; historical `camera_rtc_required` migrates to RTCP preferred with
  mandatory app fallback. Each new pair records requested/effective basis, fallback reason,
  selected delta, and distinct app/RTCP deltas. A Measurement freezes its request at start.

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
  `SCALED_IMU/2/3` uses the same command-is-intent boundary: only an actual expected-system sample
  confirms delivery. After the initial profile, Workstation retries only missing or older-than-one-
  second IMU streams up to the bounded three-attempt cycle, then waits ten seconds; any fresh sample
  clears its stream retry immediately. The existing P1/P2 IMU table shows `WAITING` before the first
  sample, `LIVE` only for fresh app-receive evidence, and `STALE` with the retained values and exact
  age otherwise. All-RX stall suppresses these retries. No automatic USB command or PX4 persistent
  write is part of this path.
- Physical NIC roles are a runtime interface. `Find network` emits candidates only; the confirmed
  `Assign Camera NIC/P1/P2` command applies a role-scoped NetworkManager delta. Settings persist
  stable NIC identity, intended roles and only the exact address/route entries Workstation created
  or explicitly adopted as matching role entries. Pre-existing unrelated profile values remain
  external and are never removed by a role move.
- `pixhawk_connection_candidates()` exposes one revisioned transient Discovery snapshot. Candidate
  rows carry source, physical device/NIC, link/kind, evidence, and role. Network role text explicitly
  marks a multi-role interface as `shared NIC`; USB role text distinguishes saved assignment from
  observed heartbeat sysid. Starting a new USB/network search increments the revision and clears
  stale selection/feedback without changing persisted configuration.
- Each Pixhawk exposes both `GPS_RAW_INT` (GPS1) and `GPS2_RAW` (GPS2). The Pixhawk workspace
  shows all four receiver rows, including MSL/ellipsoid altitude, HDOP/VDOP, H/V accuracy, fix,
  satellites, and age. Its bottom P1/P2 antenna selectors independently choose the receiver used
  by the P1→P2 GNSS baseline, NTRIP GGA, rover readiness/status, Drone reference calculation,
  Measurement calculation, and frozen GNSS contract; the default and persisted mapping is
  `P1 GPS1 → P2 GPS1`. No automatic averaging or best-fix substitution occurs.
- `baseline_yaw_alignment()` is a read-only latest-state interface over those selected receivers
  and P1/P2 MAVLink ATTITUDE. It reports horizontal
  `azimuth(P1→P2)=atan2(East,North)`, compass-normalized P1/P2 yaw,
  `Baseline−Pn=wrap180(azimuth−Pn yaw)`, and `P2−P1 yaw`. Compass angles use North=0° and
  clockwise-positive; signed values use `[-180°,180°)`. The view keeps per-source age/fix/pair
  delta and never rewrites raw NED, P1 body FRD, camera RDF, or measurement coordinates.
- `drone_pixhawk_geometry()` is a read-only latest-state interface over selected P1/P2 GNSS,
  Drone GNSS, and P1/P2 ATTITUDE. It reports midpoint→Dr in
  `gnss_baseline_center_local_level_frd`, P1→Dr in P1 `px4_frd`, and P2→Dr in P2 `px4_frd`,
  each as `F/R/D/Rg` metres. `∠Dr-P1-P2` and `∠Dr-P2-P1` are independent unsigned internal
  angles from horizontal North/East vectors only; they do not use Down or yaw, add no global
  status key, and do not change Measurement semantics or persistence.
- `magnetometers()` and `can_magnetometers()` add read-only `raw_yaw_deg`, `px4_yaw_deg`, and
  `delta_yaw_deg` projections. `magnetometers_for_imu_view()` keeps the three standard
  `SCALED_IMU*` rows and appends the existing stable-firmware `DEBUG_VECT` `CAN1_MAG`/`CAN2_MAG`
  RM3100 rows; it never relabels RM3100 data as `SCALED_IMU`. The raw value is producer-axis
  `atan2(-y,x)`; the PX4 value is correlated only on exact active device identity plus fresh
  ATTITUDE, so a standard `SCALED_IMU*` row without a physical device ID does not receive a
  copied estimator value. Stable PX4 additionally supplies RM3100 rows through the bounded PX4
  Shell `listener sensor_mag` readback path; those rows retain a
  `px4-shell/sensor_mag/<instance>` topic and are never relabelled as MAVLink `DEBUG_VECT`. The
  selected-device gate reads `listener vehicle_magnetometer -n 1` through the same exclusive Shell
  owner.
- The approved patched v6X custom firmware is
  `firmware/px4_fmu-v6x_sasl_can_mag/px4_fmu-v6x_default.px4`, with SHA-256
  `3ef5736aabdaec9047cf97b0cd5eefa2cfc074672c3ae43776c3f0e12d2349ce`. It retains the custom
  CAN1/CAN2-to-MAVLink path and guards MAVLink signing status lookup against null instance slots.
- Measurement result evidence schema v2 adds `policy_id=field_measurement_v1`,
  `validity=VALID|ESTIMATED|INVALID`, `hard_block_reasons`, and `advisory_reasons`. The fixed
  `measurements.jsonl` stream preserves successful representative `VALID`/`ESTIMATED` decisions and
  the compatibility `measurement_valid` field remains true for both. Primary
  `BLOCKED`/`FAILED`/`TIMEOUT`/`BACKPRESSURE_BLOCKED` outcomes are retained in
  `method_evaluations.jsonl` rather than creating a representative measurement row. Successful
  rows carry the method-evaluation pair/generation/snapshot IDs plus the primary outcome hash so
  they can be joined to the later submitted-method bundle without relying on positional row order.
- `MeasurementDataStore` has explicit `OPEN → CLOSING → CLOSED` lifecycle evidence. Enqueue is
  accepted only in `OPEN`; a `CLOSING`/`CLOSED` request is rejected, counted as dropped, and cannot
  strand a new priority/bulk item after shutdown has begun. `Retry Storage` uses the same
  active-operation fence and cannot move a retry record after close's empty-queue decision.
- A pending native-frame completion keeps its immutable request-to-row owner even when a newer run
  replaces the panel's current row collection. Completion updates only that original row when it is
  still available and its append lease still matches. It also emits a best-effort `native.frames`
  event under the original runtime generation; unavailable/retryable projection remains canonical
  event evidence and never writes into a same-index row owned by the next run. Completion retry is
  separate from primary-row retry, request fingerprints enforce first-terminal-wins, and exact
  duplicates are no-ops. Persistent dual rejection uses bounded background retry, then exposes an
  explicit terminal-drop counter/status and a bounded diagnostic payload cache so shutdown cannot
  wait forever; this is not a durable-write claim when every storage boundary remains unavailable.

- The canonical runtime root is `.local/workstation_data/` at the repository root, or the path in
  `SASL_WORKSTATION_DATA_ROOT`. It owns `stereo_distance_settings.json`, portable imported
  calibration profiles, `models/`, historical `recordings/`, and runtime caches. New Calibration
  photos/results default to `apps/Workstation/calibration/`; persistent restart dependencies remain
  content-addressed data-root-relative paths.
- New Measurement data defaults to `<SASL_ROOT>/recordings`. The portable
  `measurement_data_root` setting is blank for that default; an old saved value of
  `measurement_data` is normalized to blank during load. With an explicit
  `SASL_WORKSTATION_DATA_ROOT`, blank resolves to its `recordings/` child. A non-empty custom value
  remains relative to the data root. Hand-edited absolute or escaping values fall back to the
  resolved default and report `DEGRADED`; they do not reject Measurement.
- Structured files have stable type ownership and timestamped names for each new data-writer file set.
  Existing fixed names remain readable for compatibility:

  | Stream | Timestamped relative file pattern |
  | --- | --- |
  | run/capture events | `events_<timestamp>.jsonl`, `capture_events_<timestamp>.jsonl` |
  | primary pair results | `measurements_<timestamp>.jsonl` |
  | submitted-method bundles/outcomes | `method_evaluations_<timestamp>.jsonl` |
  | camera observations | `camera_left_<timestamp>.jsonl`, `camera_right_<timestamp>.jsonl` |
  | rig and target PX4 | `px4_left_<timestamp>.jsonl`, `px4_right_<timestamp>.jsonl`, `px4_drone_<timestamp>.jsonl` |
  | Base GNSS | `gnss_base_<timestamp>.jsonl` |
  | GNSS baseline-center derived evidence | `gnss_baseline_center_<timestamp>.jsonl` |
  | per-method thin results | `method_results_<timestamp>.jsonl` |
  | analysis comparisons | `geometry_comparisons_<timestamp>.jsonl` |
  | Alignment changes | `alignment_<timestamp>.jsonl` |
  | binary inventory/logs | `artifacts_<timestamp>.jsonl`, `native_frames_<timestamp>.jsonl`, `application_log_<timestamp>.jsonl` |

- `<resolved data root>/latest_measurement_<timestamp>.xlsx` is an app-owned, atomically published derived
  workbook. It contains `Summary`, Measurement events, materialized/latest and append/replace
  result history, method bundles/outcomes/inputs, Camera/Pixhawk/Target/GNSS/GNSS-baseline-center/
  geometry/native/file
  sheets, and `Issues`. Manual-record-only and unrelated application/capture/alignment streams are
  excluded. Original clock fields remain exact, UTC epoch fields gain KST ISO columns, and integers
  beyond Excel's 15-digit precision are stored as text.
  The `00_Summary` method table lists, per catalog method, the result-status counts and
  Forward/Right/Down/Range min/max, plus the **latest VALID R,t** of that method (`R,t pair`,
  `R,t rows`, `R11..R33` = R_right_from_left, `T_R/T_D/T_F` = T in right-camera RDF metres — the same
  contract as the per-method sheets) and `R spread deg` / `T spread m`, the largest deviation of the
  other VALID rows in the same Measurement from that latest transform. Methods without a VALID row
  show `N/A`.

- The writer removes Session membership fields, adds host UTC and monotonic record timestamps, and
  preserves producer coordinate/source-clock fields. A torn final JSONL fragment is left as one bad
  historical line; the next valid append begins on a new line. New writer instances use a new
  timestamped file set; existing files are never replaced by that filename allocation.
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
- Calibration uses repository-relative `apps/Workstation/calibration/` as its default reusable root
  and keeps content-addressed restart profiles under the Workstation data root. An explicit custom
  relative root remains data-root-owned. Calibration photo-set “sessions” are calibration-domain
  groupings only; they are not Measurement Session gates. Alignment events append to the data root
  and the active in-memory pose spans runs until Revert, Reset, or restart.
- Existing `recordings/<id>/` and `recordings/sessions/<id>/` trees are never scanned or recovered
  by the new workflow. `Data > Status > Open Legacy Recordings Folder` exposes them read-only through
  the file manager. Old manifests, Parquet, and `review.xlsx` remain untouched and no new legacy
  instance is generated. The root-level latest-only workbook does not inspect or convert them.
- `measurement-method-snapshot-v1` still supplies the live Measurement view. Runtime
  `method_capabilities` remains instance-driven while `method_catalog` lists all ten method IDs;
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
  `stereo_pair_saved`, and `intrinsics_only_sides`. New Live/Auto records also carry exact per-side
  `left_app_receive_monotonic_ns`/`right_app_receive_monotonic_ns`, signed `pair_delta_ns`,
  `pair_time_basis=app-receive-monotonic`, and `HOST_RECEIVE_PROXY`; host receive never sets
  `precision_timing_qualified` or `actual_exposure_verified`. A live
  CameraPane capture fails closed when a fresh native frame cannot be matched. Preview fallback is
  disabled by default and exists only behind an explicit compatibility flag for lightweight tests.
  Auto records the parser AU kind/NAL types, non-delta flag, exact AU and decoded PTS, and side
  `frame_key`; these values must agree before the image is eligible.
  The `이번 촬영` counters are UI lifecycle state rather than persisted photo-set totals, so Auto
  Start resets them while the capture inventory remains derived from the existing files/metadata.
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
  remains available as record-only wall-clock evidence. Ordinary Live/Auto pair admission is
  `HOST_RECEIVE_PROXY`; only explicitly verified per-camera exposure fields can become
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
- The Telemetry snapshot publishes additive `gnss_baseline_center_frd` evidence with frame
  `gnss_baseline_center_local_level_frd`, ordered `[forward,right,down,range]` in metres, source
  age/fix and three-source receive-time spread. During recording, primary Drone GPS reception
  additionally produces `gnss.baseline.center` rows. This adds no setting, legacy Parquet or
  canonical Measurement field.
- Frozen optical-pose frames are persisted as native PNG evidence and remain bound to
  `(frame_id, image_sha256, native_width, native_height)`. Automatic points are returned in
  native OpenCV `(u, v)` pixels and are kept separate from manual annotation payloads until
  explicit approval.
- Horizon suggestions are two endpoint candidates from a distortion-corrected gradient/Hough
  skyline line. Mountain and Cloud suggestions share one SIFT path (ORB when SIFT is
  unavailable): mutual ratio matching, then RANSAC epipolar filtering. By default every RANSAC
  inlier is kept, displayed, and stored (`max_points` 0 = all); a positive `max_points` caps
  the suggestion to that many spatially distributed points instead.
- Every Mountain/Cloud suggestion also carries `relative_rotation`: a rotation-only far-field fit
  (homography at infinity, Kabsch over all inliers with residual trimming) giving
  `rotation_right_from_left_rdf` plus `roll/pitch/yaw_right_minus_left_deg` (right camera in the
  left camera's virtual FRD, 3-2-1), `far_field_du/dv_median_px`, `fit_inlier_count`, and
  `fit_residual_px`. Negative yaw is toe-in. It assumes features at infinity; residual parallax of
  nearer features biases yaw by about `atan(B/D)`. The same mapping is repeated under each side's
  `metadata.relative_rotation`.
- Automatic suggestion evidence includes `detection_method`, `confidence`, `match_count`/
  `inlier_count`, `residual_px`, `eligible`, `copy_eligible`, `copy_block_reason`, and
  `failure_reason`. Mountain approval also records the shared exclusion indices, original/retained
  pair counts, and canonical filtered-input SHA-256. Creation and approval are
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

- When Navigation is saved hidden, move the pointer to the Workstation window's left edge to
  reveal it. It hides 400 ms after the pointer leaves; returning to the dock cancels hiding.
  Saved or manually shown Navigation remains pinned, and temporary reveals do not change the
  saved dock layout. The toolbar and View menu retain manual controls.

- Read `RAW yaw°` as a sensor-axis diagnostic, not a calibrated compass heading. Read `Corrected yaw°`
  as CAL_MAG-corrected sensor-vector yaw, still without magnetic-declination correction. Read `PX4 yaw°`
  as the firmware estimator reference for the verified active MAG; `Δyaw°` is
  `PX4−RAW` wrapped to `[-180°,180°)`. `미선택`, `ID 확인 대기`, `STALE`, and `대기` are
  intentional evidence states, not zero values or alignment verdicts.

- To change a runtime MAG before a live-Pixhawk Measurement, open `Devices > Pixhawk`, choose the
  P1 or P2 side independently, set the logical MAG priorities, and press `Apply & Verify`. Do not
  treat PARAM readback alone as success: wait for `ACTIVE VERIFIED`. Save/Reboot remains a separate
  explicit operation and is not required for an immediate runtime experiment.

- Treat `Baseline−P1` and `Baseline−P2` as separate horizontal observations. A positive value
  means P1→P2 lies right/clockwise of that board's Forward direction. Do not interpret
  `CURRENT` as physical-alignment acceptance: the UI intentionally has no target angle or
  GOOD/WARN/BAD tolerance. A stale row retains its last number with age/dashed-arrow evidence;
  use only current inputs for live adjustment.

- Telemetry `Find Network` never changes NetworkManager, PX4 parameters/files, saved settings, or
  active device ownership.
- Only a decoded HEARTBEAT with `sysid=3` is selectable as Drone; P1/P2 identities remain visible
  evidence but cannot be applied to the Drone field.
- Apply/Save use the current form value as authoritative configuration input. Periodic status
  refresh cannot overwrite a dirty edit with a stale connection endpoint.

- Treat the collapsed GNSS midpoint reference in Baseline Yaw90 as position evidence. Its line angle is the
  acute horizontal angle between the P1→P2 baseline line and midpoint→Drone GNSS line; 90° means
  the Drone is on the perpendicular through the P1/P2 midpoint. `90° error` is an absolute distance
  from that target. Do not interpret `CURRENT` as measurement acceptance; use only current inputs for
  live adjustment and keep STALE timing evidence visible.

- Use `App receive (default)` unless the operator explicitly needs RTCP/NTP frame selection.
  `RTCP/NTP preferred` requires VERIFIED/current/same-epoch frame evidence and automatically falls
  back to exact app receipt on any timing fault. Do not treat fallback as exposure time. To change
  an active run, Apply/Save the setting and start the next Measurement; the current run stays frozen.

- Keep the Calibration root setting at `calibration` for the repository-owned
  `apps/Workstation/calibration/` default. A root change applies to the next Calibration run and
  does not move or merge existing photo sets. Generated profiles are copied and hash-verified into
  the data-root import store before settings persistence; arbitrary external roots remain blocked.

- Keep `Settings > Storage > Data folder` blank for repository-root `recordings/`. Changing it while
  capture is active does not retarget open writers; the next capture uses the applied custom path.
  Existing `.local/workstation_data/measurement_data/` bytes are historical and are never moved,
  merged, deleted, or rewritten automatically.

- Treat the timestamped latest Measurement XLSX as a disposable review projection. Only `READY` with an existing
  published generation proves success; request acceptance or `GENERATING` does not. Do not use it
  as a completeness verdict or edit it as the only copy of data. A new Measurement/Manual record or
  shutdown cancels or defers export, while `Update Latest XLSX` retries after the system is idle.

- Treat NIC role changes as explicit operations, not discovery side effects. Review the selected
  NIC, exact before/after entries and active owners, then approve the OS authorization prompt.
  Ordinary Camera/Pixhawk owners on affected NICs restart automatically; owners on other NICs do
  not. Never apply during Measurement, recording, save or finalization. `APPLIED` requires
  NetworkManager+kernel readback and, for a previously active role, a new Camera frame or expected
  Pixhawk HB+ATT; a button click or socket open is not success.

- Treat the three simple Calibration tabs as an ordered workflow, not as a quality bypass.
  `FAIL` continues to block Apply, `WARNING` still requires explicit review/confirmation in
  `고급 캘리브레이션`, and Calibration's capture/Apply zoom and native-raster checks retain their
  existing behavior. After K,D is active, runtime Measurement readiness ignores live
  zoom/serial/stabilization evidence but still blocks missing/invalid profile identity and
  native-raster mismatch. The advanced area is presentation-only disclosure; collapsing it never
  disables its underlying command state or changes the active calibration.
- For local Alignment `Import Pair`, select the native Left and Right images; no zoom response is
  required. Workstation still rejects a missing/invalid K,D identity or native-raster mismatch.
  Camera, focus, and zoom provenance is operator-reviewed and does not gate readiness.
- Before `K,D 계산`, verify that every selected Left image shares one width/height, every selected
  Right image shares one width/height, and the configured inner columns/rows match the physical
  board. The mathematical minimum is three geometrically distinct detections, but the configured
  minimum and quality gate still apply. Low reprojection RMS alone is never Apply evidence.
- Auto Stop→Start resets the legacy Auto valid-pair counter but does not delete files. The primary
  `이번 촬영` inspection row is scoped to the current process and active capture root; it resets on
  root change or restart. Use `현재 캡처 세트` and Compute selection to review/exclude all retained
  files.

- Camera Preview defaults to RTSP TCP with Auto decoder. Auto uses `nvh264dec` only when its
  factory can be created, `nvidia-smi` confirms a working driver, and CUDA `cuInit(0)` succeeds;
  otherwise it reports WARN and starts with mandatory `avdec_h264`. A direct NVDEC element/health
  failure, or a 30-second decoded first-frame/stall timeout while RTP input remains current, may
  authorize one software fallback.
  RTSP/source/parser and decoded recorder/NVENC/output failures retain their own error class and
  never trigger a decoder change. Explicit `NVIDIA` has no software decode fallback.
- Decoder capability is checked before a Measurement replacement takes ownership of the live
  panes. If that preflight rejects explicit `NVIDIA`, temporary automatic recording is released and
  Measurement reports both camera/recording sources unavailable, but the existing Preview workers,
  frames, and connection state are preserved. A failure after replacement begins retains the
  bounded stop/cleanup path below.
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
- A camera QThread/process launch is only a source-level `STARTING` request. Workstation sets a side LIVE only
  after a decoded frame carrying that pane's current stream generation reaches the GUI boundary.
  The Measurement run is already `RUNNING`; after 35 seconds a side without an ACK is stopped and
  marked `UNAVAILABLE`. Zero, one, or two verified sides never roll back the observation interval.
- MP4 readiness is independent from Measurement readiness. The recording projection reports
  `video=STARTING` until each allocated side-specific path exists and grows beyond its allocation
  size, then reports provisional `RUNNING`; one failed side is `PARTIAL`, and no growing side is
  `FAILED`. The GStreamer camera pipeline supplies decoded NV12 to a bounded, oldest-drop queue;
  an isolated FFmpeg `h264_nvenc` process writes lossless H.264 MP4 with no application bitrate,
  file-size, or duration cap and no warning burn-in. Online post-run state is presence-only:
  `SAVED` means both sides produced a file, `PARTIAL` means only some files exist, and `FAILED`
  means none exist. These labels do not claim playback or integrity verification.
  Video failure never turns a frame-ready Measurement back from `RUNNING`, but Workstation does not
  claim video recording is active and preserves the per-side failure evidence.
- `Settings > YOLO / RTSP > H.264 compressed queue` controls only the two compressed H.264 queue
  boundaries and applies on the next camera Preview start. `Latest (drop old)` is the default
  low-latency policy. `Reference-safe (preserve 8)` keeps up to eight compressed access units at
  each boundary, which can reduce Workstation-induced reference loss but may increase latency or
  backpressure. It never changes WIRIS `SEIR`, decoder choice, Measurement gates, or tolerant
  decoded-frame admission.
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
  not success: Ethernet must enter `RECOVERING` and complete a fresh 10-second qualification.
- Treat the Pixhawk Discovery table as the result of the most recent accepted `Find USB` or
  `Find network` command, not as accumulated history. A second click while another Pixhawk
  Discovery worker is active is rejected without clearing the in-progress snapshot. Empty/error
  completion remains empty/error and never restores older candidates.
- USB Ethernet profile persistence and runtime stall recovery are separate operations. The verified
  recovery command writes no PX4 parameter/config/file and never flashes firmware. Automatic reboot
  is not supported.
- Workstation never starts PX4 calibration-parameter readback from startup, a packet, heartbeat,
  status poll, reconnect, or the transition to `LIVE`. The operator explicitly uses `Refresh P1/P2
  parameters` (or the existing Advanced MAG/attitude refresh); reset/apply/save operations retain
  their required verification readback. Until then the calibration rows remain `PARAM readback
  대기 중`. During an explicit parameter read, that side's normal
  ATTITUDE/GNSS receive loop is intentionally reserved. Its status
  shows `PARAM SYNC` with detail `PX4 파라미터 조회 중 (정상)` instead of falsely reporting
  `STALE` or `DISCONNECTED`. Nonmatching ATTITUDE/GPS messages consumed by the blocking PARAM or
  shell receive path are deferred and replayed through the normal handler after the reservation;
  they are not discarded. Stable-PX4 RM3100 polling uses a distinct `SHELL SYNC` status with
  detail `PX4 Shell sensor_mag 조회 중 (정상)` and is not presented as parameter lookup. P1/P2
  remain independent, and identical receive errors are reported at most once per second per side.
- Pixhawk connect and disconnect operations are mutually exclusive. Their P1/P2 buttons are
  disabled while either worker owns the subscriber; rapid repeated clicks cannot start an
  overlapping socket transition or publish a connected label for a closed transport.
- P2 Ethernet uses the configured NIC, local address, route, peer endpoint, and expected-system
  Heartbeat/ATTITUDE checks. It does not change PX4 settings automatically; board-side Ethernet
  provisioning remains an explicit per-board ETH MAV/USB operation.
- Base GNSS serial uses a one-second write timeout and a 64-item latest-preserving output queue.
  A blocked USB/radio writer emits `failed`, closes the port, and lets lifecycle shutdown continue
  instead of keeping the Base status falsely connected. PC-IP elevation through `pkexec` has a
  15-second authorization timeout and returns an actionable `sudo ip addr add ...` fallback.
- A dual-camera WIRIS command treats a broken synchronization barrier as partial failure: the
  healthy camera continues without waiting for or inheriting the peer's error. Interactive zoom
  contention is reported as control-channel busy and retried up to five times; it is not labeled
  as a camera failure. TCP command responses are accumulated across fragments until terminator,
  EOF, bounded idle, or the request deadline.
- Treat the Baseline Yaw90 midpoint reference as a GNSS antenna-midpoint local-level reference only.
  Compute `v=Drone-(P1+P2)/2` in local NED, Right from horizontal P1→P2, Down from NED Down,
  Forward as Right×Down, `Rg=√(F²+R²+D²)`, and `angle=atan2(|F|,|R|)`. Never write these values into camera
  `baseline_center_frd`, left/right optical, Measurement, or recording fields. A zero/vertical-only
  horizontal baseline waits; age over 2.0 s, future time, or three-source spread over 1000 ms is
  STALE. Fix quality is shown but is not a gate. When recording is active, save all three states to
  the dedicated bulk stream with exact selected-source clocks/provenance; never infer persistence
  from the live UI. Writer/drop/error state in Data Status is authoritative.
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
  packets. Manual and Auto Live timing uses the decoded-frame app-receive monotonic timestamps;
  missing receive evidence never falls back to RTCP/NTP. Camera-native `CPTR` stills remain
  separate `Capture HR Still`/`Capture OZ Still`
  actions, with `(FTP)` appended only when the still is downloaded immediately. Manual, Auto,
  and FTP captures over `Max Live pair Δt` still save every side whose checkerboard validation
  succeeds, mark those files `INTRINSICS ONLY`, and exclude them from stereo `R,t`. Auto capture
  consumes each new frame identity once and counts only timing-eligible two-sided captures as
  valid stereo pairs. Calibration photo-set manifest `pair_count` uses the same R,t-eligibility
  rule; K,D-only files remain visible in the selected Photo set without inflating that count.
- Auto capture is stricter than manual Live Pair: each candidate must be a parser-normalized AU
  containing NAL type 1, excluding NAL5, with `DELTA_UNIT=false`, and its AU PTS must equal the
  decoded-buffer PTS. Checkerboard detection and PNG persistence resolve that exact current-camera,
  current-generation `FrameKey`; timestamp-near and generic-latest fallbacks are forbidden.
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
- While the run is `RUNNING`, a primary `VALID`/`ESTIMATED` outcome is priority-enqueued before its
  table, history, plot, and UI banner update and without a comparison barrier. Stop preserves an
  already accepted queue record. A primary first received after Stop changes neither
  `measurements.jsonl` nor the UI. A primary received while RUNNING whose enqueue was rejected keeps
  background ownership; after Stop it may append only late evidence and cannot update the new run
  UI. Enqueue is asynchronous and is not a completed write, flush, or `fsync` claim.
- Structured data is recorded only while Measurement or Manual record is active. Explicit capture,
  import, and Alignment commands always append their own event even outside those intervals.
- Storage is best-effort. A setup/open/write/serialization/fsync failure drops the affected record
  or pauses only that stream and is reported in `Data`; it never stops the recorder or raises through
  a Qt callback into Measurement admission. `Retry Storage` clears failed-stream state.
- Native PNG completion is joined to its append-only recording row by the unique async request ID,
  append-admission lease, and original row owner,
  independently of the 1024-pair lookup cache. A stale completion for an older request cannot
  overwrite a newer pending request for the same pair or a same-index row in the next run. The
  original row owner remains bound until completion; sessionless completion also attempts a
  best-effort `native.frames` event with that original runtime generation. The disk-I/O barrier is acknowledged
  when the writer finishes even if row projection must be deferred, so a later row-construction
  failure cannot hold the next run open. Drone RTK Multi-view `BLOCKED` samples stay in audit evidence
  but are not considered collected; only `READY` samples clear the target editor when the next
  Frozen burst arrives.
- The Measurement view displays the registered methods’ frozen/live source values independently. Missing method
  instances remain visible as blocked catalog rows, while each pair persists one atomic bundle with
  method ID, instance ID, component sources, geometry hash, status, warnings, and worker timing for
  every method. That later comparison bundle does not gate or rewrite a successful representative
  row already priority-enqueued while the run was `RUNNING`. The run workbook is updated
  asynchronously at checkpoints, Stop, and late completion; Parquet is not materialized.
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
- Pixhawk Shell/PARAM results apply only to the connection on which the request started.
  Replacing or disconnecting that connection discards its late readback; Stop retains the
  running worker until it actually exits. Different sides retain their existing concurrency.
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
  Frozen detection metadata retains every descriptor-hash input, including exact integer
  `app_receive_monotonic_ns` and compatibility `timestamp_s`, so a later audit can recompute the
  digest rather than only compare copied values.
- Raw camera frames update the latest calibration snapshot without broadcasting a duplicate
  calibration-preview signal. Hidden legacy/advanced preview widgets do not render those frames;
  checkerboard overlays remain available through the throttled overlay path or explicit Refresh.
- The normal GStreamer H.264 path repeats SPS/PPS at every IDR (`h264parse config-interval=-1`) and
  uses latest-only leaky compressed queues plus an eight-buffer leaky decoded queue. A stalled
  consumer drops stale work rather than back-pressuring depay/decoder. `decoded_hz` uses a
  five-second rolling window and returns to zero after a genuine gap.
- RTP sequence gaps and absent IDR remain latched as provenance but do not suppress the current
  decoded frame. Preview, YOLO, native evidence, official Measurement, and the decoded-NV12
  recording capture continue with `INPUT UNVERIFIED`. The true decoded-frame/source timeout remains
  terminal per side; the healthy peer remains live. Workstation does not change GOP/IDR or any
  other camera encoder setting automatically, and there is no strict/tolerant runtime switch.
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
  lookup and ordinary Live/Auto Calibration pairing use the frame's application-receive monotonic
  stamp; missing receive evidence makes that capture K,D-only rather than falling back to NTP. An
  absent clock-domain reference remains `unknown`. An FTP pair with the same capture ID and command round, successful storage,
  download, and decode on both sides may use the operator-approved
  `command_round_assumed_sync` basis for R,t; it records `pair_delta_s=null` and an explicit warning
  that simultaneous exposure was assumed, not measured. Other FTP/import evidence remains
  `SYNC_UNVERIFIED` rather than borrowing a live-preview timestamp.
- Workstation live distance follows the frozen Measurement timing preference. App receive is the
  default; healthy RTCP/NTP is selected only with VERIFIED/current same-epoch evidence. Any RTCP
  timing fault falls back to exact decoded-frame app receipt, and that fallback alone does not
  change normal VALID timing. New schema-v5 rows store requested/effective basis, fallback reason,
  selected Right-minus-Left delta, and separate app/NTP deltas. Existing schema-v3/v4 rows retain
  their original meaning.
- A dual-camera `Set camera clock` operation sends Left STUS first and sends Right only after Left
  returns an exact `OK` ACK. Only STUS mutation responses may wait up to 15 seconds; generic SDK and
  HIWS retain their configured timeout. One action permits at most two rounds within 90 seconds and
  uses fresh replacement-generation SR feedback for the second round. RTCP observation runs in
  parallel, but external clock mutation remains ordered for failure isolation. STUS is available
  during Preview, Measurement, YOLO and recording and restarts neither CameraPane nor the YOLO/MP4
  path; only the decoder-free RTCP observer starts a new observation generation.
- Clock success is relative: the robust three-SR Left/Right mapping delta must be at or below the
  configured 100 ms default. Per-camera signed/raw `SR-PC` remains visible and append-only evidence,
  but its absolute magnitude never gates success, revokes `VERIFIED`, triggers Auto or blocks a
  timing result. One isolated SR outlier is tolerated by the median/MAD window; span remains
  diagnostic. Missing/stale/regressed SR, sustained relative delta failure, host-clock step and
  clock-rate evidence retain their explicit Clock failure behavior. When RTCP/NTP is preferred,
  these failures activate app-receive fallback rather than making Clock appear healthy. An
  occasional unmatched/outlier frame may be dropped without revoking Clock.
- Settings provides persistent `Manual`/`Auto` STUS mode and a 5–1440 minute interval (default
  Manual/30 minutes). Auto requires one successful manual seed after each application start or new
  camera connection. A scheduled action runs only at its deadline, including during active work.
  Clock, Frame timing and Media status remain separate throughout the transition.
- Calibration zoom A and operating zoom B are arbitrary supported zoom levels. The system
  does not prescribe 1×, 8×, or any particular A→B combination.
- Cross-zoom application uses `K,D` from operating zoom B and `R,t` estimated at calibration
  zoom A.
- Every measurement method requires the selected left/right `K,D` bundle, matching hashes, and
  matching native raster. Zoom readback, zoom agreement, camera serial, and stabilization do not
  gate or warn after application. Cross-zoom `R,t` remains allowed, but it cannot waive the
  profile-identity/native-raster intrinsics gate. Methods that
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
- Horizon and Mountain `Use suggestion` ignore detector quality state/confidence while preserving
  them as visible and persisted evidence. Horizon owner validation requires current frozen
  identities/dimensions and exactly two finite in-frame endpoints per side separated by at least
  25% of native width. Mountain requires equal ordered Left/Right counts and at least four finite
  in-frame unique/distributed pairs after shared exclusions. `NO_DATA` normally has no usable
  points and remains structurally blocked; incomplete, stale, out-of-range, duplicate or degenerate
  proposals remain blocked. Editing accepted points changes provenance to `MANUAL_EDITED`.
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
  the workbook retains explicit not-attempted states for methods without ready inputs. No fake R,t is created and the primary
  result remains blocked instead of falling back to a comparison method.
- Candidate review keeps left/right observability separate. The common Apply/Combine axis is
  enabled only when both camera sides observed that axis; one-sided evidence remains visible in
  the detail text but cannot be promoted as a two-camera contribution.

## Known Limitations and Workarounds

### Magnetometer RAW/PX4 yaw is an observation aid

`RAW yaw°` uses the raw producer-axis vector and `atan2(-y,x)` only. It does not apply
`CAL_MAG` correction, sensor/board rotation, tilt compensation, magnetic declination, or PX4
estimator fusion. `Corrected yaw°` applies CAL_MAG correction and sensor rotation but does not apply
magnetic declination. `PX4 yaw°` is the firmware `ATTITUDE.yaw` value after PX4's own selected-sensor
processing and configured declination handling, but it is shown beside a row only when that row's exact device ID matches the active
`vehicle_magnetometer` identity and its ATTITUDE sample is fresh. Standard `SCALED_IMU*` telemetry
does not carry that physical device ID, so its corrected cell may remain `ID 확인 대기` even when
RAW yaw is available.

The IMU magnetometer table also contains RM3100 rows sourced from `DEBUG_VECT` names
`CAN1_MAG`/`CAN2_MAG` or from Stable PX4 Shell `listener sensor_mag` readback. Shell rows retain
the `px4-shell/sensor_mag/<instance>` topic and Gauss unit, and use the same RAW/PX4 yaw gates as
the dedicated CAN table. Shell polling is diagnostic-rate and bounded; timeout or malformed output
ages the last sample instead of fabricating a live MAVLink value. PX4 shell
`listener vehicle_magnetometer -n 1` provides the active device identity for the PX4 yaw gate.

Workaround: when comparing a priority A/B test, change one `CAL_MAG*_PRIO` through the existing
disarmed workflow, verify `Actual selected MAG`/`vehicle_magnetometer.device_id`, and use the
fresh `PX4 yaw°` value as the estimator result. Do not interpret a stale or fallback row as proof
that the requested MAG was used.

### Baseline−P1/P2 is a latest-state physical-alignment aid

The Pixhawk view combines the latest selected P1/P2 GNSS positions and latest independent
ATTITUDE samples at the 4 Hz UI poll. It exposes GNSS ages, GNSS pair delta and each attitude age,
but it does not interpolate the streams or claim one simultaneous measurement timestamp. It also
uses only horizontal North/East direction; Down does not affect the displayed degrees.

Workaround: use the view while the assembly is stationary or moving slowly and require CURRENT
source evidence. For offline reconstruction, join the existing `pixhawk_attitude` rows
(yaw/quaternion and app/PX4 timestamps) with `gps_rtk` rows (receiver role, WGS84 position, fix and
timestamps) and apply the documented formula. No additional derived-angle field is persisted.

- Passive discovery requires the Drone to be powered and already transmitting UDP MAVLink onto the
  connected LAN. If no candidate appears, verify carrier/PX4 Ethernet configuration or use the
  preserved manual URL/USB setup path.
- A port already exclusively owned by another process cannot be scanned. Stop that owner or use a
  different configured listen port; discovery reports failure rather than interrupting it.

### GNSS BASELINE_CENTER is not the camera measurement frame

The displayed F/R/D/Rg origin is the midpoint of the P1/P2 GNSS antennas, not the optical centers.
Right is the horizontal P1→P2 direction and Down is local NED Down, so camera/body tilt,
antenna-to-camera lever arms, and camera alignment are intentionally absent. Swapping P1/P2 reverses
the signed horizontal axes. A moving platform can also move materially during the allowed 1000 ms
three-source receive-time spread even while the row is `CURRENT`.

Use this row for live GNSS relative-position reference and field diagnosis. Inspect its actual
`Max age`, `Max Δt`, and fix labels, and prefer a stationary or slowly moving setup when signed
components matter. Use the existing optical/Measurement result for camera-frame distance and pose;
recorded derived rows remain asynchronous latest-sample evidence rather than simultaneous antenna
exposure. At 5-10 Hz, allow approximately 72-144 MiB/hour of additional best-effort JSONL. Bulk
queue pressure may drop rows; compare Data Status counters and retain the raw PX4 files for audit.

### P2 Ethernet connection remains explicit

Select the configured Ethernet transport, use `Find network` and `Assign` only when the NIC/route
must change, then press `Connect` and verify the expected P2 Heartbeat/ATTITUDE. Workstation does
not run a separate topology-provisioning flow or automatic failover. If the host link is up but P2
does not answer, inspect the cable/switch path and PX4 `eth0` separately.

### USB role identity requires a current heartbeat

`Find USB` can enumerate a serial endpoint even when its air peer or Pixhawk is off, unpaired, busy,
or silent. That one physical device remains visible as `identity unverified`; USB order, tty number,
and descriptor never substitute for P1/P2/Drone identity. Power or repair the intended independent
radio pair and run `Find USB` again. Assign only the row whose observed sysid matches the named role;
Discovery itself does not assign or connect it.

### GNSS baseline↔Drone installation angle is a latest-state aid

The Telemetry view combines the latest selected P1/P2 GNSS positions with the latest Drone GNSS
position at the 4 Hz UI poll. It exposes the midpoint-relative F/R/D/Rg values, the acute horizontal
angle `atan2(|Forward|,|Right|)`, and absolute `90° error=|90°−angle|`. Drone ATTITUDE/yaw is not
used, and the view does not interpolate the streams or claim one simultaneous exposure timestamp.

Workaround: use the view while the rig is stationary or moving slowly and require CURRENT source
evidence. A STALE row retains its finite values and exposes the actual age/spread reason. The angle
is a live presentation value; no additional derived-angle field is persisted.

### Stop can precede an asynchronous representative result

The representative worker and storage writer are asynchronous. If a `VALID` or `ESTIMATED`
`pixhawk_rtk_live` outcome arrives while the Measurement is still `RUNNING`, Workstation preserves
the accepted priority-queue record even if the operator presses Stop immediately afterward. If Stop
changes the run out of `RUNNING` before that primary outcome arrives, Workstation intentionally does
not create a `measurements.jsonl` row and does not update the distance UI; the outcome remains only
in method-evaluation evidence. Priority enqueue also does not prove that the bytes have already been
written or `fsync`ed.

Workaround: when a representative distance row is required, wait until the Pixhawk attitude +
GNSS/RTK distance appears in the Measurement UI before pressing `측정 종료`, then confirm the
`measurements` stream in `Data > Status` if durable storage timing matters. There is no need to wait
for unused comparison methods or the completed submitted-method bundle. If `Data > Status` reports the
store as `CLOSING` or `CLOSED`, no new result can enter that store; start a normal new run with an
`OPEN` store rather than treating a late enqueue rejection as a saved result.

### Operating K,D does not protect against wrong zoom or camera identity

After a K,D profile is active, Workstation intentionally does not reject or warn on missing,
stale, future, or mismatched zoom readback, left/right zoom disagreement, physical-camera serial,
or stabilization state. Those values remain evidence only. The operator must therefore select the
intended calibration file; applying K,D from another zoom, physical camera, or image-warp state can
bias rays, distances, and poses without a readiness error.

`intrinsics_zoom_08_20260831.json` is structurally usable for the current native 1280x720 RTP path.
It remains incompatible with 1920x1080 native images because resolution is still a hard gate. The
file contains K,D only, so methods requiring stereo R,t also need a separately selected extrinsics
profile. This statement is about interface/applicability, not calibration quality.

### The bundled 2026-08-20 calibration photo set is not directly applicable

Verification on 2026-08-29 found that
`calibration/2026-08-20_18-21-24/AutoCapture/2026-08-20_18-49-57` contains mixed 1280x720 and
1920x1080 inputs, while the photographed board has 10x8 inner corners and the inspected runtime
setting was 13x8. Running the current selection unchanged therefore fails before K,D optimization.

Selecting only the homogeneous 1280x720 images and using the photographed 10x8 pattern proved that
the real `CheckerboardCalibrationWorker` completes: Left/Right each retained 10 representative
poses, produced finite 3x3 K matrices and five D coefficients, and reported about 0.212/0.223 px
reprojection RMS. The combined quality state was nevertheless `FAIL` because coverage, perspective
diversity, effective-pose count, principal-point stability, and subset stability were insufficient.
Do not Apply that result. Recapture at one raster/zoom with the measured square size, move the board
through image center/edges, distance, roll, yaw and pitch, then require the quality panel to pass.

### Calibration Auto capture depends on dual-camera NAL1 cadence

NAL1/non-delta is an operational non-IDR intra category, not an IDR or source-integrity guarantee.
Independent Left/Right encoder cadence may produce few candidates within `Max Live pair Δt`, so
Auto can wait longer or retain timing-ineligible observations for K,D only. If AU provenance is
missing, a FrameKey was overwritten, or a non-GStreamer compatibility source is active, Auto waits
for a new candidate and never substitutes an adjacent frame. Use manual `Capture Live Pair` when
automatic NAL1 provenance is unavailable; manual capture retains its existing freshness and timing
gates.
Resetting `이번 촬영` does not create a new AutoCapture directory. Stop→Start in the same Workstation
process continues writing to the current capture set, so use the photo selector/exclusions or begin
a separately owned calibration set when a physically isolated dataset is required.

Stopping Auto lets an already-started capture finish saving and report its result, while leaving
the stopped Auto pair count unchanged. Closing Workstation waits for that result and cleanup,
and prevents the next queued FTP capture from starting. FTP synchronization retains a photo pair
in its queue until capture startup is accepted, so a busy worker or rejected start does not silently
discard that pair.

### Previous Measurement data is not migrated automatically

Changing the default does not copy the existing `.local/workstation_data/measurement_data/`
append streams into `recordings/`; blindly concatenating active JSONL files could duplicate
operation indices or mix unrelated run histories. Treat the old folder as read-only history and
start new collection in `recordings/`. If a later offline archive needs both, merge them with an
explicit provenance-aware process. Each new writer file set is timestamped and remains append-only, so
monitor free space on the repository volume. Runtime output under repository-root `recordings/`
must not be added to source commits.

### Dynamic Camera/Pixhawk NIC roles

Role application requires a graphical PolicyKit authorization agent and NetworkManager `nmcli`.
Headless/offscreen operation fails closed with a concrete manual command instead of silently
changing the host. One-NIC Camera+P1+P2 operation is supported, but the earlier shared-L2 RTSP-load
MAVLink stall means it remains outside long-duration field qualification. If rollback is incomplete,
the UI reports the exact partial NIC state; inspect NetworkManager and use Find/Assign again rather
than assuming the saved role is active.

### Dr/P1/P2 geometry is a latest-state GNSS aid

The Pixhawk geometry group uses the latest selected P1/P2 rover positions, the latest Drone GNSS
position, and the latest P1/P2 ATTITUDE samples at the existing UI poll. `Dr from P1/P2` means
reference→Drone expressed in that Pixhawk's body-FRD frame; `Dr from Baseline Center` uses the
GNSS antenna-midpoint local-level FRD frame. The two displayed angles are `∠Dr-P1-P2` and
`∠Dr-P2-P1`, calculated from horizontal North/East rays only, so they are not 3D angles or
Pixhawk-yaw alignment verdicts. The streams are not interpolated or made simultaneous, and no
derived vector/angle row is written to the existing evidence files.

Workaround: require `CURRENT` source evidence before using the values for physical inspection.
If a P1/P2 attitude is missing, only that board's body-FRD row is unavailable; restore the
existing board connection/ATTITUDE source and let the read-only group refresh.

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
- Workstation now checks the CUDA Driver API directly with `cuInit(0)` as part of the startup
  capability gate. A `nvidia-smi` success combined with `CUDA_ERROR_UNKNOWN (999)` is reported as
  NVIDIA unavailable and deterministically selects the existing Auto→`avdec_h264` path.
- Loading GStreamer's installed `nvcodec` plugin fails CUDA initialization with `0x64`
  (`CUDA_ERROR_NO_DEVICE=100`). The plugin consequently registers no usable `nvh264dec` factory,
  and `gst-inspect-1.0 nvh264dec` reports `No such element or plugin`.
- Camera `Auto` records `gst_parse_error: no element "nvh264dec"` per side and correctly falls back
  to `avdec_h264`. The observed UI showed both sides as `CPU fallback`, about `5.9 Hz`, with one side
  becoming `STALE` under the extra CPU load.
- If the saved selection is explicit `NVIDIA`, pressing Measurement reports the unavailable decoder
  without terminating an already-live software or prior-generation Preview. Choose `Auto` or
  `Software` before retrying Measurement; explicit mode remains fail-closed.
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

App-receive monotonic is the live-distance default and mandatory timing fallback, and remains the
ordinary Live/Auto Calibration pairing source. Its provenance is decoded-frame receipt at the host,
not physical camera exposure, and includes per-side network/decode latency. Selecting healthy
RTCP/NTP can improve stereo frame selection, but PX4 attitude/GNSS association still uses host
monotonic time; no camera-to-PX4 clock conversion is claimed. Calibration continues classifying app
receipt as `HOST_RECEIVE_PROXY` and never precision-qualifies exposure from receipt alone. Keep the
checkerboard stationary; true simultaneous exposure still requires verified exposure timestamps or
qualified hardware triggering.

The nine comparison processes run concurrently while Pixhawk/RTK has its own worker. CPU or
storage contention can lengthen queue/evaluation time or fill a method's 20-pair limit. Diagnose
`queue_wait_ms`, `evaluation_duration_ms`, `BACKPRESSURE_BLOCKED`, and `Data > Status > Method 계산 상태`.
Elapsed time is observational and never establishes a dead worker. After inspecting the selected
method/run/pair, `선택 계산 중단·재시작` interrupts that current calculation as `FAILED` and replaces
only its worker. Queued jobs and other methods continue. A stale selection is rejected; queued
recovery is distinct from a confirmed replacement. If replacement fails, the same surface exposes
an explicit retry. Normal Measurement Stop keeps its existing non-cancelling behavior.


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

Camera JSONL rows describe GUI-delivered frame observations, while MP4 contains the decoded NV12
recording branch before Preview/YOLO overlays. A bounded overload drops the oldest recording frame;
fixed-rate gap duplication preserves approximate elapsed duration, so no exact 1:1 source/MP4 frame
index is claimed. Use the exact native PNG bound to a terminal pair when pixel-level replay is
required.

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

A sender that never emits H.264 IDR NAL units has no verifiable compressed-stream random-access
boundary. Workstation now keeps usable decoded output flowing instead of waiting for that boundary,
and labels it `INPUT UNVERIFIED`. It cannot reconstruct missing reference pictures or guarantee
that a decoded frame is free of concealment artifacts; existing Measurement gates evaluate the
delivered pixels, not the unavailable original compressed-media truth.

On 2026-08-27, an operator-approved exact `GEIR VISIBLE=1` readback on both FW 2.18.72 cameras still
produced zero type-5 NAL units in 15-second elementary captures: Left had 206 AUs with nine NAL1
intra AUs, Right had 112 AUs with three NAL1 intra AUs, and `ffprobe` marked every frame
`key_frame=0`. Sequential SDK reboots were later completed. Left rebooted with IDR reset to 0;
Right rebooted with Ethernet mode off and profile `0/17/16/0`, then was restored to the approved
`GETH=TRUE`, `0/1/30/1` state. On both models, fresh streams and an additional `SEIR VISIBLE 1`
issued while RTSP was active still produced zero NAL5. That sender output remains unrecoverable by
the camera/RTSP source and still requires Workswell firmware/vendor correction. It no longer blocks
decoded Preview, YOLO, native evidence, official Measurement, or decoded-lossless MP4; all retain
explicit unverified-source provenance. Clock success remains independent from Frame and Media.

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
`run.sh --check` to pass before operation. `apps/Workstation/calibration/` is the active default
Calibration owner and remains ignored by Git. The former `.local/workstation_data/calibration/`
tree is preserved as historical input and is not copied, merged, or deleted automatically.

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

Workstation still does not generate legacy Session `review.xlsx` or typed Parquet automatically.
It now generates a timestamped root-level `latest_measurement_<timestamp>.xlsx` convenience view for
the latest Measurement only; earlier Measurements and Manual-record-only intervals remain in their
timestamped JSONL source files.
Large exports are deliberately background/idle work: the representative current-data copy produced
42,425 workbook rows in about 62 seconds (23.4 MB XLSX, about 28 MiB peak RSS), and a new recording
may cancel or defer that work. Use `Update Latest XLSX` after the system is idle if the status is
stale/failed/cancelled. A foreign unmarked file with the same name is preserved; rename or move it
before retrying. Multi-run or authoritative analysis must still read timestamped/legacy JSONL, interpret
append/replace operations by `_record_index`, retain signed nanosecond timestamps, and join only
where the recorded identity/time contract supports it.

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
isolation, and verified-USB recovery followed by a 10-second product requalification plus at least
10 minutes of post-reboot observation. Until all of those pass, do not describe the field issue as
resolved or update `Pixhawk.md` with a remedy.
The two camera decoder child processes may still consume substantial CPU while Preview is active;
this is expected producer-side work and is separate from GUI responsiveness. Stop Preview when
video is not required. If a build predating 2026-08-23 was force-quit, terminate its orphaned
`multiprocessing.spawn` camera children once before starting the updated Workstation.

The NGII profile does not identify the Base serial port or receiver firmware automatically in
the settings file. Use Base receiver discovery or confirm the connected F9P device and its
firmware before field operation.

Automatic Horizon detection can confuse clouds, trees, roofs, or multiple strong boundaries
with the sky/terrain edge; inspect its confidence, residual, and overlay before approval. Quality
does not block copying, and structural endpoint readiness does not prove the detected line is a
physical horizon. Use manual endpoints when the overlay is wrong. Only a physical/geometric horizon
compatible with the selected `horizon_model` is valid -- a mountain ridge, building skyline, or
tree line is not; use the Mountain method for a ridge.
Automatic Mountain matching depends on shared visible texture and can be weak in fog,
low-texture terrain, vegetation, or repeated patterns; a failure or low-confidence result
must still be reviewed. A structurally usable low-confidence proposal may be copied for editing;
otherwise use manual clicks. Mountain auto-selection does not create terrain heights or 3-D PnP
correspondences and does not replace the GNSS-baseline rotation solver. Terrain PnP
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
paired image bearings; capture-time GNSS supplies a metric baseline; capture-request P1
Roll/Pitch and GNSS Yaw90 supply the assumed Left anchor. The *direction* of that baseline conditions a two-stage
robust epipolar-rotation fit (`(u,v)` correspondences), while the baseline's *metric length* only
scales the resulting translation -- the fitted rotation and its normalized Sampson residual are
provably invariant to the baseline's metric length. Correspondences are classified inlier/outlier
by a normalized Sampson threshold after a soft-L1 seed solve, and the candidate is only
`apply_eligible` when the final fit has full rotation rank and finite rotation/residuals.
Condition and inlier count/ratio are quality warnings under the availability policy.
GNSS translation is `apply_eligible` when the same frozen Capture contains finite
Left/Right latitude, longitude, and altitude and a finite non-zero P1-to-P2 NED baseline.
Low/unknown fix quality is a warning. `RTK_FIXED` is
retained as quality evidence, not a Mountain translation blocker; a usable non-fixed baseline
keeps explicit warnings and its `mountain_gnss` Measurement outcome is `ESTIMATED`. Mountain never
measures the Left absolute pose -- it remains the
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

- 2026-10-11: Fixed moved-module compatibility detection so recording callbacks no longer bind
  back to their own panel delegates. Clearing a missing persisted intrinsic path during settings
  save also prevents a stale asset reference from holding application shutdown open.

- 2026-10-10: Decomposed the four god modules into workspace, panel, recording and measurement
  owners. Preserved named compatibility APIs, single state/resource ownership, Qt affinity,
  generation/lease contracts and canonical coordinates. Frozen geometry now captures method
  selection/configuration before resolution. Added architecture gates and loopback-only isolated
  regression tooling; see [the plan](docs/changes/20261010-god-module-decomposition/implementation-plan.md)
  and [the result and validation report](docs/changes/20261010-god-module-decomposition/report.md).

Historical entries below retain the behavior at their stated date. Current contracts above and
`docs/MEASUREMENT_RUNTIME_CONTRACT.md` supersede retired timeout/MAG-admission descriptions.

- 2026-10-09: Fixed nested measurement payload ownership and detached JSON exports; separated Run
  state into a Qt-free owner with STARTING reservation before optional I/O; added method execution
  status and explicit identity-guarded worker recovery; repaired CI targets and content-bound
  historical path classification. See the linked RCA packet for evidence and remaining limits.


- 2026-09-08: Added `05_그래프` with twelve native Method-comparison charts for camera FRD,
  horizontal FRD and WGS84 results. The summary links to the charts; missing values retain
  gaps and empty charts show `데이터 없음`. Kept time labels below negative Down/height plots
  and removed arbitrary empty-chart scales after rendering verification. Chart source fields,
  stored precision and measurement calculations are unchanged. See
  [verification evidence](docs/changes/20260908-measurement-result-charts.md).

- 2026-09-08: Measurement displays participating Methods in one pair-consistent result
  table, switching between original FRD, existing method-midpoint/P1 local-level FRD,
  and WGS84. Quality, coordinates and R,t details come from the selected Method/pair
  record, not shared labels or the last primary history value. A displayed pair stays
  until all rows terminate or one second elapses, then jumps to the latest available
  pair. This display deadline never cancels workers or changes stored outcomes.
  Late outcomes remain in their own run/pair. BLOCKED/PENDING rows have no carried-over
  numbers; partial geolocation affects only coordinate availability. Row selection is
  read-only and does not change the representative Method. Existing preview/history
  remains separate. No coordinate formula, archive schema or calculation policy changed.

- 2026-09-08: Enabled independent 1×–8× mouse-wheel display zoom in the Baseline
  Yaw90 live previews. Zoom persists across incoming frames and returns to the
  centered full image at 1×. Corrected the Center line to clip at its transformed
  native position instead of appearing at a viewport edge when the image midpoint
  is outside the zoomed view. See [verification evidence](docs/changes/20260908-baseline-yaw90-preview-zoom.md).

- 2026-09-08: Mountain/Cloud use capture-request P1 Roll/Pitch and P1→P2 GNSS Yaw90
  (existing branch/Reverse rule) with identity initial relative rotation. Prior stereo
  R,t is not required. Camera virtual FRD is assumed aligned with P1 body FRD; antenna
  centers approximate optical centers. RTSP/FTP requests freeze inputs before completion;
  later telemetry/Reverse changes do not retarget a frozen image. Missing inputs block
  Solve, not Capture. Imported images without original reference evidence need recapture.
  Both candidate poses and the full Yaw90 reference are persisted. Canonical attempts add
  optional checksummed pose evidence: Mountain capture positions or Cloud pair positions,
  both actual orientations and R,t. CSV record_json retains it; XLSX section D expands it.
  Empty legacy fields preserve old hashes. Restart, Capture, Solve, then start a new run.

- 2026-09-08: Mountain/Cloud use `mountain-cloud-availability-v1`: finite full-rank
  rotation fits remain usable when inlier count/ratio or condition quality thresholds fail.
  Finite non-zero capture GNSS baselines remain usable with low/unknown fix quality,
  recorded as warnings. Missing inputs, invalid K,D/frame identity, rank deficiency,
  non-finite rotation/residuals and Cloud known excessive frame delta still block.
  New candidates must be solved again; existing candidates and historical records are not
  reclassified. Ready Method tooltips expose warnings; computed warned results are ESTIMATED.
  Mountain/Cloud outcome records additionally expose usable, blocking_reasons,
  quality_warnings and gate_policy; legacy hashes retain their canonical payload contract.

- 2026-09-07: Added start-click-owned Measurement directories and JSONL/CSV journal recovery.
  Reworked XLSX into five reader tabs plus preserved evidence, with a dedicated R,t tab and
  expanded R,t on every Method tab. Added CSV reconciliation, per-run late-result routing,
  and saved-input XLSX rebuilding. Coordinates and solver arithmetic are unchanged.

- 2026-09-07: Consolidated the Baseline Yaw90 installation workflow in an appended Alignment Method
  tab with live azimuth, vertex-angle and R/P tables, local Center line, and guarded Reverse
  Apply/Save. Moved Telemetry installation details and removed Calibration's Center line control;
  shared Pixhawk displays and existing measurement/recording contracts remain unchanged.
  Review follow-up: clear scoped Inspector details on tab exit and refit previews on viewport
  resize, including Diagnostics-dock changes.
  Display-query failure follow-up: retain confirmed values with advancing ages and consistent
  table/Inspector error state, recover automatically, and preserve Measurement input caches.

- 2026-09-04: The Pixhawk IMU magnetometer table now keeps the three standard `SCALED_IMU*`
  rows and appends RM3100 `CAN1_MAG`/`CAN2_MAG` rows received through stable-firmware
  `DEBUG_VECT`. RM3100 keeps its `mavlink/DEBUG_VECT` topic, Gauss unit, and the same RAW/PX4
  yaw correlation rules; no private `SASL_CAN_MAG` stream request was added.

- 2026-09-04: Added per-row Pixhawk magnetometer `RAW yaw°`, exact active-device `PX4 yaw°`, and
  wrapped `Δyaw°` readouts for IMU/CAN Magn telemetry. RAW remains explicitly sensor-axis
  diagnostic; PX4 yaw is shown only for a fresh ATTITUDE sample matched to the verified active
  device, with no priority, firmware, stream, persistence, or measurement semantic change.

- 2026-09-05: Stable PX4 RM3100 readback now polls `listener sensor_mag` and
  `listener vehicle_magnetometer` through the existing exclusive MAVLink Shell owner. The five
  fixed rows per Pixhawk keep raw/PX4/Δyaw provenance and do not require custom firmware.

- 2026-09-08: Made `PX4 yaw°` visible in the Pixhawk IMU/CAN magnetometer tables. It is the
  fresh active-device-matched `ATTITUDE.yaw` estimator value with PX4 magnetic-declination
  handling; the hidden `Δyaw°` remains `PX4−RAW` and RAW/CAL_MAG-corrected yaw semantics are
  unchanged.

- 2026-09-05: Added independent P1/P2 MAG Priority controls to `Devices > Pixhawk`. Runtime apply
  now has a visible active-device/fresh-ATTITUDE verification state, and live-Pixhawk Measurement
  start records the verified MAG selection and blocks unverified required sides.

- 2026-09-05: MAG Priority cards now retain the live stable-PX4 `sensor_mag` instance/device map
  and show the priority candidate beside the actual `vehicle_magnetometer` device. Priority
  controls remain editable, including CAN2 when it is outside the stock selector range; no
  automatic topology, slot, or calibration mutation is introduced.

- 2026-09-04: Added the read-only Pixhawk `Dr from Baseline Center`, `Dr from P1`, and
  `Dr from P2` F/R/D/Rg geometry rows plus separate horizontal `∠Dr-P1-P2` and
  `∠Dr-P2-P1` values. Dr/P1/P2 vector frames, source age, WAITING/STALE behavior, and the
  no-persistence/no-yaw/no-3D contract are explicit; existing baseline/yaw and measurement
  semantics remain unchanged.

- 2026-09-04: Added a CUDA Driver API (`cuInit(0)`) gate to the NVDEC capability check so a
  management-interface-only success cannot hide a broken CUDA context; Auto now reports the
  precise cause before using CPU decode.

- 2026-09-01: Added a read-only `P1→P2 baseline / Pixhawk yaw` group below the Pixhawk raw
  baseline. It separately reports compass-normalized P1/P2 yaw, signed `Baseline−P1` and
  `Baseline−P2`, `P2−P1 yaw`, selected GNSS/fix/age/pair-delta evidence, and a fixed-height
  north-referenced top view. Missing/stale sides remain independent; no target/tolerance verdict,
  history, worker, setting, measurement change, or derived recording field was added.

- 2026-09-02: Added Telemetry read-only Drone `Find Network`, sysid-3 candidate selection, and
  protected the accepted MAVLink URL from stale Apply/Save/refresh rollback.

- 2026-09-02: Relaxed only the NTRIP `SYSTEM_CENTER` GGA gate to accept selected P1/P2 GNSS
  positions received within 10.0 seconds with an application-receive timestamp difference of at
  most 3000 ms. Exact boundaries pass; older or more widely skewed samples still fail closed.
  Measurement and baseline readiness thresholds are unchanged.

- 2026-09-02: Fixed Measurement start to apply the currently selected
  `Settings > YOLO / RTSP > Weights path` before invoking Detection. An unapplied selection no
  longer falls back to an older persisted `yolov8n.pt`; a rejected handoff blocks start instead of
  silently using stale weights. Stop remains available without requiring a new model selection,
  while the explicit Save command and existing normal-shutdown settings save remain unchanged.

- 2026-09-02: Fixed Measurement decoder-preflight rollback so an unavailable explicit `NVIDIA`
  request no longer stops an already-live left/right Preview. The observation remains open with its
  camera/recording sources unavailable, temporary automatic recording is released, and the
  operator-visible recovery remains Stop → select `Auto`/`Software` → retry. Failures after worker
  replacement starts retain the existing teardown, and explicit NVIDIA still has no implicit
  fallback.

- 2026-09-02: Added authoritative `gnss_baseline_center.jsonl` persistence for the GNSS-derived
  Telemetry row. Measurement/Manual Record attempts one self-contained bulk row per primary Drone
  GNSS callback, including CURRENT/STALE/WAITING, selected P1/P2/Drone positions and exact clocks,
  fix/MAVLink provenance, NED intermediates and explicit frame/origin/axis contracts. Data Status
  owns write/drop/error truth and Latest Measurement XLSX adds `GnssBaselineCenter`. Raw PX4 JSONL,
  canonical Measurement fields, legacy Parquet, settings, workers and device behavior are unchanged.

- 2026-09-02: Added a read-only `GNSS BASELINE_CENTER F/R/D/Rg` table to `Devices > Telemetry`.
  Three live GNSS positions produce an antenna-midpoint local-level frame with horizontal P1→P2
  Right, NED Down, Right×Down Forward and Euclidean Rg. CURRENT uses a 2.0 s age and operator-
  selected 1000 ms three-source receive-spread limit; STALE retains finite evidence and WAITING
  never fabricates zero. The existing ideal-disparity reference reuses the same Rg/readiness.
  Camera `baseline_center_frd`, Measurement/recording schemas, settings, device behavior and worker
  topology are unchanged.

- 2026-09-03: Removed the one-time `Field topology setup` workflow. Existing per-board transport,
  NIC discovery/assignment, P2 ETH MAV, Connect/Disconnect, and explicit board-side provisioning
  controls remain unchanged.

- 2026-09-01: Simplified the shared `EngineeringValue` presentation across Workstation. Normal
  values no longer add a redundant visible `VALID` badge; age remains visible when present and
  `Quality: valid` remains available in the tooltip. `INVALID` stays visible, and authoritative
  status values such as Measurement `VALID` are unchanged.

- 2026-09-01: Made Pixhawk `Find USB` and `Find network` replace the transient Discovery snapshot,
  clear stale selection/feedback, and show only the new source. USB discovery now renders one row
  per physical serial endpoint and enriches it with heartbeat identity instead of duplicating it as
  USB plus MAVLink. Source-neutral columns distinguish shared Network NIC roles from independent
  P1/P2/Drone Radio sysids; saved assignments, settings, connections, and explicit Assign/Connect
  behavior are unchanged.

- 2026-09-03: Corrected the camera-installation alignment contract to use the GNSS-only horizontal
  angle between the P1→P2 baseline and midpoint-to-Drone GNSS vector. `Devices > Telemetry` now
  exposes the angle and absolute 90° error beside GNSS F/R/D/Rg. Drone ATTITUDE/yaw is not used;
  existing raw GNSS, measurement, recording and device contracts remain unchanged.

- 2026-09-03: Added creation-time names to new structured JSONL writer files and derived Measurement
  workbooks. Existing files are preserved; the exporter scans legacy fixed and timestamped streams,
  while the Data Status action opens the exact newly published workbook path.

- 2026-08-31: Added one derived `<data-root>/latest_measurement.xlsx` for the most recently stopped
  Measurement. A generation-exact checkpoint feeds Summary, result/history, eight-method, source,
  native/file and Issues sheets with Excel `#N/A`, exact large integers and KST display columns.
  Generation is automatic after stop and manually retryable/openable from `Data > Status`; one
  coalesced cancellable worker keeps Measurement/Manual recording and shutdown authoritative.
  Temp validation/atomic replacement protects prior app-owned output, while foreign same-name
  files and all JSONL/legacy Session data remain untouched.

- 2026-08-31: Hardened the primary-first Measurement persistence boundary. The primary worker now
  starts and publishes before comparison workers, with an independent 20-pair queue; primary
  terminal/one-second-timeout releases comparison startup, and comparison-only backlog produces
  comparison `BACKPRESSURE_BLOCKED` without rejecting an available primary slot. The enqueue
  attempt/result now precedes table/history/plot/banner projection. Method bundles commit only
  after admitted append/replace, retain a lease-bound pending row for retry, and become `late` if
  Stop wins. The data store rejects `CLOSING`/`CLOSED` enqueue and fences Retry Storage against
  close. Retired owners fall back to direct evidence without overwriting reused indices. A rejected
  primary remains independently retry-owned after bundle admission, becomes compact late evidence
  after Stop, and reuses its already-built row so native capture is never scheduled twice. Cross-run
  late results do not alter current counters/UI. Pending retries are count/byte bounded and
  round-robin budgeted; native completion uses first-terminal fingerprints and bounded terminal-drop
  diagnostics. Bundle fallback records its actual commit source.

- 2026-08-31: Decoupled the successful representative Measurement result from the eight-method
  comparison barrier. While a run is `RUNNING`, `pixhawk_rtk_live` `VALID`/`ESTIMATED` now enters
  the asynchronous priority queue before its UI update; unused comparison methods cannot delay it.
  Non-success primary outcomes remain method-evaluation-only. Stop preserves an already accepted
  queue record, while a primary arriving after Stop creates no representative row or UI update.
  "Immediate" denotes enqueue, not synchronous write or `fsync`.

- 2026-08-31: Made P1/P2 `SCALED_IMU/2/3` delivery sample-backed instead of command-backed.
  Missing/stale standard IMU streams now use bounded side-locked retries and stop immediately on a
  real sample; all-RX stall remains fail-closed. Existing IMU rows now distinguish `WAITING`, fresh
  `LIVE`, stale cached values, and fresh partial packets without changing layout or sensor axes.

- 2026-08-31: Made Measurement timing follow its persisted Settings preference. App receive remains
  the default. Healthy VERIFIED/current/same-epoch RTCP/NTP now selects pairs; missing, malformed,
  stale, transitioning, epoch-mismatched or over-limit RTCP evidence falls back to exact app receive
  without a timing-only validity downgrade. Active-run changes apply to the next Measurement. New
  schema-v5 rows and measurement-pairs Parquet v4 preserve requested/effective/fallback and distinct
  app/NTP deltas; legacy settings and rows remain readable.

- 2026-08-31: Changed operating K,D readiness so zoom readback presence/age/value, left/right zoom
  agreement, camera serial, and stabilization are saved evidence only and never blockers or
  advisories. Missing/invalid K,D identity and unknown/mismatched native raster remain fail-closed.
  Verified `intrinsics_zoom_08_20260831.json` loads as 1280x720 K,D for the current RTP path; no
  calibration-quality claim or K,D/R,t/coordinate-math change is included.
- 2026-08-31: Removed the obsolete local frozen-image Import zoom dialog. `Import Pair` now emits
  only Left/Right paths, and Calibration/Alignment guidance states that native raster remains the
  enforced applicability condition while camera/focus/zoom provenance is operator-managed.

- 2026-08-30: Restored `apps/Workstation/calibration/` as the default owner for new Calibration
  photos, FTP originals, metadata, attempts, and profiles while preserving both existing trees.
  Explicit `app_root` tests remain isolated, custom relative roots remain data-root-owned, and an
  approved K,D profile is content-addressed and hash-verified into the data root before its restart
  path is persisted.

- 2026-08-29: Added default Calibration capture feedback with shared Left/Right checkerboard
  overlays, non-color board state/corner counts, current-process inspection outcomes, persisted
  current-root candidate inventory, stale/unavailable handling, fixed-height/elided status, and an
  additive public capture snapshot. Verified the real K,D worker against the bundled photo set:
  the solver operates on a corrected homogeneous 10x8 subset, while the current mixed-raster/13x8
  selection fails and the corrected subset remains quality `FAIL`, so it must not be applied.

- 2026-08-29: Changed ordinary Calibration Live/Auto `Max Live pair Δt` admission from
  RTCP/NTP frame time to signed decoded-frame `app_receive_monotonic_ns`. New capture evidence
  records both side stamps, exact signed ns delta and `HOST_RECEIVE_PROXY`, which may admit an R,t
  pair but never precision-qualifies or verifies exposure. NTP remains record-only, missing app
  receipt fails closed to K,D-only, FTP CPTR dispatch and verified exposure paths are unchanged,
  and historical Photo sets are not rewritten.

- 2026-08-29: Changed the default Measurement output from
  `.local/workstation_data/measurement_data/` to repository-root `recordings/`. A blank setting
  selects the default, the former `measurement_data` value migrates logically without moving old
  bytes, explicit data-root overrides resolve to their own `recordings/` child, and custom relative
  folders retain the prior containment checks and non-blocking fallback behavior.

- 2026-08-29: Removed Horizon detector quality from `Use suggestion` permission while retaining
  `GOOD`/`LOW_CONFIDENCE`, confidence, residual and reason text/evidence. Owner/UI validation now
  requires current frozen identity, exactly two finite in-frame endpoints per side and the existing
  25%-width separation; explicit Solve/Apply remain unchanged.

- 2026-08-29: Documented the observed NVIDIA `595.84`/Linux 6.8 `s2idle` resume failure with two
  active NVDEC workers: paired Xid 31 MMU faults, UVM fatal error, Xid 154 reboot requirement,
  GStreamer CUDA-init failure, truthful CPU fallback symptoms, reboot-only recovery, and the
  current stop-Preview-before-suspend operational workaround.

- 2026-08-29: Extended read-only NIC discovery with explicit, confirmed NetworkManager role
  application for Camera/P1/P2. One or many NICs may hold composable roles. Target-first readback,
  role-scoped source cleanup, app-owned-entry provenance, affected-owner automatic reconnect,
  actual Camera/Pixhawk evidence, commit-last persistence and compensating rollback preserve
  unrelated NICs/settings and existing Measurement mapping locks.

- 2026-08-29: Separated Mountain automatic-suggestion quality from explicit copy permission.
  `Use suggestion` now accepts a structurally valid `LOW_CONFIDENCE` proposal, revalidates current
  frozen identities and paired native `(u,v)` geometry, applies exclusions symmetrically, and
  persists filtered-input provenance. That slice did not change Horizon; direct/automatic Solve and
  Apply behavior remain unchanged.

- 2026-08-29: Reset Calibration's visible `이번 촬영` inspection/stereo/K,D-only/failure counters on
  every successful Auto Start while preserving existing PNG/metadata files, capture IDs, and the
  cumulative `촬영 세트`; a restart rejected by an active worker leaves the current counters intact.

- 2026-08-29: Bound Calibration Auto capture to parser-observed NAL1/non-delta access units and
  propagated that evidence by exact AU/decoded PTS into the native `FrameKey`. Removed Auto's
  internal 1 ms timestamp-near lookup; evicted, stale, malformed, or old-generation keys now wait
  for the next candidate without affecting Preview, YOLO, Measurement, recording, or manual capture.

- 2026-08-29: Made Measurement observation-first. Start immediately opens a run with zero or more
  sources; camera, Pixhawk, Base GNSS/NTRIP, Target and recording join independently. Added manual
  camera rejoin with non-overwriting MP4 segments, bounded FIFO retry for essential structured
  evidence, presence-only `SAVED/PARTIAL/FAILED`, and removed artifact/integrity persistence from
  current- and next-run admission.

- 2026-08-29: Made tolerant decoded-frame admission permanent for Preview, YOLO, native evidence,
  and official Measurement. Added simultaneous `VALID` + `INPUT UNVERIFIED` provenance and replaced
  the GStreamer direct-copy recording branch with an isolated bounded decoded-NV12 → FFmpeg
  `h264_nvenc` lossless MP4 writer. Recording oldest-drops and failures do not stop YOLO/distance;
  there is no CPU encoder fallback or application bitrate/size/time cap.

- 2026-08-29: Added the persisted `H.264 compressed queue` setting with `Latest (drop old)` and
  `Reference-safe (preserve 8)` modes. A 12-second left-camera TCP A/B showed nearly equal decode
  rate (`16.72` vs `15.98` Hz mean) while the reference-safe run recorded zero observed RTP gaps
  versus 16 gaps/183 missing packets in latest mode; the setting applies on the next Preview start.

- 2026-08-29: Fixed the queue-setting handoff name from Settings to `YoloStreamWorker`. The initial
  implementation passed `rtsp_encoded_queue_mode` to a constructor that accepted only
  `encoded_queue_mode`, causing Preview to return immediately to STOPPED. A fresh real launch now
  reaches Left LIVE, changes the action to `Stop preview`, and keeps the unavailable Right isolated.

- 2026-08-28: Changed only the Mountain + GNSS translation gate from both-side `RTK_FIXED` to
  both-side `fix_type >= 3` with finite positions and a finite non-zero frozen-capture baseline.
  RTK Float/ordinary 3D Fix candidates can now supply all translation axes, while their explicit
  warning keeps `mountain_gnss` outcomes `ESTIMATED`; sub-3D fixes still block those axes.

- 2026-08-27: Reduced the operational P1/P2 HEARTBEAT+ATTITUDE qualification from ten minutes to
  60 seconds. Initial connection, reconnect, confirmed reboot, stall recovery and explicit/manual
  PARAM completion require a fresh minute; stale/stall fail-close is unchanged. Ten-minute and
  60-minute runs remain separate long-duration evidence rather than routine measurement delays.

- 2026-08-27: Removed the automatic post-`LIVE` PX4 calibration PARAM snapshot. P1/P2 and Advanced
  manual refresh plus reset/apply/save verification readback remain unchanged; ordinary
  qualification now sends no calibration PARAM requests and does not restart qualification for an
  unrequested diagnostic snapshot.

- 2026-08-27: Added a default-off, process-local WIRIS NAL1 intra field-qualification mode. It
  requires SPS/PPS, parser non-delta, clean flags and exact decoded PTS, exposes explicit
  experimental provenance, and never relaxes strict IDR/MP4 truth in normal launches.

- 2026-08-27: Made exact decoded-frame `app_receive_monotonic_ns` the then-sole Workstation
  live-distance pairing clock with a hard 100 ms bound and normal VALID semantics. This historical
  fixed-basis contract was superseded by the Settings-preferred 2026-08-31 policy; its schema-v4
  rows keep their original app-selected meaning.

- 2026-08-27: Decoupled STUS clock truth from decoded video with a dedicated RTCP-only observer,
  added Manual/Auto scheduling and logical clock epochs, and allowed ordered Left-then-Right STUS
  during active Preview/Measurement/YOLO/recording without restarting video or MP4. Changed the
  success contract to sustained Left/Right relative timing at or below 100 ms. Absolute SR-PC is
  now diagnostic-only, while occasional unmatched/outlier frames are reported and may be dropped.

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
  the then-current 10-minute qualification (superseded above by the approved 60-second operational
  gate), truthful ATTITUDE-stale versus all-RX-stalled status, no false host
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

- 2026-09-05: Distinguished bounded Stable-PX4 RM3100 Shell polling as `SHELL SYNC` so the
  operator does not mistake `sensor_mag` readback for a CAL_MAG parameter lookup.

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

### 2026-09-08 — P1/P2 Radio minimal streams

P1/P2 Radio connections automatically request ATTITUDE, GPS_RAW_INT and GPS2_RAW at
5 Hz, SYS_STATUS and BATTERY_STATUS at 1 Hz, and leave HEARTBEAT at the autopilot
default rate. Normal-mode background streams and prior optional IMU/debug/position
requests are explicitly disabled with MAV_CMD_SET_MESSAGE_INTERVAL (-1). These are
requested rates; missing sensors and Radio loss can reduce actual reception.

The profile is reapplied after reconnect and detected PX4 reboot. Healthy-link tier
escalation is disabled for P1/P2 Radio, and stale ATTITUDE recovery requests 5 Hz.
This is persistent Workstation behavior, not a saved PX4 Minimal mode or parameter
change: MAV_0_MODE, MAV_0_RATE, modem power, RTCM routing and Shell polling are unchanged.
USB/Ethernet and Drone retain their existing profiles. Radio IMU/debug/local-position
panels no longer receive periodic streams disabled by this profile; use USB/Ethernet
for those observations. ATTITUDE orientation and GPS lat/lon/MSL-alt contracts are unchanged.

The application sends interval requests; successful transmission is not an ACK/readback
claim. Link loss can prevent application. Reconnect reapplies the profile. RF coexistence
and USB supply effects remain separate unresolved limits from the measured investigation.

## Target reference discovery and connection

Data → Status → Target reference provides separate receive-only target-aircraft connectivity. Select Drone, then Find Network or Find USB. Network discovery listens for MAVLink already arriving on the configured PC UDP ports (default14570 and14550–14559); it does not scan subnets or configure the aircraft transmitter. USB/Radio discovery listens at the selected baud. Devices without a heartbeat remain visibly unverified.

Select an observed candidate and press Use selected to fill the draft endpoint and SYS/COMP, then Connect target to acquire the transport and verify identity again. UDP candidates retain their observed peer address/port through this handoff. Candidate evidence expires after30s. SYS/COMP0 are not auto-converted into verified identities; manual Any retains its explicit wildcard meaning. A valid heartbeat identifies the sender's reported IDs, not a cryptographic aircraft identity.

Target uses an exclusive UDP bind and serial device lease/exclusion. App-owned P1/P2/Telemetry and Base GNSS endpoints are rejected as busy. Discovery is single-flight, cancellable, bounded to16 UDP ports,8 serial devices and128 candidates. It never sends GCS heartbeat, stream requests, parameters, RTCM, flight or logger commands. A quiet target must be configured externally to transmit. Serial open can affect DTR/RTS depending on hardware; physical-device qualification remains required.

Connection status is separate from host-monotonic HEARTBEAT/GPS1/GPS2/ATTITUDE ages. Fresh valid GPS and RTK fix are shown independently; heartbeat alone does not establish GPS readiness. Samples older than3s show STALE. Existing Measurement admission and coordinate contracts are preserved.

Save persists only Target connection preferences in Workstation-owned settings. It does not connect. Restart restores preferences with no active connection or discovered identity. Device paths under /dev are intentionally machine-local and may need reselection on another computer. UAM uses Set UAM deferred with no live transport or discovery. Stop runs asynchronously; app shutdown cancels discovery and joins Target resources.

Target reference and the active Telemetry workspace remain independent sources. Optional lifecycle metadata records expected/verified UDP peer; telemetry coordinate fields and existing evidence formats remain compatible. Implementation and software/field evidence are in [the Target discovery packet](docs/changes/20261010-target-reference-discovery/verification.md).

## Reproducible validation environment

From the repository root on Ubuntu22.04 / Python3.10:

```bash
tools/setup_workstation_test_env.sh
apps/Workstation/tools/run_isolated_tests.sh -q -ra --tb=short
```

The setup creates the ignored repository .venv with system-site-packages for Ubuntu GI/Qt, installs declared dev/runtime dependencies with requirements/constraints-workstation-py310.txt, and disables user-site/ambient ROS Python paths. The test runner requires this environment (or an explicit executable SASL_TEST_PYTHON override) and uses only loopback networking. It no longer silently falls back to an incompatible system/user package stack.

requirements/optional-terrain.txt declares rasterio and pyproj for GeoTIFF/WGS84/ECEF/local-NED validation. Dev setup includes them; runtime-only installations can install that file separately. The missing-dependency test injects ImportError to retain failure-path coverage even when the packages are installed. Raw recording continues to be independent of optional terrain imports.

If nvidia-smi reports Driver/library version mismatch after a driver update, compare the loaded module with the installed module. Follow the [Ubuntu NVIDIA troubleshooting guide](https://ubuntu.com/server/docs/how-to/graphics/install-nvidia-drivers/#driver-library-version-mismatch-error); reboot after saving active work to activate the installed driver. NVENC is only accepted after this command executes its real lossless-recording test without skipping:

```bash
python3 tools/check_workstation_nvenc.py
```

The check fails when driver initialization or the GPU test is unavailable. A passed CPU/coordinate suite does not stand in for this hardware result. See docs/changes/20261010-runtime-validation-environment/verification.md for this machine's repair evidence.
