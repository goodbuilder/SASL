# Verification

## Requirement-to-evidence matrix

| ID | Required behavior | Independent evidence | Status |
|---|---|---|---|
| R1 | clean and sustained dual RTSP/NVDEC | fresh-client decode, RTP/RTCP/pcap, frames | prior decoded NVDEC but corrupt; strict IDR startup now blocks both FW 2.18.72 senders, so truthful unavailable / usable video FIELD FAIL |
| R2 | corruption/freeze-free motion | periodic images + identity and gap events | corrupted output exposure SW PASS via IDR+exact-PTS fail-close; sequential reboots and active-stream GEIR=1 still yield zero NAL5, so clean video FIELD FAIL pending Workswell firmware/vendor |
| R3 | sustained STUS sync | ACK targets, new workers, unique pad-probe RTCP series | 5s response-time bug fixed; 15s hardware A/B exact Left-then-Right ACK PASS; first round left 494.9ms relative offset and zero-IDR blocked verifiable generation/second round, so CRITICAL FIELD FAIL |
| R4 | P1/P2 MAVLink qualification | raw inbound HB/ATT/ANY and side state | shared-NIC load FAIL; first isolated adapter Preview 10m PASS; replacement RTL8156B direct load 45s and latest-app 10m/PARAM 124-of-124 PASS; cold boots/final 60m pending |
| R5 | SASL/NGII authentication/RTCM | raw HTTP status, CRC-valid RTCM and injection ACK/counters | isolated RTL8156B Stop/reconnect + 636.5s stream PASS: HTTP 200, GGA 120/120, station/MSM, dual injection 3026 frames/side failure0; RTK Fixed pending |
| R6 | Measurement admission and restart | UI action through new run ID/files | FIELD PASS: active-run SIGKILL left no child/socket/ring; restart did not adopt old run, showed enabled Start and created a distinct new run ID; tracker-loss startup fallback and final clean Quit also passed |
| R7 | dual MP4 finalization and usable bitstream | size/sample/decode/IDR/frame-number/timing evidence | Workstation and blank-start Legacy dual creation/EOS PASS but portable integrity FIELD FAIL; latest Legacy files also have zero IDR, decode loss, frame gaps and final stts=0 |
| R8 | exact evidence identity | FrameKey plus durable readback hash | FIELD PASS: independent no-product-import audit verified 88 real pairs/176 PNGs/487.5MB, run-scoped descriptor reconstruction, PNG CRC/decode, byte/path/ACK hashes, failure0 |
| R9 | bounded GUI/resources | time-series RSS/queues/GPU/latency/cleanup | GUI/Recv-Q PASS; 315-callback NaN log storm fixed; real SIGKILL tracker cleanup and strict orphan-startup sweep PASS; final long-load cleanup pending |
| R10 | canonical reproducible runtime | doctor/launchers/root venv | CURRENT HOST + fresh checkout/venv PASS; portability 12→0 PASS; separate clean Ubuntu OS/VM still pending |
| R11 | final integrated acceptance | 3 cold boots + 60-minute full load + failure isolation | NOT_RUN |
| R12 | method-specific readiness without global Start gate | current-zoom K,D/R,t, correspondence, attitude/GNSS/STUS permutations | FIELD/SW PASS: real all-blocked run admitted 44 pairs and persisted 352 BLOCKED outcomes; six-case UI matrix kept Start enabled without promoting validity |
| R13 | no unapproved camera setting mutation | code-path audit + pre/post readback for each hardware app | FIELD PASS: Workstation and Legacy automatic paths sent no encoder SET; Legacy all-path TCP/2240 pcap contained 0 packets, final device readback remained 0/1/30/1 |

## Focused test evidence

### Remote PX4 reboot stream-profile rearm slice

- RED: after a confirmed `120000 -> 100 -> 133 ms` remote boot regression on the same fake host
  connection, Left `_mavlink_stream_requests_sent` remained set and no second Stock profile was sent.
- Rapid-reboot RED: `5000 -> 100 -> 133 ms` remained below the original 10-second detector threshold,
  so the Left sentinel stayed set. The detector now uses the strict edge beyond the 2-second reorder
  tolerance and still requires the second increasing low-epoch sample before rearming.
- GREEN: the new regressions cover two-sample confirmation, a second real reboot on the same socket,
  rapid reboot below 10 seconds, 999 ms and exact-2000 ms reorder tolerance, natural uint32 wrap,
  reboot after the uint32 half range, delayed prior-boot packet rejection, stop/replacement baseline
  reset, same-generation liveness recovery, persistent setup-marker preservation, healthy-Right
  isolation and side-send-lock ownership.
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q
  -o cache_dir=/tmp/workstation_pytest_cache_reboot_rearm_scoped tests/test_mavlink_liveness.py
  tests/test_status_model_pixhawk.py tests/test_workstation_pixhawk_gnss.py` returned **109 passed**.
- Three focused RTCM generation/stall/qualification regressions returned **3 passed**. Python
  compilation passed. These are automated fake/model results; a restarted latest app still requires
  independent wire evidence that one rebooted side receives the 15 interval commands and returns to
  the requested approximately 140 pps profile without disturbing its peer.

### Pixhawk Connect GUI-thread/accessibility slice

- FIELD RED: Connect opened UDP 14550/14551 and sent one GCS heartbeat per side, but the UI remained
  DISCONNECTED/nonresponsive and Recv-Q reached 213,760. SIGUSR1 captured the main thread in
  synchronous `QStyle.polish()` from progress before completion could start the receive timer.
  NTRIP and Preview were off.
- Independent pcap decode found zero `MAV_CMD_SET_MESSAGE_INTERVAL` commands, continuous CRC-valid
  PX4 traffic at about 66.9/66.1 unique messages/s, zero sequence gaps and no rate burst. Application
  drain failed before profile handling; the remote-reboot detector was not exercised.
- The gated real-panel QThread regression emits 32 pre-completion statuses while the worker is held.
  GREEN proves the GUI probe keeps ticking, callbacks/completion use the GUI owner, no precompletion
  role change occurs, refresh coalesces, the real receive timer ticks and the worker clears.
- A 100-frame-per-side RTCM burst preserves 200 exact sends but emits only two success statuses in
  the five-second window. Errors and exact result counters remain unchanged.
- Focused: **3 passed**; Pixhawk/GNSS file: **90 passed**; dependents: **107 passed**; root combined
  MAVLink/liveness/status/RTCM rerun: **143 passed**. Compile, delegate check and diff check pass.
  A later 69.63-second real gate remained responsive, drained both inputs and emitted exactly one
  15-command Stock profile per side; see `evidence/mavlink-param-reservation-20260827.md`.

### MAVLink PARAM reservation and deferred replay slice

- FIELD RED: after dual LIVE, automatic calibration refresh sent 142 parameter reads per side but
  received 124 unique values. The 18 unanswered names were exactly the nonexistent `CAL_MAG4_*`
  surface and contributed about 27 seconds of 1.5-second timeouts.
- During that reservation PX4 inbound heartbeat/ATTITUDE remained continuous, while host GCS
  heartbeat had a 28.055616-second P1 gap and a 27.318256-second P2 gap. Deferred replay then emitted
  28/27 redundant ATTITUDE interval commands. Artifact paths and full hashes are in
  `evidence/mavlink-param-reservation-20260827.md`.
- GREEN software: automatic Stock refresh now uses slots 0–3 and exactly 124 parameters. The generic
  explicit parameter API still accepts arbitrary names. PARAM/shell receive loops service heartbeat
  every 50 ms from the command owner under the same re-entrant side lock; the GUI heartbeat uses a
  nonblocking attempt. RTCM remains excluded and the peer side continues.
- Deferred HEARTBEAT replay retains ingress liveness timestamps, FIFO ATTITUDE delivery and the
  one-shot profile marker but emits no setup/profile/recovery command. A new live HEARTBEAT still
  performs one warranted recovery. The existing post-command 10-minute qualification reset is
  intentionally unchanged.
- Focused new cases returned **8 passed**. The related MAVLink/liveness/status/PARAM/magnetometer/
  RTCM set returned **177 passed** with
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q`.
  Real post-fix automatic refresh passed on both isolated adapters. The replacement RTL8156B run
  requested/received 124/124 per side in 55/64 ms about 600 seconds after ownership; inbound and GCS
  heartbeat remained continuous and the ten-minute qualification passed.
- A later 9.08-second camera-only capture has outbound host heartbeat but zero inbound P1/P2 wire
  traffic. That is a separate real ETH stall; PARAM/replay root cause does not establish its PX4
  internal mechanism.

### 2026-08-27 parallel current-environment software gates

- Camera/decoder/backpressure/STUS: **227 passed** across decoder truth/recovery, start readiness,
  bounded queues/rings, continuous sync health and adjacent UI/worker contracts. The requested
  600-second synthetic run was deliberately interrupted before the real hardware gate and is
  recorded `NOT_COMPLETED`, not PASS; cleanup left no process/shared-memory residue.
- Session/recording/MP4: the early **187 passed** set proved basic container paths, but the real
  camera files later passed ffprobe/strict FFmpeg while still being defective. The current async
  validator rejects those exact files and passes a valid 60-minute/108,000-sample fixture; portable
  real camera MP4 remains FIELD FAIL because the sender supplies no IDR and has reference gaps.
- NTRIP/RTCM: **123 passed**. A corrupt length/CRC candidate previously discarded a valid RTCM frame
  enclosed later in the buffer. The parser now drops only the false sync byte and rescans; focused
  post-fix cases passed. Real isolated-topology Stop/reconnect and 636.5-second streaming now pass
  with GGA 120/120, station/MSM RTCM and dual failure-zero injection. Keyring remains `NOT_SAVED`, so
  a new app process still requires session-password entry.
- Canonical runtime: focused **53 passed**, expanded Legacy runtime **117 passed**, Workstation
  runtime/recovery **33 passed**. Launchers/doctor clear ambient Python/loader/GI/GStreamer paths,
  deployment bootstrap no longer imports `packaging`, and contaminated-environment real checks all
  exit zero. The later R10 slice closes portability and same-host fresh recreation below.
- The latest complete Workstation regression before the abnormal-restart slice returned **1448 passed,
  1 skipped**.

### Abnormal termination and new-run admission slice

- RED: two real active-Measurement `SIGKILL` runs released camera children/sockets but left two
  generic 44,236,800-byte `psm_*` rings each for more than 20 seconds. Removing the child attachment
  unregister was necessary but was not, by itself, a sufficient restart recovery contract.
- Native rings now use `sasl_ws_native_<owner-pid>_<16-hex-token>`, retain the shared spawn tracker
  registration, and have a startup fallback restricted to strict prefix, dead owner PID, same UID,
  and regular `/dev/shm` entry. It never scans or deletes arbitrary `psm_*` objects.
- GREEN real kill: PID 1093884 owned active run `run-3dba2f…`, two 44,236,800-byte rings and camera
  children 1094334/1094335. Five seconds after exact-PID `SIGKILL`, the app, children, rings and
  established camera sockets were absent.
- GREEN restart: a controlled tracker-less strict-prefix segment was reclaimed with an exact startup
  log. Event count stayed unchanged through clean launch, the UI showed `MEAS WAITING / REC IDLE`,
  accessibility reported enabled/sensitive `측정 시작`, and Press created distinct
  `run-46b999…`. Normal Quit persisted `shutdown_resource_verification.complete=true` with all four
  resource arrays empty and left no process/socket/ring or exception marker.
- Focused native-ring/evidence/camera-recovery/lifecycle regression: **84 passed**. Full artifact:
  `evidence/abnormal-restart-shm-recovery-20260827.md`.

### Exact evidence, method readiness, and Legacy recording slice

- R8 RED: persisted detection projections omitted descriptor input `timestamp_s`. The old real
  artifact chain could recover it only from the ACK-bound native artifact row. New projections retain
  it directly, and `qualify_native_evidence.py` independently verifies JSONL, run-scoped FrameKey and
  descriptor reconstruction, PNG CRC/IDAT/RGB8, file bytes, paths and pair ACKs without importing
  product writer/ring code.
- R8 FIELD PASS: `run-43dbe5…` has 88 complete pair events and 176 unique 1280×720 PNGs totaling
  487,531,968 bytes. All checks passed; aggregate manifest SHA is `1849c402…40949`.
- R12 FIELD PASS: that real run started with no active K,D and failed camera clock evidence, retained
  44 pairs, emitted all eight method outcomes per pair and marked all 352 BLOCKED. A six-case UI
  matrix for K,D/zoom/R,t/correspondence/live telemetry/STUS kept global Start enabled and validity
  blocked. Related regressions: **108 passed**.
- Legacy preflight found its saved/default camera address still recreated `10.0.0.10/24` on the
  camera NIC. Both applications now default to isolated `10.0.0.11/32`, while the Pixhawk NIC owns
  `.10/24`.
- R13 FIELD PASS: a pcap spanning Legacy startup, receive check, Preview, restarts and recordings had
  exactly zero TCP/2240 packets. Preview reported dual 1280×720 frames. RED blank-start recording
  created no workers/MP4; GREEN now auto-starts both recording streams and rolls back a rejected
  start. Distinct session `20260827_050834_833389066` produced/finalized both MP4s and cleanly quit.
- Those MP4 bytes still fail R7 independently: zero IDR, decoder-visible loss, reference gaps and
  final `stts=0`. Full artifact: `evidence/exact-evidence-readiness-legacy-20260827.md`.
- Focused Workstation evidence tests: **37 passed**. Legacy support and focused paths:
  **95 + 16 passed**. Current complete Workstation suite: **1466 passed, 1 skipped**.

### Fresh runtime and portability slice

- RED: the repository audit reproduced 12 findings. Active debt was a machine-bound Legacy model
  path and two Simulation command examples; calibration/change artifacts were immutable historical
  provenance misclassified as active settings.
- GREEN: active paths are relative/repository-derived, historical evidence is explicitly classified
  without byte rewriting, and `python3 tools/check_portable_paths.py` returns PASS. Focused policy,
  settings and runtime selections returned **48** then **71 passed**.
- A fresh current-worktree copy under `/tmp` excluded venv/local/workspace/recording/cache state. A
  new Python 3.10 `--system-site-packages` venv was created, the independently hash-matched 52,136,884
  byte YOLOv8m artifact was provisioned, and only `requirements/runtime.txt` was installed.
- RED during recreation: unconstrained Torch selected setuptools 84, conflicting with Ubuntu/ROS
  colcon `<80`. The manifest now owns `setuptools>=77,<80`, also satisfying Torch 2.13 `>=77`; fresh
  resolution selected 79.0.1.
- Deployment, execution, Workstation and Legacy checks all exited zero inside the fresh checkout.
  Workstation and Legacy also passed again under isolated HOME/config/cache. This is same-host fresh
  checkout/venv evidence; a separate clean Ubuntu OS/VM remains pending. Full artifact:
  `evidence/fresh-runtime-portability-20260827.md`.

### Legacy expected-video finalization

- RED: expected camera with zero GUI frames and no MP4 incorrectly started session-save finalization.
- GREEN: expected-side identity is persisted independently of frame count; missing/zero/non-`ftyp`
  MP4 fails closed before finalize. Focused new/adjacent cases: **6 passed**.

### Integrated first-slice regression — fresh root run

- Workstation combined focused set covering MAVLink, NTRIP/RTCM, camera/video readiness, storage
  concurrency, backpressure qualification, native ring/writer and settings: **361 passed**.
- StereoWIRIS runtime/direct-entry, RTSP enable safety, blank startup/new session and recording
  lifecycle selection: **57 passed**.
- Legacy close/save test left the tracked `stereo_distance_settings.json` hash unchanged after an
  autouse test-harness redirection was added.
- Fresh complete Workstation suite after all first slices: **1315 passed, 1 skipped** in 116.53 s.
- `git diff --check`: PASS.

### Camera/Measurement/video start-readiness slice

- RED: `QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q
  tests/test_camera_start_readiness.py` returned `6 failed`. QThread acceptance immediately set
  `camera_connected`/RUNNING and no frame/video readiness or start-failure transition existed.
- GREEN: the same focused file, expanded with bounded worker-replacement, decoder/source and
  video-partial failure cases, returned `10 passed`.
- Dependent focused regressions were run in bounded groups and returned 274 additional passes:
  camera 62; Measurement/lifecycle/recording 34; package/recovery 39; UI/contracts 46;
  recording/evidence 34; method/session 59. Total current slice evidence: `284 passed`.
- Python compilation, generated panel delegate stub check, and scoped `git diff --check` passed.
- Ruff was not available in the canonical venv (`No module named ruff`).
- At this slice, hardware frame/MP4 readiness was NOT_RUN. Later real WIRIS evidence proved file
  persistence but failed IDR/reference/timing integrity; the current validator rejects it.

### Decoder truth and per-side failure-isolation slice

- RED: `QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q
  tests/test_decoder_truth_isolation.py` returned **13 failed**. `PLAYING` status immediately set
  active decoder evidence, no boundary error classifier existed, current-generation frames did not
  ACK decoder identity, fallback was global, and recording fallback restarted both sides.
- GREEN: the focused file, expanded with RTP-input expiry, real bus-routing fakes, critical fallback
  evidence, per-side status projection and a spawned terminal-child supervisor check, returned
  **22 passed**.
- Dependent camera/worker/readiness regression returned **138 passed** across
  `test_decoder_truth_isolation.py`, `test_camera_stream_recovery.py`,
  `test_workstation_yolo_stream.py`, `test_workstation_runtime_camera_status.py`,
  `test_camera_start_readiness.py`, and `test_workstation_camera.py`.
- The then-current Workstation project regression returned **1376 passed, 1 skipped in 148.07 s** with
  `QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q`. A preceding run exposed one
  unrelated order-sensitive RTCM fsync-injection failure; the exact test passed in isolation and
  did not recur in this clean rerun.
- This slice's evidence was software/fake-boundary only. Later dual-WIRIS launch ran, but the sender
  failed current zero-IDR integrity truth; corruption-free sustained NVDEC remains FIELD FAIL.

### STUS ordering and continuous time-sync health slice

- RED: the initial focused file produced five behavior failures: dual STUS followed target/thread
  order rather than Left-first, Right was sent after a Left `ERR`, VERIFIED stopped its timer, frozen
  frame/SR metadata remained ready, and pair selection chose a healthier old pair. The UI projection
  itself already surfaced an injected FAILED state.
- GREEN: `QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q
  tests/test_camera_time_sync_continuous.py` returned **13 passed**. Automated-fake coverage includes
  exact Left-OK-before-Right dispatch and skip evidence, current monotonic/wall frame/SR age,
  regressed/missing SR, both-side freeze, freshest-pair selection, every approved hard limit,
  VERIFIED-to-FAILED long-run transition, visible FAILED detail, and configurable 10 s versus 7 s
  behavior without a hidden clamp.
- Existing controller/UI interlocks: `QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q
  tests/test_workstation_camera.py -k 'camera_sync or wiris_clock or stus or
  recording_owner_rejects_camera_swap_and_clock_reconnect'` returned **5 passed**; the Workstation
  command-level recording gate in `tests/test_workstation_contracts.py` returned **1 passed**.
- The first full-suite run exposed only a stale generated delegate-stub block from the accumulated
  controller registries. After `../../.venv/bin/python tools/gen_panel_delegate_stubs.py` and its
  `--check` oracle passed, a fresh `QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q`
  returned **1369 passed, 1 skipped** in 120.34 seconds.
- The 7.0-second value was visibly Applied and saved. Later physical evidence proved exact
  Left-then-Right ACK under the 15-second response boundary, but zero-IDR replacement generations
  blocked fresh-frame verification/round two; sustained clock quality remains CRITICAL FIELD FAIL.

### STUS high-latency closed-loop correction slice

- FIELD RED: with 7.0 s / 50 ms / 100 ms applied, ordered STUS ACKs passed but pair delta was
  510.6 ms and maximum SR-PC offset 1,312.5 ms. Pcap proved all successful 0.51–0.93 s HIWS probes
  were discarded by the 250 ms cap. Independent pad-probe SR medians were -736.135 ms Left and
  -1,307.044 ms Right; the displayed positive frame receive delays were not correction inputs.
- Focused RED had seven failures covering high RTT, missing signed formula/worker, absent correction,
  overall timeout and recording cancellation. The current focused CRITICAL set is **24 passed**;
  complete `test_camera_time_sync_continuous.py` + `test_workstation_camera.py` is **95 passed**.
- Earlier dependent automated-fake groups passed: camera/reconnect/decoder **144**, time evidence/
  status **34**, lifecycle/recording/delegates **70**. Final full-suite result is recorded below.
- Negative cases cover both-side preflight zero-mutation, Left failure skipping Right in both rounds,
  >2 s RTT, ±2 s correction, stale/old-generation/pre-ACK/poll-detected/duplicate/unstable SR,
  atomically rejected bad receive pairs, per-side missing offset, competing camera control,
  recording/Measurement conflict, timeout, second-round nonconvergence, raw STUS, cancellation,
  shutdown, prior-success supersession and late completion. ACK remains pending until external
  quality and later failure/stream reset revokes success.
- Full artifact and formula: `evidence/stus-closed-loop-correction-20260827.md`.
- First corrected-build FIELD RED then isolated STUS response latency: Left exact OK arrived 8.876
  seconds after command, after the generic 5-second client timeout/FIN, while Right was correctly
  skipped. STUS-only response wait is now bounded at 15 seconds and by the remaining 90-second
  deadline; HIWS/generic timeout is unchanged. Focused post-fix: **32 + 9 passed**. Physical retry
  PASS: Left exact OK in 3.691 seconds, then Right in 3.433 seconds. Post-seed offsets remained
  -313.712/-808.643 ms (~494.931 ms relative); zero-IDR blocked the bounded second round, so R3 is
  still CRITICAL FIELD FAIL.

### Exact-frame reservation and evidence-integrity slice

- `QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q
  tests/test_native_frame_ring.py tests/test_detection_pair_coordinator.py
  tests/test_native_evidence_writer.py` returned **27 passed**. Independent negative fixtures cover
  redundant compact/component `FrameKey` tampering, descriptor-hash tampering, delayed older stream
  generation, explicit no-detection and silent-age eviction, more-than-ring-depth GUI delay, writer
  failure, unpolled coordinator stop, pending-writer stop, and replacement stream generation.

### Shutdown, MP4 EOS and resource-truth slice

- RED: the focused lifecycle cases produced six failures: the five-second application deadline
  preempted valid 5–9 second EOS completion; OZ capture was absent from auxiliary gates; forced close
  closed DataStorage and returned success for a surviving child/QThread; no socket/resource report
  existed; and a released pinned retired ring had no shutdown sweep. Separate RED cases proved
  finalization evidence was discarded after an enqueue rejection and a kill-surviving child handle
  was forgotten.
- GREEN: `QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q
  tests/test_workstation_yolo_stream.py tests/test_camera_stream_recovery.py
  tests/test_workstation_lifecycle.py tests/test_workstation_camera.py
  tests/test_workstation_settings.py tests/test_panel_delegate_bindings.py
  tests/test_controller_delegation.py tests/test_measurement_data_store.py` returned **161 passed**.
- The cases cover EOS completion after the historical five-second boundary, a camera stop request
  delayed by earlier lifecycle work, active session-save/OZ workers, rejected then accepted video
  finalization evidence, terminate/kill-surviving children, protected camera IPC QThreads, open
  socket detection, pinned retired-ring release, explicit evidence rejection, clean mock cleanup,
  final writer ordering, and an offscreen no-worker close with zero surviving QThreads.
- Generated delegate stubs, Python compilation, and scoped `git diff --check` pass. Real dual-WIRIS
  EOS/file persistence and repeated real shutdown/resource cleanup were run: persistence PASS,
  portable bitstream FIELD FAIL, cleanup PASS. Only the long-load/final 60-minute cleanup remains.
- A fresh full Workstation run after the accumulated STUS, decoder, exact-frame and shutdown slices
  returned **1376 passed, 1 skipped** in 148.51 seconds.
- The accepted-pair reservation is acquired in the pairing worker before its latest-only result is
  exposed to GUI polling. Replaced/unconsumed results release ownership; the recording writer consumes
  the same leases and validates the incoming descriptor hash against the pinned descriptor before
  writing either side. A failed key/hash preflight writes zero PNG artifacts and reports FAILED.
- The real synthetic queue/evidence/cleanup harness plus its fail-closed matrix returned **29 passed**
  in `tests/test_camera_backpressure_qualification.py`. Related camera/evidence integration returned
  **76 passed**; Measurement/camera/controller regressions returned **124 passed**.
- Python compilation and scoped `git diff --check` passed. These are software/shared-memory and real
  temporary-file results; physical WIRIS FrameKey/hash evidence remains required for hardware R8.

## Regression and application evidence

The current full Workstation suite passes fresh (**1466 passed, 1 skipped**). A broad StereoWIRIS suite remains affected by the known
native Qt order-dependent abort; the relevant focused paths pass, and the test harness no longer
writes the tracked operator settings. Root-suite collection's duplicate module-name conflict and
the Legacy native-order residual remain visible rather than treated as product PASS evidence.

## Performance, load, and target-hardware evidence

Initial target: RTX 4050 driver 595.84, about 1 TiB free storage. Earlier cameras produced
1280x720 `nvh264dec` output, but current integrity truth rejects both zero-IDR senders. Replacement
RTL8156B telemetry, GUI, Recv-Q and resource time series cover more than ten minutes; the final
camera/NTRIP 60-minute integrated series remains pending.

Read-only current-boot MAVLink diagnostic: both `udpin:0.0.0.0:14550/14551` listeners were free,
then each waited six seconds and received no heartbeat; result `NOT_CONNECTED`. No USB candidate was
present during that first probe. After both USB cables were connected, read-only evidence mapped P1
to ttyACM1/sysid1 and P2 to ttyACM0/sysid2, proved PX4 1.17.0 and the intended `MAV_2_*` values.
Both Ethernet instances were UP/configured yet tx=0 with rising txerr. Sending GCS 255/190 heartbeat
from the bound host port restored inbound immediately on each side (P1 613, P2 596 datagrams/12 s).

The updated subscriber then sustained both sides for 610 seconds and reached `LIVE`: P1 18,300 and
P2 18,299 ATTITUDE samples, final HB age 0.103/0.579 s, ATT age about 0.020 s and no RX stall. This
crosses the old 5.5-minute failure interval but is only one current-boot qualification.

The first runtime audit found user-site dependency leakage and a broken NVIDIA boundary. The current
host has since been repaired: isolated `PYTHONNOUSERSITE=1` imports, both launcher
checks, all GStreamer factories, CUDA and NVIDIA driver 595.84 pass. A later same-host fresh
checkout/venv recreated the manifest and both launchers, and portability now passes with zero
findings. R10 still awaits a different clean Ubuntu OS/VM.

SASL/NGII authentication-only gate: the Workstation process retained an established TCP connection
to `210.117.198.83:2101`, sent/acked 153 bytes, received `ICY 200 OK` and showed
`CONNECTED / VALID` plus health `STALE`, not 401. With MAVLink disconnected, `LEFT_GNSS` explicitly
reported no message, GGA sent remained zero and no RTCM
payload followed. At that auth-only stage, RTCM receive/injection remained unverified.

Updated-app integrated NTRIP gate then passed: `ICY 200 OK`, credential source session, periodic GGA,
80 CRC-valid RTCM frames at 5.10 Hz/0.52 s age, and active P1/P2 delivery with failure 0. The app
reported READY/RTCM_ACTIVE. This proves the short real transport/parser/route chain; attached GNSS
still reported NO_FIX, so RTK convergence and sustained/cold-boot evidence remain open.

The replacement-RTL8156B repeat then provided the sustained oracle: one Stop/reconnect, one HTTP 200,
120/120 valid GGA, 2,954 CRC-valid station/MSM RTCM frames over 636.507 seconds, and final app
injection 3,026 frames/84,125 bytes per side with failure/stall-block zero. P1/P2 remained LIVE and
wire heartbeat/ATTITUDE stayed continuous. See
`evidence/ntrip-isolated-rtl8156b-repeat-20260827.md`.

Combined Preview/NTRIP field gate: both NVDEC panes became LIVE, then P2 entered RX_STALLED. A
five-second wire capture had 599 packets/drop 0, continuous P1 inbound and no P2 inbound while host
heartbeat continued; P2 USB heartbeat also timed out though ping remained alive. User physical P2
power reapply restored sysid 2/ATTITUDE on the existing host socket and both sides entered
RECOVERING. This remains a historical shared-broadcast-domain FIELD FAIL. Subsequent isolated-NIC
direct camera load and two separate application ten-minute gates did not reproduce the MAVLink
stall; an equivalent-rate NTRIP+camera repeat on the isolated topology and the final 60-minute gate
remain pending, while the exact PX4-internal mechanism is still unproven.
Preview remained active after the first attempted stop. Both recovered links later stopped again;
UI ages reached about 235/241 seconds while host heartbeat continued. The combined result is now a
dual-link FIELD FAIL. Preview was then stopped and RTSP children/sockets were confirmed absent.

Latest-app Connect freeze gate: `/tmp/mavlink_latest_connect_20260826.pcap` (SHA-256
`70856311071ecefea91c1e7166a5961f9e17b22ca3436931cd7c545938dcca42`) recorded 49.13 seconds and
12,724 valid frames. Host sent only one heartbeat to each peer; both peers replied, but no command
511/profile request followed because GUI completion and the receive timer never advanced. This is a
Workstation UI lifecycle failure, separate from PX4 #25956 wire silence.

## Feature isolation and contract evidence

No Workstation/StereoWIRIS runtime owner remained after initial setup. UDP 14550/14551 were free.
One owner will be used for all hardware runs; one-side failure and storage/decoder failure isolation
remain required.

## Visual authenticity and dynamic-layout stability evidence

No new UI is proposed. Existing text/role properties remain truthful without synchronous runtime
QStyle repolish. The gated panel regression preserves event-loop progress, geometry, focus,
selection, splitter/dock and scroll ownership under a 32-status burst.

## Visual and hardware/manual evidence

Physical prerequisites and substantial protocol gates pass: isolated device routes, two adapter
camera-load A/Bs, two application ten-minute MAVLink/PARAM gates, short NTRIP RTCM injection, STUS
ordered ACK and repeated Measurement rollback. Clean camera video/STUS convergence/portable real
MP4 remain blocked by zero-IDR; full integrated NTRIP/cold-boot/60-minute acceptance is not run.

## Residual conditions

The confirmed FW 2.18.72 zero-IDR sender is the common CRITICAL blocker for R1/R2/R3/R7. PX4
MAVLink passes isolated-NIC short and ten-minute gates, while the internal shared-L2 stall mechanism
remains unproven. Camera reboot approval, NTRIP credential/RTK, cold boots and final 60 minutes remain.
