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
| R10 | canonical reproducible runtime | doctor/launchers/root venv | PASS: portability 12→0, same-host fresh venv, and separate official Ubuntu 22.04.5 KVM exact-commit recreation; package closure, both launchers and isolated HOME pass |
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
  **95 + 16 passed**. Complete Workstation suite at that slice: **1466 passed, 1 skipped**.

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
  Workstation and Legacy also passed again under isolated HOME/config/cache. This same-host evidence
  was later independently repeated and closed on a separate clean Ubuntu VM. Full initial artifact:
  `evidence/fresh-runtime-portability-20260827.md`.

### Separate clean Ubuntu VM closure

- Official Ubuntu Jammy image SHA-256 matched its published checksum. A clean Ubuntu 22.04.5 KVM
  guest booted with cloud-init errors 0 and no `.git` or inherited installed runtime.
- Exact pushed commit `132c18544158368fc89ac98dc457b8e7a1341f9c` was exported; all 2,679 tracked
  files matched their independent archive hashes inside the guest. Both intentionally external
  YOLO files matched their tracked manifests.
- RED 1: system PyGObject left `pycairo` unsatisfied. `python3-cairo` is now in the Ubuntu manifest;
  final `pip check` reports no broken requirements.
- RED 2: Legacy's ignored model was not actually bundled. It now has a tracked size/SHA-256
  manifest, explicit provisioning, and fail-closed default-model validation. Same-size tampering
  failed the real checker; restoring approved bytes passed.
- Deployment Software, execution Software, Workstation Auto and Legacy Auto/MAVLink-only all passed.
  Both launchers passed again under a new isolated HOME/config/cache. Auto truthfully selected
  `avdec_h264` because no NVIDIA device was passed through.
- Local regressions: Legacy non-monolithic **421 passed**; Workstation **1466 passed, 1 skipped**;
  new/adjacent runtime/model contract **34 passed**; portable path policy PASS.
- Full artifact: `evidence/clean-ubuntu-vm-runtime-20260827.md`. R10 is closed; this does not promote
  any hardware requirement.

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

### STUS 30-minute relative oscillator holdover

- A read-only 1,800.022-second dual RTCP/TCP run collected 381 Left and 377 Right Sender Reports in
  one continuous segment per side with zero reconnects and no STUS/device mutation.
- Robust RTC/NTP regression confirms Right faster than Left by **16.605 ppm**, conservative 95%
  interval **+11.863 to +21.356 ppm**. Independent RTP media-clock regression agrees at
  **+16.634 ppm**, interval **+11.965 to +21.401 ppm**.
- The direct relative SR-PC difference moved +31.356 ms over 30 minutes (+17.420 ppm). Every
  five-minute checkpoint retained the same direction. At the final point estimate, 100 ms
  accumulates in about 100.4 minutes.
- Exact method, checkpoints, caveats and raw artifact hash are in
  `evidence/stus-relative-oscillator-30m-20260827.md`. This confirms oscillator-rate difference for
  the current powered run; it does not close STUS phase convergence or post-reboot repeatability.

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

The current full Workstation suite passes fresh (**1515 passed, 1 skipped**). A broad StereoWIRIS suite remains affected by the known
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
findings. A later official Ubuntu 22.04.5 KVM recreation passed exact pushed source, dependency
closure, both launchers and isolated HOME, closing R10.

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
remains unproven. Open-sky RTK Fixed, three independent cold boots and final 60 minutes remain.

An additional indoor, decoder-free SDK qualification confirmed this boundary. Left visible IDR
intervals 0/1/2/3 produced exact SDK ACK/readback but five fresh raw Annex-B artifacts across both
cameras contained NAL1/SPS/PPS/AUD and zero NAL5. Non-IDR I pictures remained `key_frame=0`.
Repeated read-only SDK access independently collapsed Right FTP/SDK/RTSP while ping remained live;
operator physical power recovery restored all services, and both cameras finish at
`GETH=TRUE`, `GSSS=0`, `IRCR=FALSE`, visible `0/1/30/1`. See
`evidence/indoor-wiris-idr-sdk-20260827.md`.

The requested pre-NTRIP shared-NIC A/B now fails on two different physical adapters. On the second
adapter, no-camera direct ETH MAVLink returned 602/613 datagrams after 12 host heartbeats per side.
Adding dual Preview then left P1 healthy but drove P2 to sustained ETH RX STALLED: a 10-second pcap
had 1,586 P1 inbound packets, zero P2 inbound and 11 host heartbeats to each peer. P2 USB heartbeat
remained alive, so the failure is Ethernet-instance-specific. NTRIP was never started. See
`evidence/shared-nic-camera-pixhawk-baseline-20260827.md`.

## Proposed app-receive fallback verification matrix — NOT_RUN / not approved

| Slice | Independent evidence required | Status |
|---|---|---|
| Basis decision | hand-built VERIFIED, revoked, unseeded, transition and recovered state table | NOT_RUN |
| Pair selection | distinct sentinel NTP/app timestamps; drifted NTP but app delta ≤100 ms selects app | NOT_RUN |
| Failure bounds | app delta >100 ms, mixed basis generation, epoch transition/mismatch produce no pair | NOT_RUN |
| Geometry isolation | identical pixels/calibration produce identical geometry across timing bases | NOT_RUN |
| Persistence | independent readback proves selected/app/NTP fields and compact `sync_delta_ns` meaning | NOT_RUN |
| Legacy read | v3 row without selected-basis fields retains historical interpretation | NOT_RUN |
| UI | NTP→fallback→transition→recovery text, ESTIMATED validity and stable geometry/focus | NOT_RUN |
| Integration | real drift/revocation keeps bounded app-timed distance and NTP evidence, then recovers | NOT_RUN |

## STUS cadence reliability evaluation

| Question | Current evidence | Status |
|---|---|---|
| Does a shorter interval reduce oscillator drift? | 16.605 ppm implies 4.98/9.96/14.94/29.89 ms per 5/10/15/30 min | PASS (model + 30m field data) |
| Is current one-cycle STUS repeatable? | two preflight failures; one exact dual-OK action changed about 140 ms to about 3.11 s and never reached correction | FAIL |
| Can 5-minute Auto make the current actuator reliable? | saves only about 25 ms versus 30 min; does not bound seconds-scale apply error | FAIL / unsafe to run repeatedly now |
| Five consecutive repaired manual cycles | final delta ≤100 ms, target ≤50 ms, no continuity/resource failure | NOT_RUN |
| Six-cycle 5-minute Auto stress | every due action converges and preserves active work | BLOCKED by manual-cycle gate |
| Selected operational cadence soak | interval derived from residual + drift + safety margin | BLOCKED |

## App-receive-authoritative distance verification — software PASS / hardware gate NOT_RUN

| Slice | Independent evidence required | Status |
|---|---|---|
| Exact boundary | integer sentinel proves monotonic ns is captured at decoded appsink receipt before YOLO | SW PASS: one `monotonic_ns` read is reused for exact ns/compatibility float; capture sites precede copy/conversion/YOLO |
| Pair truth | distinct app/NTP sentinels prove nearest pair and ≤100 ms gate use app only | SW PASS: 20 ms and exact 100 ms selected on app time |
| NTP noninterference | missing, +9h, regressed and >100 ms NTP values produce identical app pair/result | SW PASS: Clock epoch transition/mismatch, +9 h, malformed and nonfinite NTP do not alter selection/snapshot |
| App failure | missing/nonfinite/regressed or 100.1 ms app delta produces no pair/distance | SW PASS: explicit invalid/over-limit rejection, including exact 100 ms + 1 ns and malformed exact fields |
| STUS continuity | active STUS and NTP clock-epoch changes do not pause, flush or degrade valid app pairs | SW PASS: distance policy token contains no Clock/epoch state |
| Method validity | otherwise-ready app pair is normal VALID; other geometry/input blockers remain exact | SW PASS |
| Persistence | independent readback proves selected/app delta equality and distinct record-only NTP fields | SW PASS: JSONL and Parquet v4 readback |
| Schema compatibility | v3 RTCP rows retain original meaning; v4 compact sync delta is app selected truth | SW PASS |
| Backend parity | GStreamer and every supported decoded backend emit the same app timestamp contract | SW PASS by boundary/source inspection plus worker regressions; hardware GStreamer receipt remains NOT_RUN |
| UI stability | fixed APP RECEIVE, authoritative app delta, record-only NTP delta; no focus/layout churn | SW/UI PASS: offscreen tests plus AT-SPI and visual inspection |
| Integration | sustained target-hardware app pairing during Clock failure/STUS with complete saved NTP | NOT_RUN: both FW 2.18.72 streams still emit zero NAL5, so no decoded-frame pair reaches this boundary |

### App-receive-authoritative implementation evidence — 2026-08-27

- GStreamer, OpenCV and PyAV decoded-frame readers call the same exact integer timestamp helper at
  frame receipt. The exact nanoseconds and explicit `authoritative_distance` role survive worker
  transport, immutable detection snapshots, pair identity, all method fan-out and storage.
- Coordinator sentinels independently separate app and NTP values: a 20 ms app pair is selected
  despite a +9-hour NTP difference and mismatched/transitioning Clock epochs; exact 100 ms is
  accepted, while 100 ms + 1 ns or a noninteger/invalid exact app timestamp is rejected. Malformed
  or nonfinite NTP is normalized as unavailable record-only evidence and cannot block the app pair.
- Schema v4 raw JSONL and typed Parquet readback retain selected/app delta equality and a separately
  named NTP delta. Schema-aware readers preserve the historical v3 RTCP-delta interpretation.
- Focused boundary, pairing, snapshot/method, persistence, schema, settings and UI command:
  `QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q
  tests/test_workstation_yolo_stream.py tests/test_detection_pair_coordinator.py
  tests/test_measurement_canonical_evidence.py tests/test_measurement_methods.py
  tests/test_recording_raw_artifacts.py tests/test_recording_reader.py
  tests/test_session_parquet.py tests/test_workstation_settings.py
  tests/test_workstation_contracts.py tests/test_workstation_camera.py` returned **244 passed**.
- The final complete Workstation regression returned **1515 passed, 1 skipped** in 129.23 seconds.
  `tools/gen_panel_delegate_stubs.py --check`, `git diff --check` and `./run.sh --check` pass.
- The live Measurement surface was inspected offscreen and through AT-SPI: `APP RECEIVE` is fixed
  and read-only, the hard gate is 0.100 seconds, the authoritative app delta and `NTP record-only`
  labels are distinct, and the layout remains stable. Screenshot:
  `/tmp/workstation_app_receive_measurement_ui_final_20260827.png`.
- Target-hardware pairing is deliberately **NOT_RUN**, not PASS: the current camera firmware's
  zero-NAL5 stream prevents decoded frames from reaching the newly timestamped boundary. That
  external media blocker must be resolved before a sustained Clock-failure/STUS integration gate.

## Proposed indoor 60-minute experiment matrix — NOT_RUN / awaiting approval

| Slice | Independent evidence | Status |
|---|---|---|
| Strict default | flag absent still rejects every NAL1 and requires exact NAL5/PTS | NOT_RUN |
| NAL1 classifier | hand-built AU cases: missing SPS/PPS, delta, mixed/malformed, bad PTS/flags reject | NOT_RUN |
| Recovery state | exact candidate PTS opens only after decoded return; next gap relatches per side | NOT_RUN |
| Provenance/UI | experimental basis visible/saved; never IDR/keyframe; stable focus/layout | NOT_RUN |
| MP4 truth | bytes/EOS may pass; usable-IDR/portable integrity remains FAIL | NOT_RUN |
| NTRIP secret | process-local SASL credential, HTTP 200, no log/settings/keyring secret | NOT_RUN |
| Five-minute gate | dual camera/P1/P2/NTRIP/recording/UI/resource prerequisites | NOT_RUN |
| 60-minute soak | twelve checkpoints, no required terminal failure, exact 3,600 s | NOT_RUN |
| Saved evidence | app pair ≤100 ms, NTP record-only, RTCM/MAVLink counters, artifacts readable | NOT_RUN |
| Shutdown/rollback | EOS/writer drain, no child/socket/ring residue, strict relaunch | NOT_RUN |

Existing offline real artifacts provide feasibility only: 17/17 adjacent Left NAL1-I GOP segments
started a fresh FFmpeg decoder with `-xerror`, while a Right segment failed after a later packet
omission. This does not substitute for the proposed product/hardware run.

### Prepared five-minute and checkpoint oracle

The current idle real GUI exposes the required commands and diagnostics through AT-SPI, so the
monitor does not infer liveness from button text alone. A checkpoint is valid only when one bounded
AT-SPI traversal completes and the matching external PID/socket/file counters are sampled in the
same interval.

Five-minute admission requires:

- both camera rows current with nonzero decoded FPS and explicit experimental provenance;
- both Pixhawk rows have fresh expected-sysid heartbeat and attitude, not merely open UDP sockets;
  both must become `LIVE` after 60 seconds, while the first 10-minute checkpoint separately proves
  sustained continuity;
- NTRIP HTTP 200/streaming, current CRC-valid RTCM and dual delivery failures equal zero;
- Measurement/Detection active, or an exact no-target/no-geometry BLOCKED reason while frame/pair
  evidence continues; recording files show positive growth;
- GUI traversal below two seconds, no callback exception, storage OK, at least 100 GiB free, and no
  unbounded queue/RSS trend.

Every 300-second soak checkpoint records elapsed monotonic ns, side states/details/FPS, app pair
delta/count/drop reason, Clock/SR observation, MAVLink ages/rates/counters, NTRIP GGA/RTCM/delivery,
recording bytes/writer state, GUI traversal latency, process/child/socket/ring set, CPU/RSS, GPU
utilization/memory/encoder sessions, NIC errors/drops and disk free. Secrets and exact credential
environment values are excluded by schema.

The user-approved location is now outdoor. Add P1/P2 actual rover fix type and horizontal/vertical
accuracy to every checkpoint; preflight must observe current GNSS from both sides, and final
qualification reports sustained RTK Fixed separately from transport/injection. The remaining matrix
is unchanged and is now ready for implementation/execution.

## Manual-only PX4 calibration PARAM verification amendment

| Criterion | Evidence | Status |
|---|---|---|
| Repeated LIVE status refresh sends no PARAM work | controller regression with repeated status flush | SW PASS |
| P1/P2 manual full refresh remains routed | command/backend regression | SW PASS |
| Advanced MAG/attitude refresh remains routed | existing command regressions | SW PASS |
| Reset/apply/save readback remains intact | focused PX4 manager/controller regressions | SW PASS |
| No settings/schema/UI geometry change | source diff plus offscreen workspace regression | SW PASS |
| Complete Workstation regression and launcher | canonical venv commands | SW PASS |

No hardware request is required to prove removal of an implicit caller. Historical automatic PARAM
pcaps remain valid old-build evidence and are not reclassified.

Focused exact-path tests returned **3 passed**. The expanded MAVLink liveness, Pixhawk status,
calibration manager, RTCM reservation and Workstation command/UI regression returned **183 passed**.
Generated delegate check, Python compilation and `git diff --check` passed. These are software/fake
results; the running pre-change 600-second field process remains a historical baseline and must not
be presented as evidence for the restarted manual-only build.

The complete canonical Workstation suite returned **1523 passed, 1 skipped in 138.70 s**. The
documented `./run.sh --check` entry point passed with the canonical interpreter, all required
GStreamer factories, Auto→`nvh264dec`, NVIDIA driver and declared runtime packages. Final
compileall, generated-delegate check and `git diff --check` passed. The pre-change Workstation
process exited after its 600-second baseline monitor, so the next launch uses this manual-only source.

## 60-second operational MAVLink qualification verification amendment

| Criterion | Independent evidence | Status |
|---|---|---|
| Exact boundary | fake monotonic 59.999 remains QUALIFYING; 60.000 with fresh HB+ATT is LIVE | SW PASS |
| Recovery | all-RX stall/resume and remote reboot require a new 60-second interval | SW PASS |
| Explicit command | manual PARAM boundary requires post-command HB+ATT plus 60 seconds | SW PASS |
| Immediate blocking | ATTITUDE stale and all-RX stall thresholds remain unchanged | SW PASS |
| No hidden PARAM | repeated LIVE status produces no automatic readback or reset | SW PASS |
| UI/evidence | all operator/status/evidence text derives `01:00` from the health contract | SW PASS |
| Monitor separation | external 10/60-minute stability duration remains independent of 60-second LIVE | SW PASS |
| Full regression/launcher | canonical Workstation suite and documented runtime check | PASS |

Coordinate review: this slice changes duration/state presentation only. PX4 body-FRD attitude,
earth-NED, camera optical RDF, stereo-center and stored coordinate fields are unchanged.

RED focused run returned the expected **10 failed, 114 passed** against the old 600-second source.
After implementation, the exact focused set returned **124 passed** and the expanded MAVLink,
Pixhawk, status, RTCM, manual-PARAM and UI command set returned **192 passed**. A dynamic 65-second
sentinel additionally proves the transition log formats the snapshot-owned requirement rather than
a new hard-coded `01:00` literal.

The complete canonical Workstation suite returned **1524 passed, 1 skipped in 140.13 seconds**.
`./run.sh --check`, generated panel delegates, direct system-Python monitor `--help`, portable path
policy, Python compilation and full `git diff --check` all pass. The approved README target is
`apps/Workstation/README.md`.

Target-hardware latest-build 60-second confirmation is **NOT_RUN**: the operator disconnected every
wired device before the optional final probe. The attempted app launch truthfully reported the
configured NIC absent and then completed normal Quit with exit code zero; no P1/P2 packets were
available, so no hardware PASS is claimed. Earlier 10-minute captures remain historical old-timer
evidence and are not substituted for this new-build check.

## Mountain GNSS 3D-fix verification amendment

| Criterion | Independent evidence | Status |
|---|---|---|
| 3D-or-better admission | same-burst Left fix 3 + Right fix 5 exposes all translation axes | SW PASS |
| Sub-3D fail-close | Left fix 2 leaves North/East/Down unobservable with numeric reason | SW PASS |
| Baseline integrity | non-finite position and zero baseline each fail before candidate use | SW PASS |
| Rotation isolation | existing Mountain robust inlier/rank/condition solve remains required | SW PASS |
| Draft boundary | latest non-fixed 3D Mountain Draft freezes at next Measurement and is ready | SW PASS |
| Quality/persistence | warning survives worker outcome as `ESTIMATED`; method ID is `mountain_gnss` | SW PASS |
| Representative stability | canonical method order still starts with `pixhawk_rtk_live` | SW PASS |
| Scope | no setting/schema migration/UI layout/device command; docs updated | SW PASS |
| Full regression/runtime | canonical suite plus launcher/runtime and static checks | PASS |

Exact Mountain-focused integration returned **9 passed**. The expanded optical-pose, Measurement,
method-worker, canonical-evidence and UI-contract set returned **132 passed**. The complete
canonical Workstation suite returned **1529 passed, 1 skipped in 122.82 seconds**.
`./run.sh --check`, generated delegate verification, targeted compileall and targeted
`git diff --check` all pass. This gate consumes immutable captured evidence and sends no hardware
command, so a new device-side run is not required to verify the software amendment.

## Mountain Use suggestion quality-separation verification — software/UI PASS

| Criterion | Independent evidence | Status |
|---|---|---|
| Baseline | LOW_CONFIDENCE with distributed points leaves `Use suggestion` disabled | OBSERVED |
| Confidence separation | structurally usable LOW_CONFIDENCE enables/copies with warning | SW/UI PASS |
| Hard structure | <4, out-of-range, duplicate, near-collinear and malformed pairs reject | SW PASS |
| Identity | adversarially retained stale proposal rejects by current frame ID/SHA contract | SW PASS |
| Exclusions | one paired index set filters both sides and persists exact retained input | SW PASS |
| Quality persistence | LOW_CONFIDENCE remains in accepted snapshot/UI warning | SW/UI PASS |
| UI stability | same 770×164 group/control geometry; COPY READY/BLOCKED changes only state text | VISUAL PASS |
| Horizon at Mountain-slice time | LOW_CONFIDENCE rejection was the then-current contract | HISTORICAL; superseded below |
| Solver/GNSS isolation | Solve has no direct proposal path; K,D/GNSS/robust gates unchanged | SW PASS |
| Direct Solve | not approved and not implemented | OUT OF SCOPE |
| Regression/runtime | dependent and complete Workstation tests plus launcher/static checks | PASS |

Focused command:
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 QT_QPA_PLATFORM=offscreen
../../.venv/bin/python -m pytest -p no:cacheprovider -q
tests/test_optical_pose_auto_select.py tests/test_optical_pose_workspace.py
tests/test_optical_pose_session_persistence.py tests/test_optical_pose_controller_integration.py
` returned **76 passed**.

The dependent command added optical-pose workers, command/lifecycle routing, generated delegate and
controller-delegation tests and returned **139 passed**. After the final native-dimension UI
fail-close check, the complete current Workstation suite returned **1564 passed, 2 skipped in
125.59 seconds** with no warnings.
`./run.sh --check`, generated delegate verification, targeted compileall and full
`git diff --check` all pass.

Reconstructed before and actual after offscreen captures are
`/tmp/workstation_mountain_use_suggestion_before_20260829.png` and
`/tmp/workstation_mountain_use_suggestion_after_20260829.png`. Both are 770×164; the after image
retains LOW_CONFIDENCE, enables the existing button and adds COPY READY without a layout change.
This slice transforms only same-order native OpenCV `(u,v)` points and sends no hardware command;
target-device verification is not required.

## Horizon Use suggestion quality-separation verification — software/UI PASS

| Criterion | Independent evidence | Status |
|---|---|---|
| Quality separation | structurally valid LOW_CONFIDENCE enables and accepts | SW/UI PASS |
| Quality display | state, confidence, residual, reason and accepted warning remain | SW/UI PASS |
| Endpoint structure | exact two-per-side, finite bounds and 25%-width separation | SW PASS |
| Identity | stale frame ID/SHA and dimension mismatches reject | SW PASS |
| Persistence | normalized endpoints, validated-input SHA and warning reach acceptance event/review | SW PASS |
| UI stability | same 771×143 group/control geometry; COPY READY/BLOCKED changes only state text | VISUAL PASS |
| Mountain regression | structural exclusions and LOW_CONFIDENCE copy remain passing | SW PASS |
| Solve/Apply isolation | explicit Horizon Solve/Apply and exact K,D/observability gates unchanged | SW PASS |
| Regression/runtime | focused, dependent, complete suite and launcher/static checks | PASS |

Focused command:


`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 QT_QPA_PLATFORM=offscreen
../../.venv/bin/python -m pytest -p no:cacheprovider -q
tests/test_optical_pose_auto_select.py tests/test_optical_pose_workspace.py
tests/test_optical_pose_session_persistence.py tests/test_optical_pose_controller_integration.py`
returned **82 passed**. Adding optical-pose workers, command/lifecycle routing, generated delegate
and controller-delegation tests returned **145 passed**. The complete current Workstation suite
returned **1583 passed, 2 skipped in 141.24 seconds**.

Generated delegate verification, targeted compileall and full `git diff --check` pass.
`./run.sh --check` exits zero; the current host emitted an unrelated optional warning that
`nvh264dec` could not be instantiated and Auto selected `avdec_h264`. No Horizon code owns decoder
selection.

Reconstructed before and actual after captures are
`/tmp/workstation_horizon_use_suggestion_before_20260829.png` and
`/tmp/workstation_horizon_use_suggestion_after_20260829.png`. Both are 771×143. The after image
retains LOW_CONFIDENCE/confidence/residual/reason, enables the existing button and adds COPY READY.
This slice only validates/copies native OpenCV `(u,v)` endpoints and sends no hardware command;
target-device verification is not required.

## Approved Settings-preferred distance timing verification — implementing

| Criterion | Independent evidence | Status |
|---|---|---|
| Default/migration | missing/invalid default, strict legacy migration and RTCP save/reload | SW PASS |
| Healthy preference | distinct app/NTP sentinels prove selected healthy RTCP owns pair and PX4 result | SW PASS |
| Mandatory fallback | Clock/frame/timestamp/epoch/delta failures select exact app; dual failure rejects | SW PASS |
| Generation isolation | requested/effective changes flush buffers; stale revision input/result rejects | SW PASS |
| Cross-layer identity | coordinator decision freezes through snapshot, PX4 host-time lookup and workers | SW PASS |
| Persistence | schema-v5 JSON/Parquet v4 requested/effective/fallback and old-row semantics | SW PASS |
| Preview regression | valid preview reaches UI/history with local input-integrity text | SW PASS |
| UI stability | enabled combo plus fallback status at 1400×1000 without geometry expansion | SW/VISUAL PASS |
| Regression/runtime | 299 focused PASS; complete Workstation suite and launcher | FOCUSED PASS / FULL NOT_RUN |

Software implementation and verification are authorized. Hardware behavior remains separately
NOT_RUN unless current devices can provide usable decoded-frame evidence without a device mutation.

Focused command covered coordinator, canonical evidence, all method workers, Measurement boundary,
package UI, raw recording/readers/Parquet, Settings, camera status, command contracts and field
pairing and returned **299 passed**. Exact app/RTCP sentinels prove healthy RTCP 10 ms selection
versus app 90 ms, and fallback selects exact app ns without adding a measurement warning. Synthetic
Pixhawk-attitude/GNSS/K,D geometry returns a VALID approximately 9 m result under both healthy RTCP
and app fallback. The successful non-multimethod preview path now displays `input=VERIFIED` without
the former scope exception.

Offscreen visual evidence `/tmp/workstation_timing_settings_20260831.png` is 1400×1000. It shows the
existing dense Measurement layout, selectable RTCP preference, stable one-line
`RTCP/NTP → APP RECEIVE · fallback=CLOCK_NOT_VERIFIED` status, explicit next-Measurement Apply note,
no added card/modal/style, and unchanged control positions. Hardware/field timing remains NOT_RUN.
