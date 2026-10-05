# Verification

## Requirement-to-evidence matrix

| Requirement | Automated evidence | Hardware/manual evidence |
|---|---|---|
| GCS identity + 1 Hz HB | fake clock/link send assertions | packet capture P1/P2 |
| Stock stream profile | exact request/rate tests | observed message rates |
| ATT stale vs all-RX stall | deterministic transition tests | unplug/stall observation |
| Per-side RTCM fail-close | enqueue + send-race tests | counters during one-side fault |
| No false host recovery | controller command tests | app restart does not claim recovered |
| 10-minute qualification | fake-clock reset/pass tests | dual P1/P2 continuous soak |
| Safe recovery | sysid/lifecycle/rejection tests | approved side-only reboot exercise |

## Focused test evidence

### Slice 1 — pure liveness/qualification model: PASS (automated-real pure logic)

- Command: `/usr/bin/python3 -m pytest -q tests/test_mavlink_liveness.py`
- Result: `10 passed in 0.09s`.
- Independent assertions: listener/identity gate; wrong sysid, local echo, and `BAD_DATA` rejection;
  ATTITUDE `2.0/2.001 s`; all-RX `9.999/10.000 s`; heartbeat-stale/other-RX-fresh;
  qualification `599.999/600.000 s`; stalled packet resume to RECOVERING; connection generation;
  active-command suppression and post-command fresh HB+ATT requirement.
- New boundary is pure `workstation_core/mavlink_liveness.py`; Qt/pymavlink integration remains
  integrated and verified by Slice 2 below.

### Slice 2 — GCS identity/heartbeat and receive boundary: PASS (automated-fake transport)

- Commands:
  - `/usr/bin/python3 -m pytest -q tests/test_mavlink_liveness.py` → `14 passed in 0.20s`.
  - `/usr/bin/python3 -m pytest -q tests/test_workstation_pixhawk_gnss.py` →
    `62 passed in 4.74s`.
- Verified: operational `mavlink_connection` kwargs `source_system=255` and
  `source_component=190`; no heartbeat before expected sysid; one send/side/second; no missed-tick
  burst; stop cleanup; bounded exception attempts/logs; wrong-sysid rejection; raw arrival recorded
  before duplicate filtering; deferred command messages retain original monotonic receive time.
- Hardware packet capture of outbound 255/190 heartbeats remains NOT_RUN.

### Slice 3 — Stock stream profile and consumer audit: PASS (automated-fake + pure consumer regression)

- Commands:
  - `/usr/bin/python3 -m pytest -q tests/test_mavlink_liveness.py tests/test_workstation_pixhawk_gnss.py`
    → `77 passed in 5.52s`.
  - `/usr/bin/python3 -m pytest -q tests/test_pixhawk_magnetometer_workspace.py tests/test_measurement_methods.py tests/test_recording_internal_artifacts.py tests/test_workstation_contracts.py`
    → `74 passed in 2.82s`.
- Exact default profile retains 15 standard messages with named consumers: ATTITUDE; SYS_STATUS;
  GPS_RAW_INT/GPS2_RAW; GPS_RTK/GPS2_RTK; SCALED_IMU/2/3; DEBUG_VECT; RAW_IMU;
  LOCAL_POSITION_NED; GLOBAL_POSITION_INT; VFR_HUD; BATTERY_STATUS.
- `SASL_CAN_MAG` is absent from the default request and has no retry. Its unsolicited custom-firmware
  decoder remains compatible. All-RX stall blocks both initial profile and ATTITUDE recovery request.
- Hardware observed message rates/packet reduction remain NOT_RUN.

### Slice 4 — state projection, qualification readiness, and no false reconnect: PASS (automated-fake UI/controller)

- Command: `/usr/bin/python3 -m pytest -q tests/test_mavlink_liveness.py tests/test_status_model_pixhawk.py tests/test_workstation_pixhawk_gnss.py tests/test_workstation_contracts.py tests/test_workstation_ui.py tests/test_workstation_lifecycle.py`
  → `153 passed in 20.90s`.
- Verified projected states/severity/detail for CONNECTING/QUALIFYING/LIVE/ATTITUDE STALE/
  ETH MAVLINK RX STALLED/RECOVERING, fixed `mm:ss / 10:00`, HB/ATT/ANY ages, endpoint/sysid,
  RTCM blocked text, and suspected-not-proven cause wording.
- Existing recording protocol still accepts only `pixhawk.* == LIVE`; therefore 600-second model
  qualification is the READY gate. PARAM command remains a truthful temporary override.
- Re-pressing Connect on a confirmed all-RX stall emits guidance and never invokes host
  disconnect/connect. State-transition event emits once per transition, not every timer tick.
- P1/P2 fixed-width status cards stayed geometrically identical across all five dynamic states.
- Hardware/visual-manual screenshots and real 10-minute qualification remain NOT_RUN.

### Slice 5 — per-side RTCM fail-close: PASS (automated-fake boundary/integration)

- Command: `/usr/bin/python3 -m pytest -q tests/test_rtcm_pipeline_workers.py tests/test_rtcm_pipeline_end_to_end.py tests/test_rtcm_pipeline.py tests/test_workstation_ntrip.py tests/test_workstation_pixhawk_gnss.py tests/test_mavlink_liveness.py`
  → `188 passed in 10.08s`.
- Enqueue captures per-side eligibility/generation; the subscriber send lock atomically rechecks
  both immediately before `GPS_RTCM_DATA` serialization.
- Verified P1 stalled/P2 live partial delivery, enqueue-live→send-stalled race, stale generation,
  source-frame age, queue overflow, worker retirement, evidence barrier, and transport failure.
- Policy result records `blocked_stall`, target ages/generations, and `target-mavlink-stalled` without
  incrementing transport `fail` or stale-drop counters. GNSS readiness removes the stalled endpoint
  identity and records a route fault.
- RTCM sends no longer enter the blocking PARAM/SERIAL receive reservation, so a 4 Hz correction
  stream does not reset 600-second MAVLink qualification.
- Real P1/P2 one-side outage and live-side correction delivery remain NOT_RUN.

### Slice 6 — operator-confirmed verified-USB recovery: PASS (automated-fake USB/controller/UI)

- Command: `/usr/bin/python3 -m pytest -q tests/test_workstation_pixhawk_gnss.py tests/test_workstation_ui.py tests/test_workstation_lifecycle.py tests/test_workstation_contracts.py tests/test_status_model_pixhawk.py tests/test_mavlink_liveness.py tests/test_rtcm_pipeline_workers.py tests/test_rtcm_pipeline_end_to_end.py tests/test_workstation_ntrip.py`
  → `261 passed in 22.59s`.
- USB discovery preserves device/by-id path, resolved path, baud, heartbeat sysid/component, and
  monotonic observation time. Stored evidence must be unique and no older than 30 seconds; the
  one-shot worker reopens the exact device and revalidates identity immediately before reboot.
- Verified P1=sysid1/P2=sysid2 mapping, operator No, missing/duplicate/stale identity, wrong sysid,
  active Measurement, recording/save/finalization, concurrent PX4 command, and worker failure paths.
- Stalled Ethernet is rejected as a reboot transport. USB command send projects COMMAND_SENT only;
  it does not mutate liveness or claim LIVE/READY. Expected Ethernet packets must produce
  RECOVERING and a new 600-second qualification.
- No automatic reboot, PX4 parameter/config write, or firmware action was added.
- Actual USB re-enumeration, correct physical side reboot, other-side continuity, and post-reboot
  qualification remain hardware-manual NOT_RUN (no USB device was present in this session).

### Slice 7 — documentation and repository software gates: BLOCKED on pre-existing portability violations

- Supported interpreter check: root `.venv` has pytest `8.4.2` and pymavlink available.
- Focused command: `../../.venv/bin/python -m pytest -q tests/test_mavlink_liveness.py tests/test_workstation_pixhawk_gnss.py tests/test_rtcm_pipeline_workers.py tests/test_rtcm_pipeline_end_to_end.py tests/test_rtcm_pipeline.py tests/test_status_model_pixhawk.py tests/test_workstation_ntrip.py tests/test_workstation_lifecycle.py tests/test_workstation_ui.py tests/test_workstation_contracts.py`
  → `275 passed in 25.23s`.
- Full command: `../../.venv/bin/python -m pytest -q` → `1224 passed, 1 skipped in 122.72s`.
  The single warning is pytest cache write denial at the sandbox-read-only repository root, not a
  test/runtime failure.
- Generated delegate contract: `../../.venv/bin/python tools/gen_panel_delegate_stubs.py --check`
  PASS. `git diff --check` PASS.
- Portability command: `../../.venv/bin/python ../../tools/check_portable_paths.py` → FAIL with the
  same 12 pre-existing findings documented at handoff: StereoWIRIS calibration/change evidence,
  Workstation `intrinsics_zoom_08.json`, Simulation `goal.md`, and StereoWIRIS settings. The changed
  ETH code/docs/packet have no scoped home/dated-checkout match and are absent from tool findings.
  These unrelated user-owned files were not changed, so formal software Gate 1 remains BLOCKED.
- Product docs updated: `README.md`, UI workflow/status/command/implementation contracts. README
  target is the PLAN-approved workspace `./README.md`. `Pixhawk.md` diff is empty by design.
- `PLAN.md` integrity remains
  `b51477f7d526e9eb0fb959e4bd7391341adef53d11c913d04555405bcea2249d`;
  temporary read-only hardware probe file is absent.

### Canonical launcher environment repair — 2026-08-26: PASS

- Reproduced `./run.sh --check` and `run.log` startup block: the prior root `.venv` had
  `include-system-site-packages = false` plus unsupported
  `workstation-system-dist-packages.pth`.
- Installed Ubuntu `python3.10-venv 3.10.12-1~22.04.17`, preserved the prior 5.6 GB environment as
  `.venv.backup-20260826-172430`, and recreated root `.venv` with official
  `python3 -m venv --system-site-packages`. A first ensurepip-failed partial is retained as the
  28 KB `.venv.failed-20260826-172430` directory.
- Installed required runtime overrides `numpy 1.24.4` and `pyqtgraph 0.13.7`. Restored the
  independently working cached pair `torch 2.13.0+cu130` / `torchvision 0.28.0+cu130` into the
  venv; both import from venv-local paths and the configured YOLO model loads as task `detect`.
- `./apps/Workstation/run.sh --check`: PASS for canonical interpreter, system packages, all 14
  required distributions, GStreamer elements, software `avdec_h264`, and model path. Optional
  `nvh264dec` and NVIDIA driver remain WARN; software fallback is selected.
- Launcher/runtime tests: `46 passed`; MAVLink liveness/status/Pixhawk focused tests under the new
  venv: `96 passed`. Pytest cache warnings are sandbox-only.
- Actual `./run.sh`: Workstation process PID 616185 remained alive and X11 contained a maximized
  `StereoWIRIS Workstation` window. `run.log` contains normal disconnected-device/calibration
  notices and no traceback.
- Another independently owned `StereoWIRIS` process/window was also present. UDP 14550/14551 were
  unowned at inspection time, so no current port collision exists; normal field operation must
  still run only Workstation.
- `pip check` still reports unrelated user-site optional-package conflicts (`torchaudio`, `pyulog`,
  `nvidia-modelopt`, `cupy`, and PyNaCl/cffi). They do not fail the Workstation doctor, compatible
  Torch/Torchvision import, YOLO model load, or current launcher smoke evidence.

## Resume baseline evidence — 2026-08-26 16:32 KST

- `ss -lunp`: UDP 14550/14551 owner 없음.
- P1/P2 ping: 각각 2/2 replies, packet loss 0%.
- related Workstation/StereoWIRIS/QGC process: 없음.
- `/dev/serial/by-id`, `ttyACM*`, `ttyUSB*`: 장치 없음.
- passive `udpin` 및 direct `udpout` GCS heartbeat/version probe: P1/P2 모두 6초간 heartbeat 0.
- classification: host/network reachability PASS; actual MAVLink firmware/sysid/parameter inventory
  BLOCKED until safe operator recovery. No reboot, parameter write, or firmware/config change was sent.

## Regression and application evidence

Pending. Run existing MAVLink/GNSS/RTCM/status/lifecycle suites and the project test suite using the
`run-tests` skill after code changes.

## Performance, load, and target-hardware evidence

Capture RX/TX packet rate before/after GCS heartbeat peer selection, duplicate sequence counts,
Workstation CPU, queue depth/drop counters, and per-side rates. Hardware acceptance requires at
least 10 continuous minutes with both P1/P2, plus RTCM active, because the observed failure occurred
near 5.5 minutes. This is only the minimum implementation gate. Resolution documentation additionally
requires three independent cold-boot cycles of at least 10 minutes each and a 60-minute continuous
dual-link full-load soak.

## Feature isolation and contract evidence

Force one side stale while the other stays live; verify independent status, RTCM routing, and no
cross-side reboot. Verify camera/calibration/recording and NTRIP transport remain unchanged.

## Visual authenticity and dynamic-layout stability evidence

Use existing status components. Capture CONNECTING, QUALIFYING, LIVE, ATTITUDE STALE, ETH RX
STALLED, and RECOVERING at minimum/normal window size and confirm timers do not shift layout.

## Visual and hardware/manual evidence

Pending. Save screenshots, packet capture summary, event-log excerpt, and exact firmware/params.

## Hardware attempt — 2026-08-26 20:05–20:30 KST: FAIL before qualification

- Hypothesis 1, missing GCS peer: **FAIL**. With both boards ping-alive and host ports free, a
  10-second probe sent 1 Hz GCS heartbeats plus read-only version requests from source `255/190`.
  Independent tcpdump saw all 26 outbound packets and zero inbound datagrams.
- Recovery 1: **PASS only for short link restoration**. Operator power reapply restored P1/P2
  broadcast telemetry. The real subscriber then captured P1 29,066 and P2 28,882 packets for
  208 seconds, with HB/ATT/GPS present and parser errors 0. This run was deliberately interrupted
  to release the exclusive UDP ports to the already-running GUI and is not a 5-minute pass.
- GUI handoff: **PASS for truthful ownership/failure**. The first GUI Connect could not bind while
  the diagnostic subscriber owned 14550/14551. After release, the GUI owned both ports; reopening
  host sockets did not revive a subsequently silent PX4 side and did not claim LIVE.
- Recovery 2 / actual-app-from-boot attempt: **FAIL before identity**. The operator reapplied power
  while the GUI listeners were open. No new expected identity arrived. Restarted Workstation PID
  730044 again owned both ports but remained `CONNECTING`, `sysid=None`, `HB/ATT/ANY=--`.
- Independent zero-packet artifact:
  `/tmp/sasl_eth_mav_app_soak_20260826T202512KST.pcap`, 24 bytes, SHA-256
  `704e5e5b3234433c01fcfd1b20a306e77e985038120492dc53965c3edd38a4ea`.
  The capture ran from 20:25:12 until 20:29:53 and contains no packet; P1/P2 ping remained normal.
- The earlier nonzero pcap is
  `/tmp/sasl_eth_mav_workstation_soak_20260826T201424+0900.pcap`, 5,681,024 bytes, SHA-256
  `926f0e02f7b20704211d651ee64fb3526aac60d50a331f53a821e9d9a1459880`.
- New trigger candidate: first-heartbeat automatic `calibration_refresh` reads the MAG and attitude
  parameter groups on both sides through exclusive receive reservations. Its removal/delay is not
  in the approved behavior yet and its causal A/B remains `NOT_RUN`.
- Required next boundary evidence: connect one named Pixhawk USB at a time and read only heartbeat
  sysid/component, `AUTOPILOT_VERSION`, relevant `MAV_2_*`, and `mavlink status` if Stock shell
  returns output. Do not write a parameter/config or claim a runtime mitigation pass until Ethernet
  emission itself is restored.

After the three-cycle, 60-minute soak, isolation, and recovery criteria all pass, add a dated
`Pixhawk.md` record containing:

- the observed pre-change failure and evidence-backed cause classification;
- the exact Workstation mitigation/recovery behavior;
- Stock PX4 version, IP/ports, relevant parameter values, RTCM state, and soak duration;
- before/after packet-rate and duplicate-sequence evidence;
- the fact that the open PX4/NuttX defect is mitigated rather than firmware-patched.

The documentation diff is part of acceptance evidence, not a prerequisite assertion.

## Residual conditions

PX4 issue #25956 remains outside Workstation. Passing the mitigation soak supports operational
fitness but is not proof of a firmware-level fix. `MAV_2_RATE=2000000` remains an explicit follow-up
A/B variable, not silently changed in this slice.

Automated software behavior is green, but the packet cannot be completed: repository portability
has 12 pre-existing failures, actual firmware/sysid/params could not be re-read in the current
ping-alive/MAVLink-silent/no-USB condition, and all Slice 8 hardware gates are NOT_RUN.

## P1 IMU live-delivery recovery — 2026-08-31: CURRENT-RUN PASS, reboot persistence NOT_RUN

- Baseline focused regression:
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 QT_QPA_PLATFORM=offscreen PYTHONPATH=..:../../common/python
  ../../.venv/bin/python -m pytest -q tests/test_mavlink_liveness.py
  tests/test_workstation_pixhawk_gnss.py tests/test_pixhawk_magnetometer_workspace.py`
  → `126 passed in 6.08s`.
- P1 baseline artifact `/tmp/p1-imustream.pcap`: 8 seconds, `ATTITUDE=107`,
  `SCALED_IMU/2/3=0/0/0`, `RAW_IMU=0`.
- P2 comparison artifact `/tmp/p2-imu-baseline.pcap`: 6 seconds,
  `SCALED_IMU/2/3=102/102/102`, proving the common subscriber/parser/UI path works.
- P1-only GUI Disconnect/Connect artifact `/tmp/p1-imu-reconnect.pcap`: 35 seconds,
  no host outbound UDP packet and no `COMMAND_LONG`; P1 continued broadcast telemetry.
- Host evidence: `ip neigh show 10.0.0.2` → `INCOMPLETE`; two pings returned 100% loss.
  P1 wire source was `10.0.0.2` / MAC `1e:1e:b3:a4:ff:b5` to Ethernet broadcast.
- Verified USB runtime recovery sent only three non-persistent shell stream commands to Ethernet
  instance local port `14550`; no reboot, parameter save, config/file write, or firmware action.
  `mavlink status streams` then reported all three `SCALED_IMU*` streams at 20 Hz.
- Independent post-command artifact `/tmp/p1-imu-after-shell.pcap`: 6 seconds,
  `SCALED_IMU/2/3=102/102/102`; packet capture kernel drops `0`.
- Automated-real AT-SPI table readback showed all P1 rows `LIVE` with ages `0.00/0.00/0.05 s`
  and changing xmag/ymag/zmag values. Visual artifact: `/tmp/p1-imu-ui-live.png`.
- Residual: runtime stream enable is lost on PX4 reboot. The next authority boundary is an
  operator-confirmed verified-USB reboot of P1/sysid 1 followed by a fresh Ethernet-only profile,
  packet-rate, and UI readback test. Do not call this persistent recovery before that passes.

### P1 IMU delivery amendment implementation — PASS

- RED: the three new focused cases failed on the absent missing-IMU retry method and absent UI
  freshness threshold. GREEN: all three passed after implementation.
- Subscriber changes track actual `SCALED_IMU/2/3` receive timestamps, retry only missing/stale
  streams after the initial profile, share the side send lock, stop immediately on a real sample,
  enforce three attempts plus ten-second cooldown, suppress duplicate outage logs, clear state on
  stop/reboot, and send nothing during all-RX stall.
- Presentation changes preserve cached values but project `STALE` beyond one second; no sample
  remains `WAITING`, fresh complete values are `LIVE`, and fresh incomplete values are `PARTIAL`.
- Focused MAVLink/Pixhawk/IMU suite: `129 passed in 7.03s`.
- Expanded MAVLink/status/RTCM/NTRIP/lifecycle/contracts suite: `306 passed in 20.47s`. One existing
  NTRIP test emitted a background measurement-writer `StopIteration` warning; no test failed.
- Full Workstation suite:
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 QT_QPA_PLATFORM=offscreen PYTHONPATH=..:../../common/python
  ../../.venv/bin/python -m pytest -q`
  → `1682 passed, 1 skipped in 128.86s`.
- `compileall`, delegate-stub freshness, `git diff --check`, and `./run.sh --check` pass. The
  repository portability checker still reports 13 pre-existing/out-of-scope paths; no changed IMU
  source, test, README, product doc, or packet file is reported.
- Automated-real failure/recovery: runtime-stopping only P1 `SCALED_IMU3` changed its existing UI
  row to `STALE` with retained values and age `12.76s`; restoring it changed the row to `LIVE` at
  `0.04s`. P1 streams 1/2 remained `LIVE` throughout.
- Final new-code process PID `60907` owns UDP `14550/14551`. Independent artifact
  `/tmp/p1-imu-final-new-code.pcap` contains, over six seconds, P1 `SCALED_IMU/2/3=102/102/102`,
  `ATTITUDE=77`, `HEARTBEAT=5`, with zero kernel capture drops.
- Two AT-SPI reads 1.2 seconds apart showed all three P1 values changed and every row stayed
  `LIVE` with `0.01–0.04s` age. Visual evidence is `/tmp/p1-imu-ui-new-code.png`; final live table
  readback after the full suite again showed all three P1 rows `LIVE` at `0.02–0.04s`.
- README and current UI workflow/status/implementation contracts describe sample-backed delivery,
  bounded retry, stale-value truth, no automatic USB command, and no persistent PX4 mutation.

## P1 USB-free reboot persistence — 2026-08-31: FAIL at P1 Ethernet RX boundary

- Precondition: Workstation PID `60907` was sole owner of UDP `14550/14551`; cameras and P1/P2 were
  `LIVE`; Measurement `WAITING`; recording `IDLE`; one USB candidate uniquely revalidated as
  P1 `sysid/component=1/1` at `/dev/ttyACM0`.
- The approved one-shot worker revalidated the same identity immediately before sending only
  `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`. No parameter/config/file/firmware write occurred.
- Independent pcap reset evidence:
  `ATTITUDE.time_boot_ms 3,474,164 -> 3,579`. P1 entered `RECOVERING`; P2 stayed `LIVE`.
- Post-reboot automatic result: P1 heartbeat/ATTITUDE resumed via Ethernet broadcast, but
  `SCALED_IMU/2/3=0/0/0`; host `COMMAND_LONG=0` before ARP bypass because neighbor
  `10.0.0.2` remained `INCOMPLETE`.
- Temporary static-neighbor A/B used captured MAC `1e:1e:b3:a4:ff:b5`. Pcap then showed host
  `255/190` heartbeat, one 15-command Stock profile and bounded three-stream retries sent to
  `10.0.0.2`; P1 returned `COMMAND_ACK=0` and `SCALED_IMU/2/3=0/0/0`.
- USB `mavlink status` readback showed Ethernet instance #1 received only self-originated
  `sysid=1/component=1`, with external `255/190` absent. `mavlink status streams` showed the
  default Ethernet instance lacked all three `SCALED_IMU*` streams.
- `ifdown eth0` succeeded, `ifup eth0` reported failure and P1 became `ETH MAVLINK RX STALLED`;
  a second verified USB reboot restored broadcast telemetry. This interface reset did not repair RX.
- USB runtime stream activation restored P1 Ethernet data. The complete 620-second artifact
  `/tmp/p1-persistent-10m-eth-data.pcap` contains approximately 19.76 Hz per IMU stream; its single
  7.55-second P1 gap is the later operator-approved cable exchange and remained below the 10-second
  all-RX stall threshold. Tcpdump reported zero kernel drops.
- The independent GUI/process monitor reached `450.15s` with cameras/P1/P2 `LIVE`, NTRIP
  intentionally disconnected, failures `0`, and was deliberately interrupted for the cable swap;
  it is not claimed as an uninterrupted 10-minute application soak.
- Cable/port A/B artifact `/tmp/p1-p2-cable-swap.pcap` (180 seconds, zero kernel drops):
  P1 ARP requests/replies `147/0`; P2 `5/5`. Both telemetry links recovered from the intentional
  exchange, but the receive fault followed P1 rather than the physical cable/switch branch.
- Final classification: **USB-free Ethernet recovery FAIL**. P1 telemetry can be received over ETH
  after USB runtime stream activation, but P1 cannot receive host GCS heartbeat, stream commands or
  RTCM over ETH. Repair/replace P1's board-side PHY/driver/NuttX RX path before rerunning the
  USB-free reboot plus uninterrupted 10-minute gate. `Pixhawk.md` remains unchanged.
- Final restored-state artifact `/tmp/p1-final-restored-eth.pcap`: four seconds, P1
  `SCALED_IMU/2/3=61/61/62`, `ATTITUDE=46`, `HEARTBEAT=3`, kernel drops `0`; Workstation PID
  `60907`, P1/P2 and cameras all `LIVE`.

## P1 official stock reflash and ATTITUDE/GNSS soak — 2026-08-31

### Firmware reinstall: PASS

- Source-of-truth documents: `Pixhawk/P1/README.md`, `Pixhawk/P1/manifest.json`, and
  `firmware/stock/MANIFEST.md` selected official PX4 v1.17.0 `px4_fmu-v6x_default`, board ID `53`,
  SHA-256 `8f2c4278...abba0d7f`. The custom SASL CAN-MAG image was not used.
- Preflight: unique USB heartbeat P1 `1/1`, FMUv6X, GUID match, `Disarmed`, failsafe `no`; fresh
  2,393-byte BSON and 58-byte extras backups were saved in `Pixhawk/P1/` with hashes recorded in
  `firmware-reinstall-20260831T1003KST.json`.
- Official PX4 v1.17 `Tools/px_uploader.py` detected bootloader board `53,0`, protocol revision 5,
  STM32H7 rev V and 1,966,080-byte flash. No board force or force-erase was used. Erase, program,
  verify and reboot all reached 100%; uploader exit `0`, elapsed `26.914s`.
- Postflight: same PX4 `d6f12ad...`, sysid `1`, GUID, all required `MAV_2_*`, `10.0.0.2/24`,
  netman, extras and 1,178/1,912 parameter storage readbacks passed.

### USB-free inbound Ethernet: FAIL unchanged

- Artifact `/tmp/p1-post-reflash-eth.pcap`, 142.275 seconds, SHA-256
  `dc5a32e...9050bab`: before USB stream activation P1 sent heartbeat `87`, ATTITUDE `1,299`, and
  `SCALED_IMU/2/3=0/0/0`; P1 ARP requests/replies were `90/0`, P2 `5/5`.
- Reflash therefore excluded a corrupt/unverified installed image but did not restore P1 inbound
  ARP/GCS/RTCM. USB runtime activation restored IMU over ETH (`62/62/62` in the final four-second
  capture), so the outbound data plane remained usable.

### P1/P2 differential diagnosis: physical damage NOT PROVEN

- P1: no PC ARP entry; static insert `ioctl failed: Out of memory`; originated ping `sendto errno
  114`; host replied to P1-originated ARP but P1 did not learn it. Free heap was 498,080 bytes,
  largest block 420,608, DMA 1,024/5,120.
- Official v6X NuttX defconfig at v1.17.0 has `CONFIG_IOB_NBUFFERS=24`. P1 Ethernet MAVLink RX was
  4,313 B/s, reported loss 126.9%, source only self `sysid=1`.
- P2 read through a temporary side-exclusive ETH shell had the same FMUv6X, PX4 git hash,
  `MAV_2_CONFIG=1000`, broadcast `1`, rate `2,000,000`, and same extras pattern. P2 had the PC ARP
  entry, Ethernet RX 38.9 B/s, loss 0.1%, and external GCS `255/190` received.
- This proves a P1-specific board-side boundary but leaves PHY/MAC damage versus NuttX IOB/ARP or
  duplicate-init state unresolved.

### Ten-minute P1 outbound ATTITUDE/GNSS transport: PASS; GNSS solution: NO FIX

- Artifact `/tmp/p1-post-reflash-attitude-gnss-10m.pcap`, 3,186,079 bytes, SHA-256
  `980461f4...3a38fbc`, zero kernel drops and parser exceptions.
- Raw MAVLink2 sequence: 32,034 packets, every delta exactly 1; duplicates/missing/reordered `0/0/0`.
- HEARTBEAT: 619 at 1.000 Hz, maximum gap 1.0037s.
- ATTITUDE: 9,292 at 15.000 Hz, maximum gap 0.0715s.
- GPS_RAW_INT: 3,097 at 5.000 Hz, maximum gap 0.2044s.
- GPS2_RAW: 620 at 1.000 Hz, maximum gap 1.0029s.
- Twenty-one UI checkpoints over 602.93 seconds kept P1/P2 `LIVE`, all-RX stall `0`, P1 displayed
  ATTITUDE age at most 0.06s and GNSS message age at most 0.21s.
- Both GNSS streams consistently carried `fix_type=0`, satellites `0`, lat/lon `0`; message
  transport passed but a valid GNSS position solution did not.
