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
