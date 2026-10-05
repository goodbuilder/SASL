# Verification

## 2026-08-31 software implementation evidence

- Settings, transport, liveness, Radio policy, UI, management, MAVFTP and firmware focused suite:
  `224 passed`.
- Offset-aware MAVFTP independent fake-server tests cover raw request headers, restart offsets,
  exact prefix resume, mismatch rejection, download partial recovery, remote size/CRC32 and atomic
  evidence.
- Firmware tests verify the repository stock bundle, exact uploader SHA-256
  `5cae28e8ebb0cdb1c25035034afdb44b2fc09f3893c32d54f2a31e96e0ac144c`, no force flags,
  direct-USB sysid 3 pre/post identity and board 53 phase parsing through a fake process boundary.
- Full Workstation suite excluding one reproducible unrelated NVIDIA/GStreamer recorder failure:
  `1780 passed, 1 skipped, 1 deselected`. The excluded test
  `test_decoded_nv12_recorder_finalizes_synthetic_lossless_mp4` rejects its second synthetic frame
  in the current GPU environment both in-suite and alone; no Pixhawk code participates in it.
- Hardware acceptance remains `NOT_RUN`: no third Drone ground/air pair is available in the recorded
  inventory. Existing P1/P2 20-second NETID isolation evidence is retained but is not a substitute
  for the approved three-pair soak.

Read-only hardware inventory on 2026-08-31 found one FT231X ground serial endpoint with stable serial
`DU0E49VU` and no process owner. A 57600-baud eight-second passive probe received zero bytes and sent
no AT/MAVLink/configuration traffic. Product model, configured UART rate, air rate, peer identity,
and the second independent radio remain unverified.

Later dual-radio read-only evidence found `DU0E49VU` and `D30JNMVH`. At 57600 baud, ten-second
passive probes produced `0` messages on the former and `141` on the latter; the latter contained
both P1 sysid 1 and P2 sysid 2. Direct USB independently confirmed both powered boards and their
identities. Independent-pair acceptance therefore FAILS for the current RF configuration. No AT,
configuration, parameter, or MAVLink command was sent.

If the FT231X radios expose SiK AT commands, isolation qualification must first capture read-only
local/remote firmware and `ATI5` parameters. Each pair must match internally on firmware, AIR_SPEED,
frequency range, channel count, NETID, ECC, LBT, and MAX_WINDOW; P1/P2 pairs require distinct NETIDs
and must pass a post-change cross-sysid isolation capture. No AT write is authorized yet.

Read-only AT inventory PASS: both ground radios reported local SiK parameters with NETID 25;
`D30JNMVH` reported RFD SiK 2.0 and one remote with the same NETID/radio profile, while `DU0E49VU`
had no remote response. No `ATS/RTS`, `AT&W`, `ATZ`, reset, or persistent command was sent. `ATO`
recovery was independently confirmed by renewed passive MAVLink on `D30JNMVH`.

P1-only pair mapping PASS: with P2 off, `D30JNMVH` received 169 messages including 8 heartbeat from
P1 sysid 1 over eight seconds; `DU0E49VU` received zero. No radio or Pixhawk command was sent.

P2-only pair mapping PASS: with P1 off, `D30JNMVH` received 154 messages including 8 heartbeat from
P2 sysid 2 over eight seconds; `DU0E49VU` received zero. This proves both air radios currently use
the D30JNMVH/NETID-25 network rather than independent ground pairs.

P2 NETID split WRITE/READBACK PASS but RF LINK FAIL: approved remote/local NETID 25→26 commands,
readbacks, EEPROM save acknowledgements, and reboot commands succeeded; DU local persistence was
independently re-read as 26. P2 USB stayed live, yet 10-second and 20-second passive checks saw zero
messages on both ground radios. Further writes stopped pending a physical P2 air-radio power cycle.

P2 post-power-cycle RF isolation PASS: over 15 seconds `DU0E49VU` received 298 messages including
15 heartbeat from only sysid 2, `D30JNMVH` received zero, and direct P2 USB stayed live. This proves
the saved NETID-26 pair requires/accepted the full air-radio power reapply. Dual-board simultaneous
cross-isolation remains NOT_RUN.

Dual-board simultaneous isolation PASS: over approximately 20 seconds DU/NETID26 received 393
messages from only sysid 2 and D30/NETID25 received 400 messages from only sysid 1. Heartbeat counts
were 19/20 and maximum gaps 1.932/1.124 seconds; cross-sysid count was zero on both. This is a short
transport/isolation pass, not the later RTCM/load/long-duration acceptance soak.

Required evidence will include fake serial/UDP boundary tests, expected/wrong/duplicate sysid,
device busy/disconnect/reconnect, per-side RTCM isolation, low-bandwidth saturation, settings
migration/restart, shutdown, UI stability, and real independent-radio tests with RTCM age and
RTK fix evidence.

Low-bandwidth tests must prove critical RTCM/heartbeat/ATTITUDE/GNSS continuity, bounded stale RTCM
drops, visible optional-stream reduction, side isolation, and absence of persistent PX4 writes.
Timing tests must prove the exact 10-second transport qualification, 30-second initial diagnostic
recovery, 15-second inter-tier dwell, timer reset on regression, and independence from RTK readiness.
Removal tests must prove immediate side-only RTCM revocation, no automatic reacquire on same/different
device return, explicit Connect requirement, generation isolation, and unaffected opposite side.
Migration tests must cover old settings, repeated load/idempotence, missing Radio devices, preserved
Ethernet values, explicit Ethernet reselection, restart, and rollback-compatible serialization.
Discovery tests must permute USB order/path, use identical serial descriptors, exercise expected,
wrong, duplicate, and changing sysids, and prove no candidate can be owned by both sides.
Connection tests must prove zero automatic parameter/calibration/shell/persistent/FTP traffic for
both Ethernet and Radio, while explicit manual commands still own their bounded lifecycle.
Bulk-transfer tests must cover progress, cancellation, resume/retry policy, timeout, partial file,
hash mismatch, disconnect, wrong generation, opposite-side isolation, exact artifact readback, and
the absence of RTCM-specific warning/blocking. When RTCM is active they must assert exactly one
informational start event with side/transport/category/time/age and no focus or command delay.
Firmware tests must distinguish staged image evidence from independently proven installation/boot
identity.
USB-flash integration must reject wrong/ambiguous board identity, preserve staged evidence across
the handoff, and require erase/program/verify plus post-reboot firmware identity before success.
Transfer recovery tests must cover clean resume, restart, missing partial, changed remote bytes,
wrong side/generation, disconnect during verification, cancel, and final full-file hash mismatch.
Concurrency tests must cover simultaneous P1/P2 transfer, same-side duplicate rejection, one-side
disconnect/cancel isolation, RTCM independence, and global rejection of a second USB flash.

## 2026-09-01 Discovery follow-up matrix

- USB start clears stale network/PC/MAVLink rows, selection, feedback, and status while preserving
  saved assignments/settings: `PASS` (real panel/Qt with fake serial boundary).
- Network start clears stale USB/PC/network rows and shows only the new network result: `PASS`
  (real panel/worker with fake network inventory).
- Empty and failed current discovery never resurrect stale rows: `PASS`.
- A duplicate click rejected during the same active worker does not clear the in-progress snapshot:
  `PASS`.
- Two physical serial devices with one heartbeat produce two rows, with the heartbeat row labelled
  by observed sysid rather than a duplicate endpoint: `PASS`.
- Table headings, per-board Radio labels, tooltips, selection reset, and stable geometry are covered
  by focused Qt tests and an offscreen current-code render: `PASS`; rendered candidate group was
  1292×408 px and transport group was 1292×158 px without clipping or control movement.

Commands and results:

- Focused Discovery/UI/settings:
  `QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q tests/test_pixhawk_network_assignment.py tests/test_workstation_pixhawk_gnss.py::test_camera_and_pixhawk_discovery_candidates_are_visible_and_assignable tests/test_workstation_settings.py::test_workstation_exposes_complete_system_settings_surface`
  -> `23 passed in 3.00s`.
- Direct Pixhawk/Radio/status/command/settings/lifecycle regression -> `256 passed in 42.04s`.
  Its first run had one transient phase-one command enablement failure; the same test then passed
  alone and the identical 256-test command passed on rerun. No Discovery assertion failed.
- Full Workstation suite from `apps/Workstation`:
  `QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q`
  -> `1811 passed, 1 skipped in 151.52s`.
- Runtime boundary: `./run.sh --check` -> exit 0, all declared runtime checks PASS, selected/effective
  decoder `auto -> nvh264dec`.
- Visual artifacts: `/tmp/pixhawk-discovery-after.png` and
  `/tmp/pixhawk-transports-after.png`; automated-fake/offscreen, not hardware evidence.

No post-change serial device was opened for this UI/state correction. The existing real FT231X
inventory remains baseline evidence only; hardware discovery rendering after restarting the field
application is unverified and does not weaken the automated one-device/one-row contract.

## Planned 2026-09-02 guided-topology amendment matrix

- Pure contract/journal: old/no record, every forward/partial/terminal stage, malformed record,
  generation mismatch, idempotent same-evidence update, crash between device readback and journal
  write, restart re-probe, Resume boundary, inverse rollback plan.
- SiK boundary: independently built ASCII fixtures for ATI/ATI5/RTI/RTI5, exact `RTS3=26`, query,
  save and ATO order; no write on wrong/multiple/no sysid, malformed/partial response, timeout,
  disconnect, busy port, cancellation, stale result, or failed readback.
- PX4 boundary: unique direct-USB sysid2 and UID, disarm, current profile, targeted command/parameter
  ACK/readback, extras evidence, reboot/reappearance, carrier/route/UDP expected-system evidence;
  reject wrong/duplicate/armed/busy/stale identity and any shared-RF shell path.
- State/settings: preserve pre-operation settings until both device postconditions, atomic final
  commit, failed settings write restoration, old settings compatibility, no startup acquisition,
  explicit Connect only, P2 failure isolation.
- UI: fixed group/row/column geometry for IDLE through long PARTIAL errors, correct current-action
  enablement, persistent-write confirmation, focus/selection/scroll preservation, no moving
  transport/Connect controls, manual controls preserved outside active operation.
- Lifecycle/integration: duplicate clicks, Check/worker concurrency, Cancel before/after write,
  close during every active phase, late callback generation rejection, restart Resume/rollback,
  opposite-side continuity, full forward and inverse fake-device flows.
- Regression commands: new focused module; existing Pixhawk network/settings/GNSS/status/UI/
  lifecycle suites; full `QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q`;
  `./run.sh --check`; offscreen screenshots and dynamic-state sequence.
- Hardware/manual: forward configuration and rollback with current D30/DU/P1/P2/Drone inventory,
  then telemetry-only qualification followed by RTCM age, queue/drop, RTK fix, cross-identity and
  long-duration evidence. Status remains NOT_RUN until executed on the real rig.

## 2026-09-02 guided-topology software result

- Pure journal/state, SiK boundary, P2 USB/profile/UDP boundary, controller and fixed UI focused
  suite: `53 passed in 2.12s`.
- Affected Pixhawk network/settings/contracts/status/lifecycle/GNSS/topology regression:
  `256 passed in 29.46s`.
- The P2 mutation boundary is two-phase. The first action captures UID, every owned parameter, and
  exact `extras.txt`/`net.cfg` presence/bytes and atomically journals them. A separately confirmed
  action re-probes 20-second Radio absence, unique disarmed USB identity and UID before writing.
  Missing durable backup evidence prevents the command. Post-reboot verification checks exact
  profile readback, current sysid2 HEARTBEAT+ATTITUDE, a quiet unexpected-sysid window, and a fresh
  final cross-radio/UDP pre-commit probe.
- The Drone SiK fixture proves local/remote NETID preconditions and the exact
  `RTS3=26 -> RTS3? -> RT&W -> RTS3? -> ATO` order. Empty/error ATO evidence, external ownership,
  wrong/multiple identities, an armed board, changed isolation, cancellation, or missing readback
  fails closed. Persistent-write intent must be durably recorded before the worker starts.
- Offscreen render inspection PASS at 1920x1080: fixed group 1292x316 px, table 1274x168 px, five
  rows, all three manual transport/Connect rows and four topology actions visible. At 1600 width
  with both documented side docks open, the existing central workspace uses horizontal scrolling;
  collapsing a dock remains the documented narrow-screen behavior. A separate PARTIAL render kept
  the same 316 px group height, selected and locked the preset/manual mutation controls, retained
  Disconnect semantics, and enabled `Check / Resume`, Cancel and Guided rollback without reflow.
- Runtime preflight: `./run.sh --check` exited 0. Required Python/GStreamer/FFmpeg/runtime checks
  passed. The optional `nvh264dec` element was unavailable, so decoder selection truthfully fell
  back to `avdec_h264`; this does not affect Pixhawk topology.
- Earlier unfiltered full Workstation pytest: `1854 passed, 1 skipped, 1 failed in 180.73s`. The
  pre-existing, unrelated
  `test_decoded_nv12_recorder_finalizes_synthetic_lossless_mp4`. It failed again alone, and direct
  FFmpeg reproduction failed before encoding with `cuInit(0) -> CUDA_ERROR_UNKNOWN`, while
  `nvidia-smi` and encoder enumeration remained available. No topology source participates in that
  path. The full suite is therefore not reported as green.
- Final-code rerun with only that reproduced environment-owned test explicitly deselected:
  `1867 passed, 1 skipped, 1 deselected in 187.32s`. This includes every Pixhawk topology,
  lifecycle, settings, network, UI and device-boundary test; it is reported as a qualified pass,
  not as an unqualified full-suite pass.
- Real D30/DU/P1/P2/Drone forward setup, explicit Connect/qualification, restart/Resume, Guided
  rollback, RTCM/RTK, and soak remain `NOT_RUN`. No serial, MAVLink, SiK, PX4, or hardware mutation
  was executed by this software-verification phase.

## 2026-09-02 three-radio hardware qualification (terminal path, not the guided UI)

This section records real device evidence obtained outside the guided surface. It does not satisfy
the UI-driven acceptance above, which stays `NOT_RUN`.

Read-only diagnosis PASS. `D30JNMVH` and `DU0E4O0F` both reported NETID 25 and a strong symmetric
peer (RSSI ~213/215, rxe +1..2 per 20 s) while delivering zero MAVLink bytes at 57600, 115200,
38400 and 9600. `RTI7` from each returned the other's local counters (`temp=24 rxe=199` against
`temp=37 rxe=1968` and the reverse), proving the two ground radios had paired with each other.
`DU0E49VU` read NETID 26 with RSSI 164/164 and carried P2 sysid 2, so the earlier P2 25→26 split is
now RF-live. Direct USB showed `MAV_SYS_ID` 1, 2 and 3 on three v6X boards, all with
`MAV_0_CONFIG=101` and `SER_TEL1_BAUD=57600`. The Drone USB endpoint additionally carried sysid 1
telemetry and a `sys 51 comp 68` RADIO_STATUS (`rssi=185 remrssi=183`), proving the two air radios
had paired with each other. Four radios on NETID 25 had therefore formed a 2+2 mis-pairing.

Drone pair NETID 25→27 WRITE/READBACK PASS. With `D30JNMVH` detached and P1's air radio unpowered,
`DU0E4O0F` observed exactly sysid 3 (HEARTBEAT 8, ATTITUDE 35, GPS_RAW_INT 11 in 8 s), no armed
board, and local/remote NETID 25. `RTS3=27` → `OK`, pre-save readback `27`, `RT&W` → `OK`,
post-save readback `27`; then `ATS3=27` → `OK`, readback `27`, `AT&W` → `OK`, `ATZ`. The ground
radio returned on NETID 27 with remote RSSI 0 until the Drone air radio received a full power
reapply, which is the expected saved-but-not-running state.

Three-link qualification PASS. After the Drone power reapply and P1 air-radio restore, a 12-second
passive capture per endpoint gave `D30JNMVH`/NETID25 sysid 1, `DU0E49VU`/NETID26 sysid 2 and
`DU0E4O0F`/NETID27 sysid 3, each with HEARTBEAT 11-12, ATTITUDE 48-52 and GPS_RAW_INT 14-18, no
foreign sysid on any endpoint and no armed board. Post-link counters were RSSI 187/186 rxe 153 on
`D30JNMVH` and RSSI 154/152 rxe 6 on `DU0E4O0F`.

Settings now select Telemetry Radio for all three boards with `D30JNMVH`, `DU0E49VU` and
`DU0E4O0F` at 57600, so the application is configured for this topology.

Not covered: RTCM forwarding, RTK fix quality, soak, restart/Resume, Guided rollback, and any
Workstation-owned Connect. All evidence above is passive terminal capture, not an application
connection.

## 2026-09-03 guided-topology removal verification

- Source reference scan confirms no current Python code or test imports the withdrawn topology
  controller, worker, core contract, journal path, command IDs, UI group, or lock state.
- Focused preserved Pixhawk/UI/lifecycle/USB/NIC/contract tests: `217 passed in 27.22s`.
- Full Workstation suite: `1893 passed, 1 skipped in 150.38s`; one existing non-fatal
  `StopIteration` thread warning was emitted by the NTRIP test's monkeypatched clock.
- Launcher check: `./run.sh --check` exited `0`; all declared runtime checks passed.
- Offscreen UI regression confirms the topology group and `pixhawk.topology.*` commands are absent,
  while the Board transport/Connection/Discovery/ETH MAV controls remain present.
- The machine-local `pixhawk_topology_operation.json` journal was intentionally preserved as old
  user evidence; removed code no longer loads or writes it. Hardware connectivity is unchanged and
  is not claimed by this source change.
- Repository-root `pytest` was attempted as required by the generic test procedure, but collection
  stops before tests because `apps/StereoWIRIS/tests/test_px4_attitude_cal_manager.py` and
  `apps/Workstation/tests/test_px4_attitude_cal_manager.py` share the same imported module name.
  This pre-existing cross-app collection collision is unrelated to the removal; the documented
  Workstation suite above is the affected regression boundary.

One operator-visible interruption occurred during read-only diagnosis: a `+++` probe left
`DU0E49VU` in AT command mode and P2 telemetry stopped until `ATO` restored it. Passive MAVLink
resumed immediately and cleanly (`bad_data 0`). Entering AT mode on a live link should be avoided
unless the endpoint is known to be idle.

## 2026-09-04 P1/P2 IMU observation

- RTCM stop plus P1/P2 reconnect: operator-reported IMU recovery `FAIL`; this is a field
  observation, not a new packet capture.
- Saved settings readback: `left/right_pixhawk_transport=ethernet`, with Radio endpoints populated
  as `D30JNMVH` for P1 and `DU0E49VU` for P2. `PASS` as a configuration observation.
- Application log readback: P1/P2 Connect attempts ended in `ROUTE MISMATCH`; Radio Connect and
  Radio IMU request evidence are absent. `PASS` as evidence that the tested path was Ethernet,
  `NOT_RUN` for Telemetry-only IMU acceptance.
- Required next boundary: save/select Radio, explicitly Connect and qualify expected sysid, then
  observe `SCALED_IMU/2/3` requests and samples while RTCM remains the higher-priority path.
