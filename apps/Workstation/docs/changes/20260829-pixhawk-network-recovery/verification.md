# Verification

## Requirement-to-evidence matrix

| Requirement | Evidence | Status |
|---|---|---|
| read-only physical NIC/address/route discovery | pure JSON/route tests, real host inventory | PASS |
| Assign P1/P2 persists same/different NIC intent | controller + real-panel restart tests | PASS |
| Connect fails invalid side before socket and keeps valid side eligible | route-preflight partition tests | PASS |
| LIVE-side reassign disconnects only that side | controller regression | PASS |
| legacy settings remain usable | additive default/round-trip tests | PASS |
| Camera/Pixhawk candidate separation and same-NIC no-warning policy | Workstation UI contract tests | PASS |
| candidate-only discovery rows and independent P1/P2 assignment cards | UI/model regression + visual capture | PASS |
| target-hardware P1/P2 heartbeat and 60-second LIVE | requires new app restart and valid existing route | NOT RUN |

## Network-role application amendment matrix (pre-implementation)

| Requirement | Required evidence | Status |
|---|---|---|
| explicit confirmation; Find/startup never mutate | UI/controller negative tests and command capture | PASS automated-fake |
| one-/many-NIC role union preserves unrelated entries | independent NM fixtures + raw delta assertions | PASS automated-fake + real readback |
| privilege denial/timeout/partial write/readback mismatch roll back | adapter/controller failure matrix | PASS automated-fake |
| target verified before role-scoped source cleanup; persistence last | ordered call/readback/state tests | PASS automated-fake |
| active affected owners auto stop/reconnect on selected NIC | Camera-generation and expected-sysid HB/ATT integration tests | PASS simulated; hardware NOT RUN |
| unrelated NIC owners never restart | cross-feature isolation tests and hardware observation | PASS simulated; hardware NOT RUN |
| Measurement/recording/save/finalization rejects before mutation | mapping-lock and zero-command assertions | PASS automated-fake |
| restart/close/interruption reports or restores exact host state | persistence/shutdown/recovery matrix | PASS automated; mutating hardware NOT RUN |
| compact UI remains stable through all phases and long errors | target-size/font/focus/selection/scroll captures | PASS visual-manual |
| real one-NIC combined and multi-NIC move/rollback | NetworkManager + packet/frame/device evidence | NOT RUN |

## Network-role amendment evidence

Focused role command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 QT_QPA_PLATFORM=offscreen \
  ../../.venv/bin/python -m pytest -q tests/test_network_role_application.py -x
```

Result: **22 passed**. Coverage includes role union/deduplication, shared-NIC movement, invalid
input, exact provenance/adoption, unrelated-value preservation, target-first order, reverse
rollback, new/existing/empty profile commands, settings round-trip, mapping lock, affected-owner
isolation, worker errors, commit-after-device-evidence and composed-role UI projection.

Affected-workflow result: **274 passed in 35.07s** across network role, Pixhawk assignment/settings,
Camera/Pixhawk UI, Camera, contracts and lifecycle suites.

Final full command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 QT_QPA_PLATFORM=offscreen \
  ../../.venv/bin/python -m pytest -q
```

Result: **1611 passed, 2 skipped in 158.70s**. The only warnings are pytest cache writes blocked at
the read-only repository-root `.pytest_cache`; test behavior and workspace files are unaffected.

`compileall`, delegate-stub freshness, `git diff --check`, and `./run.sh --check` pass. Runtime doctor
selected actual `NVIDIA GeForce RTX 4050 Laptop GPU / nvh264dec` with no missing GStreamer elements.

An escalated **read-only** real NetworkManager snapshot of active `pixhawk-dds` proved the planner
preserves unrelated `10.41.10.1/24`, `192.168.0.254/24`, Camera `.11/32` and `.231/.232` routes while
adding only Pixhawk `.10/24` and `.2/.3` role entries. No host setting was changed.

Visual-manual captures `/tmp/network_roles_1366.png` and `/tmp/network_roles_1600.png` show composed
`Camera,P1`/`P2` assignments, disabled Assign controls during the operation, and fixed-position
`NETWORK ROLE RECONNECTING`. Long values use the existing elide+tooltip/Inspector policy; table
selection, scroll, docks and adjacent controls remain stable.

## Focused test evidence

Baseline before source edit: `133 passed in 32.97s` for Pixhawk/contracts/settings tests.

Final focused command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 QT_QPA_PLATFORM=offscreen \
  ../../.venv/bin/python -m pytest -q \
  tests/test_pixhawk_network_assignment.py \
  tests/test_workstation_pixhawk_gnss.py \
  tests/test_pixhawk_magnetometer_workspace.py \
  tests/test_rtcm_pipeline_workers.py \
  tests/test_workstation_settings.py \
  tests/test_workstation_contracts.py \
  tests/test_workstation_lifecycle.py -x
```

Result after the assignment-card amendment: **221 passed in 25.49s**. The new dedicated file contributes 13 passing tests covering
normal, missing/mismatched route, read-only command boundary, persistence rollback/restart,
same-NIC assignment, settings invalidation, live-side reassignment and real-panel worker lifecycle.

## Regression and application evidence

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 QT_QPA_PLATFORM=offscreen \
  ../../.venv/bin/python -m pytest -q \
  -k 'not test_decoded_nv12_recorder_finalizes_synthetic_lossless_mp4'
```

Fresh full result after the assignment-card amendment: **1583 passed, 2 skipped in 158.40s**.
A preceding non-fresh full run emitted one transient failure around 13% and later stalled, so it was
interrupted; the immediate `-x` rerun completed cleanly with the result above.

`./run.sh --check`, compileall, delegate-stub freshness and `git diff --check` pass. Runtime doctor
truthfully warns that `nvh264dec` is unavailable and selects software decode.

## Performance, load, and target-hardware evidence

The bounded discovery worker completes without GUI-thread I/O. Actual host discovery returned three
IPv4 choices on `enx588694fce07e`; independent kernel readback classified both P1/P2 routes as
`ROUTE MISMATCH` because they currently select Wi-Fi. This is the intended truthful result and no
host network setting was changed.

The synthetic NVENC test is deselected only because NVIDIA is in power-saving state. Independent
FFmpeg returned `cuInit(0) -> CUDA_ERROR_UNKNOWN`; the user confirmed a reboot will restore it.
This is unrelated to the NIC slice and must be rerun after reboot.

## Feature isolation and contract evidence

Camera discovery remains unchanged. Pixhawk owns only its candidate snapshot/controller/worker;
the existing single MAVLink subscriber remains the socket owner. Expected sysid, 60-second
qualification, stalled-side RTCM blocking and verified-USB recovery regressions pass.

## Visual authenticity and dynamic-layout stability evidence

Offscreen 1366x768, 1600x900 and 1920x1080 captures place the bounded candidate table directly below
Connection, retain the compact native Qt language, expose horizontal scroll/tooltips for long
routes and keep assignment values visible. A 1.25x font-scale UI test passes. Candidate selection
survives assignment-driven refresh and the table is not rebuilt by unrelated liveness ticks.

The amended capture `/tmp/pixhawk_network_assignment_cards_20260829.png` shows a five-column
candidate-only table (`Source/NIC/Link/Local IPv4/Assigned`) and independent P1/P2 cards containing
only that side's State/NIC/Local IPv4/Endpoint/Route. The two-row card layout removes the overlap
seen in the first vertical-stack probe.

## Visual and hardware/manual evidence

- visual-manual: PASS at the three target sizes; temp captures
  `/tmp/pixhawk_network_after_1366_v2.png`, `/tmp/pixhawk_network_after_1600x900.png`, and
  `/tmp/pixhawk_network_after_1920x1080.png`;
- automated-real host boundary: PASS for NIC/address/route readback and truthful mismatch;
- hardware-manual MAVLink: NOT RUN because the currently running pre-change app owns UDP ports and
  the host has no valid `.2/.3` Pixhawk route.

## Residual conditions

1. Restart Workstation from the new source after the planned reboot, establish the intended existing
   host routes, then run Find -> Assign P1/P2 -> Connect and observe expected sysid plus 60-second LIVE.
2. Alternate P1/P2 NIC and shared Camera/Pixhawk topology stability remain hardware-manual NOT RUN;
   the UI intentionally does not warn or gate shared-NIC selection.
3. Repository portability remains blocked by one pre-existing out-of-scope dated checkout string in
   `../../docs/changes/20260828-yolo-tolerant-input-trial/current-behavior.md:5`; no new packet file
   is reported.
4. The new privileged NetworkManager mutation was not exercised against the currently operating
   field network. Run confirmed Assign Camera/P1/P2 on representative one-NIC and multi-NIC wiring,
   observe actual profile/kernel readback plus Camera frame/Pixhawk expected HB+ATT, and force one
   target/reconnect failure to prove real rollback before closing this amendment.

## 2026-08-29 real split-NIC field evidence

- automated-real host/device: OWC power-cycle enumerated RTL8156B as
  `enx0023a40a2e39` (`00:23:a4:0a:2e:39`); the original Camera NIC remained
  `enx588694fce07e` (`58:86:94:fc:e0:7e`).
- hardware-real failure: assigning while P2 emitted no Ethernet traffic entered reconnect, timed out
  on whole-affected-owner device evidence and ended `ROLLBACK PARTIAL`. Assign P1/P2 were correctly
  locked while the transaction was active, but the completion boundary blocked the desired offline
  assignment.
- automated-real host mutation/readback: Camera role retained `.11/32` plus `.231/.232` routes on the
  original NIC; P1/P2 role retained `.10/24` plus `.2/.3` preferred-source routes on the RTL8156B.
  `ip route get` selected the intended device and source for all four peers.
- hardware-real MAVLink: raw capture on the RTL8156B observed both
  `10.0.0.2:14550 -> 10.0.0.255:14550` and
  `10.0.0.3:14551 -> 10.0.0.255:14551`. Workstation independently reported P1 sysid 1 and P2 sysid
  2 with fresh HEARTBEAT and ATTITUDE, both `LIVE`.
- hardware-real USB-free recovery: removing `/dev/ttyACM0` caused P2 to reboot and visibly re-enter
  qualification; after 60 continuous seconds it returned to `LIVE`. A later three-second raw capture
  with USB absent counted 120 P1 and 122 P2 datagrams. The Workstation run subsequently exited
  cleanly with exit code 0.
- verdict: final split topology and USB-free P1/P2 operation PASS. Automatic reconnect/rollback when
  another affected role is offline FAILS the newly discovered operator requirement and reopens the
  policy contract.
