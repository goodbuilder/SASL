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
