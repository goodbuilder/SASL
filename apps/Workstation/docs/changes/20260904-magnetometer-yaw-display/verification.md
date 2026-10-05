# Verification

## Requirement-to-evidence matrix

| Requirement | Evidence planned | Status |
|---|---|---|
| RAW yaw is finite, wrapped, and sensor-axis labelled | Pure tests + table/Inspector test | PASS |
| PX4 yaw appears only for exact active identity and fresh ATTITUDE | Evidence tests | PASS |
| Missing/stale/near-zero/fallback states are explicit | Negative evidence/UI tests | PASS |
| Both tables retain rows/selection/geometry under refresh | Qt UI tests | PASS |
| Existing priority/telemetry/calibration paths remain unchanged | Dependent regression | PASS for changed path; suite residual below |
| Real priority A/B estimator response | Hardware/manual procedure | UNVERIFIED |

## Focused test evidence

PASS — `../../.venv/bin/pytest tests/test_pixhawk_magnetometer_workspace.py -q -k
'imu_magnetometer_view or pixhawk_workspace_renders_p1_p2_imu_magnetometer_table or
debug_vect_decoder or can_magnetometer_evidence_correlates_only_exact_active_device_with_px4_yaw'`.

Result: 4 passed, 11 deselected. This covers the consolidated five-row-per-side IMU view,
`mavlink/DEBUG_VECT` topic/unit preservation, RM3100 RAW/PX4/Δyaw rendering, Inspector selection,
and stable-firmware `CAN1_MAG` identity decoding.

PASS — `../../.venv/bin/python -m pytest tests/test_magnetometer_yaw.py tests/test_pixhawk_magnetometer_workspace.py -q -k 'magnetometer_yaw or mavlink_magnetometer_evidence_keeps_three_imu_streams_and_axes or pixhawk_workspace_renders_p1_p2_imu_magnetometer_table or can_magnetometer_evidence_correlates_only_exact_active_device_with_px4_yaw or can_magnetometer_evidence_requires_dedicated_message_and_times_out or dedicated_can_mag_sample_wins_over_legacy_scaled_imu_regardless_of_arrival_order'`

Result: 12 passed, 8 deselected.

PASS — `../../.venv/bin/python -m py_compile workstation_core/magnetometer_yaw.py evidence.py workspaces.py tests/test_magnetometer_yaw.py tests/test_pixhawk_magnetometer_workspace.py`.

PASS — `git diff --check` for changed source, tests, product docs, and this packet.

## Regression and application evidence

The directly dependent command
`../../.venv/bin/python -m pytest tests/test_workstation_pixhawk_gnss.py tests/test_status_model_pixhawk.py tests/test_workstation_ui.py tests/test_baseline_yaw_alignment.py tests/test_baseline_yaw_alignment_ui.py -q`
resulted in 153 passed and 3 failures. The failures are outside this packet: camera discovery
feedback text, an existing Pixhawk layout expectation, and an existing recovery-contract text
expectation. The new Magn test path passed within the same run.

The full command `../../.venv/bin/pytest -q` resulted in 1,902 passed, 2 skipped, and 29 failed.
The failures are existing dirty-worktree mismatches in calibration capture, time sync, measurement
package/settings, native-frame cleanup, network role/USB/telemetry, optical-pose Import, the two
pre-existing MAG stale/fallback expectations, Pixhawk recovery/layout, and related Workstation
surfaces. None of the failures names the new consolidated RM3100/DEBUG_VECT assertions. Exact
reproduction command is the full command above.

PASS — `./run.sh --check`. Canonical interpreter, imports, GStreamer elements, NVIDIA/CUDA gate,
and `auto -> nvh264dec` selection all passed.

## Performance, load, and target-hardware evidence

The existing per-camera Shell worker is reused at a 2-second cadence with a 4-second timeout and
manual-command exclusion. Record UI refresh/geometry evidence; do not claim target hardware or
PX4 estimator behavior from fakes.

## Feature isolation and contract evidence

PASS by source review and focused tests: no priority write, private stream request, or firmware
change was added; RM3100 keeps the existing DEBUG_VECT path and now has a separate Stable PX4 Shell
readback path with explicit provenance. Raw frame/unit contract remains, and unrelated
Measurement/Recording/Alignment paths are untouched.

## Visual authenticity and dynamic-layout stability evidence

PASS by Qt table/Inspector assertions and fixed header/scroll policies: three SCALED_IMU rows plus
two Shell/DEBUG_VECT RM3100 rows per Pixhawk render, RAW/PX4/delta cells remain bounded, and row
updates use the existing in-place model.
Full screenshot/manual DPI evidence remains unverified.

## Visual and hardware/manual evidence

Software visual evidence is automated/simulated. A real P1/P2 priority A/B run must still confirm
the active device ID, raw vector, fallback behavior, and resulting PX4 yaw on the rig.

## Residual conditions

Real PX4 fusion selection, fallback, calibration, and tilt response remain hardware/manual
conditions. An unmatched standard SCALED_IMU stream intentionally has no corrected cell. The
broader project regression remains non-green because of the pre-existing failures listed above.

## Stable PX4 Shell RM3100 amendment verification — 2026-09-05

| Slice | Evidence | Status |
|---|---|---|
| Shell decoder | `parse_px4_sensor_mag_shell_output()` keeps only known RM3100 IDs and preserves a `px4-shell/sensor_mag/<instance>` topic; `parse_px4_vehicle_magnetometer_shell_device_id()` reads the selected device ID. | PASS |
| Boundary command | `DeviceRuntimeController._px4_sensor_mag_shell_command()` requests `sensor_mag` instances 0–4 plus `vehicle_magnetometer`; unit test proves all six commands are present. | PASS |
| State handoff | Shell RM3100 samples route to `latest_can_mag_sensors`; active device ID updates the existing yaw gate; manual Shell/PARAM commands block the same side's poll. | PASS (fake boundary) |
| Status semantics | `RM3100 SHELL POLL` is projected as `SHELL SYNC`, while explicit PARAM reservations remain `PARAM SYNC`. | PASS |
| Evidence/UI | Five rows per Pixhawk, Shell topic/source preservation, RAW yaw, exact active-device PX4 yaw, Δyaw, Inspector, and fixed geometry are covered by Qt/evidence tests. | PASS (simulated) |
| Failure/recovery | Poll timeout/failure is throttled, last samples age into stale/disconnected states, and reconnect starts polling again. | PASS (logic/source review; live timeout unverified) |
| Stable launcher | `./run.sh --check` completed all runtime/GStreamer/NVIDIA gates. | PASS |
| `--start-pixhawk` workflow | Qt test proves the flag selects the Pixhawk page and invokes the real guarded Connect owner, which is the prerequisite for Shell polling. | PASS (simulated) |
| Real P1/P2 Shell response | Live Radio run with `./run.sh --no-start --start-pixhawk` opened the configured P1/P2 serial links, polled all five `sensor_mag` instances plus `vehicle_magnetometer`, and populated the UI model with five RAW-yaw cells per Pixhawk. | PASS |

Focused commands:

- `../../.venv/bin/python -m pytest tests/test_pixhawk_magnetometer_workspace.py tests/test_magnetometer_yaw.py -q` → **30 passed**.
- `../../.venv/bin/python -m pytest tests/test_workstation_pixhawk_gnss.py -q -k 'shell or serial_control'` → **5 passed, 93 deselected**.
- `../../.venv/bin/pytest tests/test_status_model_pixhawk.py -q` → **6 passed**.
- `../../.venv/bin/python -m py_compile workstation_core/pixhawk_attitude.py sensor_presentation_controller.py evidence.py device_runtime_controller.py standalone_gui.py tests/test_pixhawk_magnetometer_workspace.py` → **PASS**.
- `git diff --check` on the amended source/tests/docs → **PASS**.

The dependent command `tests/test_workstation_pixhawk_gnss.py tests/test_status_model_pixhawk.py
tests/test_workstation_ui.py tests/test_workstation_telemetry.py -q` returned 137 passed and 7
pre-existing dirty-worktree failures. The failures concern discovery feedback, Pixhawk layout/
recovery wording, and TelemetryWorkspace baseline/dirty-url APIs; none names the new Shell polling,
RM3100 routing, or five-row yaw assertions.

## Live Radio runtime evidence — 2026-09-05

PASS — Started the actual Workstation with `./run.sh --no-start --start-pixhawk` while P1/P2 were
connected through Radio-backed serial links. The live application log records the three configured
MAVLink links, then six separate Shell commands per P1/P2 side (`sensor_mag -i 0..4` and
`vehicle_magnetometer -n 1`). Sending them separately is required by the Radio transport's short
serial input frame.

PASS — The Qt workspace refresh/model emitted
`[픽스호크 UI] P1/P2 5-MAG yaw cells populated` in
`.local/workstation_data/logs/run.log` (line 60 in the captured run). That record contains ten
finite RAW yaw cells: P1 IMU streams 1–3 plus RM3100 CAN1/CAN2, and P2 IMU streams 1–3 plus
RM3100 CAN1/CAN2. The same record preserves the live topics, including
`px4-shell/sensor_mag/3` and `/4`; example RM3100 values were P1 `-143.78°`, `+147.92°` and
P2 `-148.64°`, `+144.07°`.

The raw five-row yaw requirement is therefore verified against a running Workstation and live
Radio/PX4 responses, not only a fake boundary. `PX4`/`Δ` cells may still show `STALE`, `ID 확인
대기`, or `미선택` when Radio ATTITUDE is not fresh or the active identity is not selected; that
is the intended exact-identity/freshness gate and does not suppress the RAW yaw cells. Direct CUA
window capture was unavailable in this environment, so the evidence is the running Qt model's
own UI-population log rather than a screenshot.

## Direct RM3100 calibration-parameter readback — 2026-09-05

Read-only MAVLink `PARAM_VALUE` readback was performed directly over the P1/P2 Radio serial links,
using the same integer bit-decoding convention as the Workstation. No parameter was written.

| Pixhawk | RM3100 slot/device | Priority | Rotation | Offset X/Y/Z | Scale X/Y/Z | Cross/diag terms |
|---|---|---:|---:|---|---|---|
| P1 | `CAL_MAG2_ID=8944899` (CAN1) | 100 | 0 | 0 / 0 / 0 | 1 / 1 / 1 | all 0 |
| P1 | `CAL_MAG3_ID=8944387` (CAN2) | 75 | 0 | 0 / 0 / 0 | 1 / 1 / 1 | all 0 |
| P2 | `CAL_MAG2_ID=8944643` (CAN1) | 100 | 0 | 0 / 0 / 0 | 1 / 1 / 1 | all 0 |
| P2 | `CAL_MAG3_ID=8944387` (CAN2) | 100 | 0 | 0 / 0 / 0 | 1 / 1 / 1 | all 0 |

The initially missing Radio replies (`P1 CAL_MAG2/3_YCOMP`, `P1 CAL_MAG3_ROLL`, and
`P2 CAL_MAG3_ZODIAG`) were individually reread and were also `0.0`. Result: both RM3100s are
registered and enabled by priority, but no non-identity hard/soft-iron correction or rotation is
currently stored in these RM3100 calibration slots. This explains why the UI's intentionally raw
`atan2(-y, x)` yaw can retain a large CAN1/CAN2 difference; the parameters show registration, not
a completed effective magnetometer calibration.

After readback, the actual Workstation was restarted with `./run.sh --no-start --start-pixhawk`.
The live process resumed P1/P2 Shell polling and five-row UI updates; for example the current log
contains P2 RM3100 values `-165.67°` and `+157.74°` at lines 188–189.

## Active standard-magnetometer PX4 yaw identity handoff — 2026-09-05

Root cause of the empty PX4-yaw column: Stable PX4 Shell provides the standard `sensor_mag`
instance `device_id`, but the faster `SCALED_IMU*` sample for the same presentation slot does not.
The latter replaced the cached Shell state with `device_id=None`, so the exact active-device gate
returned `ID 확인 대기` even when the active device was the board magnetometer.

Fix: preserve the latest known device identity when a subsequent `SCALED_IMU*` sample updates the
same slot. This does not copy PX4 yaw to non-active rows; it only keeps the correct identity for
the matching standard stream. RM3100 rows remain `미선택` while PX4 selects an internal/GPS mag.

PASS — `tests/test_pixhawk_magnetometer_workspace.py` now verifies Shell standard identity survives
multiple `SCALED_IMU` replacements. The focused combined command returned **60 passed**, status
tests returned **6 passed**, and Shell/serial tests returned **5 passed, 93 deselected**.

PASS — After restarting the real Radio-connected Workstation, P1 and P2 both rendered a selected
standard-magnetometer row with PX4 yaw and Δ repeatedly. Example: [run.log:146](/home/moontaehyung/바탕화면/SASL-20260821/.local/workstation_data/logs/run.log:146)
shows P1 `PX4=-164.68° Δ=+21.02°` and P2 `PX4=+85.51° Δ=-67.14°`; both remain tied to
`mavlink/SCALED_IMU`/Shell instance identity rather than an RM3100 row.

## Direct RM3100 raw/vector and UAVCAN health check — 2026-09-05

Read-only Shell checks were performed on both Radio links after stopping the Workstation owner,
then the Workstation was restarted. The five `sensor_mag` instances were queried directly and
the RM3100 vectors were:

| Side | Node/device | x / y / z (G) | Raw yaw | Norm (G) |
|---|---|---|---:|---:|
| P1 CAN1 | node `0x7D`, `8944899` | `-0.22070 / +0.07873 / +0.44091` | `-160.367°` | `0.499308` |
| P1 CAN2 | node `0x7B`, `8944387` | `-0.27905 / -0.11853 / +0.37451` | `+156.986°` | `0.481846` |
| P2 CAN1 | node `0x7C`, `8944643` | `-0.18310 / +0.04843 / +0.43554` | `-165.185°` | `0.474938` |
| P2 CAN2 | node `0x7B`, `8944387` | `-0.16394 / -0.06939 / +0.46362` | `+157.059°` | `0.496623` |

Both UAVCAN buses reported `Internal failures: 0`, `Transfer errors: 0`, `HW errors: 0`, and
`IO errors: 0`; all visible nodes were `OK OPERAT`. P1 had node 125/123 and P2 node 124/123,
with node 123 as the common CAN2 device. The repeated CAN1-vs-CAN2 yaw offset therefore follows
the sensor/node group, not Radio loss or a Workstation parser asymmetry. The raw vectors are
healthy and finite, but the RM3100 `CAL_MAG` corrections read back as identity; the remaining
checkable causes are sensor/node frame orientation or uncalibrated hard/soft-iron/environmental
effects. A physical sensor-swap test or PX4 compass calibration is required to distinguish those
two causes.

## CAL_MAG-corrected yaw display — 2026-09-05

The operator table now exposes a visible `Corrected yaw°` column for both the consolidated IMU
magnetometer rows and the dedicated CAN rows. The calculation follows PX4's magnetometer
calibration contract: `R * S * (data - offset)`, using the device ID's `CAL_MAG` slot, offset,
diagonal/off-diagonal scale terms, and sensor rotation. A nonzero power/current compensation is
reported as pending because the Shell sensor sample has no contemporaneous power value. PX4's
calibration library defines the same correction order in `Magnetometer::Correct()`.

The corrected column remains `PARAM 대기` until the user performs the existing manual P1/P2
`QGC 결과 새로고침`; no automatic parameter lookup was added. PX4 yaw/Δ remain in the backend
diagnostic model but are not returned to the operator table. Pure tests cover identity, offset,
scale, rotation, and missing-readback states; the combined focused run returned **62 passed**.

## User-requested CAN2 priority trial — 2026-09-05

`CAL_MAG3_PRIO=100` was written/read back on both disarmed Pixhawks. At the time of the trial both
sides already read back `100`, so no effective priority change occurred. The active
`vehicle_magnetometer.device_id` remained GPS2 `396817` on both sides rather than CAN2 RM3100
`8944387`. A short direct ATTITUDE sample gave P1 `+77.09°` and P2 `+85.21°` (about `+8.1°`
shortest difference), but this improvement cannot be attributed to CAN2 selection because the
active device did not switch. No `param save` or Pixhawk reboot was performed; a valid CAN2 trial
requires applying persistence/reinitialization and verifying active ID `8944387` afterward.

## Operator display preference — 2026-09-05

The operator requested that `PX4 yaw` and `Δ yaw` not be shown in the magnetometer tables. The
two columns are now hidden in both the consolidated IMU magnetometer table and the dedicated CAN
magnetometer table. Raw yaw, sensor values, source, age, and topic remain visible. PX4/Δ evidence
and the exact-identity calculation remain in the backend read model and Inspector for diagnostics;
they are not removed from the data path.

PASS — Table visibility assertions cover both hidden column pairs. The final focused command returned
**60 passed**, the status suite returned **6 passed**, and Shell/serial tests returned **5 passed,
93 deselected**. The actual Workstation was restarted with Radio links after the UI change.

## User-requested EKF parameter parity check — 2026-09-05

Read-only precheck confirmed both Pixhawks were disarmed. The requested `param set` commands were
sent through PX4 Shell to both sides. P2 matched on the first pass; P1's `SENS_IMU_MODE`,
`EKF2_HEAD_NOISE`, and `EKF2_MAG_NOISE` were retried individually and then read back as `1`, `0.3`,
and `0.05`. Final readback matched the requested values on both P1/P2. No `param save` or Pixhawk
reboot was performed.

PASS — After Workstation restart and more than 30 seconds of live Radio reception, both sides
reached `MAVLink liveness: LIVE`, but PX4 yaw remained approximately P1 `-164.69°` and P2
`+85.59°` (shortest difference about `109.7°`), as recorded in `run.log` around line 204. This
shows the requested noise/gate/mode parity does not itself align the two independent EKF yaw
references; a reboot/EKF yaw reset and effective magnetometer calibration/common heading reference
remain separate operational steps.

## ATTITUDE age regression amendment — 2026-09-05

Root cause: the six-command Radio Shell poll owns `recv_match` for the duration of each poll
sequence. Non-`SERIAL_CONTROL` packets were correctly deferred by the command worker, but the Qt
receive timer skipped the side while any command was active. ATTITUDE therefore waited until the
whole `sensor_mag`/`vehicle_magnetometer` sequence finished before it could update its receive
timestamp.

Fix: `spin_once()` now drains only the already-deferred telemetry queue while a command owns the
receive path. It never calls `recv_match` during the command, so Shell responses remain owned by
the worker while deferred HEARTBEAT/ATTITUDE samples update liveness and the UI immediately.

PASS — Regression test
`../../.venv/bin/python -m pytest tests/test_mavlink_liveness.py -q` → **29 passed**. The new test
proves an ATTITUDE deferred during an active `RM3100 SHELL POLL` is replayed before the command
ends and keeps the receive timestamp current.

PASS — Live Radio verification after restarting the real Workstation with
`./run.sh --no-start --start-pixhawk`: P1 and P2 each reached `MAVLink liveness: LIVE` repeatedly
while Shell polling continued (for example `run.log` lines 183, 195, 246, 260, 308, 322, 370,
and 383). No `ATTITUDE STALE` transition was emitted in this post-fix run, and the same run kept
emitting the ten-cell `[픽스호크 UI] P1/P2 5-MAG yaw cells populated` record (lines 420–421 at the
latest check). `LIVE` is the liveness contract that heartbeat and ATTITUDE ages are both within
their fresh thresholds; the temporary `QUALIFYING` messages occur while the diagnostic Shell
command is active.
