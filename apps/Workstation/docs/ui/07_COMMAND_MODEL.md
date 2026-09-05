# StereoWIRIS Command Model

- Status: Revised after audit; command/accepted/status boundary implemented
- Depends on: 00–06 design documents

## 1. Fundamental Rule

> Commands request change. Status confirms change.

## 2. Command Categories

- Global
- Contextual
- Stateful
- Toggle
- Immediate
- Destructive
- Workflow
- Diagnostic

## 3. Global Commands

Only high-frequency cross-context operations. Candidates: Connect/Disconnect/Reconnect, Preview,
Measurement Start/Stop, Manual Record, open Data, and Diagnostics. There is no Measurement
Session Save/Finalize command.

## 4. Contextual Commands

Operate on selected object/workspace: camera connect/preview/capture, calibration capture/calculate/save, etc.

`Data > Status` adds two local derived-artifact commands:

- `Update Latest XLSX` submits/coalesces a request for the newest saved Measurement only. It is
  disabled while Measurement/Manual recording or XLSX generation is active. Acceptance means
  `WAITING/GENERATING`, not success; status readback owns the result.
- `Open Latest XLSX` is enabled only when the published path exists and an app-owned generation is
  reported. It delegates to the system spreadsheet application and does not edit source data.

Automatic Measurement-stop submission uses the same backend request after the run is already
`IDLE`. Duplicate requests coalesce; a newer completed generation supersedes an older pending one.
Cancellation, checkpoint timeout, malformed input, publication failure, and manual retry remain
local to Data and cannot reject Measurement.

`Find network` is a read-only contextual command that inventories physical NIC identity,
address/carrier and route evidence. `Assign Camera NIC/P1/P2` is a separate explicit mutation:
native before/after confirmation -> OS authorization -> target-first app-owned NetworkManager delta
-> dual readback -> role-scoped source cleanup -> `PRESS CONNECT`. Pixhawk owners never reconnect
automatically after the host transaction; the operator's later Connect owns device evidence.
One or many NICs and same-NIC role composition use the same command.

Ordinary healthy Camera/Pixhawk owners on source/target NICs follow the selected role automatically;
unrelated NIC owners never restart. Measurement/recording/save/finalization rejects before any stop
or mutation. Assignment persistence commits last, rollback restores the original arrangement where
possible, and incomplete rollback remains explicit. Connect continues side-specific route preflight.
Assign acknowledgement, NetworkManager write, or socket open is never device success.

## 5. Stateful Commands

Transitions matter. Examples:
- Connect: DISCONNECTED → CONNECTING → CONNECTED
- Measurement: IDLE → QUEUED(optional) → STARTING → RUNNING → STOPPING → IDLE
- Manual recording: IDLE → STARTING → ACTIVE → STOPPING → IDLE
- Preview: STOPPED → STARTING → LIVE → STOPPING → STOPPED
- STUS: OBSERVER_PREFLIGHT → LEFT_COMMAND/ACK → RIGHT_COMMAND/ACK → RTCP_OBSERVING →
  VERIFIED or one RELATIVE_CORRECTION round → VERIFIED/FAILED

Avoid naive toggles where backend transitions are meaningful.

## 6. Toggle Commands

Use only when ON/OFF are symmetrical, transition is simple, backend state is observable, and ambiguity is low.

## 7. Immediate / Destructive Commands

Immediate examples: export, copy, screenshot, open file. Destructive actions require appropriate confirmation.

The P1/P2 Ethernet stall action is a guarded destructive command, not a generic reboot. It is
enabled only for `ETH MAVLINK RX STALLED`, while Measurement/recording/save/finalization and other
PX4 commands are idle. It requires one current unique USB heartbeat mapping (P1=sysid1/P2=sysid2),
names side/device/sysid in the confirmation, and revalidates the same identity in a one-shot worker.
Operator No, missing/stale/duplicate/changed identity, or revalidation failure sends no reboot.

Telemetry Radio and Ethernet are per-board selections. Transport selection, endpoint edits,
device return, startup flags and recovery actions cannot open a link; only explicit `Connect` can.
Persistent `MAV_2_*` setup is a separate manual command and is never a first-heartbeat side effect.

`Find USB` and `Find network` are mutually exclusive read-only Discovery commands. An accepted
command clears the previous transient candidate snapshot before starting and publishes only its
new result. A duplicate/concurrent request is rejected without clearing the active snapshot.
Discovery never changes saved endpoints, NIC assignments, settings, or connection owners.

Telemetry has its own bounded `Find Network` projection for the Drone role. It passively observes
UDP MAVLink, permits `Use selected` only for current sysid-3 evidence, and fills the editable URL
without Apply, Save, Disconnect or Connect. Apply/Save responses return the newly accepted URL;
stale connection status cannot rewrite that configuration field.

Explicit parameter, shell, reboot, persistent Ethernet, MAVFTP file/log and firmware-staging
commands are available on Radio. RTCM activity creates no warning, modal or block; one bounded Log
event records role, transport, category, start time and RTCM age. Upload/download interruption
leaves a partial artifact. `Resume` requires exact prefix readback; otherwise `Restart` is required.
Radio firmware staging is `STAGED`, not installed. USB flash is globally serialized, verifies
direct-USB sysid and bootloader board ID, uses the pinned uploader without force flags, and leaves
telemetry disconnected afterward.

The topology workflow has four contextual commands owned by one controller:
`pixhawk.topology.check`, `pixhawk.topology.start`, `pixhawk.topology.cancel`, and
`pixhawk.topology.rollback`. The current-row action advances the controller state machine; it is
never inferred from button text. `Check` is read-only. Every persistent SiK/PX4/SD-file action
requires a fresh unique expected-sysid probe, physical isolation, disarmed evidence, exclusive
endpoint ownership, and an explicit confirmation naming the target and mutation. The final settings
commit occurs only after a fresh cross-radio/UDP readback. P2 provisioning first journals the exact
pre-change parameter/file profile in a separate read-only action; a device write cannot begin if
that evidence is not durable. Cancel after any persistent change remains `PARTIAL`, and
rollback uses captured pre-change evidence. No topology command opens MAVLink, presses Connect,
implements failover, or treats command acceptance as qualification.
While the operation is active or partial, the duplicated Drone Telemetry Apply/Connect/Discovery
surface and all conflicting Pixhawk administration are locked; an existing Disconnect remains
available. Mid-operation topology-settings drift blocks the next write and final commit.
The stalled Ethernet connection is never used as the reboot transport. The command writes no
parameter/config/file and does not imply recovery. The product postcondition is `RECOVERING` plus a
1-minute qualification; a separate 10-minute post-recovery observation is required only for
field-stability acceptance. Automatic reboot is not a supported command.

## 8. Command Ownership

One authoritative action/controller path. Menu and Toolbar should reuse the same QAction/command logic.

## 9. Toolbar

Candidate Phase 1:
[Connect] [Preview] | [Measurement] | [Manual Record] | [Data] | [Diagnostics]

Toolbar remains compact; configuration fields do not belong there.

## 10. Menu Structure

Candidate: File, View, Device, Camera, Measurement, Calibration, Recording, Tools, Diagnostics, Help. Exact final menus depend on audited command ownership.

## 11. Enablement / Preconditions

Commands shall be enabled/disabled based on actual runtime conditions. Disabled reasons should be discoverable where non-obvious.

`Use suggestion` ignores detector quality state/confidence for both Horizon and Mountain while
retaining those values as status/evidence. Horizon requires current frozen identities/dimensions,
exactly two finite in-frame endpoints per side and 25%-width separation. Mountain requires at least
four equal-order finite in-frame unique/distributed pairs after one shared exclusion-index set. The
controller repeats validation; widget state is never the geometry oracle. The command does not run
Solve or Apply.

## 12. Postconditions / Failure

Stateful commands have expected postconditions. Failure feedback should identify command, target, factual reason, timestamp, and possible recovery.

## 13. Save / Apply Semantics

Save must have defined scope. Inspector values may be immediate, deferred-with-Apply/Revert, or read-only; avoid inconsistent mixtures.

Calibration evidence preservation and runtime promotion are separate commands. A quality
`WARNING` or `FAIL` calculation may be saved under the calibration `attempts/` evidence path, but
that save never updates the active alias, persisted settings, or Measurement geometry. `WARNING`
still requires explicit confirmation before Apply; `FAIL` cannot be applied. Structural validation
of K,D/R,t and exact operating-profile applicability remain hard Apply gates.

`측정 시작` immediately creates a transient observation run. It does not require a parent, save
completion, storage health, camera, P1/P2, GNSS/NTRIP, Target, Alignment, exact K,D, or a ready
method. Missing or invalid source configuration marks only that source unavailable; method-local
gates still block unsupported results. Those gates accept a retained last valid attitude/GPS sample
for a side that has already produced one, including a sample received before the current
Measurement, and still block a side that never produced one. No command, confirmation, or clear
action is added for that reuse. Manual recording and busy camera resources transition only
their branches while the run remains RUNNING. A duplicate live interval is still rejected.

The same command requests per-side decoded-NV12 lossless NVENC recording. Camera rejoin allocates a
new segment rather than reopening a prior path. File existence produces `SAVED/PARTIAL/FAILED` and
does not claim usability. Finalization, integrity, artifact enqueue, NVENC, queue or disk failure is
recording-local and never commands a Detection/native-frame/Measurement stop or blocks the next
run. There is no strict/tolerant toggle or CPU encoder fallback command.

Manual and scheduled STUS stay enabled during Preview, Measurement, YOLO and recording. They own a
separate decoder-free RTCP observer and never stop/restart CameraPane, YOLO or MP4. Observation is
parallel; mutation is ordered Left exact-OK then Right. Per-camera SR-PC absolute magnitude is a
diagnostic payload only. The command postcondition is fresh/stable Left/Right relative RTCP timing
at or below 100 ms, not ACK, PC absolute proximity, or zero dropped frames. Auto requires one manual
VERIFIED seed in the current application/camera-connection generation and executes only at its
configured deadline.

STUS is not a live-distance command or prerequisite. With the default App receive setting,
distance continues on host monotonic time. With `RTCP/NTP preferred`, VERIFIED/current RTCP may
select future pairs; STUS transition, failed/revoked Clock or unusable frame evidence causes exact
app-receive fallback without an operator retry. STUS still owns only Clock mutation/status and
cannot turn fallback into a successful Clock state.

PX4 calibration PARAM readback is an explicit contextual command. Reaching `LIVE` never invokes
it. `Refresh P1/P2 parameters` and the Advanced MAG/attitude controls retain the existing bounded
worker, `PARAM SYNC`, result/error, and retry path. Reset/apply/save may invoke the same explicit
verification readback after their operator-requested mutation.

The visible Pixhawk page additionally exposes side-specific `Refresh MAG` and `Apply & Verify`
commands. They reuse the existing per-camera PARAM/Shell owner; `Apply & Verify` succeeds only
after PARAM readback, exact active `vehicle_magnetometer.device_id`, and fresh ATTITUDE. P1/P2
commands are independent. A Measurement start that requires live Pixhawk data consumes this
verification result and fails closed when a required side is unverified.

## 14. Detection / Recording / Calibration Separation

- Detection enable/disable → Command
- model/confidence → Configuration
- runtime detection state → Status

- Recording start/stop → Command
- output/data-root config → Settings/Data Workspace
- recording state → Status

- Calibration global entry → Navigation/command
- calibration steps → local workflow commands
- Calibration Auto intent does not imply capture success. Success requires positive NAL1/non-delta
  AU evidence, exact decoded PTS and native FrameKey, then the existing checkerboard/timing/save
  postconditions. Missing/evicted evidence remains waiting; manual capture stays independent.

## 15. Command Logging

Important system-changing commands should be logged separately from resulting state transitions where useful.
For verified-USB recovery, log discovery identity, operator cancellation or acceptance, immediate
worker revalidation, command send/failure, Ethernet recovery transition, and qualification result as
distinct events.

## 16. Anti-Patterns

Do not use one button as command+status, infer backend success from visual button state, duplicate backend logic, keep impossible actions enabled, invent unsupported commands, or show routine success modals.

## 17. Summary

Navigation = context
Configuration = desired parameters
Command = request
Status = confirmed state
Diagnostics = evidence

> A button click is an event, not a fact.

Implementation note: `CommandRegistry` emits `invoked`, `accepted`, `completed`, or
`failed`. `accepted`/`completed` only mean the existing handler returned without an
exception; asynchronous postconditions remain visible through `StatusModel` and the
source-backed workspace/Inspector evidence.
