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

Pixhawk `Find network` is a read-only contextual command. It inventories wired NIC/address/carrier
and P1/P2 route evidence without changing host networking. The discovery row remains candidate-only;
P1/P2 assignment cards separately expose each side's endpoint and route. `Assign P1/P2` persists operator intent;
it disconnects only the selected side when that side is already open. `Connect` revalidates the
stored interface/address against the current kernel route and attempts valid disconnected sides
independently. Assign or socket-open acknowledgement is never link success.

## 5. Stateful Commands

Transitions matter. Examples:
- Connect: DISCONNECTED → CONNECTING → CONNECTED
- Measurement: IDLE → QUEUED(optional) → STARTING → RUNNING → STOPPING → IDLE
- Manual recording: IDLE → STARTING → ACTIVE → STOPPING → IDLE
- Preview: STOPPED → STARTING → LIVE → STOPPING → STOPPED

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
The stalled Ethernet connection is never used as the reboot transport. The command writes no
parameter/config/file and does not imply recovery; `RECOVERING` plus 10-minute qualification is the
observable postcondition. Automatic reboot is not a supported command.

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
gates still block unsupported results. Manual recording and busy camera resources transition only
their branches while the run remains RUNNING. A duplicate active run is still rejected.

Camera rejoin allocates a new MP4 segment. File existence produces `SAVED/PARTIAL/FAILED` without
claiming usability. Finalization, integrity or artifact enqueue failure never stops or gates a run.

## 14. Detection / Recording / Calibration Separation

- Detection enable/disable → Command
- model/confidence → Configuration
- runtime detection state → Status

- Recording start/stop → Command
- output/data-root config → Settings/Data Workspace
- recording state → Status

- Calibration global entry → Navigation/command
- calibration steps → local workflow commands

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
