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

Only high-frequency cross-context operations. Candidates: Connect/Disconnect/Reconnect, Preview, Detection, Start/Stop Recording, Save, Export, Diagnostics.

## 4. Contextual Commands

Operate on selected object/workspace: camera connect/preview/capture, calibration capture/calculate/save, etc.

## 5. Stateful Commands

Transitions matter. Examples:
- Connect: DISCONNECTED → CONNECTING → CONNECTED
- Recording: IDLE → STARTING → ACTIVE → STOPPING → SAVED/IDLE
- Preview: STOPPED → STARTING → LIVE → STOPPING → STOPPED

Avoid naive toggles where backend transitions are meaningful.

## 6. Toggle Commands

Use only when ON/OFF are symmetrical, transition is simple, backend state is observable, and ambiguity is low.

## 7. Immediate / Destructive Commands

Immediate examples: export, copy, screenshot, open file. Destructive actions require appropriate confirmation.

## 8. Command Ownership

One authoritative action/controller path. Menu and Toolbar should reuse the same QAction/command logic.

## 9. Toolbar

Candidate Phase 1:
[Connect] [Preview] | [Detection] | [Record] [Stop] | [Save] | [Diagnostics]

Toolbar remains compact; configuration fields do not belong there.

## 10. Menu Structure

Candidate: File, View, Device, Camera, Measurement, Calibration, Recording, Tools, Diagnostics, Help. Exact final menus depend on audited command ownership.

## 11. Enablement / Preconditions

Commands shall be enabled/disabled based on actual runtime conditions. Disabled reasons should be discoverable where non-obvious.

## 12. Postconditions / Failure

Stateful commands have expected postconditions. Failure feedback should identify command, target, factual reason, timestamp, and possible recovery.

## 13. Save / Apply Semantics

Save must have defined scope. Inspector values may be immediate, deferred-with-Apply/Revert, or read-only; avoid inconsistent mixtures.

## 14. Detection / Recording / Calibration Separation

- Detection enable/disable → Command
- model/confidence → Configuration
- runtime detection state → Status

- Recording start/stop → Command
- output/session config → Inspector/Workspace
- recording state → Status

- Calibration global entry → Navigation/command
- calibration steps → local workflow commands

## 15. Command Logging

Important system-changing commands should be logged separately from resulting state transitions where useful.

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
