# Frontend Contract

## Controls and inputs

`Find network`, NIC/local-IPv4 candidate table, independent P1/P2 assignment summaries,
`Assign P1`, `Assign P2`, 기존 `Connect/Disconnect/Find USB`.

Amendment: an explicit Camera/Pixhawk role action opens a native confirmation showing the selected
NIC, current address/routes, exact proposed address/routes and whether another NIC currently owns
that role. Find, startup and row selection alone never change the host.
If the selected NIC already has another app role, the confirmation says that role will remain and
the assignment display may show `Camera,P1,P2` together without implying a conflict.
The table has no fixed NIC count and exposes every discovered physical wired NIC. It does not ask
for `single` versus `multiple` topology mode; its `Assigned` text shows the current role set for
each row and remains stable across unrelated status refreshes.

## Payloads and commands

Network Assign payload는 stable candidate ID와 target side다. Connect는 저장된 side assignment를
controller preflight로 보낸다.

## Visible states and transitions

NOT RUN -> DISCOVERING -> FOUND/ERROR; UNASSIGNED/LEGACY AUTO ROUTE -> ASSIGNED ->
REASSIGN REQUIRED 또는 기존 CONNECTING/QUALIFYING/LIVE.

Role application adds `REVIEW REQUIRED -> AUTHORIZING -> APPLYING -> VERIFYING -> APPLIED` and
terminal `CANCELLED/AUTH DENIED/APPLY FAILED/READBACK MISMATCH` states. A successful button handler
is not success; visible `APPLIED` requires independent kernel/NetworkManager readback.
An add-role failure retains the previous role list and reports that existing Camera/P1/P2 traffic
was not intentionally changed.
Move progress distinguishes `VERIFYING TARGET`, `CLEANING SOURCE`, `VERIFYING RETAINED ROLES` and
`ROLLING BACK`. A failed move keeps the last verified assignment visible; a partial rollback shows
both actual NIC states and a direct retry/recovery action instead of claiming either assignment.
When affected owners were active, progress additionally shows `STOPPING AFFECTED`, `RECONNECTING`
and the per-owner device-evidence result. The confirmation names every Camera/P1/P2 owner that will
restart and says that owners on other NICs continue.

## Errors and recovery actions

INTERFACE MISSING/LINK DOWN/ADDRESS MISSING/NO ROUTE/ROUTE MISMATCH/ENDPOINT INVALID/PORT IN USE를
inline 표시하고 Find/Assign/Connect 재시도를 안내한다.
An active Measurement/recording/save/finalization rejects role Apply with the preserved mapping-lock
reason. Ordinary LIVE/Preview state is not a rejection; it enters automatic restart.

The 2026-08-30 field refinement keeps automatic restore when required device evidence is missing.
`DEVICE NOT SEEN` is shown as the rollback cause, followed by `RESTORED` only after previous host
routes and required previously running owners are independently verified. The new NIC is not shown
as assigned after restoration.

An affected owner that was already offline before the click is excluded from reconnect progress and
cannot disable another role's success. The confirmation distinguishes `healthy; will reconnect` from
`already offline; unchanged`. Mere socket/worker/configuration presence is not shown as healthy.
Selected-role behavior when Assign precedes its first Connect remains an open interview decision.

## Keyboard, focus, accessibility, and layout

QTableView row selection, native keyboard focus와 fixed header policy를 사용하며 refresh에서 selection,
focus, scroll을 보존한다.

## Cross-feature visible behavior and isolation

Camera table은 Camera candidate만, Pixhawk table은 Pixhawk network/USB candidate만 표시한다.
같은 physical NIC가 양쪽에 나타나도 warning이나 gate가 없다.

Discovery table columns are `Source / NIC or Device / Link / Local IPv4 / Assigned`; a row never
mixes P1 and P2 routes/endpoints. Each side summary independently displays
`State / NIC / Local IPv4 / Endpoint / Route`.
