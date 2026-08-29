# Frontend Contract

## Controls and inputs

`Find network`, NIC/local-IPv4 candidate table, independent P1/P2 assignment summaries,
`Assign P1`, `Assign P2`, 기존 `Connect/Disconnect/Find USB`.

## Payloads and commands

Network Assign payload는 stable candidate ID와 target side다. Connect는 저장된 side assignment를
controller preflight로 보낸다.

## Visible states and transitions

NOT RUN -> DISCOVERING -> FOUND/ERROR; UNASSIGNED/LEGACY AUTO ROUTE -> ASSIGNED ->
REASSIGN REQUIRED 또는 기존 CONNECTING/QUALIFYING/LIVE.

## Errors and recovery actions

INTERFACE MISSING/LINK DOWN/ADDRESS MISSING/NO ROUTE/ROUTE MISMATCH/ENDPOINT INVALID/PORT IN USE를
inline 표시하고 Find/Assign/Connect 재시도를 안내한다.

## Keyboard, focus, accessibility, and layout

QTableView row selection, native keyboard focus와 fixed header policy를 사용하며 refresh에서 selection,
focus, scroll을 보존한다.

## Cross-feature visible behavior and isolation

Camera table은 Camera candidate만, Pixhawk table은 Pixhawk network/USB candidate만 표시한다.
같은 physical NIC가 양쪽에 나타나도 warning이나 gate가 없다.

Discovery table columns are `Source / NIC or Device / Link / Local IPv4 / Assigned`; a row never
mixes P1 and P2 routes/endpoints. Each side summary independently displays
`State / NIC / Local IPv4 / Endpoint / Route`.
