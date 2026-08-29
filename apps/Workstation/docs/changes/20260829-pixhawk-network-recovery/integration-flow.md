# Integration Flow

## Action-to-postcondition traces

`Find network -> worker read -> candidate snapshot -> table`; `Assign -> optional side disconnect ->
settings atomic save -> assignment summary`; `Connect -> route preflight -> connection worker -> socket
open -> expected sysid HB/ATT -> StatusModel`.

## Transition tables

Discovery and Assign are intent/evidence separated. A successful Assign means persistence only;
successful Connect handler means socket attempt only; LIVE remains the device postcondition.

## Failure and recovery flows

Preflight failure creates no socket for that side. Partial success preserves the other side. Changed NIC
name requires new Find/Assign. ETH RX stall keeps the existing verified-USB/manual power recovery.

## Cross-feature contracts and dependency direction

UI -> facade -> controller -> read-only network adapter / subscriber -> evidence/status. Pixhawk code does
not call Camera widgets/workers and Camera does not consume Pixhawk candidate state.
