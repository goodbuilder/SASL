# Integration Flow

## Action-to-postcondition traces

`Find network -> worker read -> candidate snapshot -> table`; `Assign -> optional side disconnect ->
settings atomic save -> assignment summary`; `Connect -> route preflight -> connection worker -> socket
open -> expected sysid HB/ATT -> StatusModel`.

Amended host-role trace: `Find -> select physical NIC -> explicit role action -> before/after review
-> operator confirm -> privilege result -> NetworkManager mutation -> NM + kernel readback -> role
state/assignment persistence -> visible APPLIED or factual failure`. No socket is opened by Apply.

Same-NIC trace: `existing role set -> add Camera/P1/P2 intent -> union desired entries -> apply only
missing app-owned entries -> read back every retained and added role -> publish combined role set`.

Move trace: `snapshot source/target -> add role to target -> verify target -> remove only moved role
from source -> verify target plus retained source roles -> persist assignment`. Any pre-cleanup
failure leaves source untouched; cleanup failure triggers target rollback and truthful partial-state
reporting if restoration cannot be independently proven.

Active-owner trace: `snapshot active owners on affected NICs -> stop/release those owners -> host
role transaction -> restart previous active set -> Camera generation frame / Pixhawk expected
HB+ATT -> publish moved assignment`. On device-evidence failure: `stop partial new owners -> restore
host snapshot -> restart original owner set -> publish restored or exact partial failure`.

## Transition tables

Discovery, role application and Connect are separate evidence boundaries. A successful role action
means verified host address/route only; successful Connect handler means socket attempt only; LIVE
remains the device postcondition.
Removing or moving one role may remove only entries whose role reference count reaches zero.

## Failure and recovery flows

Preflight failure creates no socket for that side. Partial success preserves the other side. Changed NIC
name requires new Find/Assign. ETH RX stall keeps the existing verified-USB/manual power recovery.
Network-role move failure never becomes `APPLIED` or updates the side assignment without the full
two-NIC postcondition.
Owners on unrelated NICs do not enter any transition. Mapping-locked Measurement/recording rejects
before active-owner snapshot or host mutation.

## Cross-feature contracts and dependency direction

UI -> facade -> controller -> read-only network adapter / subscriber -> evidence/status. Pixhawk code does
not call Camera widgets/workers and Camera does not consume Pixhawk candidate state.
