# Workflow Impact

## Affected operators and workflow IDs

Field operator; `WF-PX4-NETWORK-ASSIGN-CONNECT`.

## Current workflow

Settings 또는 후보 표에서 host/URL을 선택하고 Connect한다. NIC/route는 외부 명령으로 확인한다.

## Proposed workflow

`Find network -> NIC/address 행 선택 -> Assign P1/P2 -> Connect -> route preflight ->
socket open -> expected sysid -> QUALIFYING/LIVE`.

Amended role workflow: `Find network -> NIC 선택 -> Camera/Pixhawk 역할 적용 요청 ->
변경 예고 확인 -> OS 인증 -> NetworkManager 적용 -> kernel address/route readback ->
assignment save`. `Connect` remains a separate device operation and still requires route preflight
plus expected-system MAVLink evidence.

Applying a role to a NIC already used by another role is additive. The confirmation shows both the
new role and the Camera/P1/P2 roles that will remain on that NIC; no intermediate exclusive-role
transition is exposed to the operator.
The same workflow covers one or many discovered NICs. The operator never chooses a topology mode:
they assign roles to the current inventory, and the resulting role sets define the topology.

## Preserved, added, removed, and reordered steps

Find USB, custom endpoint Settings, Connect/Disconnect, liveness와 recovery는 보존한다. visible
`Find host IP`와 static URL-only discovery를 제거하고 NIC/route discovery와 Assign persistence를
추가한다. Role application is explicit and confirmation-gated; discovery and startup never
apply a role silently.

## Decisions, confirmations, handoffs, waits, and artifacts

Role Apply가 operator의 host-network 변경 결정이고 Connect는 그 결과를 다시 검증하는
별도 장치 연결 결정이다. 동일 NIC를 Camera/Pixhawk, P1/P2가 공유해도 기존
승인대로 차단하지 않는다. Persistence must distinguish intended role from verified active
NetworkManager state.
Role assignment is a set, not a single-choice label: a row may visibly be assigned to
`Camera,P1,P2` at the same time.
Candidate count and order may change after plug/unplug; stable physical identity preserves only
matching selections and stale saved roles become `REASSIGN REQUIRED` rather than being remapped by
row position.

## Interruption, cancellation, retry, restart, and resume

중복 discovery/connect는 기존 busy gate로 거절한다. LIVE side 재배정은 해당 side disconnect
완료 후 저장한다. restart는 마지막 배정을 복원하되 interface가 없으면 자동 대체하지 않는다.

Close/shutdown during stop/apply/reconnect attempts bounded rollback and records actual host/owner
state. A late worker result cannot overwrite a newer operation generation.

## Degraded, error, and recovery behavior

interface missing, link down, address missing, no route, route mismatch, invalid endpoint, port in use를
side별로 표시한다. Privilege denial, NetworkManager rejection, partial write and readback
mismatch become explicit role-application failures. The old role remains usable unless the approved
role move reaches its complete verified postcondition.
Failure while adding one role must leave the already-working roles on that NIC unchanged.

A move uses `snapshot A/B -> apply role to B -> verify B -> remove only moved role from A -> verify
B and every role retained on A -> persist`. Target failure leaves A unchanged. Cleanup failure rolls
back B toward the original arrangement and exposes any incomplete rollback as a blocking partial
state with retry/recovery guidance.

If affected Camera/P1/P2 owners are active, the sequence continues as `confirm affected owners ->
stop only affected owners -> move role -> reconnect the previously active set -> verify real device
evidence`. Owners on other NICs continue. Ordinary live operation follows the operator-selected
NIC; only Measurement/recording/save/finalization retains the hard mapping lock.

## Completion and next actions

socket open은 완료가 아니다. operator는 expected sysid, heartbeat/ATTITUDE와 60초 LIVE를 확인한다.

## Training, operating-procedure, and compatibility impact

새 기본 절차는 세 동작이다: Find, Assign, Connect. 구 settings는 legacy auto-route로 계속 동작한다.

Role Apply adds one confirmation/authentication wait. An ordinary active connection may briefly
show `STOPPING -> APPLYING -> RECONNECTING`; no manual Stop/Disconnect is required.

## Workflow-to-screen, command, state, and regression traceability

Pixhawk workspace -> discovery/assign facade -> device runtime controller/worker -> kernel route oracle ->
DualPixhawkMavlinkSubscriber -> StatusModel/EvidenceAdapter.

## Open field refinement — host assignment versus device readiness

The real split-NIC run showed that the approved all-in-one completion rule prevents assigning a
currently offline device. The proposed refinement is to make verified host assignment the durable
postcondition of Assign and keep device reacquisition as a separate per-role result:

`Assign -> apply/read back host role -> persist selected NIC -> reconnect previously healthy owners
-> show each owner as LIVE/QUALIFYING/DEVICE NOT SEEN`.

The operator chose automatic restoration instead of retaining an unproven new mapping. Missing
required HEARTBEAT/ATTITUDE/frame evidence therefore enters `ROLLING BACK`, restores the previous
host role and previously active connection set, and leaves the saved assignment unchanged. A
successful rollback ends `RESTORED · DEVICE NOT SEEN ON TARGET`; incomplete restoration remains a
blocking partial state.

The operator decided that a role already offline before the click does not veto another selected
role. The pre-operation snapshot separates `healthy` from merely `open/configured`: only the
selected role and other affected owners with current Camera-frame or expected-system Pixhawk
HB+ATT evidence enter the required reconnect set. Pre-existing offline owners remain offline and
are not started or awaited. A required previously healthy owner that fails to recover still triggers
automatic previous-NIC restoration.

The remaining workflow decision concerns the normal Assign-before-Connect path: when the selected
role itself was not connected before Assign, there is no prior healthy device session to restore or
compare. The operator must decide whether verified host routes alone may persist until a later
explicit Connect, or Assign must require new target device evidence and restore on timeout.
