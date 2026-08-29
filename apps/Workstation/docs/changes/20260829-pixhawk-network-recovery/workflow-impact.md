# Workflow Impact

## Affected operators and workflow IDs

Field operator; `WF-PX4-NETWORK-ASSIGN-CONNECT`.

## Current workflow

Settings 또는 후보 표에서 host/URL을 선택하고 Connect한다. NIC/route는 외부 명령으로 확인한다.

## Proposed workflow

`Find network -> NIC/address 행 선택 -> Assign P1/P2 -> Connect -> route preflight ->
socket open -> expected sysid -> QUALIFYING/LIVE`.

## Preserved, added, removed, and reordered steps

Find USB, custom endpoint Settings, Connect/Disconnect, liveness와 recovery는 보존한다. visible
`Find host IP`와 static URL-only discovery를 제거하고 NIC/route discovery와 Assign persistence를
추가한다. 별도 Apply 단계는 없다.

## Decisions, confirmations, handoffs, waits, and artifacts

Assign이 operator 결정이다. Connect는 OS 설정을 변경하지 않는다. 동일 NIC를 Camera/Pixhawk,
P1/P2가 공유해도 경고하지 않는다. 저장 artifact는 기존 settings JSON의 additive fields다.

## Interruption, cancellation, retry, restart, and resume

중복 discovery/connect는 기존 busy gate로 거절한다. LIVE side 재배정은 해당 side disconnect
완료 후 저장한다. restart는 마지막 배정을 복원하되 interface가 없으면 자동 대체하지 않는다.

## Degraded, error, and recovery behavior

interface missing, link down, address missing, no route, route mismatch, invalid endpoint, port in use를
side별로 표시한다. 한쪽 실패 시 다른 쪽은 유지한다.

## Completion and next actions

socket open은 완료가 아니다. operator는 expected sysid, heartbeat/ATTITUDE와 60초 LIVE를 확인한다.

## Training, operating-procedure, and compatibility impact

새 기본 절차는 세 동작이다: Find, Assign, Connect. 구 settings는 legacy auto-route로 계속 동작한다.

## Workflow-to-screen, command, state, and regression traceability

Pixhawk workspace -> discovery/assign facade -> device runtime controller/worker -> kernel route oracle ->
DualPixhawkMavlinkSubscriber -> StatusModel/EvidenceAdapter.
