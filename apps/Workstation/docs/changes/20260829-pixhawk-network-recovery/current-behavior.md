# Current Behavior

## Version and worktree context

- Baseline: `37417a222cf8b6053094c33bb0bb666507041dc6`.
- Worktree에는 통합 안정성 및 Measurement 관련 대규모 user-owned 변경이 이미 존재한다.
- 관련 기준선은 2026-08-29에 `133 passed`였다.

## Reproduction

실장비에서 Workstation은 UDP `14550/14551`을 열고 P1/P2 broadcast packet은
`enx588694fce07e`에서 관찰됐지만 `.2/.3` outbound heartbeat route는 Wi-Fi로 향했다.
Pixhawk 전용 `enx0023a40a2e39`는 존재하지 않았고 현재 NIC에는 camera-owned `.11/32`와
`.231/.232` route만 있었다.

## Observed frontend behavior

Pixhawk top row에는 `Find network`와 `Find host IP`가 따로 있고, candidate 표는 host/USB/
MAVLink URL을 한 목록에 섞는다. `Find network`는 실제 NIC/route 상태 없이 정적 URL 후보만
추가한다. 후보 Assign은 endpoint text를 바꾸지만 MAVLink 후보를 즉시 저장하지 않는다.

## Observed backend and persistence behavior

`PixhawkHostIpDiscoveryWorker`는 이미 주소가 붙은 interface만 열거한다. Connect는 configured
URL을 열고 peer를 seed하지만 특정 NIC 배정이나 `ip route get` 검증은 없다. Settings에는
side별 endpoint/peer는 있으나 side별 interface/local address는 없다.

## Documentation, code, test, and runtime conflicts

README의 field-qualified 구성은 camera/Pixhawk NIC 분리를 요구하지만 사용자는 같은 NIC도
경고 없이 선택 가능하도록 승인했다. 새 UI는 topology를 차단하지 않되 기존 shared-NIC stall
field evidence를 문서에서 삭제하거나 안정성이 검증됐다고 표현하지 않는다.
