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

## 2026-08-29 Network-role amendment implementation

The current source keeps Find read-only but makes confirmed Assign a role-scoped NetworkManager
transaction. Runtime-discovered NICs carry composable Camera/P1/P2 sets; target readback precedes
source cleanup, exact Workstation-owned entries are persisted, and ordinary active affected owners
stop/reconnect automatically. Mapping-locked Measurement/recording remains unchanged. Automated,
launcher, read-only real-NM and visual evidence pass; real mutating device moves remain unverified.

## 2026-08-29 Real multi-NIC field finding

The first mutating field run exposed a contract mismatch rather than a NetworkManager write
failure. After the OWC RTL8156B appeared as `enx0023a40a2e39`, Camera traffic was physically on
`enx588694fce07e` and P1 traffic was physically on the new NIC. P2 was temporarily silent until a
USB-triggered PX4 boot, after which it broadcast `10.0.0.3:14551` normally.

Moving one Pixhawk role marked every open Camera/P1/P2 owner on the source and target interfaces as
affected. The coordinator then required fresh device evidence from that whole set before committing
the host assignment. A silent P2 therefore caused `device evidence timeout on selected NIC`, locked
both Assign buttons during the operation, and produced `ROLLBACK PARTIAL` even though the selected
host address/route delta itself was valid. This prevents an operator from intentionally assigning an
offline or not-yet-powered device to its desired NIC.

With Workstation stopped, the same role-scoped NetworkManager transaction was applied and read back
successfully: Camera `.231/.232` routes use `enx588694fce07e` with source `.11`, while P1 `.2` and P2
`.3` use `enx0023a40a2e39` with source `.10`. A fresh Workstation launch then reached expected-system
P1/P2 `LIVE`; after USB removal P2 rebooted, completed the visible 60-second qualification, and
returned to `LIVE` without a USB device present. The remaining issue is assignment/reconnect policy,
not the final host topology or PX4 Ethernet parameters.
