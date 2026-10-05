# Change Brief

## Operator outcome

`Devices > Telemetry`에서 `Find Network`를 눌러 현재 유선망에서 실제 MAVLink를
보내는 Drone Pixhawk(`sysid=3`)를 찾고, 검증된 UDP URL을 선택해 기존
`Apply -> Save -> Connect`로 연결한다. 성공은 소켓 생성이 아니라 실제
`sysid=3` HEARTBEAT/ATTITUDE/GNSS 수신으로 판정한다.

## Motivation

현재 Telemetry는 USB 탐색만 제공해, Ethernet Drone이 정상 송신 중이어도
예전 USB 경로가 남아 `heartbeat=WAITING`으로 보이는 문제를 UI에서 해결하기
어렵다. 2026-09-02 실장 패킷에서 `10.0.0.3:14551`, `sysid=3`, 5초
HEARTBEAT 5개가 독립 확인됐다.

## Included scope

- Telemetry 지역 `Find Network` 명령과 후보 표시/선택.
- 기존 공용 Pixhawk 네트워크/수신 소유자를 통한 비동기, 읽기 전용 탐색.
- `sysid=3`이 검증된 후보만 Drone 선택 가능.
- 선택은 URL만 채우며 Apply/Save/Connect는 기존 명시적 명령으로 보존.
- Apply/Save 완료 후 현재 폼의 MAVLink URL이 이전 런타임/저장값으로
  되돌려지지 않고, 실제 적용·저장된 값으로 유지된다.
- 설정 재시작 복원, 연결/해제, 실패/타임아웃, 기존 USB 경로 회귀.
- 현재 실장 Drone을 검색·선택·연결하고 실제 수신을 확인.

## Excluded scope and non-goals

- PX4 IP/파라미터/SD 파일을 자동으로 변경하지 않는다.
- NetworkManager 주소/라우트, P1/P2 배정, 카메라 네트워크를 변경하지 않는다.
- 포트를 자동 추측해 미검증 주소를 저장하지 않는다.
- 탐색 즉시 자동 연결하거나 기존 연결을 임의로 끊지 않는다.

## Acceptance summary

1. 현재 실장 `sysid=3` 송신자가 사실적 IP/UDP 포트와 함께 표시된다.
2. 후보 선택은 `MAVLink URL`만 채우고 다른 설정/연결은 변경하지 않는다.
3. Connect 후 `sysid=3` HEARTBEAT와 Drone GNSS가 UI에 표시된다.
4. `sysid=1/2`, 조용한 링크, 중복 후보, 포트 점유는 실패 사실과 회복 안내를 남긴다.
5. Apply/Save 직후와 주기적 refresh 후에도 새 URL과 포커스가 유지된다.
