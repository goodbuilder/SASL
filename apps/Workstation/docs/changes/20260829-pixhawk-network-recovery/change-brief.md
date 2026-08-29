# Change Brief

## Operator outcome

Pixhawk 화면에서 `Find network`를 누르면 현재 유선 NIC, 로컬 IPv4, P1/P2 커널 route와
MAVLink endpoint를 한 표에서 확인하고, 선택 행을 `Assign P1` 또는 `Assign P2`로 저장한 뒤
`Connect`로 실제 연결을 시도한다. P1/P2는 같은 NIC 또는 서로 다른 NIC를 사용할 수 있고,
한 side 실패가 다른 side 연결을 해제하지 않는다.

## Motivation

현재 `Find network`는 NIC를 찾지 않고 고정 MAVLink URL 문자열만 만든다. NIC 교체로 `enx...`
이름이 바뀌거나 `10.0.0.2/.3` route가 Wi-Fi로 빠져도 후보와 실제 경로의 차이를 보여주지 못해
socket open이 장치 연결처럼 보일 수 있다.

## Included scope

- read-only 유선 NIC/IPv4/carrier/route discovery;
- operator-owned P1/P2 NIC+endpoint Assign와 마지막 배정 저장;
- Connect 직전 side별 interface/address/route preflight와 독립 partial connect;
- 기존 연결 side 재배정 시 그 side만 비동기 disconnect;
- Pixhawk와 Camera discovery 데이터의 화면별 분리;
- 기존 MAVLink liveness, expected sysid, 60초 qualification과 RTCM stall gate 보존.

## Excluded scope and non-goals

- IP/route/NetworkManager 프로필 생성·변경 또는 관리자 권한 요청;
- PX4 firmware, parameter, SD-card 설정의 자동 변경;
- Camera/Pixhawk 동일 NIC 배정 경고 또는 topology readiness gate;
- 좌표, GNSS, attitude, Measurement 계산 변경.

## Acceptance summary

`Find network -> Assign P1/P2 -> Connect`가 UI thread를 막지 않고 동작하고, kernel route와
배정이 일치한 side만 socket을 열며 expected sysid heartbeat/ATTITUDE가 기존 상태 모델을 통해
`QUALIFYING/LIVE`를 만든다. 잘못된 side는 사실적인 원인으로 실패하고 정상 side는 유지된다.
