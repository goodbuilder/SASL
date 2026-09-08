# P1 실기 송신 설정과 최소 스트림 예비 검증

## 결론과 범위

P1에서 많은 기본 스트림이 남아 있는 상태의 낮은 수신률을, 송신 상한을 올리지 않고
불필요 스트림을 실제 중지하여 개선했다. P2는 이번 재연결 후 원격 Radio 응답이 없었다.
따라서 **P2 및 RTCM 동시 주입 조건의 검증은 완료되지 않았다**.

기존 설정의 NTRIP GGA는 P1/P2 중앙 위치를 요구한다. P2가 없으므로 NTRIP은 연결되지만
RTCM stream이 나오지 않았다. 아래 전/후 구간 모두 RTCM 전달은 0이며, 앞선 RTCM ON
Shell 시험과 조건이 다르다. 사용자에게 P2 전원을 확인해 달라는 질문을 보냈다.

## 실제 읽기 결과

P1 PARAM_VALUE를 통해 확인:

- MAV_0_CONFIG=101, MAV_0_MODE=0 (Normal), MAV_0_RATE=1200 B/s.
- MAV_0_RADIO_CTL=1, SER_TEL1_BAUD=57600.
- MAV_1_CONFIG=0, MAV_2_CONFIG=1000, MAV_2_RATE=2000000.

`mavlink status`로 instance #0이 `/dev/ttyS6 @57600`인 Radio 링크이며
`tx rate max: 1200 B/s`임을 교차 확인했다.

MAV_CMD_GET_MESSAGE_INTERVAL로 실제 stream interval을 읽었다. ATTITUDE/GPS_RAW_INT는
각 200000us(5Hz)였지만 ATTITUDE_QUATERNION 10Hz, SCALED_IMU/2/3 각각 10Hz,
LOCAL_POSITION_NED 5Hz와 여러 상태/목표 메시지도 활성 상태였다.

## 임시 변경

P1의 HEARTBEAT, SYS_STATUS, GPS_RAW_INT, GPS2_RAW, ATTITUDE, BATTERY_STATUS는 유지했다.
읽기에서 양수 interval로 확인된 나머지 22개 stream에 MAV_CMD_SET_MESSAGE_INTERVAL의
interval=-1을 적용하고, 각 ACK accepted와 GET_MESSAGE_INTERVAL=-1을 확인했다.
기존 interval은 `left-original-intervals.json`에 남겨 복원할 수 있게 했다.

PARAM_SET/param save/재부팅은 실행하지 않았다. Shell 자동 polling, 자동 Radio tier 변경,
ATTITUDE/IMU rate 재요청, 자동 PARAM 조회를 시험 관찰기에서 일시 중지했다.

## 실측 — 각 60초

| P1 | 변경 전 | 최소 스트림 |
| --- | ---: | ---: |
| ATTITUDE 수신 | 1.77Hz (106개) | 4.48Hz (269개) |
| GPS_RAW_INT 수신 | 1.78Hz (107개) | 4.52Hz (271개) |
| ATTITUDE source timestamp 간격 중앙값 | 0.560s | 0.199s |
| GPS source timestamp 간격 중앙값 | 약 0.598s | 약 0.200s |
| 전체 MAVLink RX | 1072 B/s | 539.25 B/s |

최소 스트림 적용 후 `mavlink status`의 Radio instance에서 `tx rate mult: 1.000`,
`tx rate max: 1200 B/s`, `tx: 583.5 B/s`, `txerr: 0.0 B/s`를 읽었다.
일부 Shell 출력은 다른 instance 구간에서 누락됐으나 위 Radio 구간은 읽혔다.

약 5Hz 송신 간격으로 회복됐지만 호스트 실수신은 약 4.5Hz이고 최대 공백은 약 0.92초였다.
패킷 무손실이나 정확히 5Hz의 연속 수신을 입증한 것은 아니다.

## 현재 상태와 다음 검증

P1 최소 stream interval은 임시 적용 상태로 유지한다. 앱/펌웨어의 영구 코드는 수정하지
않았다. P2 전원/Radio 연결을 회복한 후 실제 P2 설정을 읽고, 세 장치 RTCM 주입이 다시
증가하는 조건에서 P1/P2 모두 같은 개선이 유지되는지 검증해야 한다.

증거는 Workstation data root `diagnostics/20260907-minimal-profile-p1/`에 보관했다.

Coordinate contract checked: ATTITUDE body-FRD→NED와 GPS WGS84/MSL 값/단위는 그대로
유지했고, 원래 timestamp와 host 도착 빈도를 비교했다.
