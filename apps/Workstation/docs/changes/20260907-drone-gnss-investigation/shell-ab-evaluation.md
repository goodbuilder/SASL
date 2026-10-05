# P1/P2 Shell 조회 영향 분리 시험

## 시험 계약

사용자 요청: P1/P2의 낮은 ATTITUDE 수신률에서 자력계 Shell 조회가 차지하는 영향을
분리한다. 실내 시험의 GNSS fix 품질을 이번 평가의 원인으로 사용하지 않는다.

- 동일 Workstation, 동일 P1/P2/Drone Radio 연결과 baud를 사용한다.
- NTRIP RTCM은 네 구간 내내 세 장치에 계속 전달한다.
- 자동 Radio tier 변경, 누락 IMU/ATTITUDE rate 재요청, 별도 PARAM 조회를 시험 동안
  일시 중지한다. 초기 CRITICAL의 ATTITUDE/GPS 요청률은 5Hz다. 기존 PX4 기본/optional
  stream을 이번 시험에서 새로 정리하지 않으며, 측정 구간 중 stream 설정 요청이
  추가로 전송되지 않는지 wire 기록으로 검증한다.
- A1 ON → B1 OFF → A2 ON → B2 OFF, 각 60초. 전환 후 최소 15초를 제외하며,
  OFF 구간은 이미 실행 중인 Shell worker까지 모두 종료된 뒤 시작한다.
- 전송/수신 관찰은 기존 connection의 send/recv_msg를 감싼다. 별도 serial reader는
  열지 않는다. GUI callback 기록과 실제 parser 수신 기록을 따로 비교한다.
- 각 구간의 ATTITUDE/GPS Hz, 도착 간격 p95/최대, source timestamp 간격,
  Shell 실제 byte 수, RTCM bytes/실패, stream 재설정 유무를 비교한다.
- OFF 개선이 두 반복에서 같은 방향인지 확인한다. 25% 이상의 Hz 증가 또는 p95
  공백 감소를 반복하면 실질적 영향으로 분류한다. 그보다 작은 차이나 방향 불일치는
  개선을 단정하지 않는다. Shell OFF가 요청 5Hz를 회복하는지도 별도로 판정한다.
- 시험 후 원래 Shell timer/자동 정책/파라미터 조회 방법을 복원한다.

Coordinate contract checked: ATTITUDE body-FRD→NED, GPS WGS84/MSL의 값/축은 변경하지
않는다. host monotonic 도착 간격과 원래 time_boot_ms/time_usec를 구분하여 분석한다.

## 검증 slice

| Slice | 독립 증거 | 상태 |
| --- | --- | --- |
| 연결/RTCM 유지 | sysid 및 각 RTCM TX counter/wire | PASS |
| 단일 변수 | phase marker, OFF SERIAL_CONTROL=0, 측정 중 stream/PARAM 요청=0 | PASS |
| 수신률/지연 | parser RX와 앱 callback, source timestamp | PASS |
| 반복성 | A1/B1/A2/B2 비교 | PASS (P1 부분 개선, P2 일관된 개선 없음) |
| 복원 | 완료 marker, timer와 RTCM 상태 | PASS |

이 시험은 영구 앱 소스/펌웨어 설정 변경이 아니다.

## 결과

**Shell 조회만 꺼서는 P1/P2의 낮은 수신률이 해결되지 않았다.**
P1에는 부하 감소 효과가 있었지만 P2에서는 같은 개선이 재현되지 않았다.

| 실제 parser 수신률 | A1 ON 60초 | B1 OFF 61초 | A2 ON 61초 | B2 OFF 61초 |
| --- | ---: | ---: | ---: | ---: |
| P1 ATTITUDE | 1.73 Hz | 2.07 Hz | 1.48 Hz | 1.93 Hz |
| P2 ATTITUDE | 2.00 Hz | 2.02 Hz | 1.77 Hz | 1.72 Hz |
| P1 GPS_RAW_INT | 1.75 Hz | 1.92 Hz | 1.70 Hz | 1.82 Hz |
| P2 GPS_RAW_INT | 2.03 Hz | 1.98 Hz | 1.62 Hz | 1.72 Hz |
| Drone GPS_RAW_INT (참조) | 1.30 Hz | 1.44 Hz | 1.33 Hz | 1.59 Hz |

Qt timer 전환에 따라 실제 측정 구간은 60~61초였다. Hz는 실제 경과 시간으로 계산했다.
ON/OFF 구간을 각각 합쳐 시간 가중 평균을 내면 P1 ATTITUDE는 1.603→2.000Hz
(+24.75%), P2는 1.884→1.869Hz(-0.81%)였다. P1의 두 OFF 개선은 약 +19%와 +31%로,
사전 기준인 두 반복 모두 25% 이상의 Hz 개선에는 미달했다. P2에는 일관된 개선이 없다.
두 장치 모두 요청 5Hz 회복은 FAIL이다. 짧은 실내 단일 배치 시험이며 실제 RF 환경까지
불변으로 만들지는 못했으므로, 작은 차이를 보편적인 효과 크기로 일반화하지 않는다.

| ATTITUDE parser 도착 공백 p95 / 최대 | A1 ON | B1 OFF | A2 ON | B2 OFF |
| --- | --- | --- | --- | --- |
| P1 | 0.954 / 1.388s | 0.893 / 1.376s | 1.389 / 2.003s | 0.913 / 1.767s |
| P2 | 0.928 / 1.336s | 0.952 / 1.272s | 1.063 / 1.397s | 1.031 / 1.422s |

## 분리 조건과 추가 근거

- 모든 측정 snapshot에서 NTRIP=STREAMING이었다. 각 장치의 RTCM wire TX는
  약 1.197~1.219 kB/s로 유지됐고 전달 실패 증가 0이었다.
- 모든 구간에 MAV_CMD_SET_MESSAGE_INTERVAL=0회, PARAM 요청=0회,
  그 외 COMMAND_LONG=0회였다. HEARTBEAT와 RTCM은 계속 송신됐다.
- OFF 구간 P1/P2의 SERIAL_CONTROL 송신과 수신은 모두 0이었다. ON의 Shell 수신은
  약 118~183 B/s, 관찰된 전체 MAVLink 수신 bytes의 약 12.5~19.4%였다.
  이전에 말한 '조회 작업 80% 활성'은 무선 대역폭 80% 점유라는 뜻이 아니다.
- parser 수신부터 앱 callback까지의 p95 지연은 Shell ON에서 약 22~46ms,
  OFF에서 약 0.28~0.42ms였다. 조회를 꺼서 전달 지연은 개선됐지만, OFF의 parser
  자체 수신률부터 약 2Hz였으므로 GUI 지연만으로 낮은 Hz를 설명할 수 없다.
- 실제 ATTITUDE/GPS wire bytes 2,472개를 별도 common MAVLink decoder로 다시
  파싱하여 CRC, source identity와 source timestamp를 검증했다.
- 완료 후 Shell timer와 자동 정책, PARAM 조회 방법을 원복했다. 실제
  RM3100 SHELL POLL 재개와 NTRIP STREAMING, 세 RTCM counter 증가를 확인했다.

## 다음 해결 대상

Shell OFF에서도 ATTITUDE_QUATERNION, SCALED_IMU 3종, LOCAL_POSITION_NED,
ATTITUDE_TARGET, SERVO_OUTPUT_RAW, OPEN_DRONE_ID 등 여러 기본 stream이 계속 수신됐다.
P1/P2의 2026-09-06 parameter 보관본에는 MAV_0_RATE=1200 B/s와 MAV_0_MODE=0이
있다. **이는 보관본 값이며 이번 시험의 live parameter readback은 아니다.**
현재 송신 instance/rate와 실제 stream 목록을 조회하고, 필수 ATTITUDE/GNSS를
우선하는 고정 profile로 불필요 stream을 실제 중지한 뒤 5Hz 회복을 검증하는 것이
다음 단계다. 전송률 상한과 많은 기본 stream이 남아 있는 상황을 우선 조사해야 한다.

기록: Workstation data root `diagnostics/20260907-shell-poll-ab/`의
`analysis.json`, `transport.jsonl`, `mavlink.jsonl`, `snapshots.jsonl`, `events.jsonl`,
`manifest.json`. 실행한 임시 관찰기/분석기 사본도 함께 보관했다.
