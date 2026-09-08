# P2 재연결과 P1/P2 최소 스트림·RTCM 검증

## 결과

P2의 sysid/compid 2/1 heartbeat, ATTITUDE와 GPS가 다시 수신됐고 세 장치 RTCM 전달이
재개됐다. P2에도 불필요한 기본 스트림을 실제 중지하여 수신률을 개선했다.
**RTCM을 함께 주입할 때 P1/P2의 실수신 5Hz를 안정적으로 보장하는 단계까지는
완료되지 않았다.**

## 설정과 읽기 증거

- P2의 live PARAM_VALUE에서 MAV_0_MODE=0, MAV_0_RATE=1200, SER_TEL1_BAUD=57600을 확인했다.
  일부 다른 parameter 읽기는 timeout으로 None이므로 값이 확인됐다고 주장하지 않는다.
- P2 `mavlink status`에서도 Radio instance #0, `/dev/ttyS6 @57600`, max 1200 B/s를 확인했다.
- P1은 이전 단계에서 22개 불필요 stream을 중지한 상태였다. P2의 실제 interval을
  확보한 뒤 나머지 19개 periodic stream을 중지했다.
- HEARTBEAT, ATTITUDE, GPS_RAW_INT/GPS2_RAW, SYS_STATUS, BATTERY_STATUS는 유지했다.
  ATTITUDE/GPS의 nominal interval은 200000us(5Hz)를 유지했다.
- 각 중지 명령 ACK와 interval readback을 확인했다. PX4는 제거된 stream에 -1,
  interval 0으로 존재하는 비활성 stream에 0을 반환할 수 있으므로, 후자는 실제
  주기 송신이 사라지는지 수신 기록으로 함께 확인했다.
- 관리 조회 응답이 누락돼 설정 읽기/적용 구간에는 RTCM을 일시 정지했다. OFF에서도
  일부 응답은 재확인이 필요했다. 이후 성능 측정은 별도의 무관리 명령 구간으로 나눴다.
- firmware parameter 저장, 재부팅, 앱의 영구 소스 변경은 하지 않았다.

## 이 기록에서 말하는 '최소 스트림'의 정확한 정의

P1/P2의 위치·자세 수신과 기본 연결/상태 확인에 필요한 주기 메시지를 유지하고,
읽기에서 활성화된 것으로 확인된 나머지 주기 메시지를 중지한 **이 시험의 임시 구성명**이다.
`MAV_0_MODE=0 (Normal)`, `MAV_0_RATE=1200 B/s`, baud=57600을 유지한 상태에서
메시지별 interval만 변경했다. PX4의 `minimal` 모드로 전환한 설정을 뜻하지 않는다.

### 유지한 Pixhawk → PC 메시지

| 메시지 | ID | 유지한 nominal 주기 | 용도 |
| --- | ---: | --- | --- |
| HEARTBEAT | 0 | 기존 기본 약 1Hz 유지; SET_MESSAGE_INTERVAL로 변경하지 않음 | 장치 식별·연결 확인 |
| ATTITUDE | 30 | 200000us = 5Hz | P1/P2 roll/pitch/yaw와 각속도 |
| GPS_RAW_INT | 24 | 200000us = 5Hz | GPS1 위경도·MSL 고도·fix·위성 수·정확도 |
| GPS2_RAW | 124 | 200000us = 5Hz | GPS2를 선택하는 기존 경로 보존 |
| SYS_STATUS | 1 | 1000000us = 1Hz | 기본 센서·시스템 상태 |
| BATTERY_STATUS | 147 | 1000000us = 1Hz | 배터리 상태 |

위 표는 설정/요청 주기다. 실제 도착 Hz는 아래 실측 표로 구분한다. GPS2나 배터리
등은 해당 producer가 메시지를 생성하지 않으면, 유지한 interval이 있어도 수신되지 않는다.

### 중지한 주기 메시지

| 메시지 | ID | P1 | P2 |
| --- | ---: | --- | --- |
| PING | 4 | 중지 | 중지 |
| LINK_NODE_STATUS | 8 | 중지 | 중지 |
| SCALED_IMU | 26 | 중지 | 읽기 시 이미 비활성 |
| SCALED_PRESSURE | 29 | 중지 | 중지 |
| ATTITUDE_QUATERNION | 31 | 중지 | 중지 |
| LOCAL_POSITION_NED | 32 | 중지 | 중지 |
| SERVO_OUTPUT_RAW | 36 | 중지 | 중지 |
| VFR_HUD | 74 | 중지 | 중지 |
| ATTITUDE_TARGET | 83 | 중지 | 중지 |
| POSITION_TARGET_LOCAL_NED | 85 | 중지 | 중지 |
| SCALED_IMU2 | 116 | 중지 | 읽기 시 이미 비활성 |
| SCALED_IMU3 | 129 | 중지 | 읽기 시 이미 비활성 |
| ALTITUDE | 141 | 중지 | 중지 |
| ESTIMATOR_STATUS | 230 | 중지 | 중지 |
| VIBRATION | 241 | 중지 | 중지 |
| HOME_POSITION | 242 | 중지 | 중지 |
| EXTENDED_SYS_STATE | 245 | 중지 | 중지 |
| TIME_ESTIMATE_TO_TARGET | 380 | 중지 | 중지 |
| CURRENT_MODE | 436 | 중지 | 중지 |
| ID 441 (관찰기의 UNKNOWN_441 표기) | 441 | 중지 | 중지 |
| OPEN_DRONE_ID_LOCATION | 12901 | 중지 | 중지 |
| OPEN_DRONE_ID_SYSTEM | 12904 | 중지 | 중지 |

총 P1 22개, P2 19개를 중지했다. P2의 IMU 3종은 이번 읽기 시 이미 비활성이어서
중지 개수에 포함하지 않았다. 각 이전 interval은 원본 JSON에 보관했다.
GPS_RTK(127)/GPS2_RTK(128)는 읽기 시 이미 비활성이었다.

중지에는 `MAV_CMD_SET_MESSAGE_INTERVAL`(511)의 `param1=message ID`,
`param2=-1`을 사용했다. ACK accepted와 `MAV_CMD_GET_MESSAGE_INTERVAL`(510) 결과를
확인했으며, 비활성 readback -1/0은 실제 주기 메시지의 수신 소멸로 교차 확인했다.

### 같이 유지하거나 제한한 동작

- PC → 각 Pixhawk의 GCS HEARTBEAT 약 1Hz는 유지했다.
- PC → 각 Pixhawk의 GPS_RTCM_DATA(233)는 별도 방향의 보정 주입이다. RTCM ON 측정에서
  세 장치에 계속 전달했으며, 주입 빈도는 수신 RTCM에 따랐다.
- 주기 stream 설정과 별도로 발생하는 COMMAND_ACK, 미션/이벤트 메시지, Radio 자체
  상태는 수신할 수 있다. 실제 최소 구성 기록에도 MISSION_CURRENT(42),
  CURRENT_EVENT_SEQUENCE(411), Radio의 RADIO_STATUS(109)가 포함됐다.
- 시험 관찰기에서는 자동 Shell polling, 자동 PARAM 조회, 자동 Radio tier 확대,
  누락 ATTITUDE/IMU에 대한 자동 rate 재요청을 일시 중지해 설정을 고정했다.
- 위 22/19개 중지 구성의 적용 대상은 P1/P2다. Drone은 이 프로파일의 비교 적용 대상에
  포함하지 않았으며, 세 장치 RTCM 동시 주입 조건에는 포함했다.
- 현재 적용은 runtime interval 변경이며 앱의 일반 시작/재연결 경로에 영구 구현된
  프로파일이 아니다. Pixhawk 재부팅이나 다른 프로그램의 stream 재설정으로 달라질 수 있다.

## P2 최소화 전/후 — RTCM ON

전 구간은 P1 최소/P2 기존 구성 60초, 후 구간은 양쪽 최소 구성 120초다.
둘 다 NTRIP STREAMING이고 Shell/PARAM/stream 변경 명령은 측정 구간에 없었다.

| 항목 | P2 최소화 전 | 양쪽 최소화 후 |
| --- | ---: | ---: |
| P1 ATTITUDE | 3.90Hz | 3.78Hz |
| P1 GPS_RAW_INT | 3.87Hz | 3.76Hz |
| P2 ATTITUDE | 1.92Hz | 3.67Hz |
| P2 GPS_RAW_INT | 1.92Hz | 3.70Hz |

후 구간에서 ATTITUDE 도착 간격 p95/최대는 P1 0.619/1.181초,
P2 0.600/1.415초였다. Source ATTITUDE timestamp 간격 중앙값은 양쪽 약 0.203초였다.
세 장치 각각 RTCM wire 송신 약 1.172kB/s, 전달 실패 증가 0을 확인했다.

P2의 후속 `mavlink status` Radio 구간에는 `tx rate mult: 1.000`, `tx: 583.4 B/s`,
`txerr: 0.0 B/s`, `tx rate max: 1200 B/s`, `rx: 1111.9 B/s`가 기록됐다.
이는 status를 읽은 시점의 값이며 전체 측정 구간의 연속 송신 계측은 아니다.

## 남은 수신 공백을 위한 고정 profile RTCM ON/OFF/ON

최소 profile을 그대로 유지하고 RTCM만 OFF 후 ON으로 되돌렸다.
OFF는 전환 뒤 10초를 제외한 약 69초, ON 복원 후는 60초였다.

| 항목 | ON1 120초 | OFF 약 69초 | ON2 60초 |
| --- | ---: | ---: | ---: |
| P1 ATTITUDE | 3.78Hz | 4.32Hz | 3.45Hz |
| P1 GPS | 3.76Hz | 4.42Hz | 3.32Hz |
| P2 ATTITUDE | 3.67Hz | 4.58Hz | 4.00Hz |
| P2 GPS | 3.70Hz | 4.52Hz | 4.02Hz |
| P1 MAVLink 순번 공백 추정 | 24.9% | 12.9% | 32.4% |
| P2 MAVLink 순번 공백 추정 | 25.0% | 9.3% | 19.0% |

각 ON 구간에서 RTCM은 장치당 약 1.139~1.172kB/s로 계속 전달됐고 앱 전달 실패 증가는
0이었다. OFF는 실제 GPS_RTCM_DATA TX가 0이었다. 세 구간에 관리 명령 전송은 없었다.

순번 공백은 같은 sysid/compid의 실제 MAVLink2 header sequence에서
`(next-prev-1) mod 256`으로 계산했다. 큰 역행/재정렬 step은 없었다. 이는 수신 측
순번 공백 추정치이며, 유실이 FC 송신 큐·무선 modem·PC 중 정확히 어디에서 일어났는지
단독으로 확정하는 계측은 아니다. ON1의 지상 Radio rxerrors는 P1 6809→6931,
P2 6285→6427로 증가했다.

해석: 불필요 stream 정리는 P2 수신률을 크게 개선했다. 그래도 동일 최소 profile에서
RTCM 주입 시 양쪽 수신률이 낮아지고 순번 공백이 증가하는 현상이 남았다.
RTCM 동시 송신 부하와 Radio 경로의 영향을 추가 분리해야 한다. Radio air rate,
실제 양방향 처리량, 주입 burst/배치, 장치별 링크 상태를 읽고 검증하는 것이 다음 단계다.
현재 증거로 특정 RF 장치나 간섭원을 단정하지 않는다.

## 현재 적용 상태

P1/P2에 최소 stream을 임시 유지하고 세 장치 NTRIP RTCM을 다시 켜 두었다.
실험 관찰기에서는 자동 stream 확대/자동 Shell/자동 PARAM 조회가 중지된 상태다.
따라서 일반 실행 경로에 영구 반영된 최적화가 아니다.

P2 전체 원래 interval은 `right-original-intervals-complete.json`에 보존했고,
canonical `right-original-intervals.json`도 전체 원본으로 복원해 두었다. 현재 관찰기의
P2 in-memory restore map은 마지막 남은 stream 처리에 사용한 부분집합이므로,
전체 복원 시 전체 원본을 다시 로드한 helper를 사용해야 한다.

증거: Workstation data root `diagnostics/20260908-p1-p2-minimal-rtcm/`.
설정 readback, 실제 wire, snapshot, 통계, 최종 연결 상태와 관찰기 사본을 보관했다.

Coordinate contract checked: ATTITUDE body-FRD→NED, GPS WGS84/MSL 값/단위를 유지했다.
위치 정확도 평가는 이번 실내 통신 시험의 판정 범위에서 제외했다.
