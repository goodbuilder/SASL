# Radio 설정·출력 의존성 조사 — 2026-09-08

## 확인된 범위

최소 stream 이후의 낮은 Hz는 **송신 출력과 다중 link 송신에 민감한 Radio 구간 손실**로
좁혀졌다. 같은 RTCM 전송량에서 지상 Radio 출력 설정을 낮추면 수신이 개선됐고,
원래 출력으로 복원하면 다시 악화됐다. 현재 측정의 serial buffer overflow 증가나
GUI 이후 처리 누락으로 설명되지 않는다.

동일 대역 다중 Radio의 RF 간섭/수신 성능 저하가 유력하다. 다만 출력 변경은 전류
소모도 바꾸므로, 공유 USB 전원 영향을 독립적으로 분리한 것은 아니다. 물리적 메커니즘을
RF 간섭 하나로 완전히 확정했다고 주장하지 않는다.

## 배치 정정과 재시험

사용자는 P1 Pixhawk·GPS 안테나·텔레메트리를 함께 이동했다고 설명했다. PC 측/기체 측
중 어느 Radio 끝단인지 독립 확인되지 않아, 앞서 요청했던 'PC 측 P2 안테나만 분리'와
동일한 단일 변수 시험으로 해석하지 않았다. 이동 중 P2 전원 단절과 재부팅이 있었고,
그 구간은 제외했다. P2 기본 stream을 다시 최소화하고 연결을 확인한 뒤 재시험했다.

이동 배치에서도 P1_ONLY 시 P2 ATTITUDE는 3.30Hz, 시작/종료 NONE은 4.48/2.92Hz로
변동했다. 따라서 거리 변경만으로 원인이 해결됐다고 판정하지 않았다.
배치 재시험 증거는 data root `diagnostics/20260908-rtcm-route-cause-moved/`에 보관했다.

## 읽기 전용 Radio 설정 조회

앱의 serial 소유권을 해제한 뒤 command mode에서 ATI/ATI5 및 RTI/RTI5를 읽었다.
각 조회 후 ATO로 데이터 모드로 나왔고, 이후 MAVLink 수신을 확인했다.

| 로컬(PC 측) 설정 | P1 | P2 | Drone |
| --- | --- | --- | --- |
| 펌웨어 보고명 | RFD SiK 2.0 on HM-TRP | 동일 | 동일 |
| SERIAL_SPEED | 57 (57600 baud) | 57 | 57 |
| AIR_SPEED | 64 kbps | 64 kbps | 64 kbps |
| NETID | 25 | 26 | 27 |
| TXPOWER — 원래 설정 | 20 dBm | 20 dBm | 20 dBm |
| MIN_FREQ / MAX_FREQ | 915000 / 928000 kHz | 동일 | 동일 |
| NUM_CHANNELS | 50 | 50 | 50 |
| ECC / OPPRESEND | 0 / 0 | 0 / 0 | 0 / 0 |
| MAVLINK | 1 | 1 | 1 |
| DUTY_CYCLE / LBT_RSSI | 100 / 0 | 동일 | 동일 |
| RTSCTS / MAX_WINDOW | 0 / 131 | 동일 | 동일 |

P2/Drone 원격 펌웨어 보고명은 RFD SiK 2.0 on HM-TRP였다. P2 원격 frequency 응답의
누락 부분은 개별 RTS8?/RTS9? 조회에서 915000/928000으로 확인했다.
P1 원격은 SiK 2.2 on HM-TRP로 보고됐고, 여러 원격 숫자 응답이 불완전하거나
서로 모순됐다. 그 숫자를 유효한 설정으로 채택하거나 덮어쓰지 않았다. P1의 버전 차이는
별도 확인 대상이며, 이번 Hz 저하의 원인이라고 단독으로 확정하지 않았다.

## 출력 시험

변경 대상은 명확히 PC에 연결된 로컬 Radio의 TXPOWER였다. 주파수·AIR_SPEED·NETID·
MAV_0_RATE·stream 구성은 변경하지 않았다. ATS4로 일시 변경하고 ATS4? 숫자 응답을
확인했다. 요청 10은 11로 읽혔고, 요청 2와 복원 20은 각각 2와 20으로 확인됐다.
이는 Radio의 설정 readback이며 교정된 RF 출력계 측정값은 아니다.
AT&W는 실행하지 않았다.

| 출력 설정 / RTCM 대상 | 시간 | P1 ATT | P2 ATT | P2 순번 공백 추정 |
| --- | ---: | ---: | ---: | ---: |
| P1만 11 dBm / P1_ONLY | 90s | 3.81Hz | 3.16Hz | 38.5% |
| P1만 2 dBm / P1_ONLY | 90s | 4.69Hz | 4.29Hz | 13.9% |
| P1만 2 dBm / ALL | 60s | 3.48Hz | 3.33Hz | 31.3% |
| 지상 세 Radio 모두 2 dBm / ALL | 60s | 4.08Hz | 4.23Hz | 14.4% |
| 지상 세 Radio 모두 20 dBm 복원 / ALL | 60s | 3.60Hz | 2.58Hz | 48.7% |

핵심 ALL 비교의 장치별 RTCM wire TX는 2 dBm에서 1160.7 B/s,
20 dBm 복원에서 1160.3 B/s로 거의 같았다. 두 구간 모두 NTRIP STREAMING이고
측정 중 PARAM/stream/Shell 명령 전송은 없었다. 각각의 원래 serial bytes를 별도
common MAVLink decoder로 재생한 core 메시지 개수와 앱 parser 개수가 30개 비교에서
모두 일치했다. 아카이브된 raw bytes만으로 재생해도 같은 결과임을 검증했다.

설정 확인에 실패한 중간 시도는 성능 측정을 시작하지 않고 원복했다. 출력 비교를 위한
AT 모드 진입 때문에 구간 사이 serial/앱/NTRIP 재연결은 있었다. 무선 hopping 위상이나
외부 RF 환경까지 불변인 무작위 실험은 아니므로 정확한 효과 크기를 일반화하지 않는다.

## Radio 내부 오류 counter

ATI7에서 txe/rxe/stx/srx를 읽었다. SiK 구현에서 stx/srx는 serial TX/RX overflow,
rxe는 Radio 수신 오류 counter다.
[SiK counter 정의와 출력](https://github.com/ArduPilot/SiK/blob/master/Firmware/radio/tdm.c).

| 지상 Radio | 복원 실행 전 rxe → 후 rxe | stx 전→후 | srx 전→후 |
| --- | --- | --- | --- |
| P1 | 11747 → 12289 | 0 → 0 | 0 → 0 |
| P2 | 11057 → 11728 | 0 → 0 | 10464 → 10464 |
| Drone | 3938 → 4368 | 0 → 0 | 0 → 0 |

위 전후는 복원 실행의 초기화·대기·측정·종료를 포함하며, 60초 측정만의 counter 차이는
아니다. P2의 10464는 누적값으로, 이 실행에서 추가 serial RX overflow는 관찰되지 않았다.
별도로 RADIO_STATUS로 자른 60초 구간의 rxerrors 증가는 2→20 dBm 비교에서
P1 41→82, P2 49→88, Drone 4→60이었다. txe는 전후 모두 0이었다.

## 결론과 남은 분리

- 앞서 확인한 불필요 stream과 1200 B/s 송신 예산 문제는 최소 stream으로 개선됐다.
- 남은 손실은 원래 serial bytes에 도달하기 전부터 존재하고, 출력 및 다른 Radio의
  RTCM 송신에 영향을 받는다. 현재 측정에서는 새 serial overflow가 관찰되지 않았다.
- NETID가 서로 달라도 로컬 세 Radio는 같은 frequency 범위를 사용한다.
  동일 대역 복수 modem의 출력/배치가 손실에 영향을 줄 수 있다는 제조사 설명과
  관찰이 부합한다.
  [RFDesign 근접 modem 운용 설명](https://files.rfdesign.com.au/Files/documents/Modem%20Support%20FAQs%20V1.2.pdf#page=7).
- USB topology에서 세 FTDI는 같은 hub 아래에 있었다. 외부 전원 사용 여부를
  사용자에게 질문했으며, 전원만 독립 분리하는 시험은 아직 하지 않았다.
- 모든 조건에서 실수신 5Hz가 보장된 것은 아니다. 저출력의 장거리 운용 성능도
  이번 실내 시험 범위 밖이다. 이 결과는 수신 Hz 저하에 관한 것이며, 오후의
  동일 GNSS 좌표 반복 현상 전체를 같은 원인으로 확정하는 증거는 아니다.

## 복원과 기록

P1/P2/Drone 지상 Radio 모두 TXPOWER=20 readback과 데이터 모드 복귀를 확인했다.
앱을 다시 연결하고 세 장치 RTCM 주입을 복원했다. MAV_0_RATE는 올리지 않았고,
Radio/PX4 firmware를 교체하거나 Radio 설정을 EEPROM에 저장하지 않았다.
P1/P2 최소 stream과 실험 관찰기의 자동 polling/rate 변경 제한은 유지한다.

증거: Workstation data root `diagnostics/20260908-radio-power-cause/`.
설정 조회, 출력 readback/복원, 실제 serial chunks, RX/TX, snapshot, 분석 JSON,
counter와 SHA-256 manifest를 보관했다.

Coordinate contract checked: ATTITUDE body-FRD→NED, GPS WGS84/MSL 값·단위는 유지했다.
출력 설정, 송수신 bytes, 시각, 순번과 수신 빈도만 비교했다.
