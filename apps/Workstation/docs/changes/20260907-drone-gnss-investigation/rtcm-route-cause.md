# RTCM 대상별 수신 누락 원인 분리

송신 상한을 높이지 않고, 최소 P1/P2 stream 상태의 남은 수신 누락을 분리한다.
Radio 안테나 배치 질문에 사용자는 P1/P2가 거의 붙어 있고 Drone은 약 1.5m
떨어져 있다고 답했다. 후속 설명에서는 P1 Pixhawk·GPS 안테나·텔레메트리를 함께
이동했다고 정정했다. PC 측/기체 측 중 어느 끝단의 배치인지 독립적으로 확인하지
못했으므로 PC 측 간섭으로 위치를 단정하지 않는다. 실내 GNSS fix 품질은 판정에 쓰지 않는다.

## 사전 시험 계약

- 동일 앱/포트/baud/MAV_0_RATE와 P1/P2 최소 stream을 유지한다.
- 자동 Shell/PARAM/stream 확대 및 자동 rate 재요청은 중지한 상태를 유지한다.
- NTRIP 연결은 유지하며 RTCM 주입 대상만 바꾼다.
- NONE1 → P1_ONLY → P2_ONLY → DRONE_ONLY → ALL → NONE2,
  각 전환 뒤 10초 제외, 각 측정 약 60초. 완료 후 세 장치 주입으로 복원한다.
- 각 구간의 실제 GPS_RTCM_DATA TX 대상/bytes를 검증한다. 비대상 link에는
  GCS HEARTBEAT 등 기존 연결 유지 메시지만 남는다.
- 기존 serial connection의 원래 recv bytes를 별도로 기록하고 parser RX 및
  GUI callback과 비교한다. 별도 serial reader를 열지 않는다.
- ATTITUDE/GPS Hz, 도착 간격, source 시각, MAVLink sequence 공백,
  RADIO_STATUS rxerrors/txbuf/RSSI를 비교한다.

## 판정 방향

- 자기 link 주입 시에만 악화되면 해당 link의 양방향 처리/flow control/FC 입력 경로를 우선 확인한다.
- 다른 link에만 주입해도 악화되면 여러 Radio에 공통인 환경을 우선 확인한다.
  이 결과만으로 RF와 USB 등 물리 경계를 단정하지 않고 추가 시험으로 구분한다.
- 원래 serial bytes를 독립 decoder로 읽은 결과와 앱 parser 수신이 일치하면,
  GUI 표시나 앱 이후 처리에서 빠진 것으로 해석하지 않는다.
- NONE1/NONE2 차이도 확인해 시간 변화에 따른 link 변동을 과도하게 원인으로 단정하지 않는다.

Coordinate contract checked: ATTITUDE body-FRD→NED, GPS WGS84/MSL 값은 변경하지 않는다.
RTCM 대상과 수신 빈도/byte/sequence만 비교한다. 영구 parameter/앱 코드 변경은 없다.

## 현재 배치 실측 결과

| RTCM 대상 | P1 ATTITUDE | P2 ATTITUDE | Drone GPS | P2 순번 공백 추정 |
| --- | ---: | ---: | ---: | ---: |
| 없음 — 시작 기준 | 4.41Hz | 4.23Hz | 2.03Hz | 15.6% |
| P1만 | 4.00Hz | 3.03Hz | 1.93Hz | 37.8% |
| P2만 | 4.23Hz | 4.82Hz | 1.95Hz | 4.6% |
| Drone만 | 4.05Hz | 4.33Hz | 2.02Hz | 12.2% |
| 세 장치 모두 | 3.95Hz | 3.83Hz | 1.98Hz | 21.0% |
| 없음 — 종료 기준 | 3.64Hz | 4.97Hz | 1.95Hz | 1.4% |

각 구간 실제 길이는 60~61초이며 실제 길이로 Hz를 계산했다. NTRIP은 모든 구간에서
STREAMING이었다. 주입 대상의 RTCM wire TX는 약 1.127~1.159kB/s였고, 비대상의
GPS_RTCM_DATA TX는 0이었다. 측정 구간에는 PARAM/stream/Shell 명령 전송이 없었다.

## 확인된 범위

- P2는 자신에게만 RTCM을 주입할 때 약 4.82Hz를 받았다. 반면 P1에만 주입할 때
  P2 자체의 RTCM 송신은 0인데도 약 3.03Hz로 낮아졌다. 따라서 P2의 자기 link
  양방향 부하나 FC의 RTCM 처리만으로 이 저하를 설명할 수 없다.
- P1_ONLY 구간 P2 Radio rxerrors 증가는 98이었고, P2_ONLY 구간에는 8이었다.
  이 값은 modem의 누적 counter 차이이며 원인 위치를 단독으로 확정하지 않는다.
- 원래 serial read bytes를 별도 common MAVLink decoder에 공급한 ATTITUDE/GPS 개수와
  앱 parser 수신 개수가 6구간×3장치×2메시지 = 36개 비교에서 모두 일치했다.
  부족한 메시지가 GUI 표시 이후 처리에서만 빠진 현상으로는 설명되지 않는다.
- 시작/종료 NONE 기준에서도 P1/P2 수신률이 달랐다. 특히 P1은 RTCM 없이도
  4.41→3.64Hz로 변했으므로, 모든 저하를 단일 원인으로 단정하지 않는다.

## 현재 해석과 남은 검증

**P1 측 송신이 P2 수신에 영향을 주는 현상이 확인됐으며, P1/P2 Radio의 근접 배치가
유력한 원인 후보이다.** PC 측/기체 측 중 어느 물리 경계인지 아직 확정하지 않았다.
거리 변경 후의 유효한 A/B는 아직 완료하지 않았다.
여러 Radio에 공통인 USB/전원 등의 영향까지 완전히 배제한 것은 아니다.

RFDesign의 모뎀 기술 자료도 근접한 복수 modem 운용에서 안테나 간격과 송신 출력에
따라 데이터 유실이 달라질 수 있음을 설명한다. 이 자료는 원리상의 참고이며,
현재 장치의 정확한 모델/출력/채널 구성을 읽은 증거나 특정 거리의 성능 보장은 아니다.
[RFDesign Modem Support FAQs, 7~8쪽](https://files.rfdesign.com.au/Files/documents/Modem%20Support%20FAQs%20V1.2.pdf#page=7).

계획은 PC 측 P2 Radio/안테나만 분리하는 것이었지만, 실제 사용자 설명은 P1 본체·GPS·
텔레메트리를 함께 이동한 것이다. 후속 비교는 'P1 이동 배치'로 기록하며, Radio 한 끝단의
거리만 바꾼 시험으로 해석하지 않는다. 자체 link 경로/안테나 방향 변화도 구분해야 한다.

이동 직후 P2 전원이 빠졌다는 사용자 확인과 P2 GPS source timestamp 초기화를 통해
재부팅을 확인했다. 그 구간은 비교에서 제외한다. 재부팅으로 기본 stream이 복원됐으므로
P2 최소 stream을 재적용하고 세 link 및 RTCM을 확인한 뒤 새 구간을 측정한다.

## 증거와 복원

Workstation data root: `diagnostics/20260908-rtcm-route-cause-near/`.
원래 serial chunks, parser RX/TX, 앱 callback, snapshot, phase marker, 분석 JSON,
실행한 관찰기/분석기와 SHA-256 manifest를 보관했다.

현재 배치의 6구간 실험은 완료했고, RTCM 주입 대상은 P1/P2/Drone 모두로 복원했다.
P1/P2 최소 stream과 실험용 자동 polling/rate 변경 제한은 유지한다.
원인 확정 상태: 거리 변경 검증 대기.
