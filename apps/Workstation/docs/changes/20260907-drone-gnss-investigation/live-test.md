# Drone 이동 실기 확인 — 2026-09-07

## 결과

이번 전원 투입 후 시험에서는 **Age만 갱신되고 좌표가 고정되는 증상이 재현되지 않았다**.
Drone의 실제 MAVLink 좌표와 source timestamp가 모두 변했고, 앱 상태에도 그대로 반영됐다.
다만 GNSS 품질이 나빠 정상적인 이동 측위가 확인됐다고 결론 내릴 수는 없다.
앞서 오후 기록의 동일 좌표 반복 원인이 해결됐다는 증거도 아니다.

## 시험 조건

- 실제 Workstation과 기존 연결/NTRIP/RTCM worker를 사용했다. 앱 소스와 장비 파라미터는 수정하지 않았다.
- P1/D30JNMVH=sysid 1, P2/DU0E49VU=sysid 2, Drone/DU0E4O0F=sysid 3,
  각 57,600 baud Radio 연결을 확인했다.
- 저장 설정은 P1/P2 RTCM만 활성화돼 있었고 `routing.drone_enabled=false`였다.
  사용자의 세 장치 주입 요청에 따라 시험 세션에는 Drone도 활성화했다.
- Drone이 나중에 켜져 연결되었으므로, 세 장치가 연결된 뒤 NTRIP을 재시작하여
  Drone까지 실제 전달되는 상태를 확인했다.
- 준비 시 Drone은 fix=4였다. 원시 패킷 bytes, decoded fields, 앱 snapshot,
  RTCM 전달 카운터를 별도 관찰기로 기록했다.
- PX4 `gps status` 응답 일부에서 Drone 내부 `rtcm_injection_rate: 6.99861`,
  `rtcm_crc_failed: False`를 확인했다. Radio Shell 응답에는 일부 누락이 있어
  전체 출력이 완전하다고 주장하지 않는다.

## 관찰 구간과 검증

22:49:06.445 ~ 22:51:07.446 KST, 약 121초: 이동 안내 전 기준값 저장부터 사용자의
"멈췄어" 보고 직후까지다. 실제 이동 시작 시각을 별도로 받지 않았으므로 이 전체가
연속 이동 시간이라는 뜻은 아니다.

| 항목 | 결과 |
| --- | --- |
| Drone GPS_RAW_INT 수신 | 181개, 전부 sysid/compid 3/1 |
| 서로 다른 위도 / 경도 / 고도 | 181 / 180 / 180개 |
| 서로 다른 원래 `time_usec` | 181개, 역행 0회 |
| 독립 MAVLink decoder 재검증 | 실제 wire bytes 181개 CRC/필드 일치 PASS |
| 앱 snapshot과 원시 좌표 대조 | 122개 일치 PASS |
| P1/P2/Drone RTCM 전달 증가 | 각각 701 frames, 각각 실패 증가 0 |
| Drone fix | fix=4 139개, fix=0 42개 |
| Drone 위성 수 | 5~7개 |
| Drone 수평 오차 추정치 `h_acc` | 35.301~102.306m |

위경도/고도는 실제 수신 필드가 바뀌었으므로 UI 고정 또는 수신 상태 캐시 고정은
이번 구간에서 관찰되지 않았다. 정지 보고 뒤 첫 30초에도 50개 GPS가 들어왔고,
보고 좌표는 수평 약 1.61m, 고도 -5.62m 변했다. 이는 정지 후 좌표 변동이며,
실제 추가 이동량으로 해석하지 않는다.

Coordinate contract checked: raw lat/lon degE7 → WGS84 degrees; raw alt mm → MSL m,
positive up. h_acc/v_acc mm → m. 정지 후 수평 변화는 지구 반지름 6,378,137m의
국소 위경도 차이 근사이며 카메라/FRD 좌표나 측정 거리로 사용하지 않았다.

## 기록 위치

Workstation data root 기준 `diagnostics/20260907-drone-movement-224906/`:

- `summary.json`: 통계와 파일 SHA-256.
- `mavlink.jsonl`: 실제 wire bytes와 decoded fields.
- `snapshots.jsonl`: 앱 좌표, 수신 Age, 링크 상태, RTCM 카운터.
- `events.jsonl`: 시험 marker와 해당 구간 이벤트.
- `ready_baseline.json`, `stopped_baseline.json`: 비교 시작/종료 snapshot.

아카이브는 준비 전 20초부터 정지 보고 뒤 30초까지로 제한했다.
진단용 Workstation 프로세스는 결과 작성 시 연결과 RTCM을 유지하고 있다.

## 남은 확인

1. 사용자가 이번 시험은 실내였다고 확인했다. 이번 fix 품질 저하는 실내 시험 조건으로
   취급하며, 이 기록으로 수신기 고장이나 외부 간섭원을 추정하지 않는다.
2. 오후 좌표 고정의 최초 원인은 아직 미확정이다. 이번 시험의 재현 실패와 구분한다.
3. UTC가 없는 경우를 "GPS UTC 진행"으로 분류하는 기존 진단 오류는 미수정이다.
   이번 검증에서는 원래 MAVLink `time_usec`를 직접 기록해 비교했다.
