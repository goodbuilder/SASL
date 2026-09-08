# Radio 수신률과 RTCM ON/OFF 확인

사용자가 실내 시험임을 확인했다. 직전 추가한 fix 상실/RF 간섭 해석과 별도 추출
warning 파일은 삭제했다. 원래 MAVLink 캡처는 보존했다.

## 실측

기존 Workstation의 세 Radio 연결을 유지하며 NTRIP/RTCM을 OFF 후 ON으로 복원했다.
ON 전/후 각 60초, OFF는 전환 뒤 10초를 제외한 약 95초를 비교했다.

| 수신 항목 | RTCM ON 전 | RTCM OFF | RTCM ON 복원 후 |
| --- | ---: | ---: | ---: |
| P1 ATTITUDE | 1.73 Hz | 1.63 Hz | 1.77 Hz |
| P2 ATTITUDE | 1.07 Hz | 1.81 Hz | 1.73 Hz |
| Drone GPS_RAW_INT | 1.62 Hz | 2.16 Hz | 2.10 Hz |
| Drone ATTITUDE | 8.32 Hz | 2.16 Hz | 2.08 Hz |

앱이 처리한 실제 패킷 수/관찰 시간을 사용했다. 송신 요청률 자체가 아니다.
RTCM ON에서 각 장치로 전달한 원래 보정 payload는 약 1.12~1.15 kB/s였고,
이 값에는 MAVLink framing/fragmentation 및 무선 링크 overhead가 포함되지 않는다.
57600 baud를 실제 무선 유효 대역폭과 동일시하지 않았다.

세 구간 모두 앱의 RTCM 전달 실패 증가량은 0이었다. 이는 serial 송신 결과이며
downlink ATTITUDE/GPS 수신률 또는 각 GNSS 수신기의 보정 사용을 보증하지 않는다.

RTCM을 껐다고 요청된 5Hz로 복구되지는 않았다. 재주입 후에도 P2/Drone GPS 수신률은
OFF와 비슷했다. 따라서 이 시험은 RTCM 단독 원인을 입증하지 않는다. 시험 중 자동
Radio tier가 변했으므로 고정 스트림 조건의 순수 대역폭 실험도 아니다.

## 확인된 코드 구조

- `workstation_core/pixhawk_radio_policy.py`: 세 역할에 같은 정책을 적용한다.
  CRITICAL에서도 ATTITUDE/GPS1/GPS2, RTK status, SYS_STATUS/BATTERY를 요청한다.
  상위 tier에는 IMU/DEBUG_VECT/RAW_IMU/local/global position/VFR_HUD가 추가된다.
- `_request_mavlink_sensor_streams`: 현재 tier 목록에 있는 메시지의 양수 rate만 설정한다.
  CRITICAL로 낮아져 목록에서 빠진 optional 메시지에 `interval=-1`을 보내지 않는다.
  따라서 기존 stream이 실제 중지됐다고 볼 수 없다. PX4 기본 stream도 별도 확인 대상이다.
- `device_runtime_controller.py`: P1/P2에 2초 timer로 sensor_mag 5개 instance와
  vehicle_magnetometer Shell 조회를 시도한다. 이 시험 snapshot의 약 77~86%에서
  `RM3100 SHELL POLL`이 활성 상태였다. 이것은 command 활성 시간 비율이지
  전체 무선 byte 중 Shell 비율은 아니다.
- `sensor_presentation_controller.py`: Drone ATTITUDE는 stereo camera attitude 저장에
  사용하지 않는다. 그러나 공통 liveness의 LIVE 판정에는 ATTITUDE freshness가 필요하다.
  따라서 Drone ATTITUDE를 제거하려면 Drone 전용 liveness 계약도 함께 바꿔야 한다.

PX4는 configured rate 또는 physical link가 포화되면 stream rate를 낮춘다.
[공식 MAVLink module 설명](https://docs.px4.io/main/en/modules/modules_communication#mavlink).

## 필요한 방향 (이번 조사에서 미구현)

Drone 기능상 필요한 위치/fix/sats/accuracy는 GPS_RAW_INT 하나에 들어 있다.
HEARTBEAT identity/liveness와 실제 필요한 저주기 상태를 더한 Drone 전용 profile이
적합하다. P1/P2의 자세·선택한 GNSS를 우선하고, 자력계 Shell 조회는 별도 저주기 또는
명시적 조회로 제한할 필요가 있다. 역할별 allowlist와 실제 stream disable, Drone의
GPS 기반 freshness를 함께 검증해야 한다. 단순히 요청 목록에서 ATTITUDE를 빼는
것만으로 실제 송신 중지나 올바른 LIVE 판정이 보장되지 않는다.

원시 기록/통계: Workstation data root의 `diagnostics/20260907-radio-bandwidth-ab/`.
조사 종료 시 세 장치 RTCM은 STREAMING으로 복원됐고 각 전달 frame 카운터가 증가했다.

Coordinate contract checked: GPS WGS84 위경도/MSL 고도, ATTITUDE body-FRD→NED 계약은
그대로 두고 패킷 도착 빈도와 시각 차이만 분석했다. 앱 소스/펌웨어 설정은 수정하지 않았다.
