# 2026-09-07 Drone GNSS 위치 고정 조사

GPS 안테나도 Pixhawk와 함께 위치 이동했다는 사용자 확인을 기준으로 조사했다.
이번 작업은 기록 분석과 기존 코드 검증이며 애플리케이션 코드나 장비 설정을 변경하지 않았다.

## 확인된 사실

오늘 저장된 Drone `gps_rtk` 749개 모두 위치가 같았다. 이 기록은 화면 문자열의
사본이 아니라 MAVLink GPS callback이 전달한 값을 기록한다. 같은 경로의 P1/P2
좌표는 변했고, 별도 합성 입력으로 Drone 좌표를 바꾸면 실제 Telemetry 표까지
변하는 것도 확인했다. 화면에 표시되기 전부터 Drone 위치 값이 고정돼 있었다는
판단을 뒷받침한다. 수신기 내부와 PX4 GPS 처리 중 어느 곳이 최초 원인인지는 미확정이다.

| 저장 파일의 세션 | GPS 수신 구간 (KST) | GPS 행 수 | 서로 다른 lat/lon/alt 조합 | 송신 sysid/compid |
| --- | --- | ---: | ---: | --- |
| `20260907_143158 헬기 착륙` | 14:32:26.151712 ~ 15:06:25.197624 | 744 | 1 | 3/1 |
| `20260907_150913 헬기 착륙` | 15:10:48.712697 ~ 15:10:55.806934 | 5 | 1 | 3/1 |

이 구간은 녹화된 샘플의 범위다. 녹화되지 않은 시간까지 연속 수신했다고 주장하지 않는다.

- 공통 위치: 위도 `34.610784000°`, 경도 `127.210620800°`, MSL 고도 `2.418000 m`.
- 타원체 고도 `23.161000 m`, 수평 정확도 `0.816 m`, 수직 정확도 `0.577 m`도 동일.
- `fix_type=4`, 지상 속도 `0.000000 m/s`; 위성 수는 23~32로 변했다.
- 모든 GPS 행의 송신 ID는 `3/1`, endpoint는 Drone에 저장된 DU0E4O0F radio였다.
- 첫 파일의 MAVLink sequence는 서로 다른 값 244개, callback 수신 시각은 744개였다.
- 첫 파일의 기록 시점 `age_s`는 0.000072~0.006021초였다. 이는 GUI의 장시간 최대
  Age가 아니라 callback 실행 당시 수신 후 경과 시간이다.
- 같은 첫 세션의 GPS1 위치 조합: P1은 178개 중 161개, P2는 453개 중 397개가 서로 달랐다.

원본은 repository root 기준 `.local/workstation_data/recordings/`에 있다:

| 파일 | 첫/마지막 GPS 행 번호 | SHA-256 |
| --- | --- | --- |
| `20260907_143158 헬기 착륙/px4_drone_20260907_143158_492054523+0900.jsonl` | 5 / 5771 | `edb54ae6dcc2ec96145f10d468245b5175c768ca066dddd948568266de7e5912` |
| `20260907_150913 헬기 착륙/px4_drone_20260907_150913_745730656+0900.jsonl` | 3 / 32 | `bf506a285ad38c2352642c9ac4ce59821345a2fc0c39bc54a54498246c404bee` |

## Age와 좌표의 계약

`workstation_core/pixhawk_attitude.py::_handle_mavlink_message`는 GPS_RAW_INT마다
새 `GpsPositionState`를 만든다. 수신 시각과 좌표 모두 해당 callback에서 설정한다.
`sensor_presentation_controller.py::_update_gps`는 이를 `latest_gps['drone']`에 넣고
원시 GPS 행을 기록한다. `drone_telemetry_controller.py::_gps_mapping`의 Age는
`now - gps.timestamp_s`이며, 좌표가 마지막으로 바뀐 뒤 경과 시간이 아니다.
`workspaces.py::TelemetryWorkspace._refresh_position_table`은 같은 snapshot의
좌표와 Age를 함께 표시한다. 따라서 같은 좌표가 새 패킷에 반복되면 Age도 다시 작아진다.

좌표 계약: MAVLink lat/lon은 degE7에서 WGS84 도로, alt는 mm에서 MSL m로 변환한다.
고도는 위쪽이 양수다. 타원체 고도는 별도 필드이며, 이 표시 경로에는 NED/ENU,
body-FRD 또는 카메라 optical-RDF 변환이 없다.
[MAVLink GPS_RAW_INT 정의](https://mavlink.io/en/messages/common.html#GPS_RAW_INT).

## 별도로 재현된 진단 오류 (미수정)

`sensor_presentation_controller.py::_observe_gnss_frozen_position`은 다음처럼 판정한다:

```python
utc_frozen = utc is not None and previous_utc == utc
```

`utc=None`이면 False가 되고, 후속 분기는 이를 곧바로 **GPS UTC 진행 / 수신기 정지**로
해석한다. 실제로 오늘 749개 GPS 행의 `gps_utc_us`는 모두 비어 있고
`time_source=mavlink_time_usec_unresolved`다. 첫 세션 application log 33행 등에는
이 조건에서도 "GPS UTC는 진행"이라고 기록돼 있다. 그 문구는 진행 증거로 사용할 수 없다.
같은 분류는 Telemetry FROZEN 설명과 evidence의 원인 설명에도 전파된다.

MAVLink `time_usec`는 epoch 또는 부팅 기준일 수 있다. PX4 v1.16 공식 GPS_RAW_INT
송신 구현은 `sensor_gps.timestamp`를 넣는다. 현재 수신 코드는 epoch로 인식되는
큰 값만 `gps_utc_us`에 담으며, 원래 `raw.time_usec`는 메모리에 있지만 `gps_rtk`
저장 행에는 남기지 않는다. 따라서 저장된 오늘 기록만으로 원래 timestamp의
진행/반복 여부를 복구할 수 없다. 이는 UTC가 비어 있다는 사실과 수신기 시계 고장을
구분해야 하는 이유다. 실제 Drone 펌웨어 버전은 이 기록에서 확인하지 못했다.
[PX4 v1.16 공식 GPS_RAW_INT 송신 코드](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/mavlink/streams/GPS_RAW_INT.hpp#L64).

후속 수정 범위는 원인 분류의 UNKNOWN 처리, 원래 MAVLink timestamp 및 시간 기준
보존, 반복 위치를 수신기 고장으로 단정하지 않는 설명이다. timestamp가 진행해도
GNSS/PX4 경계의 어느 구성요소가 문제인지 단독으로 확정할 수 없다.

## 검증 기록

| Slice / 근거 | 입력 → 관찰 결과 | 상태 |
| --- | --- | --- |
| 좌표 계약 / MAVLink 정의와 producer 코드 | degE7/mm → WGS84 deg/MSL m, 축 변환 없음 | PASS |
| 저장 기록 / 위 원본 JSONL | 749개 GPS callback → 동일 위치, 다른 시각/위성 수 | PASS |
| 수신·상태·UI / 독립적인 합성 GPS 필드 | 변경 lat/lon/alt → 실제 Telemetry 셀 변경, FROZEN 해제 | PASS (offline) |
| GPS1/GPS2 분리 / 합성 GPS2_RAW | GPS2 변경 → Drone primary GPS1 유지 | PASS (offline) |
| 동일 위치와 Age / 합성 GPS_RAW_INT 10개 | 좌표 동일 + 수신 시각 진행 → Age 매번 0 | PASS (offline) |
| UTC 원인 분류 / UTC 없음, boot-relative time_usec | UTC 없음 → 잘못된 "GPS UTC는 진행" 로그 | FAIL (결함 재현, 미수정) |
| 장비 내부 경계 / 직접 GNSS와 PX4 비교 | 현재 serial 장치와 Workstation 프로세스 없음 | BLOCKED (하드웨어 미연결) |

기존 회귀 테스트는 **44 passed**:

```bash
../../.venv/bin/python -m pytest -q \
  tests/test_workstation_telemetry.py \
  tests/test_gnss_baseline_center_recording.py \
  tests/test_mavlink_sources.py \
  tests/test_workstation_pixhawk_gnss.py::test_workstation_mavlink_packet_provenance_reaches_state_and_timeline \
  tests/test_workstation_pixhawk_gnss.py::test_mavlink_dual_gps_decodes_source_specific_altitude_and_accuracy
```

추가 offline probe는 실제 subscriber → sensor controller → snapshot → Telemetry 표를
연결했다. 합성 입력 `lat=346107941, lon=1272106408, alt=3418`이 셀에서
`34.61079410`, `127.21064080`, `3.418`로 표시됨을 검증했다.
이 probe는 hardware bytes를 캡처한 것이 아니며 장비 수리 증거가 아니다.

아래는 잘못된 UTC 분류만 좁혀 재현하는 명령이다 (`apps/Workstation`에서 실행):

```bash
PYTHONPATH=..:../..:../../common/python ../../.venv/bin/python - <<'PY'
from types import SimpleNamespace
from Workstation.sensor_presentation_controller import SensorPresentationController
from Workstation.workstation_core.rtk_geometry import GpsPositionState
logs = []
panel = SimpleNamespace(_log=logs.append)
owner = SensorPresentationController(panel)
for i in range(10):
    gps = GpsPositionState(
        camera='drone', namespace='', topic='mavlink/GPS_RAW_INT',
        timestamp_s=100.0+i, latitude_deg=34.610784,
        longitude_deg=127.2106208, altitude_m=2.418,
        gps_utc_us=None, raw=SimpleNamespace(time_usec=4_000_000_000+i*200_000),
    )
    owner._observe_gnss_frozen_position('drone', gps)
print('gps_utc_us:', gps.gps_utc_us)
print(logs[-1])
PY
```

## 남은 원인 분리

다음 장비 연결 시 안테나 이동과 함께 Drone의 MAVLink GPS_RAW_INT 원래 `time_usec`,
PX4 `sensor_gps`, GNSS 수신기의 직접 위치 출력을 같은 구간에서 비교해야 한다.
직접 GNSS 위치부터 고정돼 있으면 수신기 설정/출력 쪽, 직접 위치는 변하고
PX4 sensor_gps만 고정돼 있으면 GPS 드라이버/입력 경계, sensor_gps는 변하고
MAVLink만 고정돼 있으면 송신/선택 경계로 좁힐 수 있다.
오늘 데이터만으로 static hold, 수신기 fixed mode 또는 특정 firmware 결함을
확정하지 않았다.
