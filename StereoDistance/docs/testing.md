# 테스트와 검증

## 기본 정적 확인

FFmpeg 원본 녹화 의존성:

```bash
ffmpeg -version
```

The GStreamer preview path uses FFmpeg H.264 stream-copy recording, so this
command must resolve before a session can produce raw MP4 files.

Python 문법 컴파일:

```bash
python3 -m compileall main.py wiris_stereo_distance
```

## Canonical 검증 세트

Codex 또는 작업자가 검증 명령을 다시 조합해야 할 때는 아래 세트를
우선 기준으로 삼는다. 변경 범위가 더 좁으면 해당 파일의 targeted test를
먼저 실행하고, 더 넓으면 이 세트를 포함해 확장한다.

### Core logic

하드웨어 없이 실행 가능한 계산, 좌표계, 설정, 기록 경로의 기본 세트:

```bash
python3 -m pytest \
  tests/test_measurement_controller.py \
  tests/test_triangulation.py \
  tests/test_coordinate_contract.py \
  tests/test_settings.py \
  tests/test_recording_session.py \
  -q
```

### Coordinate/profile

triangulation, coordinate contract, reprojection/profile 관련 변경의 기본 세트:

```bash
python3 -m pytest \
  tests/test_measurement_controller.py \
  tests/test_triangulation.py \
  tests/test_coordinate_contract.py \
  tests/test_reprojection_profile.py \
  -q
```

### UI smoke

GUI 배치나 저장/측정 workflow가 바뀌었을 때의 최소 PyQt 세트:

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest \
  tests/test_stereo.py::test_session_buttons_toggle_labels_and_roles \
  tests/test_stereo.py::test_camera_network_settings_are_single_compact_row \
  tests/test_stereo.py::test_measurement_settings_save_button_triggers_current_settings_save \
  -q
```

## 의존성 파일

- `requirements/runtime.txt`: 앱 기본 실행 의존성.
- `requirements/calibration.txt`: OpenCV 카메라 보정 의존성.

## 심사용 Evidence Checklist

StereoDistance의 최종 주장은 "정확도 보장"이 아니라 "검증 가능한 현장
stereo measurement workstation"이다. 따라서 보고서나 발표에는 아래
evidence를 분리해서 제시한다.

### 1. 계산과 좌표계

필수 증거:

- `measurement_controller`, `triangulation`, `coordinate_contract`,
  `reprojection_profile` 대표 테스트 결과
- NED, PX4 body FRD, OpenCV optical RDF, stereo-center frame 계약
- OpenCV가 사용 가능한 runtime에서 distortion coefficient가 live ray
  생성에 적용되는지 확인
- mixed-frame 비교가 최종 정확도 지표가 아니라는 명시
- raw와 corrected row의 source, frame, metric 분리

대표 명령:

```bash
python3 -m pytest \
  tests/test_measurement_controller.py \
  tests/test_triangulation.py \
  tests/test_coordinate_contract.py \
  tests/test_reprojection_profile.py \
  -q
```

### 2. UI와 운용 workflow

필수 증거:

- offscreen PyQt smoke 실행 여부
- Live/Connection/Measurement/Save-Analyze 흐름 스크린샷 또는 로그
- session start/stop, measurement row 저장, settings 저장 동작 확인
- `gui.py`가 workflow orchestration을 담당하고 계산 로직은 별도 모듈에
  있다는 구조 설명

### 3. Hardware readiness

실제 장비가 있을 때 1회 세션 기준으로 아래 표를 채운다.
MAVLink와 ROS2/uXRCE가 동시에 연결되는 상태를 최우선 목표로 두되,
Rover readiness PASS는 MAVLink heartbeat와 제어 가능성을 기준으로
판정한다. ROS2 sample만 있고 MAVLink 제어 채널이 없으면 RTK/복구 준비
완료로 기록하지 않는다.

| 항목 | PASS 기준 | 기록할 값 |
| --- | --- | --- |
| WIRIS left/right video | 포트 open이 아니라 첫 프레임 수신 | FPS, latency, timestamp |
| WIRIS time check | still image EXIF/GPS timestamp 비교 가능 | left/right time delta |
| Pixhawk MAVLink | heartbeat 수신과 제어/복구 경로 확인 | system id, component id, link endpoint |
| ROS2/uXRCE | publisher와 sample 확인, MAVLink와 동시 연결 여부 확인 | topic name, Hz, sample age |
| RTCM injection | `GPS_RTCM_DATA` 전송과 rover RTK 반응을 분리 확인 | sent bytes/packets, response status |
| RTK/GNSS | baseline 또는 fix 상태 확인 | baseline m, fix type, covariance |
| MAG/RM3100 | device id와 freshness 확인 | device id, age, anomaly flag |
| Recording | session CSV/JSON 생성 | session path, row count |

### 4. Calibration and geometry

필수 증거:

- zoom별 intrinsics 파일 이름과 생성 조건
- checkerboard 이미지 수, 해상도, detector method, accepted/rejected image list
- 전체 reprojection error와 accepted image별 reprojection error
- checkerboard center의 screen coverage와 scale variation warning
- stereoCalibrate RMS, accepted/rejected pair list, pair별 epipolar residual
- reprojection profile JSON의 `calibration_envelope`와 warning
- baseline source: manual, RTK/GNSS, 또는 fixed scenario value
- `nominal-shared-rig-v1` 적용 여부와 실패 조건
- 장착 오차를 보정으로 제거한 것이 아니라 최종 오차에 포함한다는 설명

### 5. Post-flight validation

필수 증거:

- truth source: ULog local NED, ULog GPS/global-derived NED, live telemetry,
  recorded-row fallback 중 하나
- time basis: UTC, PX4 boot/local timestamp, recorded timestamp 중 하나
- matched count, unmatched row count, mean/max timestamp delta
- `estimated_range_m`, `truth_range_m`, `range_error_m`, `position_error_m`
- `error_frd_forward/right/down_m` 또는 `estimated_north/east/down_m`와
  `truth_north/east/down_m`
- Simulation evidence와 hardware evidence를 합치지 않는 결과 표

## 빠른 로직 테스트

하드웨어 없이 실행 가능한 핵심 계산/저장 테스트다. 새 작업에서는 위
`Canonical 검증 세트`의 `Core logic`을 우선 사용하고, 기존 기록 재현이
필요할 때만 아래 축약 세트를 사용한다.

```bash
python3 -m pytest \
  tests/test_measurement_controller.py \
  tests/test_triangulation.py \
  tests/test_settings.py \
  tests/test_recording_session.py \
  -q
```

## 대표 PyQt 테스트

오프스크린 Qt 환경에서 일부 UI 테스트를 실행한다. 새 작업에서는 위
`Canonical 검증 세트`의 `UI smoke`를 우선 사용한다.

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest \
  tests/test_stereo.py::test_session_buttons_toggle_labels_and_roles \
  tests/test_stereo.py::test_camera_network_settings_are_single_compact_row \
  tests/test_stereo.py::test_measurement_settings_save_button_triggers_current_settings_save \
  -q
```

## 전체 PyQt 테스트 주의점

전체 `tests/test_stereo.py`는 로컬 `stereo_distance_settings.json` 값에 영향을 받을 수 있다. 예를 들어 `rtsp_transport=tcp`가 저장되어 있으면 UDP 기본값을 기대하는 테스트가 실패할 수 있다.

오프스크린 Qt 전체 테스트는 환경에 따라 이벤트 처리 중 세그폴트가 날 수 있으므로, 실패 시 먼저 개별 테스트로 좁혀 확인한다.

## 시각 검증

GUI를 실제로 띄울 수 없는 환경에서는 오프스크린 렌더링으로 최소 확인한다.

```bash
QT_QPA_PLATFORM=offscreen python3 -c 'from PyQt5 import QtWidgets; import wiris_stereo_distance.gui as gui; gui.WirisStereoDistancePanel._initial_status = lambda self: None; app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([]); w = gui.WirisStereoDistancePanel(); w.resize(1500, 920); w.show(); app.processEvents(); w.grab().save("/tmp/stereo_design_check.png")'
```
