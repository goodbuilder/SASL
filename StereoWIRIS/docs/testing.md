# 테스트와 검증

## 기본 정적 확인

Python 문법 컴파일:

```bash
cd <SASL_ROOT>
python3 -m compileall sasl_common StereoWIRIS/wiris_stereo_distance
```

## 의존성 파일

- `requirements/runtime.txt`: 앱 기본 실행 의존성. Live undistortion/ray 계산용 OpenCV·NumPy, rig alignment용 PyYAML, 세션 Parquet 저장용 PyArrow를 포함한다.
- `requirements/calibration.txt`: calibration bootstrap이 단독 복구할 수 있도록 runtime의 OpenCV·NumPy·PyYAML 항목을 의도적으로 반복한 부분집합.

## 빠른 로직 테스트

하드웨어 없이 실행 가능한 핵심 계산/저장 테스트:

```bash
cd <SASL_ROOT>
PYTHONPATH=. python3 -m pytest \
  StereoWIRIS/tests/test_stereo_geometry_common.py \
  StereoWIRIS/tests/test_coordinate_contract.py \
  StereoWIRIS/tests/test_measurement_attempt.py \
  StereoWIRIS/tests/test_measurement_controller.py \
  StereoWIRIS/tests/test_triangulation.py \
  -q
```

이 묶음은 immutable attempt/pair identity, class·ambiguity fail-closed,
`camera_optical_rdf -> left_camera_rdf -> baseline_center_frd`, checkerboard
`X_right = R X_left + T` 역변환, Pixhawk body FRD/NED 변환,
selected attitude/GPS snapshot 공유, selected baseline-frame midpoint 원점 이동,
FRD 축 구성과 midpoint Range 재계산, invalid-ray/frame-basis fail-closed를
확인한다. 평행 0.30 m baseline의 중심선 표적은
`left_camera_rdf=(0.15,0,5.4)`에서
`baseline_center_frd=(5.4,0,0)`으로 변환돼야 하며, 비평행 optical-forward
case에서도 Right/Forward 직교성과 `Down=Forward×Right`를 만족해야 한다.
`test_measurement_attempt.py`는 factory/direct-constructor timestamp metadata가
같은 allowlist 순서로 canonicalize되는지, mutable/nonfinite/duplicate 입력이
snapshot 생성 시점에 거부되는지, source·`asdict()` mutation이 attempt
canonical SHA-256을 바꾸지 않는지도 확인한다. 또한 outer container와 각
pair item을 명시적인 `None`/tuple/list 계약으로 제한하고 Mapping·set·문자열
·bytes·임의 iterable이 two-element pair로 오인되지 않는지 검증한다. 실제
`DetectionObservation.from_payload()` 경로도 같은 shape 계약으로 먼저
검증하며, `DetectionSnapshot` subclass가 factory·`replace()`를 통해 attempt와
canonical SHA 경계에 진입하지 못하는지도 확인한다.

## 대표 PyQt 테스트

오프스크린 Qt 환경에서 일부 UI 테스트를 실행한다.

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest \
  tests/test_stereo.py::test_pixhawk_rtk_can_drive_triangulation \
  tests/test_stereo.py::test_checkerboard_triangulation_marks_pixhawk_attitude_not_used \
  tests/test_stereo.py::test_raw_recording_saves_bbox_geometry_and_pixhawk_status_fields \
  tests/test_stereo.py::test_session_buttons_toggle_labels_and_roles \
  tests/test_stereo.py::test_camera_network_settings_are_single_compact_row \
  tests/test_stereo.py::test_measurement_settings_save_button_triggers_current_settings_save \
  -q
```

좌표/기록 테스트의 PASS 기준은 v2 row가
`measurement_frame=baseline_center_frd`, 명시적
`baseline_center_forward/right/down/range_m`, attempt-scoped frame instance를
포함하고 `x/y/z`, public `left_camera_*`, `stereo_center_*` alias를 생성하지
않는 것이다. `source_left_camera_rdf_*`는 source diagnostic으로만 허용한다.

## 전체 PyQt 테스트 주의점

전체 `tests/test_stereo.py`는 로컬 `stereo_distance_settings.json` 값에 영향을 받을 수 있다. 예를 들어 `rtsp_transport=tcp`가 저장되어 있으면 UDP 기본값을 기대하는 테스트가 실패할 수 있다.

오프스크린 Qt 전체 테스트는 환경에 따라 이벤트 처리 중 세그폴트가 날 수 있으므로, 실패 시 먼저 개별 테스트로 좁혀 확인한다.

## 시각 검증

GUI를 실제로 띄울 수 없는 환경에서는 오프스크린 렌더링으로 최소 확인한다.

```bash
QT_QPA_PLATFORM=offscreen python3 -c 'from PyQt5 import QtWidgets; import wiris_stereo_distance.gui as gui; gui.WirisStereoDistancePanel._initial_status = lambda self: None; app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([]); w = gui.WirisStereoDistancePanel(); w.resize(1500, 920); w.show(); app.processEvents(); w.grab().save("/tmp/stereo_design_check.png")'
```
