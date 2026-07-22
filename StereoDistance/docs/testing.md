# 테스트와 검증

## 기본 정적 확인

Python 문법 컴파일:

```bash
cd <SASL_ROOT>
python3 -m compileall sasl_common StereoDistance/wiris_stereo_distance
```

## 의존성 파일

- `requirements/runtime.txt`: 앱 기본 실행 의존성. Live undistortion/ray 계산용 OpenCV·NumPy와 rig alignment용 PyYAML을 포함한다.
- `requirements/calibration.txt`: calibration bootstrap이 단독 복구할 수 있도록 runtime의 OpenCV·NumPy·PyYAML 항목을 의도적으로 반복한 부분집합.

## 빠른 로직 테스트

하드웨어 없이 실행 가능한 핵심 계산/저장 테스트:

```bash
cd <SASL_ROOT>
PYTHONPATH=. python3 -m pytest \
  StereoDistance/tests/test_stereo_geometry_common.py \
  StereoDistance/tests/test_coordinate_contract.py \
  StereoDistance/tests/test_measurement_attempt.py \
  StereoDistance/tests/test_measurement_controller.py \
  StereoDistance/tests/test_triangulation.py \
  -q
```

이 묶음은 immutable attempt/pair identity, class·ambiguity fail-closed,
`camera_optical_rdf -> left_camera_rdf`, checkerboard `X_right = R X_left + T`
역변환, Pixhawk body FRD/NED 변환, selected-left quaternion/timestamp 공유,
왼쪽 optical-center 원점 유지와 invalid-ray fail-closed를 확인한다.
`test_measurement_attempt.py`는 factory/direct-constructor timestamp metadata가
같은 allowlist 순서로 canonicalize되는지, mutable/nonfinite/duplicate 입력이
snapshot 생성 시점에 거부되는지, source·`asdict()` mutation이 attempt
canonical SHA-256을 바꾸지 않는지도 확인한다.

## 대표 PyQt 테스트

오프스크린 Qt 환경에서 일부 UI 테스트를 실행한다.

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
