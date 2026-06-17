# 테스트와 검증

## 기본 정적 확인

Python 문법 컴파일:

```bash
python3 -m compileall main.py wiris_stereo_distance
```

## 의존성 파일

- `requirements/runtime.txt`: 앱 기본 실행 의존성.
- `requirements/calibration.txt`: OpenCV 카메라 보정 의존성.

## 빠른 로직 테스트

하드웨어 없이 실행 가능한 핵심 계산/저장 테스트:

```bash
python3 -m pytest \
  tests/test_measurement_controller.py \
  tests/test_triangulation.py \
  tests/test_settings.py \
  tests/test_recording_session.py \
  -q
```

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
