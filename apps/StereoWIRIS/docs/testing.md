# 테스트와 검증

## 기본 정적 확인

Python 문법 컴파일:

```bash
cd <SASL_ROOT>
python3 -m compileall common/python/sasl_common apps/StereoWIRIS/wiris_stereo_distance
```

## 의존성 파일

- `requirements/runtime.txt`: 앱 기본 실행 의존성. Live undistortion/ray 계산용 OpenCV·NumPy, rig alignment용 PyYAML, 세션 Parquet 저장용 PyArrow를 포함한다.
- `requirements/calibration.txt`: calibration bootstrap이 단독 복구할 수 있도록 runtime의 OpenCV·NumPy·PyYAML 항목을 의도적으로 반복한 부분집합.

## 빠른 로직 테스트

하드웨어 없이 실행 가능한 핵심 계산/저장 테스트:

```bash
cd <SASL_ROOT>
PYTHONPATH=. python3 -m pytest \
  apps/StereoWIRIS/tests/test_stereo_geometry_common.py \
  apps/StereoWIRIS/tests/test_coordinate_contract.py \
  apps/StereoWIRIS/tests/test_measurement_attempt.py \
  apps/StereoWIRIS/tests/test_measurement_controller.py \
  apps/StereoWIRIS/tests/test_triangulation.py \
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

## Legacy startup/session 집중 회귀

새 프로세스가 미완료 spool과 intrinsics를 자동 채택하지 않고, 기록 버튼 시점에만 새 session을
만드는지 확인한다. 명시적 recovery와 수동 intrinsics 경로가 남아 있는지도 함께 검증한다.

```bash
cd <SASL_ROOT>/apps/StereoWIRIS
QT_QPA_PLATFORM=offscreen PYTHONPATH=. ../../.venv/bin/python -m pytest -q \
  tests/test_stereo.py::test_startup_does_not_invoke_spool_recovery_or_intrinsics_auto_load \
  tests/test_stereo.py::test_startup_ignores_retryable_spool_and_new_recording_preserves_it \
  tests/test_stereo.py::test_gui_discovers_and_loads_retryable_pair_spool_after_restart \
  tests/test_stereo.py::test_gui_starts_new_recording_after_discarding_recovered_session \
  tests/test_stereo.py::test_recording_stop_waits_for_video_worker_restart_before_session_save
```

실제 startup에서는 보정 탭이 저장 파일의 존재를 표시할 수 있어도 `active_intrinsics`와 거리
계산 source는 미적용이어야 한다. 앱 실행만으로 새 session directory가 생기지 않아야 하며 기존
spool/intrinsics 파일 hash도 변하지 않아야 한다.

## WIRIS preview corruption 집중 회귀

RTP sequence wrap/duplicate/old/SSRC reset, forward gap 및 missing-PTS/DISCONT/CORRUPTED AU
latch와 clean-intra resume,
Auto/NVIDIA health 정책, 초기 PLAYING 실패의 0.5/1/2/4초 bounded retry, non-ASCII filesink
property round trip, plain MP4 mux policy, gap log throttle, immutable process bytes와
deep-copied QImage identity를 확인한다.

```bash
cd <SASL_ROOT>/apps/StereoWIRIS
QT_QPA_PLATFORM=offscreen PYTHONPATH=. ../../.venv/bin/python -m pytest \
  tests/test_yolo_stream.py -q
```

실장비 acceptance에서는 앱을 완전히 종료한 뒤 다음 환경변수를 지정한 새 프로세스를
사용한다.

```bash
export STEREO_PREVIEW_IDENTITY_LOG=/tmp/wiris-preview-identity.jsonl
```

Left/Right 동시 YOLO 부하에서 최소 15분 동안 window-id recording과 CPU/GPU/NIC 상태를
수집한다. JSONL의 `producer_queue_match`와 `queue_qimage_match`는 모두 true,
`queue_reordered_total`은 0이어야 한다. `queue_dropped_total`은 latest-only preview queue가
완전 frame을 의도적으로 교체한 경우만 허용한다. 화면 한 장만 정상인 것은 합격 근거가
아니다. 기록 시작 직전의 non-recording worker PID와 기록용 worker PID가 달라야 하며, 양쪽
worker는 bounded retry window 안에 PLAYING/첫 frame에 도달하고 `video/raw_left.mp4`와
`video/raw_right.mp4` 크기가 증가해야 한다. 종료 후 두 MP4 모두 `ffprobe`와 strict software
full decode(`ffmpeg -v error -xerror -i <file> -f null -`)를 통과해야 한다.

## 전체 PyQt 테스트 주의점

전체 `tests/test_stereo.py`는 로컬 `stereo_distance_settings.json` 값에 영향을 받을 수 있다. 예를 들어 `rtsp_transport=tcp`가 저장되어 있으면 UDP 기본값을 기대하는 테스트가 실패할 수 있다.

오프스크린 Qt 전체 테스트는 환경에 따라 이벤트 처리 중 세그폴트가 날 수 있으므로, 실패 시 먼저 개별 테스트로 좁혀 확인한다.

## 시각 검증

GUI를 실제로 띄울 수 없는 환경에서는 오프스크린 렌더링으로 최소 확인한다.

```bash
QT_QPA_PLATFORM=offscreen python3 -c 'from PyQt5 import QtWidgets; import wiris_stereo_distance.gui as gui; gui.WirisStereoDistancePanel._initial_status = lambda self: None; app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([]); w = gui.WirisStereoDistancePanel(); w.resize(1500, 920); w.show(); app.processEvents(); w.grab().save("/tmp/stereo_design_check.png")'
```
