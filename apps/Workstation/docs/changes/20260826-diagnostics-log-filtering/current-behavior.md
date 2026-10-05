# Current Behavior

## Version and worktree context

- Baseline Git identity: `c439799`.
- SASL와 Workstation에 다수의 기존 tracked/untracked 변경이 있다. 이번 패킷은 이를
  소유하거나 정리하지 않는다.
- 다른 Workstation 변경 패킷은 `paused` 또는 `verifying` 상태다. 이번 변경은 별도
  `20260826-diagnostics-log-filtering` 패킷으로 격리한다.

## Reproduction

1. Workstation에서 하단 Diagnostics의 `Log`를 연다.
2. 카메라, Pixhawk, GNSS/NTRIP 이벤트를 발생시킨다.
3. 현재 화면은 source widget 세 묶음을 순서대로 표시하며 장비·수준·검색 필터가 없다.

기준선 테스트:

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 QT_QPA_PLATFORM=offscreen \
../../.venv/bin/python -m pytest -q -o cache_dir=/tmp/workstation_pytest_cache \
  tests/test_workstation_ui.py::test_diagnostics_tab_preserves_workspace_inspector_selection_and_row_inspection \
  tests/test_workstation_ui.py::test_diagnostics_log_preserves_manual_scroll_and_offers_follow_latest \
  tests/test_workstation_ntrip.py::test_handle_gnss_network_diagnostic_forwards_to_the_visible_log \
  tests/test_workstation_camera.py::test_diagnostics_ftp_tab_surfaces_the_ftp_lifecycle_log
```

결과: `4 passed in 0.86s`. Workstation 로컬 `.venv`에는 PyQt5가 없고, pytest 자동
플러그인은 ROS `launch_testing`의 누락된 `yaml` 때문에 실패하므로 canonical root venv와
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`이 현재 재현 가능한 기준선이다.

## Observed frontend behavior

- `DiagnosticsContent`의 Log는 read-only `QPlainTextEdit`이다.
- 사용자가 위로 스크롤하면 위치를 유지하고 `New logs: N`/`Follow latest`를 표시한다.
- Diagnostics 선택은 중앙 Workspace와 Inspector/행 선택을 보존한다.
- 보이는 Diagnostics 탭만 4 Hz status poll에서 갱신된다.
- FTP는 별도 탭과 별도 600-line buffer를 갖는다.

## Observed backend and persistence behavior

- `StandalonePanelBackend.diagnostic_text()`는 `camera_yolo_log`, `pixhawk_log`,
  `wiris_log`를 이 순서로 이어 붙인다. 세 source 사이의 실제 시간순 병합은 없다.
- `WirisStereoDistancePanel._log(text)`는 문자열 접두어/키워드로 기존 세 위젯에 배분하고,
  분류되지 않은 메시지는 `wiris_log`에 넣는다.
- worker thread 로그는 `_threadsafe_log_requested: pyqtSignal(str)`을 거쳐 GUI thread에서
  텍스트 위젯에 기록된다.
- 활성 기록 중 raw message는 기존 `application.log` 또는
  `audit/application_log.jsonl` 경로에도 기록될 수 있다. 이 persistence 계약은 이번
  변경에서 유지한다.

## Documentation, code, test, and runtime conflicts

- `04_COMPONENT_SYSTEM.md`는 Time/Level/Source/Message 구조화 열을 권장하지만 실제 Log는
  plain text다.
- `02_INFORMATION_ARCHITECTURE.md`는 Diagnostics를 factual communication history로
  정의하지만 현재 device/source 구분은 문자열 휴리스틱과 widget fallback에 의존한다.
- GNSS controller는 `[GNSS][NTRIP]` 이벤트를 visible log에 전달하지만 `_log`의 기존
  분류 규칙은 `[GNSS]`를 인식하지 않아 WIRIS widget fallback으로 보낼 수 있다.
- 런타임 실제 장비 화면/부하 증거는 아직 수집하지 않았으며 승인 후 소스 수정 전에
  baseline screenshot과 burst 로그 증거를 추가한다.
