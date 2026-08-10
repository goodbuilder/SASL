# StereoWIRIS Workstation UI 설계 감사 결과

Current boundary note (2026-08-09): the Workstation cleanup slice removed ROS2/uXRCE
Pixhawk connection/discovery and still-image/EXIF TimeSync from the standalone runtime.
MAVLink FTP remains only as an advanced PX4 Ethernet-settings owner operation, not as
the removed still-image path.
Older findings below may mention those paths as audit history; such paths remain Legacy-only.
The Workstation launcher now skips ROS2/PX4 overlay bootstrap, and the shared telemetry
module lazy-loads ROS2 Python bindings only for a Legacy ROS2 operation.

- 감사일: 2026-08-08
- 감사 브랜치: `audit/20260808-audit-prompt`
- 감사 시작 기준 커밋: `b5f11bc`
- 감사 범위: `AUDIT_PROMPT.md`의 00–10 설계 문서, 실제 PyQt5 실행 경로, 카메라·Pixhawk·GNSS/RTK·검출·측정·기록·설정·종료 흐름, 테스트와 좌표/시간 계약
- 감사 단계: 초기 감사에서는 UI/백엔드 동작 코드를 변경하지 않았고, 이후 사용자 승인으로 같은 브랜치에서 구현 단계로 전환함.
- 현재 판정: **GO WITH CONDITIONS** — 실장비 장시간 검증 이전의 소프트웨어 migration slice와 full Qt 회귀까지 통과했으나 장비·장시간 종료 검증은 남아 있음.

## 2026-08-09 분리 단계 추가 사실

Legacy 구현은 `legacy_gui.py`로 이동했고 `gui.py`는 기존 import와 monkeypatch를
보존하는 모듈 alias가 되었다. Workstation은 저장소 루트 `workstation/`의
`standalone_gui.py`와 `standalone_backend.py`만 entrypoint owner로 사용한다. 따라서 두 UI의 panel owner는
실행 경로상 분리되며, 측정·삼각측량·저장 schema·좌표 계약을 제공하는 공통 모듈은
의도적으로 공유한다.

## 감사 판정 규칙

본문에서 다음 표기를 사용한다.

- **CURRENT FACT**: 현재 저장소의 코드·테스트·실행 경로에서 확인한 사실
- **DESIGN INTENT**: 00–10 문서가 요구하는 목표
- **UNSUPPORTED ASSUMPTION**: 코드·테스트로 뒷받침되지 않은 설계 전제
- **RECOMMENDED CHANGE**: 구현 전에 해야 할 안전한 수정 또는 범위 조정

## Executive Summary

초기 감사 시점의 StereoWIRIS는 측정·좌표·시간·세션 저장 코어가 비교적 잘 분리된 기존 PyQt5 운용 도구였고, 화면 오케스트레이션은 `gui.py`의 하나의 대형 패널에 집중되어 있었다. 초기 adapter slice는 그 패널을 단일 backend/resource owner로 유지하면서 Workstation shell을 추가했지만, 2026-08-09 분리 단계에서 Workstation은 복사된 standalone panel/backend owner로 전환되었다.

가장 중요한 사실은 다음과 같다.

1. **해결 — 런처 선택기:** `gui.main()`이 `--ui legacy|workstation`을 parse하며 legacy가 기본값이다. 한 프로세스는 하나의 operational window만 만든다.
2. **해결 조건부 — 종료:** top-level close가 `LifecycleCoordinator`를 통해 설정·보조 worker·GNSS·runtime worker·session save 순서를 공유한다. 실제 장비/장시간 timeout은 별도 검증이 필요하다.
3. **해결 — 코어 재사용:** `MeasurementAttempt`/`measurement_controller.py`와 `recording_session.py`를 Workstation에서 재구현하지 않고 `StandalonePanelBackend`가 복사된 panel의 public handler를 호출한다. Legacy는 원본 panel/backend 경로를 계속 사용한다.
4. **해결 조건부 — 상태/명령:** `BackendStatusAdapter`, `StatusModel`, `CommandRegistry`가 source/clock/age를 보존한다. 모든 transport/failure 상태의 현장 의미는 아직 hardware smoke가 필요하다.
5. **유지 — 좌표 계약:** `ned`, `px4_frd`, OpenCV optical `RDF`, `left_camera_rdf`, `baseline_center_frd`, ENU 정렬 벡터를 명시 라벨로 표시하고 변환은 기존 backend에 남긴다.

따라서 현재 Workstation adapter-backed slice는 **GO WITH CONDITIONS**다. 소프트웨어 acceptance와 source projection은 충족했지만 실제 RTSP/Pixhawk MAVLink/GNSS와 장시간 recording finalization을 확인하기 전에는 현장 운용 완료로 간주하지 않는다. ROS2/uXRCE와 still-image/FTP/EXIF TimeSync는 현재 Workstation 범위가 아니라 Legacy 전용이다.

## Post-audit Implementation Evidence

| 영역 | 구현 증거 | 현재 판정 |
|---|---|---|
| Runtime selection | `main.py` lazy dispatch, `legacy_gui.py` + `gui.py` alias, `standalone_gui.py`, `run.sh`, `tests/test_workstation.py` | PASS; legacy default 유지 |
| One owner / shell | `workstation/main_window.py`, `standalone_backend.py` | PASS; Workstation은 복사된 panel이 유일한 resource owner |
| Lifecycle | `gui.py:shutdown_for_close`, `sasl_common/lifecycle.py` | PASS in fake/pending-step tests; hardware timeout pending |
| Status | `workstation/status_model.py` | PASS for source-backed projection; camera sync and GNSS receiver/NED baseline evidence added |
| Commands | `workstation/command_registry.py`, `standalone_backend.py` | PASS; QAction와 standalone public handler 연결, camera/Pixhawk/GNSS/calibration operation direct facades, backend failure raises and is not completed |
| Workspaces | Overview/Camera/Measurement/Alignment/Calibration/Recording-Review/Advanced owner tools | PASS in offscreen smoke; source-backed tables plus operation controls are exposed |
| Diagnostics | System/Camera/Pixhawk/GNSS/Log/Measurement/Performance plus Advanced owner tools | PASS from existing logs/status/attempt/telemetry evidence; raw SDK/PX4/MAG/field-watch, preview/zoom, and media review now use typed owner facades |
| Coordinates/time | `coordinate_contract.py` labels and receive/source clock metadata | PASS by contract review; no geometry duplicated |
| Regression | 58 Workstation tests plus the refreshed full suite of 672 tests, including advanced-owner and device-I/O worker tests | PASS for software scope; hardware/manual validation remains |

## Repository Baseline

### CURRENT FACT

| 항목 | 확인 결과 |
|---|---|
| 브랜치 | `audit/20260808-audit-prompt`에서 감사 수행 |
| 시작 커밋 | `b5f11bc` |
| 실행 진입점 | `StereoWIRIS/main.py:40-43`에서 `wiris_stereo_distance.gui.main` 호출 |
| 실행 스크립트 | `StereoWIRIS/run.sh`가 환경/ROS overlay를 준비한 뒤 `main.py`로 인자를 전달 |
| 실제 GUI | `gui.py:2907`의 `WirisStereoDistancePanel` 하나가 연결·보정·정렬·YOLO·측정·저장·고급 기능을 소유 |
| GUI 규모 | `gui.py` 약 21,000줄; 패널은 `QWidget`이면서 명령·계산 연결·파일·워커 수명주기를 함께 처리 |
| 현재 상위 탭 | `연결`, `시간`, `이미지`, `보정`, `파라미터`, `정렬`, `YOLO`, `측정`, `저장·분석`, `고급` (`gui.py:3238-3247`) |
| Workstation 패키지 | 저장소 루트 `workstation/`에 shell, adapter, status, command, workspace 모듈 구현; lifecycle은 `sasl_common/`에 중립화 |
| 런처 모드 | `--ui legacy|workstation` 추가; legacy default, one operational window per process |
| 작업 트리 | 사용자 수정·untracked 파일을 보존한 채 구현 파일을 추가함; 최종 status는 아래에서 다시 확인 |

### 감사 대상 문서의 성격

00–08은 제품·워크플로·IA·레이아웃·컴포넌트·네비게이션·상태·명령·디자인 토큰의 설계 의도다. 09는 조건부 구현 계획이고 10은 감사 프롬프트다. 따라서 문서의 “candidate”, “should”, “initial”을 현재 기능으로 간주하지 않았다.

### 실행 증거

- `python3 -m compileall -q StereoWIRIS/wiris_stereo_distance StereoWIRIS/main.py`: 통과
- Workstation offscreen/contract suite: **58 passed**; 1366×768 resize/navigation/close, source
  projections, upstream evidence, device-candidate controls, deferred completion, worker shutdown,
  standalone backend conformance, status transition, command failure, and device-I/O worker smoke 포함
- 좌표·삼각측량·immutable attempt·controller·recording 회귀와 Workstation command/settings/alignment/advanced/session facade: **672 passed** in the full suite
- Workstation device-I/O workers cover camera-network I/O, WIRIS STUS/GDTI/STIP/SDK/zoom, Pixhawk connect/USB bootstrap/stream setup/PARAM/Shell/FTP/reboot, and GNSS RTCM queue delivery.
- 실제 RTSP, Pixhawk MAVLink, GNSS receiver/caster, video writer 하드웨어 검증: 수행하지 않음

## Architecture Findings

다음 finding 본문은 구현 전 baseline을 보존한 감사 기록이다. 각 차단 조건의 현재
해결 상태와 남은 조건은 `Post-audit Implementation Evidence` 및 `Implementation
Blockers` 표를 우선한다.

### UI-AUDIT-001 — HIGH / Architecture / 런처와 공존 모델이 현재 구현과 불일치

- **Design claim:** 09는 `python main.py --ui legacy`와 `python main.py --ui workstation`을 제안하고, legacy를 기본 fallback으로 둔다.
- **CURRENT FACT / evidence:** `main.py:40-43`은 기존 `gui.main`만 호출한다. `gui.py:21051-21081`의 `main()`은 `--ui`를 읽지 않는다. `WirisStereoDistanceGui`와 `IntegratedWirisPixhawkConsole`은 각각 같은 `WirisStereoDistancePanel`을 central widget으로 만든다(`gui.py:20982-21025`).
- **Why:** Workstation을 추가해도 어떤 프로세스가 어떤 UI를 소유하는지, 기본 모드와 fallback이 무엇인지 결정되지 않는다. `--ui`를 먼저 구현하지 않고 인자를 전달하면 Qt 인자 처리와 실제 선택 결과가 검증되지 않는다.
- **Failure scenario:** 현장 운용자가 Workstation 모드로 실행했지만 legacy 패널이 열리거나 인자 오류가 발생한다. 두 창을 동시에 띄우는 임시 공존은 동일한 카메라 큐·Pixhawk 구독·기록 경로를 이중 소비할 수 있다.
- **RECOMMENDED CHANGE:** QApplication 생성 전에 명시적 parser로 `legacy|workstation`을 검증하고, 한 실행당 한 operational window만 만들도록 한다. 새 UI는 먼저 read-only adapter와 legacy handoff를 사용하고, default는 검증 전까지 legacy로 유지한다.
- **Verification:** `--help`, 잘못된 mode, 두 mode의 offscreen launch, 한 프로세스당 worker/subscriber 생성 수를 테스트한다.

### UI-AUDIT-002 — HIGH / Architecture / 상태·명령·비즈니스·수명주기가 대형 패널에 결합

- **Design claim:** WorkstationMainWindow는 shell/presentation만 소유하고 측정·카메라·PX4·GNSS·기록·보정 구현은 소유하지 않는다.
- **CURRENT FACT / evidence:** `WirisStereoDistancePanel`은 `gui.py:2907`에서 시작해 UI 생성, `CameraPane` 연결, Pixhawk/ROS2/MAVLink, GNSS, `MeasurementAttempt` 생성(`:18021`), 측정 갱신(`:18304`), 세션 시작/저장(`:13864`, `:14126`), 설정 저장과 종료(`:17497`)를 모두 처리한다.
- **Module classification:** 패널은 **D UI + business logic**이면서 **E lifecycle/resource owner**다. `measurement_attempt.py`, `triangulation.py`, `rtk_geometry.py`, `recording_session.py`는 주로 **A backend**로 재사용할 수 있다. `camera_pane.py`, `yolo_stream.py`, `pixhawk_attitude.py`, `SessionSaveWorker`는 **B backend with Qt / E resource owner** 성격이 섞여 있다.
- **Why:** Workstation이 패널 메서드를 직접 복제하면 계산·기록·수명주기의 두 번째 소유자가 생긴다. 반대로 버튼 클릭을 호출하면 Workstation이 legacy 위젯 구현에 종속된다.
- **Failure scenario:** 한 UI에서 수정한 measurement gate 또는 recording state가 다른 UI에 반영되지 않거나, 두 화면이 동일한 one-consumer queue를 경쟁적으로 소비한다.
- **RECOMMENDED CHANGE:** 기능별로 최소 facade/adapter를 먼저 정의한다. `MeasurementAttempt`/controller와 `save_session_files`는 그대로 backend authority로 두고, legacy 패널의 직접 메서드는 vertical slice별로 호출 경계를 문서화한 뒤 단계적으로 추출한다.
- **Verification:** workstation 모듈이 `gui.py`의 위젯 필드에 직접 접근하지 않는 정적 검사, measurement/recording characterization test, worker 생성 수 검사를 추가한다.

### Required table — Migration Hotspots

| 모듈/클래스 | 분류 | 현재 결합/위험 | 판정 및 이동 방법 |
|---|---|---|---|
| `WirisStereoDistancePanel` | D + E | 모든 화면·명령·계산 연결·파일·종료를 한 클래스가 소유 | 직접 재사용하지 말고 facade 경계부터 정의 |
| `CameraPane` | B + C + E | Qt signal, QThread, worker 상태, preview 보관과 stop 소유 | 좌/우별 단일 owner 유지, aggregate status adapter 추가 |
| `YoloStreamWorker` | B + E | multiprocessing spawn, frame/detection queue, child reader/writer | 한 consumer 원칙 보존, Workstation은 signal adapter만 사용 |
| `DualPixhawkAttitudeSubscriber` | B + E | rclpy context/node/executor와 Qt timer 수명주기 | telemetry source adapter로 감싸고 stop idempotency 검증 |
| `DualPixhawkMavlinkSubscriber` | B + E | heartbeat·polling·connection close·parameter path가 섞임 | transport evidence와 command channel을 분리해 표시 |
| `GnssOperationManager` 및 GNSS workers | A/B + E | readiness core와 GUI worker/boolean이 분산 | readiness snapshot을 authoritative adapter로 사용 |
| `MeasurementAttempt` / `measurement_controller.py` | A | immutable snapshot, frame/unit/timestamp gate | 계산 로직 재구현 금지; thin command facade만 추가 |
| `recording_session.py` / `SpoolRows` | A | spool, generation, manifest, CSV/Parquet/XLSX publish | 저장 authority로 재사용; UI는 lifecycle command만 발행 |
| `SessionSaveWorker` | B + E | QThread 비동기 finalization | close coordinator가 완료/실패/보류를 기다리게 함 |
| `theme.py` + `gui.py` QSS | C/설정 | 토큰은 분리되어 있으나 실제 QSS는 gui.py inline | 기존 토큰/QSS를 확장하고 두 번째 theme 금지 |

## State Ownership Findings

### UI-AUDIT-003 — HIGH / State / UI 요청 플래그와 실제 증거가 중복됨

- **Design claim:** 06은 status model이 authoritative source가 되고, command intent와 state evidence를 분리해야 한다.
- **CURRENT FACT / evidence:** `_start_camera_streams()`는 `CameraPane.start_stream()`을 호출한 직후 `camera_connected=True`, `yolo_enabled=enable_detection`을 기록한다(`gui.py:13596-13616`). 실제 worker는 이후 `status_changed`, frame, stats를 비동기로 보낸다(`camera_pane.py:67-115`). `camera_stream_stats`의 `timestamp_s`도 GUI가 signal을 받은 시각으로 덮어쓴다(`gui.py:12600-12604`).
- **Why:** “연결 요청이 수락됨”, “RTSP decoder가 실행 중”, “실제 frame 수신 중”, “검출 output이 fresh”가 하나의 boolean/label로 축약된다.
- **Failure scenario:** 시작 버튼 직후 Overview가 카메라 connected로 표시되지만 첫 프레임·decoder·YOLO output은 실패한다. Workstation이 이 boolean을 실제 status로 사용하면 measurement enable 또는 recording readiness를 잘못 표시한다.
- **RECOMMENDED CHANGE:** `command_requested`, `transport_open`, `decoder_running`, `frame_fresh`, `inference_running`, `target_fresh`를 별도 evidence field로 만들고 `CameraPane`/worker signal에서 adapter가 계산한다. 라벨은 projection으로만 사용한다.
- **Verification:** 첫 frame 지연, decoder failure, stale frame, detection disabled/enabled 각각의 상태 전이를 fake worker로 검증한다.

### Required table — State Ownership

| 상태 | 실제 근거 | 현재 UI/패널 소유 | 분류 | Workstation 사용 규칙 |
|---|---|---|---|---|
| Camera L/R connected | `CameraPane.current_status`, worker status, 실제 frame | `camera_connected`, `camera_connection_label`, frame cache | UI-OWNED + DERIVED + DUPLICATED | aggregate adapter가 per-camera state와 evidence age를 계산 |
| FPS/stats | `YoloStreamWorker.stats_ready`, frame receive history | `camera_stream_stats`, runtime labels | DERIVED; receive time 재기록 | source/process/receive clock을 분리해 표시 |
| P1/P2 transport | ROS2 subscriber/node, MAVLink open/heartbeat | `pixhawk_ros2_subscribers`, `mavlink_pixhawk`, `pixhawk` alias, label | DUPLICATED | ROS2 publisher, sample, MAVLink heartbeat를 별도 row로 표시 |
| attitude freshness | `AttitudeSnapshot`/receive monotonic, PX4 source fields | `pixhawk_attitude_times`, label | DERIVED + DUPLICATED | `FRESH/STALE/VERY_STALE`와 source timestamp를 보존 |
| GNSS operation | `GnssOperationManager`, readiness snapshot, receiver worker | 여러 `gnss_*` boolean과 labels | authoritative core + UI duplicate | manager snapshot을 source로, transport와 readiness 분리 |
| RTK baseline | `GpsPositionState`, `RtkBaselineEstimate`, selected attempt | `latest_rtk_baseline` 및 alignment labels | DERIVED; latest cache는 diagnostic | selected pair의 NED baseline만 geometry input으로 사용 |
| detection enabled/running | worker inference stats/status, detection signal | `yolo_enabled`, buttons, labels | UI-OWNED + DERIVED | configured/model-loaded/enabled/running/target-present/fresh 분리 |
| target | `DetectionObservation`, timed buffers, pair matcher | `latest_left/right`, buffers | DERIVED | class, bbox pixel unit, frame/timestamp metadata를 함께 보존 |
| measurement readiness/result | `MeasurementAttempt`, controller result/rejection | `last_distance_status`, history, labels | backend authority + UI duplicate | attempt/rejection object를 source로 사용 |
| rejection reason | attempt warnings, GNSS/geometry gate | distance label/text | DERIVED but text-only projection | stable reason code와 human message를 함께 제공 |
| recording active | `SpoolRows`, session metadata, save worker/manifest | booleans, record button text, status label | UI-OWNED + DUPLICATED | `requested/active/finalizing/saved/failed/recoverable` state machine 필요 |
| active session | session id, recovery metadata, spool paths | `recording_session_id`와 panel fields | DERIVED + UI owner | session identity와 generation/manifest를 source로 사용 |
| calibration identity | applied intrinsics result/profile path/hash | `active_intrinsics`, labels, combo state | DERIVED + DUPLICATED | zoom/profile/hash/mode/frame contract을 immutable identity로 표시 |

### UI-AUDIT-004 — MEDIUM / State / 시간 상태는 강하지만 aggregate source가 없음

- **CURRENT FACT:** 카메라 metadata는 app receive `timestamp_s`, source PTS, RTCP/NTP를 별도 보존한다(`yolo_stream.py:1463-1481`, `:1610-1643`). Pixhawk/GPS state도 receive monotonic과 PX4/GPS timestamp를 함께 갖고, `MeasurementAttempt`가 selected pair와 age를 고정한다.
- **Risk:** Overview status가 sample age를 repaint 시각 또는 GUI stats callback 시각과 혼동할 수 있다. 현재 `_update_camera_connection_status()`는 최근 stats/frame 수신을 이용해 표시하지만 authoritative aggregate model은 없다.
- **RECOMMENDED CHANGE:** status adapter에 `source_clock`, `receive_monotonic_s`, `process_time`, `displayed_at`을 명시하고 freshness는 sample/receive 기준으로만 계산한다. `displayed_at`은 freshness 입력으로 사용하지 않는다.
- **Verification:** source PTS가 멈추고 frame receive만 계속되는 경우, callback 지연, stale stats, 좌우 pair skew를 각각 테스트한다.

## Command Findings

### UI-AUDIT-005 — HIGH / Command / Workstation 명령 재사용 경계가 정의되지 않음

- **Design claim:** 07은 command intent를 backend handler/controller와 연결하고, `button.click()` 기반 중복을 피하라고 한다.
- **CURRENT FACT:** 현재 명령은 대부분 패널 slot에서 입력값 검증, backend 호출, UI flag 갱신, label 갱신, cleanup을 한 번에 수행한다. recording은 `_start_session_recording()`에서 session/spool/anchor/booleans를 만들고 camera를 재시작하며, stop은 비동기 save worker를 시작한다.
- **Failure scenario:** Workstation이 기존 버튼을 호출하면 동작은 재사용되지만 UI 위젯 존재·탭 전환·label side effect에 결합된다. 새 controller를 만들면 measurement gate나 save manifest가 두 개가 된다.
- **RECOMMENDED CHANGE:** command ID와 result/evidence contract를 먼저 문서화하고, 기존 handler를 그대로 노출하는 얇은 facade부터 만든다. command result는 `accepted`, `pending`, `completed`, `failed`, `recoverable`과 source/session ID를 가져야 한다.
- **Verification:** 아래 command mapping의 precondition·success evidence·failure evidence를 fake backend로 테스트한다.

### Required table — Command Mapping

| 명령 | 현재 handler → 실제 경로 | 성공 증거 | 실패/보류 증거 | Phase 1 판정 |
|---|---|---|---|---|
| Camera connect / preview | `toggle_camera_connection` → `_start_camera_streams` → `CameraPane.start_stream` → `YoloStreamWorker` | per-camera status + 실제 frame + fresh age | worker status/error, no frame, stale | adapter 없이는 보류 |
| Detection on/off | `toggle_yolo_detection` → pane start/stop, inference process | inference stats/detection signal | model/process error, no output | camera adapter 이후 |
| Recording start/stop | `toggle_session_recording` → `SpoolRows`/video paths → async `SessionSaveWorker` | active spool/session metadata; final manifest/artifacts | save worker error, recoverable spool | lifecycle blocker 해결 전 금지 |
| Save/export | `save_distance_history_csv` → `save_session_files` | published generation + manifest | XLSX/artifact failure, spool retained | backend facade로만 |
| Calibration apply | calibration UI → `active_intrinsics`, K/D and mode refresh | applied profile/zoom/hash visible | invalid result or not applied | workflow-specific; legacy handoff |
| Pixhawk MAVLink connect | MAVLink connect → heartbeat → ATTITUDE/GPS/SCALED_IMU | open link + heartbeat + message sample | socket open without heartbeat | transport evidence 분리 |
| GNSS/RTK operation | `start_gnss_network` / base serial → worker + `GnssOperationManager` | readiness snapshot and contract ID | NO DATA/NO FIX/FLOAT/stale/operation error | manager adapter로 제한 |

### UI-AUDIT-006 — MEDIUM / Command / 일부 장비 명령이 UI thread에서 block할 수 있음

- **CURRENT FACT:** `send_zoom()`은 `send_tcp_line()`을 직접 호출한다(`gui.py:16342-16363`). 시간/일부 WIRIS command는 raw `threading.Thread`를 만든 뒤 GUI handler가 `join(timeout)`한다(`gui.py:16428-16469`, `:16606-16618`). MAVLink parameter/shell helper에는 blocking `recv_match()` 루프가 있다(`pixhawk_attitude.py:2096-2124`).
- **Why:** timeout이 있어도 응답이 늦거나 여러 대상이 동시에 실패하면 UI 이벤트 처리가 지연된다. Workstation toolbar에서 동일 handler를 더 쉽게 반복 호출할 수 있다.
- **RECOMMENDED CHANGE:** 장비 command를 command worker/future로 감싸고 UI에는 pending/result signal만 전달한다. WIRIS 시간 동기화의 동시 실행·skew 계산은 보존하되 join을 presentation thread 밖으로 이동한다.
- **Verification:** no-response, timeout, partial-left/right response, duplicate command, close-during-command를 offscreen fake transport로 검증한다.

**2026-08-09 implementation status:** Workstation의 WIRIS command/zoom/time/STIP, camera-network
queries, Pixhawk connect/stream setup/USB bootstrap/PARAM/Shell/FTP/reboot, and GNSS RTCM
delivery now use dedicated workers or a bounded queue. MAVLink command workers reserve an
exclusive per-camera receive slot so the Qt timer cannot consume PARAM/SERIAL_CONTROL responses.
Fake transport tests cover partial timeout, exclusive command access, USB cleanup, shell/FTP
ordering, and partial RTCM delivery. The historical direct paths remain only in the Legacy
owner unless separately requested.

## Threading/Lifecycle Findings

### UI-AUDIT-007 — HIGH / Lifecycle / 최상위 창 종료가 패널의 완전한 종료 경로를 우회

- **Design claim:** 09와 10은 worker stop, thread termination, recording finalization, camera/subscriber shutdown, pending save 보존을 요구한다.
- **CURRENT FACT / evidence:** 패널의 `closeEvent()`(`gui.py:17497-17525`)는 settings 저장, calibration/auto-detect/RTSP/GNSS thread wait, GNSS stop, `stop_all()`, session save wait를 수행한다. 그러나 실제 `WirisStereoDistanceGui.closeEvent()`와 `IntegratedWirisPixhawkConsole.closeEvent()`(`gui.py:20996-21025`)는 `panel.stop_all()`과 `_wait_for_session_save()`만 호출한다. `stop_all()`(`gui.py:13693-13709`)은 네 종류의 auto/RTSP thread만 wait하고, `gnss_network_thread`, base serial lifecycle와 settings save를 직접 처리하지 않는다. `_stop_camera_panes()`(`gui.py:13651-13653`)는 `CameraPane.stop()` 기본값 `wait=False`를 사용한다. `CameraPane.stop()`은 worker에 request만 보내고 반환할 수 있다(`camera_pane.py:120-142`).
- **Why:** top-level close는 panel close cleanup을 호출하지 않으므로 active GNSS/RTSP/camera/recording 조합에서 종료 순서가 코드 경로별로 달라진다. video writer와 child process가 아직 flush 중일 때 finalization이 실행될 수 있다.
- **Failure scenario:** 세션 기록 중 창을 닫으면 camera worker/AsyncVideoWriter가 아직 row/frame을 내보내는 동안 spool 저장이 끝나거나, GNSS worker가 남은 상태에서 Qt 객체가 파괴된다. 현재 spool은 recoverable하도록 설계되어 있지만, 정상 종료의 completeness를 증명할 수 없다.
- **RECOMMENDED CHANGE:** idempotent `LifecycleCoordinator`를 두고 top-level과 panel이 같은 shutdown sequence를 호출하게 한다. 순서는 command intake 차단 → recording stop/finalize → camera/decoder/writer wait → Pixhawk/ROS2/MAVLink stop → GNSS stop → auxiliary workers wait → settings save → final close로 고정한다. timeout 시 창을 닫지 말고 상태와 recoverable spool 경로를 표시한다.
- **Verification:** fake camera process, writer, ROS2/MAVLink, GNSS worker를 이용해 close 중 각 단계 지연·실패·재진입을 테스트하고 `QThread.isRunning()`과 manifest 상태를 검사한다.

### UI-AUDIT-008 — MEDIUM / Lifecycle / recording core는 견고하지만 UI state machine이 없다

- **CURRENT FACT:** `recording_session.py`의 `SpoolRows`와 `save_session_files()`는 append-only spool, update record, `.generations/*.preparing`, manifest, atomic published pointer, XLSX 실패 시 spool 보존을 구현한다(`recording_session.py:418`, `:1771`, `:1846-1969`). 이는 좋은 backend 경계다.
- **Risk:** 패널은 `_start_session_recording()`에서 `distance_recording_enabled=True`, `video_recording_enabled=True`를 path/writer의 실제 준비 신호와 별개로 설정한다(`gui.py:13864-13949`). stop은 flag를 먼저 끄고 camera를 재시작한 뒤 저장 worker를 시작한다(`gui.py:13952-13971`).
- **RECOMMENDED CHANGE:** recording command의 `requested → preparing → active → stopping → finalizing → saved/failed/recoverable` 전이를 manifest와 worker signal에 연결한다. camera/video writer readiness를 별도 evidence로 기록하고, UI flag는 projection으로만 둔다.
- **Verification:** start 중 camera failure, stop 중 XLSX failure, close 중 finalization, recovered `.spool` 재개를 각각 검증한다.

## Navigation Findings

### UI-AUDIT-009 — MEDIUM / Navigation / 제안 트리의 선택과 실행 의미를 분리해야 함

- **DESIGN INTENT:** 05의 Navigation tree는 Overview, Cameras, Pixhawk, GNSS/RTK, Detection, Measurement, Synchronization, Alignment, Calibration, Recording, Playback, Export, Diagnostics를 후보로 둔다. selection은 command/stop을 실행하지 않아야 한다.
- **CURRENT FACT:** 실제 기능은 10개 top-level tab와 nested tab으로 구성되고, 연결/정렬/고급 화면 안에 Pixhawk, GNSS, MAG, WIRIS command가 함께 있다. Selection-driven workspace host나 dedicated Playback API는 확인되지 않았다.
- **RECOMMENDED CHANGE:** Navigation node는 stable ID를 가진 context selection만 수행하고 command는 toolbar/action registry로 분리한다. disconnected device도 선택 가능하게 하되 selection이 connection을 시작하지 않게 한다.
- **Verification:** navigation selection이 worker/thread/recording state를 변경하지 않는 UI test, device disconnected selection test를 추가한다.

## Workspace Findings

### Required table — Workspace Decision

| 후보 | 판정 | 근거 | Phase 1 처리 |
|---|---|---|---|
| Overview | KEEP, read-only | 여러 source의 요약은 유용하지만 현재 authoritative model 없음 | adapter가 제공한 source/age/quality만 표시 |
| Cameras | MERGE with Detection | 실제 `CameraPane`와 YOLO가 같은 stream worker lifecycle을 공유 | Camera local tabs; adapter 전까지 legacy handoff |
| Pixhawk | MERGE into Alignment/Diagnostics | ROS2/MAVLink 두 transport와 MAG/PX4 control이 분산 | dedicated command workspace 금지, evidence panel부터 |
| GNSS/RTK | MERGE into Alignment/Diagnostics | `GnssOperationManager` readiness와 GPS sample/baseline이 별도 source | readiness/source/age를 한 inspector group으로 |
| Detection | MERGE with Cameras | inference 상태는 camera stream과 결합 | model/running/target freshness를 local section으로 |
| Measurement | KEEP, but backend facade required | `MeasurementAttempt`와 controller는 명확한 vertical slice | geometry logic 재구현 없이 read-only result + guarded command |
| Synchronization | KEEP as Time / Sync | PC UTC, WIRIS RTC, RTCP/NTP evidence, and editable sync limits form an independent field workflow | Measurement keeps read-only synchronization state and rejection reason |
| Alignment | KEEP as workflow-specific | baseline/attitude/MAG/GNSS 검증은 순차 workflow가 있음 | 기존 정렬 기능을 직접 복제하지 않음 |
| Calibration | KEEP as workflow-specific | capture/calculate/apply/save의 상태 전이가 분명함 | Phase 1은 legacy handoff/read-only profile |
| Recording | MERGE with global action + Save/Review local tab | 실제 기록은 cross-cutting lifecycle이고 save backend가 강함 | coordinator 이후에만 action wiring |
| Playback | DEFER | 현재 dedicated playback source/API가 확인되지 않음 | 구현 약속 금지 |
| Export | DEFER or MERGE with Save/Review | `save_session_files()`는 export backend지만 workspace semantics는 없음 | 파일/manifest 목록만 adapter로 |
| Diagnostics | KEEP as dock | 실제 log/measurement/worker evidence가 있음 | source-backed tabs만 허용 |

## Inspector Findings

### UI-AUDIT-010 — LOW / MEDIUM / Inspector / 정보의 빈도와 위험도 구분 필요

- **FREQUENT:** camera live state, pair Δt, measurement validity, recording state, P1/P2 freshness, GNSS readiness summary.
- **OCCASIONAL:** camera URLs, stream transport, Pixhawk namespaces, baseline source, applied intrinsics profile.
- **ADVANCED:** PX4 parameters, MAVLink shell, WIRIS SDK commands, MAG priority, Ethernet bootstrap.
- **WORKFLOW-SPECIFIC:** calibration capture/apply, GNSS survey-in/NTRIP/base serial, optical alignment, recording recovery.
- **READ-ONLY:** `MeasurementAttempt` provenance, frame/axis/unit contract, selected GPS/attitude age, manifest/generation, rejection reason.

`x/y/z`를 일반 Inspector 값으로 만들면 의미가 사라진다. 실제 모델은 `N/E/D`, `Forward/Right/Down`, `Right/Down/Forward`, `east/north/up`, pixel `u/v`를 각각 이름과 frame annotation과 함께 표시해야 한다.

## Diagnostics Findings

### UI-AUDIT-011 — MEDIUM / Diagnostics / “실제 source-backed tab” 범위가 설계안보다 작음

- **CURRENT FACT:** 실제 source는 camera worker stats/frame metadata, `camera_yolo_log`, `pixhawk_log`, `wiris_log`, GNSS operation/readiness, measurement attempt/rejection, recording spool/manifest, telemetry history에 있다.
- **UNSUPPORTED ASSUMPTION:** `System`, `Log`, `Measurement`만 있으면 Camera/PX4/GNSS/Performance를 동일한 품질로 즉시 제공할 수 있다는 전제. 현재는 각 source의 lifecycle과 ownership이 통합되지 않았다.
- **RECOMMENDED CHANGE:** Phase 1 Diagnostics는 `System lifecycle`, `Source-backed logs`, `Measurement attempt` 세 가지로 제한한다. Camera/PX4/GNSS/Performance는 각 adapter가 source, clock, age, quality, fault를 제공한 뒤 추가한다. Diagnostics는 source of truth가 아니라 evidence projection이어야 한다.
- **Verification:** 각 diagnostic field에 producer path와 sample age를 표시하고, no-data/stale/fault 상태가 빈 문자열이나 정상 색상으로 축약되지 않는지 테스트한다.

## Settings Findings

### UI-AUDIT-012 — LOW / Settings / engineering 설정과 UI preference를 분리해야 함

- **CURRENT FACT:** `settings.py`의 `AppSettings`가 RTSP, calibration, Pixhawk, GNSS, YOLO 등 engineering configuration을 JSON으로 load/save한다. `gui.py:_apply_visual_style()`는 `theme.py` 토큰과 inline QSS를 사용한다.
- **DESIGN INTENT:** 08은 centralized design tokens와 UI-only geometry/dock/last workspace 저장을 제안한다.
- **RECOMMENDED CHANGE:** engineering settings를 두 번째 Workstation store로 복제하지 않는다. UI geometry/dock/last workspace만 별도 namespaced preference로 두고, existing `AppSettings` path/serialization을 재사용한다. theme.py token과 기존 QSS를 확장하되 두 번째 theme system을 만들지 않는다.
- **Verification:** legacy/workstation 양쪽에서 같은 engineering settings path와 calibration identity를 읽고, UI preference가 measurement/recording config를 덮어쓰지 않는지 테스트한다.

## Reusable Component Findings

### 재사용 등급

| 항목 | 등급 | 조건 |
|---|---|---|
| `MeasurementAttempt`, `measurement_controller.py`, `triangulation.py` | REUSE AS-IS backend | 호출 전에 immutable snapshot/frame contract를 유지 |
| `recording_session.py`, `SpoolRows`, `save_session_files()` | REUSE AS-IS backend | lifecycle coordinator가 flush/finalize를 소유 |
| `GnssOperationManager`, readiness dataclasses | REUSE WITH ADAPTER | transport state와 measurement readiness를 분리 표시 |
| `CameraPane` | REUSE WITH ADAPTER | 하나의 pane/worker owner, status/frame/stats signal만 소비 |
| `YoloStreamWorker` | REUSE WITH ADAPTER | multiprocessing/queue/one-consumer/stop semantics 보존 |
| `DualPixhawkAttitudeSubscriber`, `DualPixhawkMavlinkSubscriber` | REUSE WITH ADAPTER | ROS2/MAVLink source와 heartbeat/sample/age를 분리 |
| `SessionSaveWorker` | REUSE WITH LIFECYCLE FACADE | close가 완료/실패/보류를 기다림 |
| `DistanceHistoryPlot`, `TelemetryHistoryPlot`, `Triangulation3DView`, `RtkAlignmentPlot` | REUSE UI component | data model을 직접 읽지 말고 view model을 받음 |
| `theme.py` tokens / existing QSS | REUSE/EXTEND | duplicate visual system 금지 |
| `WirisStereoDistancePanel` | REFACTOR FIRST | Workstation의 source-of-truth로 직접 사용 금지 |
| label text, button text, log widgets | DO NOT REUSE AS STATE | display projection으로만 사용 |

## Test Coverage Findings

### UI-AUDIT-013 — MEDIUM / Test / Workstation 계약 테스트는 추가됐지만 전체 Qt 종료 검증은 남음

### CURRENT FACT

| 영역 | 현재 증거 | 분류 | 공백 |
|---|---|---|---|
| coordinate/triangulation | `test_coordinate_contract.py`, `test_triangulation.py`, geometry common | unit | Workstation field mapping/label smoke 추가; hardware geometry는 별도 |
| measurement attempt/controller | immutable snapshot, pair identity, fail-closed tests | unit/integration-ish | Workstation은 기존 result/history만 표시 |
| recording | `test_recording_session.py`, selected `test_stereo.py` | unit/integration-ish | fake close coordinator 추가; active writer 장시간 검증은 남음 |
| GNSS/RTK | `test_gnss_operation.py`, `test_rtk_geometry.py`, network tests | unit/mock | real receiver/caster/Pixhawk chain 없음 |
| camera/YOLO | `test_yolo_stream.py`, selected Qt tests | unit/component | multiprocessing stop, stale/failure aggregate status partial |
| Pixhawk | `test_pixhawk_attitude.py`, selected triangulation | unit/mock | MAVLink auxiliary lifecycle, close path 없음 |
| settings/calibration | settings/calibration tests | unit | legacy/workstation shared preference contract 없음 |
| launcher | `_parse_ui_mode`, `test_workstation.py` | unit/smoke | invalid mode/help and real process mode validation remains |
| Workstation UI | `tests/test_workstation.py` + offscreen real-panel smoke | component/smoke | field hardware and full Qt teardown remain |

### 실행한 테스트

- Workstation targeted regression **58개 통과**; 좌표·삼각측량·immutable attempt·controller·recording 회귀는 아래 full suite에 포함
- Workstation standalone panel + `StandalonePanelBackend` no-device shell smoke 통과: overview/camera/detection/Pixhawk/GNSS/alignment/calibration/recording/session-review/artifacts/advanced/diagnostics 전환 및 close
- 전체 `StereoWIRIS/tests` **672 passed**; device-candidate discovery, deferred command completion, WIRIS/Pixhawk blocking-I/O worker, Pixhawk connect/USB/stream/RTCM worker, retryable camera shutdown, standalone backend conformance를 포함해 새로 실행
- compileall 통과

### 구현 후 판정

다음 항목은 구현으로 추가됐다.

- launcher mode parser/offscreen window construction test
- Workstation shell/navigation/command/close smoke
- coordinate-labelled Measurement workspace with left/right frame proxies and source-backed
  detection/sync/GNSS/recording/FPS evidence
- source-backed Alignment/Pixhawk/GNSS tables, Calibration profile/metric tables, and
  Recording/Review/Export session artifact tables
- Workstation Measurement settings and Time/Sync limits Apply/Save, Calibration setup Apply/scan/Save,
  Alignment source/sampling/profile/swap commands, and session CSV preview/open controls
- phase-one direct camera discovery/RTSP/control, Pixhawk discovery/MAVLink, and GNSS
  Base serial/Survey-in/NTRIP command facades, plus calibration capture/run/save/load/apply/
  validation commands, with explicit legacy-owner delegation
- read-only Inspector context details and Detection configuration/target bbox-pixel evidence
- model-based System/Measurement tables and command failure propagation
- retryable lifecycle coordinator unit test

상태 전이의 하드웨어 timing 검증과 hardware smoke는 별도 검증으로 남긴다. 현재
통과는 software shell/adapter correctness의 증거이지 현장 장비
운용 완료의 증거가 아니다.

전체 재현 명령은
`QT_QPA_PLATFORM=offscreen PYTHONPATH=StereoWIRIS Simulation/.venv/bin/python -m pytest -q StereoWIRIS/tests -x`이다.
operator의 `NETWORK_RTK_DIRECT`/`pixhawk_ethernet_mode=mavlink` 설정에 의존하던
`test_gnss_base_survey_in_command_status_moves_past_waiting`와
`test_ros2_failure_starts_mavlink_recovery_and_fallback`는 테스트 안에서 repository
default `AppSettings`를 사용하도록 격리했다. session artifact list를 현재 저장 owner의
GNSS/workbook 산출물과 맞추고 close test의 `stop_all(wait=True)` stub을 명시한 뒤,
`QT_QPA_PLATFORM=offscreen PYTHONPATH=StereoWIRIS Simulation/.venv/bin/python -m pytest -q StereoWIRIS/tests -x`
가 **672 passed**로 완료됐다. preview worker lazy-start와 새 Pixhawk device-I/O worker
검증도 포함한다.

## Two-Day Feasibility

### Required table — Phase 1 Feasibility

원래 09 계획에 대한 판정:

| 계획 항목 | 2일 내 가능성 | 이유 |
|---|---|---|
| branch/package/shell/menu/toolbar/dock | 완료 | launcher와 close contract를 먼저 구현하고 offscreen 확인 |
| navigation/workspace host | 완료 | stable context ID와 QStackedWidget host 구현 |
| Overview/basic status | 완료 조건부 | backend evidence adapter 구현; field semantics는 hardware 확인 필요 |
| Camera workspace | 완료 조건부 | CameraPane signal만 연결; actual RTSP/frame은 hardware 확인 필요 |
| Measurement workspace | 완료 조건부 | 기존 history/attempt와 명시 contract에 좌/우 frame proxy, sync, GNSS/NED baseline, detection, recording, FPS evidence 추가; geometry 재계산 없음 |
| Recording command + review evidence | 완료 조건부 | 기존 session owner facade, artifact projection, and close coordinator 사용; 장시간 writer 검증 필요 |
| Diagnostics | 완료 | 기존 logs/status/measurement attempt 기반 System/Log/Measurement |
| full inspector/calibration/PX4/GNSS parity | operation projection 완료 조건부 | source-backed values/tables and owner-backed commands are present; setup-field parity and hardware validation remain |
| regression/smoke/visual review | 최소 0.5–1일 필요 | 새 launcher/status/lifecycle 계약 테스트가 필요 |

원래의 계획은 launcher/close/adapter gate를 선행하는 조건부 MVP로 실행됐고, shell·camera
evidence·measurement evidence·recording facade·diagnostics까지 구현됐다. 다만 아래
하드웨어 및 전체 Qt 조건을 충족하기 전에는 현장 operational parity로 승격하지 않는다.

## Revised Two-Day MVP

NO-GO 차단 조건을 해소한 **adapter-backed Phase 1 MVP**다. 측정·기록 계산/저장 로직을
새 UI가 소유하지 않는 조건에서 화면·명령 facade까지 구현한다.

### Day 1 — 실행·수명주기·어댑터 기반

- `--ui legacy|workstation` parser와 legacy default를 구현했다.
- one-window/one-owner 실행 계약과 기존 legacy fallback을 유지한다.
- top-level close가 공유 lifecycle coordinator를 호출하며 retryable step test를 통과한다.
- `StatusModel`/`BackendStatusAdapter`가 producer/source/age/quality/clock을 표시한다.
- 좌표 필드는 `N/E/D`, `FRD`, `RDF`, `baseline_center_frd`처럼 명시 이름만 허용한다.

### Day 2 — 안전한 Workstation shell

- menu/toolbar/navigation tree/central host/diagnostics dock/status bar를 구현했다.
- Overview는 adapter가 확인한 상태만 표시한다.
- Camera/Measurement는 기존 signal/history/attempt를 이용하는 evidence view다. Measurement는
  좌/우 frame proxy, detection, sync, GNSS/NED baseline, recording, FPS를 함께 표시한다.
- Recording/Review/Export와 Advanced owner tools는 기존 session owner command facade와 source-backed session/artifact projection을 사용한다. session catalog, manifest error, CSV preview, XLSX/Parquet/video artifact open은 Workstation에서 제공하며, workbook 생성은 기존 session-save writer, video 재생은 시스템 연결 player를 사용한다.
- Alignment/Pixhawk/GNSS와 Calibration은 owner-backed operation facade와 source-backed evidence를 제공한다. Calibration setup-field parity, measurement tuning, alignment sampling/profile 작업과 advanced preview/zoom·PX4·field browser 작업은 Workstation에서 직접 호출하며 계산/저장/resource owner는 기존 panel이다.
- diagnostics는 System/Camera/Pixhawk/GNSS/Log/Measurement/Performance source-backed 탭이다.
- offscreen smoke와 1366×768 layout 확인을 수행했다.

### MVP acceptance gate

- 두 mode가 실제로 선택되고, 기본 legacy 실행이 유지된다.
- Workstation이 worker/queue/subscriber를 두 번째로 만들지 않는다.
- close가 active session의 spool/manifest 상태를 잃지 않고, 모든 in-scope worker 종료를 관찰한다.
- engineering values에 가능한 source, age/clock, unit, frame/axis가 표시되며, no-data는 명시적으로 `--`/`UNKNOWN`으로 표시된다.
- measurement/recording backend 코드를 복제하지 않는다.

## Design Documents Changed

감사 및 후속 구현에서 다음 문서를 추가·수정했다.

- `11_DESIGN_AUDIT_RESULT.md`: baseline findings, 구현 증거, 조건부 판정 갱신
- `09_IMPLEMENTATION_PLAN.md`: 구현 상태와 Phase 1 acceptance 조건 반영
- `10_DESIGN_AUDIT.md`: 감사 결과와 후속 구현 상태 링크 반영
- `docs/ui/workstation/README.md`: Workstation 문서 패키지 안내 유지
- `StereoWIRIS/UPDATE.md`: 감사 및 구현 기록을 한국어 changelog 형식으로 추가
- `workstation/`: shell/adapter/evidence/status/command/workspace/runtime-path 구현
- `sasl_common/lifecycle.py`: Legacy/Workstation이 공유하는 UI-neutral retryable lifecycle 구현
- `StereoWIRIS/tests/test_workstation.py`: offscreen shell/launcher/lifecycle/status/upstream/source-projection/command 계약 테스트

00–08과 `AUDIT_PROMPT.md`는 감사 기준으로 읽었으며 구조를 임의로 덮어쓰지 않았다. 사용자 기존 코드 변경과 untracked 파일은 감사 범위에 포함시키되 삭제하지 않았다.

## Remaining Unknowns

- 실제 WIRIS RTSP decoder/AsyncVideoWriter가 stop request 뒤 flush를 완료하는 시간과 종료 실패 양상
- 실제 Pixhawk MAVLink subscriber가 재연결·종료될 때의 thread affinity와 heartbeat/sample 회복 시간
- GNSS Base serial, NTRIP caster, Rover Pixhawk heartbeat/RTK Fixed가 결합된 현장 readiness와 재연결 시간
- 1920×1080, 1600×900, 1366×768에서 새 source-backed table/path 값이 실제 운용 데이터로 채워졌을 때의 clipping/scroll/읽기성
- 실제 장비에서 두 카메라 WIRIS/Pixhawk timeout·partial response가 UI와 장비 재연결에 미치는 지연
- 시스템 연결 프로그램의 artifact open 결과와 장시간 recording state 영향

이 항목들은 정적 감사만으로 확정할 수 없으며 hardware/manual smoke와 fake lifecycle test가 필요하다.

## Implementation Blockers

| ID | Severity | 차단 조건 | 해제 기준 |
|---|---|---|---|
| B-01 | HIGH | 실제 `--ui` mode selector 없음 | **해결** — parser/default/one-window smoke 통과 |
| B-02 | HIGH | top-level close가 panel cleanup을 우회하고 camera/GNSS/save 종료를 완전히 기다리지 않음 | **소프트웨어 해결 조건부** — coordinator/retry test 통과, 실장비 timeout pending |
| B-03 | HIGH | Workstation이 사용할 authoritative status/command adapter 없음 | **해결 조건부** — `EvidenceAdapter`/`BackendStatusAdapter` 구현, field transition은 hardware pending |
| B-04 | HIGH | measurement/recording lifecycle이 giant panel에 갇혀 있음 | **해결** — thin facade로 기존 backend 단일 owner 유지 |
| B-05 | MEDIUM | 실제 WIRIS/Pixhawk command timeout·partial response와 MAVLink 재연결 미검증 | **잔여 조건** — Workstation 범위 밖의 현장/manual validation 필요 |
| B-06 | MEDIUM | Workstation shell/navigation/lifecycle/offscreen test 없음 | **해결** — `test_workstation.py`와 offscreen smoke 추가 |

## Final Recommendation

### Verdict: GO WITH CONDITIONS

Workstation의 shell, Camera/Measurement evidence, Alignment/Calibration/Recording source
projection, recording command facade, diagnostics, and Advanced owner workspace 구현을 승인한다.
코어 측정·저장·좌표 계약은 기존 panel과
기록 코어가 단일 owner로 유지한다. 다음 조건을 충족하기 전에는 현장 운용 완료나
legacy parity를 주장하지 않는다.

1. 실제 RTSP 두 스트림에서 frame/fps/stop 및 preview 단일 소비를 확인한다.
2. Pixhawk MAVLink heartbeat/sample, GNSS/RTK readiness, recording finalization을
   장비와 함께 확인한다.
3. clean settings와 clean Qt teardown 조건에서 전체 `StereoWIRIS/tests`를 재실행한다.
4. 1920×1080, 1600×900, 1366×768 화면에서 clipping/읽기성을 수동 확인한다.

현재 구현 범위는 위 조건을 충족할 때까지 기존 panel을 기술적 resource owner로 유지하면서
Workstation typed facade로 조작 surface를 이관하는 단계적 migration이다.

Coordinate contract checked: `coordinate_contract.py`, `triangulation.py`, `measurement_attempt.py`, `rtk_geometry.py`, `baseline_alignment.py`, `recording_session.py`, `docs/COORDINATE_CONTRACT.md`; world NED, PX4/body FRD, camera optical/left-camera RDF, baseline-center FRD, ENU alignment, WGS84 GPS, pixel px, monotonic/source timestamps와 해당 회귀 테스트를 대조했다.
