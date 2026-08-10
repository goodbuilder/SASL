# StereoWIRIS Workstation UI — Design Documentation + Adversarial Code Audit

## 역할

당신은 이 저장소의 UI 구현자가 아니다.

당신은 다음 역할을 동시에 수행하는 독립적인 수석 검증 엔지니어다.

- Senior Software Architect
- Senior Qt / PyQt Engineer
- Engineering Workstation UI Architect
- Adversarial Code Auditor
- Regression Risk Reviewer

현재 목표는 새로운 UI를 구현하는 것이 아니다.

현재 목표는:
1. Workstation UI 설계 문서를 저장소에 정착시키고
2. 설계를 실제 StereoWIRIS 코드와 대조하고
3. 잘못된 가정, 누락 기능, 상태/명령 소유권 문제, migration 위험을 찾고
4. 구현 가능한 수준으로 설계를 수정하는 것이다.

기존 설계를 호의적으로 해석하지 마라. 문서보다 실제 코드와 런타임 의미를 우선하라.

---

## 0. 절대 규칙

이번 작업에서는 실제 Workstation UI를 구현하지 마라.

금지:
- 기존 UI 대규모 수정
- gui.py 대규모 refactor
- WorkstationMainWindow/NavigationTree/Toolbar/Inspector/Workspace 구현
- backend behavior 변경
- measurement algorithm 변경
- coordinate/frame semantics 변경
- recording format 변경
- calibration math 변경
- PyQt5 → PyQt6 migration
- dependency upgrade
- 불필요한 formatting/refactoring

허용:
- 문서 작성/수정
- 코드 읽기
- 정적 분석
- 기존 테스트 실행
- 안전한 read-only 실행 검사
- 최소 분석용 임시 스크립트
- 감사 결과 문서 작성

---

## 1. 최우선 원칙

우선순위:
1. 실제 코드
2. 실제 runtime behavior
3. 기존 테스트
4. 실제 데이터 흐름
5. 기존 설정/파일 포맷
6. 설계 문서
7. 이름/주석/commit message

반드시 구분:
- CURRENT FACT
- DESIGN INTENT
- UNSUPPORTED ASSUMPTION
- RECOMMENDED CHANGE

---

## 2. 먼저 저장소를 조사하라

수정 전에 repository root, current branch, HEAD, working tree status, untracked files, docs/test 구조, application entry point, Python package 구조를 확인한다.

사용자 미커밋 변경을 덮어쓰거나 삭제하지 마라.

---

## 3. 설계 문서 위치

기존 docs convention을 확인한 뒤 적절한 위치(예: `Workstation/docs/ui/`)를 선택한다.

확인 대상:
- 00_PRODUCT_VISION.md
- 01_USER_WORKFLOW.md
- 02_INFORMATION_ARCHITECTURE.md
- 03_WORKSPACE_LAYOUT.md
- 04_COMPONENT_SYSTEM.md
- 05_NAVIGATION.md
- 06_STATUS_MODEL.md
- 07_COMMAND_MODEL.md
- 08_DESIGN_SYSTEM.md
- 09_IMPLEMENTATION_PLAN.md
- 10_DESIGN_AUDIT.md

감사 전에는 FINAL로 취급하지 않는다. 목표는 DRAFT → AUDIT → REVISED다.

---

## 4. 실제 코드 조사

다음 또는 동등한 파일을 실제 저장소에서 찾아 읽는다.

- main.py
- wiris_stereo_distance/gui.py
- camera_pane.py
- measurement_widgets.py
- measurement_controller.py
- measurement_attempt.py
- pixhawk_attitude.py
- px4_mag_manager.py
- rtk_geometry.py
- recording_session.py
- settings.py
- network.py
- calibration.py
- alignment_managers.py
- sync.py
- theme.py

파일명이나 주석이 아니라 실제 classes, signals/slots, state variables, handlers, timers, threads, cleanup paths, read/write ownership, UI-side calculations를 근거로 판단한다.

---

## 5. gui.py 특별 감사

다음으로 분류한다.
- Presentation only
- Command logic
- Business logic
- Lifecycle logic
- State ownership
- File/configuration ownership

UI widget state 또는 label text가 application state의 사실상 source of truth인 부분을 반드시 찾는다.

---

## 6. Backend/UI 결합도 분류

A — Pure backend
B — Backend with Qt dependency
C — Reusable UI component
D — UI + business logic mixed
E — Lifecycle/resource owner
F — Unknown / requires runtime verification

D/E를 migration hotspot으로 취급한다.

---

## 7. 실제 데이터 흐름 추적

### Camera
source → receiver/decoder → frame ownership → UI → FPS/state

구분: connection, RTSP availability, decoder, actual frame reception, frame timestamp, FPS, preview state.

### Pixhawk
PX4/MAVLink/ROS2 → receiver → parser/model → UI

확인: connection semantics, heartbeat, attitude, timestamps, freshness, multiple transport paths.

### GNSS / RTK
receiver → fix/RTK → baseline/geometry → UI

보존: NO DATA, NO FIX, FLOAT, FIX, stale, eph/epv, satellites, timestamp.

### Detection
model/runtime → inference → target → UI/measurement

구분: configured, model loaded, enabled, running, inference producing, target present, output stale.

### Measurement
camera/detection + timing + attitude + calibration + baseline + geometry → attempt → validation/rejection → final result → UI/recording

반드시 확인: required inputs, coordinate frames, units, timestamps, calibration, attitude, synchronization, baseline validity, rejection reason, quality, stale semantics.

### Recording
start request → validation → session creation → active → stop → flush/finalization → saved/error → shutdown

확인: authoritative state owner, session ID, output validation, partial failure, async behavior, stop, shutdown.

---

## 8. 명령 경로 감사

추적:
- Camera Connect/Disconnect
- Preview Start/Stop
- Detection Enable/Disable
- Recording Start/Stop
- Save/Export
- Calibration Run/Save/Activate
- Pixhawk Connect
- GNSS/RTK-related commands

각각:
Command → UI handler → controller/helper → backend → success evidence → failure evidence → state update → cleanup

---

## 9. 상태 소유권 감사

다음의 authoritative source를 찾고 분류한다.

- Camera Left/Right
- FPS
- Pixhawk 1/2
- attitude freshness
- GNSS
- RTK
- detection runtime/target
- measurement/latest measurement/rejection reason
- recording/active session
- calibration identity

분류:
AUTHORITATIVE / DERIVED / UI-OWNED / DUPLICATED / UNKNOWN

UI-OWNED 또는 DUPLICATED이면 finding을 만든다.

---

## 10. 시간 의미 감사

확인:
- timestamp source
- clock domain
- unit
- receive/sample/processing/display time
- monotonic vs wall clock

금지:
UI repaint time = data freshness
GUI timer callback = sample timestamp

---

## 11. 좌표계와 단위 보호

position, baseline, attitude, roll/pitch/yaw, range, pixel coordinate, timestamps에 대해 가능한 경우 Frame, Unit, Axis convention, Rotation convention, Reference, Timestamp를 기록한다.

NED/ENU/FRD/RDF/camera optical frame 등을 임의로 통일하지 마라.

---

## 12. Navigation / Workspace / Inspector 감사

각 proposed node/workspace를 KEEP / MERGE / MOVE / REMOVE / DEFER로 판정한다.

질문:
- dedicated workspace가 정말 필요한가?
- Inspector면 충분한가?
- Diagnostics tab이면 충분한가?
- command/dialog이어야 하는가?
- 현재 기능이 빠졌는가?

Inspector 설정은 FREQUENT / OCCASIONAL / ADVANCED / WORKFLOW-SPECIFIC / READ-ONLY로 분류한다.

---

## 13. Diagnostics 감사

System, Log, Measurement, Camera, PX4/MAVLink, GNSS, Performance 각각 실제 데이터 source가 있는지 확인한다. 없는 것은 Phase 1에 약속하지 않는다.

Diagnostics는 evidence를 보여주며 새로운 truth를 만들지 않는다.

---

## 14. Threading / concurrency 감사

검색:
QThread, threading.Thread, multiprocessing, Timer, QTimer, queue, callback, signal/slot, worker object

확인:
- ownership
- one-consumer queue
- shared mutable state
- UI thread affinity
- shutdown ordering
- blocking calls
- duplicate polling

---

## 15. Legacy coexistence / shutdown

안전한 기본 해석:
`--ui legacy` 또는 `--ui workstation`으로 한 run에서 하나의 operational UI만 instantiate.

두 operational window를 동시에 띄우는 방식은 backend ownership 안전성이 증명되지 않으면 제안하지 않는다.

Shutdown 경로를 반드시 추적한다:
closeEvent → recording finalize → worker stop → camera/network release → thread join → file flush → exit

Shutdown regression은 HIGH 이상, data loss 가능성은 CRITICAL 가능.

---

## 16. Settings / reusable widgets / theme

실제 설정 source와 owner를 찾는다. Workstation용 별도 engineering setting store를 만들지 않는다. UI layout preferences만 별도 허용 가능.

기존 widget은 REUSE AS-IS / REUSE WITH ADAPTER / REFACTOR FIRST / DO NOT REUSE로 분류한다.

08_DESIGN_SYSTEM.md를 current theme.py/QSS와 비교하고 가능하면 기존 theme infrastructure를 확장한다.

---

## 17. 화면 밀도 / workflow / 2일 feasibility

검토 해상도:
1920x1080, 1600x900, 1366x768

대표 workflow:
W1 Startup
W2 Camera
W3 Measurement
W4 Recording
W5 Failure
W6 Calibration

2일 scope 항목을 GREEN/YELLOW/RED로 평가한다:
- shell
- navigation
- overview
- status
- camera workspace
- measurement workspace
- recording command
- diagnostics
- inspector
- launcher

필요하면 2일 MVP를 더 작게 줄인다. 시각적 완성보다 정확한 state/backend reuse가 우선이다.

---

## 18. 테스트

기존 테스트를 unit/integration/UI/hardware/manual로 분류한다.

coverage 확인:
measurement, recording, settings, shutdown, camera, PX4, GNSS/RTK.

안전한 테스트만 실행한다. 하드웨어를 움직이거나 외부 설정을 변경하거나 데이터를 삭제하는 테스트는 안전하다는 증거 없이 실행하지 않는다.

---

## 19. Findings 형식

UI-AUDIT-XXX

Severity: CRITICAL / HIGH / MEDIUM / LOW
Category: Architecture / State / Command / Threading / Lifecycle / Navigation / Workspace / Inspector / Diagnostics / Settings / Testing / Visual / Migration

Design Claim:
Evidence:
Code Location:
Why It Matters:
Failure Scenario:
Recommended Correction:
Verification:

HIGH/CRITICAL finding에는 구체적인 code evidence가 필요하다.

---

## 20. Audit Result 문서

`11_DESIGN_AUDIT_RESULT.md`를 생성한다.

포함:
- Executive Summary
- Repository Baseline
- Architecture Findings
- State Ownership Findings
- Command Findings
- Threading/Lifecycle Findings
- Navigation Findings
- Workspace Findings
- Inspector Findings
- Diagnostics Findings
- Settings Findings
- Reusable Component Findings
- Test Coverage Findings
- Two-Day Feasibility
- Revised Two-Day MVP
- Design Documents Changed
- Remaining Unknowns
- Implementation Blockers
- Final Recommendation

표:
1. Migration Hotspots
2. State Ownership
3. Command Mapping
4. Workspace Decision
5. Phase 1 Feasibility

---

## 21. 최종 구현 Gate

판정:
GO / GO WITH CONDITIONS / NO-GO

GO WITH CONDITIONS이면 조건을 번호로 명시한다. NO-GO이면 blocker를 명시한다.

이번 작업에서는 실제 Workstation UI를 구현하지 않는다.

---

## 22. Git 안전 규칙

현재 branch/status를 확인한다. 별도 branch 권장: `feature/ui-workstation-refactor`.

사용자의 미커밋 변경을 임의로 이동/삭제하지 마라.

금지:
- git reset --hard
- git clean -fd
- git checkout -- .
- git restore .

사용자가 요청하지 않았다면 임의 commit하지 마라.

---

## 23. 완료 조건

- repository baseline 확인
- 11개 설계 문서 확인
- 실제 코드 감사
- gui.py coupling 분석
- command paths 분석
- state ownership 분석
- threading/lifecycle/shutdown 분석
- settings ownership 분석
- representative workflow 분석
- tests 조사 및 안전한 테스트 실행
- findings 작성
- 2일 계획 재평가
- 필요한 설계 문서 수정
- 11_DESIGN_AUDIT_RESULT.md 작성
- GO / GO WITH CONDITIONS / NO-GO 판정

---

## 24. 최종 응답 형식

### Audit Verdict
GO / GO WITH CONDITIONS / NO-GO

### Critical Findings
CRITICAL/HIGH만 간략히.

### Two-Day Scope
INCLUDE / DEFER

### Documents Changed

### Tests
실행 / 결과 / 미실행

### Remaining Blockers

### Next Step
Audit 결과를 기준으로 Workstation UI implementation plan을 확정할 준비가 되었습니다.

---

## 핵심 원칙

문서가 맞다는 것을 증명하려 하지 마라.

성공 기준은 스크린샷이 아니다.

- 기존 기능 보존
- 상태 의미 보존
- measurement semantics 보존
- coordinate/frame semantics 보존
- recording lifecycle 보존
- shutdown correctness 보존
- backend logic duplication 방지
- UI 구조 개선

> Architecture is approved only after its assumptions survive contact with the implementation.
