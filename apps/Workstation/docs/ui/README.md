# StereoWIRIS Workstation UI Design Package

StereoWIRIS를 산업용/엔지니어링 Workstation UI로 재설계하기 위한 문서 세트입니다.

포함 파일:
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
- AUDIT_PROMPT.md
- 11_DESIGN_AUDIT_RESULT.md
- 12_UI_REFINEMENT_PLAN.md

권장 순서:
1. 문서를 저장소의 `apps/Workstation/docs/ui/`에 배치합니다.
2. `AUDIT_PROMPT.md`를 Codex에 입력합니다.
3. 실제 코드 감사를 수행하여 `11_DESIGN_AUDIT_RESULT.md`를 생성합니다.
4. CRITICAL/HIGH finding과 NO-GO/조건부 판정을 검토합니다.
5. 승인된 구현에서는 `09_IMPLEMENTATION_PLAN.md`의 gate와 `11_DESIGN_AUDIT_RESULT.md`의
   조건을 따라 launcher, lifecycle, adapter, shell을 단계적으로 검증합니다.
6. 실장비·장시간 종료·전체 Qt 회귀가 통과하기 전에는 `GO WITH CONDITIONS`를 현장
   parity 승인으로 해석하지 않습니다.

현재 구현된 Workstation은 저장소 루트 `apps/Workstation/`의 `standalone_gui.py`/`standalone_backend.py`
런타임 owner를 사용한다. Legacy 구현은 `legacy_gui.py`에 있고 `gui.py`는 호환 alias로 남는다. lifecycle 순서 제어만
UI-neutral `common/python/sasl_common/lifecycle.py`로 공유하며 panel/resource owner와 runtime data root는 분리한다. Workstation 설정과
recordings는 `.local/workstation_data/` 아래, Legacy 설정과 recordings는 `apps/StereoWIRIS/` 아래에 저장된다. 두 경로는 shared domain 모듈과
저장·좌표 계약만 공유한다. Workstation에는 기존 CameraPane 신호를 재사용한 Camera/Measurement 프리뷰,
측정 품질, detection, RTCP/NTP 동기화, GNSS/RTK NED baseline, recording/FPS 증거와 함께
Alignment/Pixhawk/GNSS 표, Calibration profile/intrinsics/R/T 표, Recording/Review/Export
session/artifact 표가 포함됩니다. Pre-run Calibration에는 좌/우 라이브 카메라 프리뷰가
표시되고, 체커보드가 검출되면 프레임 위에 코너 그리드와 검출 상태가 표시됩니다. 프리뷰
하단에서 카메라/검출 및 캘리브레이션 작업 버튼을 바로 조작할 수 있습니다.
Inspector와 Diagnostics도 같은 `EvidenceAdapter` source를
읽습니다. Workstation 테스트는 source/clock/age, 좌표 라벨, 명령 실패 전파와 context 종료를
확인하며, Camera의 IP/RTSP/control 명령과 Pixhawk/GNSS의 discovery/MAVLink/Base/Survey-in/NTRIP
명령도 기존 owner를 호출하는 phase-one facade로 제공합니다. Calibration 실행/적용,
validation 실행도 동일한 owner-backed facade로 제공합니다. Calibration setup fields,
measurement tuning, playback/export와 실제 RTSP/Pixhawk/GNSS 장비·장시간 writer 종료는
별도 조건입니다.

Measurement 실행은 상위 세션과 개별 측정을 분리합니다. 최상위 `Session` 화면의
`Open Session`은 날짜·시간 기반 당일 작업 폴더를 열고, `New Measurement`는 그 아래
`measurements/mNNNNNN`을 생성합니다. Session 화면은 현재 child의 증거 수집/유효 측정
준비 상태를 차단 원인 중심으로 표시합니다. 모든 측정 방식은 항상 보이며, 선택한 방식의
실제 입력값·출처·좌표계만 상세 영역에 표시합니다. Detection,
spool, 영상, 캡처, 결과, revision은 현재 child에만 기록되며 `Seal Measurement` 뒤 같은
세션에서 다음 child를 만들 수 있습니다. `Close Session`은 모든 child manifest SHA-256과
세션 공통 증거를 상위 manifest에 봉인합니다. 저장 리뷰도 `(session_id, measurement_id)`를
키로 사용합니다.

Optical Pose는 현재 Horizon, Mountain + GNSS, Terrain PnP, Drone RTK Rotation,
Drone RTK Multi-view R,t, References, Combine 방식별 탭으로 분리됩니다. Measurement는
backend readiness snapshot에서 준비된 방식만 표시하며, 부분 후보의 누락 회전은 live
Pixhawk, 누락 Translation은 live GNSS로 보완합니다. 이 보완은 Horizon과 Drone RTK
Rotation의 명시적 계약에만 적용되며 full 6-DOF solver는 부분 결과를 미준비로 처리합니다. 자동 대체 순서는
`Settings > Runtime / Calibration`에서 재정렬하고, 모든 전환은 detection-pair 경계와
기록 provenance를 보존합니다. 상세 운용 규칙은 `01_USER_WORKFLOW.md`, 배치는
`03_WORKSPACE_LAYOUT.md`를 참조합니다.

Alignment Navigation에 들어오면 공통 Readiness와 frozen 좌·우 preview가 먼저 표시되고,
그 아래 `Horizon`, `Mountain + GNSS`, `Terrain PnP`, `Drone Rotation`,
`Drone Multi-view`, `References`, `Combine`이 Alignment의 상단 방식 탭으로 직접 노출됩니다.
각 탭은 공통 Candidate DOF와 별도로 해당 방식의 입력 개수, 관측 축, 품질 metric, profile/hash,
실패 사유를 표시하며, `References`는 source 등록만, `Combine`은 후보 조합만 담당합니다.
별도의 `Optical Pose` 또는 바깥 `R,t methods` 탭은 없습니다. 넓은 화면은 preview 좌우 배치와
방식/Candidate 60:40 배치를 사용하고, 좁은 화면은 preview와 Candidate review를 세로로 전환합니다.
Workstation은 ROS2/uXRCE Pixhawk 연결·탐색과 still-image/FTP/EXIF TimeSync를 제공하지
않으며, 해당 Legacy workflow는 `apps/StereoWIRIS/` 런타임에 유지됩니다.
