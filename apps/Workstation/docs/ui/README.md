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
새 측정 데이터는 `.local/workstation_data/measurement_data/` 아래, 기존 recordings는 그대로 남고, Legacy 설정과 recordings는 `apps/StereoWIRIS/` 아래에 저장된다. 두 경로는 shared domain 모듈과
저장·좌표 계약만 공유한다. Workstation에는 기존 CameraPane 신호를 재사용한 Camera/Measurement 프리뷰,
측정 품질, detection, RTCP/NTP 동기화, GNSS/RTK NED baseline, recording/FPS 증거와 함께
Alignment/Pixhawk/GNSS 표, Calibration profile/intrinsics/R/T 표, Data writer/capture
표가 포함됩니다. Pre-run Calibration에는 좌/우 라이브 카메라 프리뷰가
표시되고, 체커보드가 검출되면 프레임 위에 코너 그리드와 검출 상태가 표시됩니다. 프리뷰
하단에서 카메라/검출 및 캘리브레이션 작업 버튼을 바로 조작할 수 있습니다.
Inspector와 Diagnostics도 같은 `EvidenceAdapter` source를
읽습니다. Workstation 테스트는 source/clock/age, 좌표 라벨, 명령 실패 전파와 context 종료를
확인하며, Camera의 IP/RTSP/control 명령과 Pixhawk/GNSS의 discovery/MAVLink/Base/Survey-in/NTRIP
명령도 기존 owner를 호출하는 phase-one facade로 제공합니다. Calibration 실행/적용,
validation 실행도 동일한 owner-backed facade로 제공합니다. Calibration setup fields,
measurement tuning, playback/export와 실제 RTSP/Pixhawk/GNSS 장비·장시간 writer 종료는
별도 조건입니다.

Measurement 실행은 persisted Session 없이 transient
`IDLE/QUEUED/STARTING/RUNNING/STOPPING/ERROR` 상태만 사용합니다. `측정 시작`은 기존
OPEN/RECOVERABLE 데이터와 writer 오류를 검사하지 않습니다. 수동 기록은 먼저 중지하고,
실제 카메라 작업이 진행 중이면 QUEUED로 기다렸다가 자동 시작합니다. 구조화 자료는
기본 `<SASL_ROOT>/recordings`의 유형별 고정 JSONL에 append되고 MP4/사진/native PNG/ULog는
생성 시각과 UTC offset 파일명을 사용하며 기존 파일을 덮어쓰지 않습니다. `측정 종료`는
실제 카메라 owner를 해제한 뒤 즉시 다음 실행이 가능하며 Parquet, review.xlsx, manifest,
Finalize/Seal/Retry Save를 기다리지 않습니다. 종료 뒤에는 최근 Measurement 한 건의 파생
`latest_measurement.xlsx`를 백그라운드에서 만들며 이 작업도 다음 실행을 막지 않습니다.
`Data > Status`는 writer 상태와 `Update/Open Latest XLSX`, 고정 상태 줄을 제공하고,
`Capture`는 기존 캡처 도구를 유지하며 과거 Session recordings는 file manager에서만 엽니다.

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
