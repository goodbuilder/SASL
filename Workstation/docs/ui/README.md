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

권장 순서:
1. 문서를 저장소의 `Workstation/docs/ui/`에 배치합니다.
2. `AUDIT_PROMPT.md`를 Codex에 입력합니다.
3. 실제 코드 감사를 수행하여 `11_DESIGN_AUDIT_RESULT.md`를 생성합니다.
4. CRITICAL/HIGH finding과 NO-GO/조건부 판정을 검토합니다.
5. 승인된 구현에서는 `09_IMPLEMENTATION_PLAN.md`의 gate와 `11_DESIGN_AUDIT_RESULT.md`의
   조건을 따라 launcher, lifecycle, adapter, shell을 단계적으로 검증합니다.
6. 실장비·장시간 종료·전체 Qt 회귀가 통과하기 전에는 `GO WITH CONDITIONS`를 현장
   parity 승인으로 해석하지 않습니다.

현재 구현된 Workstation은 저장소 루트 `Workstation/`의 `standalone_gui.py`/`standalone_backend.py`
런타임 owner를 사용한다. Legacy 구현은 `legacy_gui.py`에 있고 `gui.py`는 호환 alias로 남는다. lifecycle 순서 제어만
UI-neutral `sasl_common/lifecycle.py`로 공유하며 panel/resource owner와 runtime data root는 분리한다. Workstation 설정과
recordings는 `workstation_data/` 아래, Legacy 설정과 recordings는 `StereoWIRIS/` 아래에 저장된다. 두 경로는 shared domain 모듈과
저장·좌표 계약만 공유한다. Workstation에는 기존 CameraPane 신호를 재사용한 Camera/Measurement 프리뷰,
측정 품질, detection, RTCP/NTP 동기화, GNSS/RTK NED baseline, recording/FPS 증거와 함께
Alignment/Pixhawk/GNSS 표, Calibration profile/intrinsics/R/T 표, Recording/Review/Export
session/artifact 표가 포함됩니다. Inspector와 Diagnostics도 같은 `EvidenceAdapter` source를
읽습니다. Workstation 테스트는 source/clock/age, 좌표 라벨, 명령 실패 전파와 context 종료를
확인하며, Camera의 IP/RTSP/control 명령과 Pixhawk/GNSS의 discovery/MAVLink/Base/Survey-in/NTRIP
명령도 기존 owner를 호출하는 phase-one facade로 제공합니다. Calibration 실행/적용,
validation 실행도 동일한 owner-backed facade로 제공합니다. Calibration setup fields,
measurement tuning, playback/export와 실제 RTSP/Pixhawk/GNSS 장비·장시간 writer 종료는
별도 조건입니다.
Workstation은 ROS2/uXRCE Pixhawk 연결·탐색과 still-image/FTP/EXIF TimeSync를 제공하지
않으며, 해당 Legacy workflow는 `StereoWIRIS/` 런타임에 유지됩니다.
