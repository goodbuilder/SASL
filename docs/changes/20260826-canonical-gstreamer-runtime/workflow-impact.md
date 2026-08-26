# Workflow Impact

## Affected operators and workflow IDs

- `RT-01`: 최초 배포/공식 venv 생성
- `RT-02`: 실행 전 deployment/execution doctor
- `RT-03`: StereoWIRIS/Workstation `--check` 및 정상 시작
- `RT-04`: Camera Preview/YOLO decoder 선택과 장애 복구

## Current workflow

운영자는 여러 venv 후보 중 우연히 import 가능한 Python으로 앱을 시작하며, 일부 dependency는
launch 중 설치될 수 있다. decoder mode와 실제 fallback 정책이 일치하지 않을 수 있다.

## Proposed workflow

운영자는 manifest의 Ubuntu 패키지를 수동 설치하고 루트 `.venv`를
`--system-site-packages`로 생성한 뒤 root pip manifest를 설치한다. doctor 또는 앱 `--check`로
준비 상태를 확인하고 앱을 시작한다. Qt는 공통 필수 GStreamer capability가 없으면 열리지 않는다.

## Preserved, added, removed, and reordered steps

- 보존: 설정 schema, RTSP URL/transport, camera/session/recording lifecycle, ROS conditional overlay
- 추가: interpreter gate, runtime doctor, capability/decoder report
- 제거: 앱/Simulation/system Python fallback, 수동 `.pth` 지원, launch-time apt/pip
- 순서: interpreter/GStreamer gate가 ROS overlay source와 Qt 생성보다 앞선다.

## Decisions, confirmations, handoffs, waits, and artifacts

설치는 전적으로 운영자 결정이다. doctor는 상태만 읽고 apt/pip/venv를 변경하지 않는다.
복구 명령과 venv backup/rollback 절차를 문서에 남긴다.

## Interruption, cancellation, retry, restart, and resume

Auto에서 NVIDIA runtime 장애가 발생하면 기존 recorder finalization을 거쳐 software로 한 번
재시작한다. NVIDIA 모드는 동일 장애에서 software로 내려가지 않고 중지/오류를 표시한다.

## Degraded, error, and recovery behavior

Auto에서 driver/nvh264dec가 시작 전 불가하면 WARN과 함께 `avdec_h264`가 유효 decoder가 된다.
NVIDIA 선택 시 같은 조건은 카메라 시작 거부이며 Auto 변경 안내를 표시한다. gi/typelib/필수
element 누락은 앱 전체 시작 FAIL이다.

## Completion and next actions

자동화된 software/launcher regression 통과 후 실제 좌/우 RTSP를 NVIDIA와 software로 각각
검증하고, software 두 스트림 지속 실행·GUI 응답·고아 process 부재를 현장 확인한다.

## Training, operating-procedure, and compatibility impact

운영자는 앱별 venv나 `PYTHON_BIN` 우회 대신 루트 venv 복구 명령을 사용해야 한다. decoder
설정 값은 호환 유지되지만 명시적 NVIDIA의 silent fallback은 제거된다.

## Workflow-to-screen, command, state, and regression traceability

`RT-01/02`는 doctor/manifest tests, `RT-03`은 launcher/direct-entry tests, `RT-04`는
capability policy, camera-start rejection, MP4-safe Auto fallback tests로 추적한다.
