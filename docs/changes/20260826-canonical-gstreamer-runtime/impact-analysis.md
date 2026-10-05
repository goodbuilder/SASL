# Impact Analysis

## Preserved invariants

좌표/측정 schema, RTSP 재연결 owner, session/MP4/NTRIP/MAVLink 동작, settings의
`auto|nvidia|software`, 한 camera child/recorder owner를 보존한다.

## Affected layers and components

root dependency manifests/tools/common runtime module, 두 shell launcher/direct entry,
두 camera worker 복사본, decoder camera-start gate, runtime/launcher tests, 운영 문서.

## Feature dependency, ownership, and coupling changes

공통 `sasl_common.operator_runtime`이 interpreter/capability/decoder policy의 단일 owner가 된다.
앱별 settings/model/import 검사는 각 앱에 남고 device/recording owner는 이동하지 않는다.

## Worker CPU/GPU resource impact

worker 수, queue, rate, thread budget은 바뀌지 않는다. Auto는 NVIDIA driver가 실제 사용
가능할 때만 GPU decode를 선택하고 아니면 기존 software path를 명시적으로 사용한다.

## Compatibility and migration

기존 root venv는 삭제하지 않고 backup 이름으로 이동한 뒤 공식 방식으로 재생성한다.
앱별/Simulation venv는 남지만 두 운영 앱에서 선택되지 않는다. 수동 `.pth`는 unsupported다.

## Rollback and recovery

코드 revert와 기존 venv backup 복원으로 rollback한다. 데이터/settings migration은 없다.

## Risks and assumptions

Ubuntu 22.04/Python 3.10/ROS Humble 기준이다. factory 생성과 `nvidia-smi`는 startup
capability이지 실제 두 카메라 decode 성공 증거가 아니므로 hardware verification이 남는다.
