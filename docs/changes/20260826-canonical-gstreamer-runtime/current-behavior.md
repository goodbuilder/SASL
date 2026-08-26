# Current Behavior

## Version and worktree context

- Baseline commit: `5bec92529e0954a96522d1de7c5e1e5f2ece38ca`
- Branch: `fix/workstation-optical-pose-evidence-geometry-hardening`, baseline에서 1 commit ahead
- 작업 트리는 이미 StereoWIRIS, Workstation, Simulation 전반의 사용자 변경으로 dirty이다.
  이 변경은 해당 내용을 보존하고 runtime 관련 현재 파일에만 덧붙인다.
- 루트 `.venv/pyvenv.cfg`는 `include-system-site-packages = false`이며
  `workstation-system-dist-packages.pth`가 `/usr/lib/python3/dist-packages`를 수동 주입한다.

## Reproduction

- `apps/StereoWIRIS/run.sh`는 앱 venv, 루트 venv, Simulation venv, system Python 순으로
  후보를 고른다.
- `apps/Workstation/run.sh`도 루트 venv가 없으면 system Python으로 넘어간다.
- `apps/StereoWIRIS/scripts/bootstrap_network_tools.sh`는 launch 중 조건에 따라 apt를 실행하고,
  calibration bootstrap은 임의 `PYTHON_BIN`/Simulation venv를 허용하며 pip를 실행한다.
- Ubuntu package audit에서 GStreamer/PyGObject/ffmpeg는 설치되어 있으나
  `python3.10-venv`는 `unknown ok not-installed`이다.
- `nvidia-smi` binary는 있지만 driver와 통신하지 못한다.

## Observed frontend behavior

두 UI는 `auto|nvidia|software` 설정을 유지한다. 현재 Auto와 명시적 NVIDIA 모두
`nvh264dec` 실패 후 `avdec_h264` 후보를 사용할 수 있어 NVIDIA 선택이 fail-closed가 아니다.
Workstation과 Legacy에는 Auto runtime fallback 중 녹화를 정리하는 기존 상태 경로가 있다.

## Observed backend and persistence behavior

카메라는 Qt worker가 `multiprocessing` spawn child를 만들며 MP4/queue shutdown owner를
보존한다. `DEFAULT_YOLO_PYTHON`은 `PYTHON_BIN` 환경변수를 우선해 부모 `sys.executable`과
다른 interpreter 계약을 암시한다. 기존 recording/session persistence와 reconnect owner는
이번 변경에서 유지 대상이다.

## Documentation, code, test, and runtime conflicts

문서는 앱/Simulation/system Python fallback과 launch-time 설치를 안내하지만 승인 계약은
루트 공식 venv 단일 사용과 수동 복구만 허용한다. 기존 테스트도 명시적 NVIDIA의 software
fallback 및 Simulation venv 사용을 기대하므로 새 정책에 맞게 갱신해야 한다.
