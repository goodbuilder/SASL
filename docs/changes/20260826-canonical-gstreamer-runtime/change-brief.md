# Change Brief

## Operator outcome

StereoWIRIS와 Workstation은 저장소 루트 `.venv` 하나만 사용하고, Qt가 열리기 전에
GStreamer/PyGObject/decoder 준비 상태를 판정한다. 운영자는 자동 설치나 숨은 Python
fallback 없이 `PASS/WARN/FAIL`, 실제 decoder 선택, 누락 Ubuntu 패키지와 정확한 복구
명령을 확인할 수 있다.

## Motivation

현재 launchers는 앱/Simulation/system Python으로 넘어갈 수 있고 루트 venv는 수동 `.pth`로
시스템 `dist-packages`를 노출한다. 이 구성은 배포 재현성이 없고, 명시적 NVIDIA 모드도
software decoder로 조용히 내려갈 수 있어 설정 의미와 실제 동작이 다르다.

## Included scope

- Ubuntu 22.04 system dependency manifest와 read-only runtime doctor
- `python3 -m venv --system-site-packages .venv` 공식 환경 계약
- 공통 pip manifest, 선택 PyAV manifest, 앱별 호환 manifest
- 두 launcher/direct entry/runtime check와 camera child interpreter 계약
- Gst/GstRtp 및 pipeline element factory 검사, Auto/NVIDIA/Software decoder 정책
- Auto NVIDIA runtime 장애 시 기존 MP4 안전 종료 후 software 전환
- 루트/앱 문서와 자동화 테스트

## Excluded scope and non-goals

세션/MP4 형식, NTRIP/MAVLink/ROS overlay 의미, RTSP 손상 재연결 정책, 측정/좌표 계약,
Simulation 자체 venv와 worker 정책은 변경하지 않는다. 앱은 apt 또는 pip 설치를 자동으로
수행하지 않는다.

## Acceptance summary

공개 doctor와 두 `run.sh --check`가 합의된 종료 코드와 진단을 제공하고, 잘못된 interpreter는
2, 필수 runtime 누락은 1, 준비 완료 또는 허용된 Auto software 상태는 0을 반환한다.
