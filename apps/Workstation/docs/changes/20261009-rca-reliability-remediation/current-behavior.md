# Baseline

- Baseline commit: 42b16e0. Workstation 및 품질 검사 파일은 시작 시 clean.
- 사용자 소유 변경: apps/Simulation, docs/changes/20261008-stereo-campaign-v1-1 및 작업지시서. 이 작업에 포함하지 않는다.
- 이전 RCA의 193개 선별 시험은 이전 관찰이다. 이번 변경의 증거로 대체하지 않고 새로 실행한다.
- FrozenMeasurementPairSnapshot은 nested dict/list를 그대로 보관하며 canonical_payload도 얕은 복사다.
- CI는 2026-08-24 삭제된 test_workstation.py를 호출한다.
- 이식성 검사 이전 RCA: 20 regex matches. 주로 historical provenance이며 20개 실행 의존성 결함은 아니다.
- MeasurementRunController가 panel의 _measurement_run_* 값을 읽고 쓰며 CameraStreamLifecycleController도 active_sides를 직접 쓴다.
- method worker는 start ACK 이후 deadline이 없고 process exit만 복구한다. 이 no-deadline 정책은 느린 계산을 보존하기 위한 현재 계약이다.
- README의 MAG 차단 설명과 advisory 설명이 충돌한다. 실제 시작은 advisory다.
- 이번 checkout에는 .venv가 없다. 초기 시험은 /usr/bin/python3와 사용 가능한 설치 패키지로 실행한다.
