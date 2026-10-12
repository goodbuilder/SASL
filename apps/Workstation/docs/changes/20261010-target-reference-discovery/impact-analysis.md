# 영향 범위와 소유권

현재 RecordingWorkspace → StandalonePanelBackend → TargetDroneController → TargetDroneReceiver/Worker/기록 owner. Telemetry는 독립 controller와 탐색 worker를 가진다.
제안 RecordingWorkspace → TargetReferenceView → 명명된 backend 명령 → TargetDiscoveryController + 기존 TargetDroneController. 탐색 owner는 후보·세대·취소·임시 자원만 소유하고, 실제 연결/원본 기록 owner는 기존 Target이다. view가 Telemetry/P1/P2 private widget·worker에 접근하지 않는다.

예상 파일:
- 신규 workspace_views/target_reference.py, target_discovery_controller.py, target_discovery_workers.py, workstation_core/target_discovery.py.
- 연결부 workspace_views/recording.py, standalone_backend.py, target_drone_controller.py, workstation_core/target_drone_stream.py.
- 구성/자원 목록/종료: panel_controller_bindings.py, panel_application.py, app_lifecycle_controller.py 및 타입 생성 도구의 기존 계약. 대형 facade에 탐색 로직을 넣지 않는다.
- 설정: workstation_core/settings.py의 Target 전용 선호 설정, typed snapshot. machine-local /dev 경로만 명시적으로 분류한다.
- endpoint 점유 보호는 기존 pixhawk_attitude의 장치 잠금/UDP 독점 확인 primitive를 검토해 좁은 중립 인터페이스로 사용한다. 필요한 경우에만 기존 연결 owner의 자원 조회/획득 경계에 최소 변경한다. 새로운 범용 연결 broker나 Telemetry 전면 재작성은 계획에 포함하지 않는다.

Telemetry의 SYS=3 및 기존 탐색 계약은 유지한다. 공통으로 재사용할 것은 장치 열거·route 조회·검증된 parser·표 스타일다. 기존 SYS=3 정책, GCS 송신 probe, SO_REUSEADDR 설정은 Target에 복제하지 않는다.

자원 제안: Target 탐색은 앱 전체에서1개, 대기0개, GPU0개, Target 수신 worker와 탐색은 동시 실행하지 않는다. network 최대16 포트/5초, USB 최대8 장치×선택 baud1개×1초, 전체10초, 후보 최대128개/최근 패킷 통계만 보관. 취소 확인 주기≤100ms, 종료 후 자원 해제 목표1초(필드 검증 대상). 입력량이 많으면 bounded batch로 처리하고 overflow 표시한다. UI I/O는 비동기이며 library 내부 thread fanout을 추가하지 않는다. 수치들은 제안값이며 실측 성능 주장이 아니다.

설정은 선택적 새 필드와 현재 UAM 기본값으로 후방 호환한다. 저장 기록/CSV 변환은 없다. rollback은 이 패킷의 source diff와 새 설정 필드만 대상으로 하며 기존 미커밋 변경을 reset하지 않는다. 새 의존성·런타임 설치·모델 복사·실장비 설정 변경은 이 계획의 일부가 아니다.

제품문서 갱신 대상(구현 승인 후): README.md, workflow.md, docs/ARCHITECTURE.md, docs/ui/01_USER_WORKFLOW.md,03_WORKSPACE_LAYOUT.md,06_STATUS_MODEL.md,07_COMMAND_MODEL.md,09_IMPLEMENTATION_PLAN.md. 현재 동작 문서는 계획만으로 수정하지 않는다.
