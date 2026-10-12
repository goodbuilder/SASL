# 조사한 현재 동작

기준 HEAD: 42b16e0 + 기존 미커밋 RCA/갓 모듈 분리 변경. 다른 Simulation/campaign 작업은 독립 변경이다. 이전 여러 change packet의 상태는 이번 요청으로 재개하지 않는다. 이번 대화에서 명확히 지정한 Target reference 개선을 새 패킷으로 만든다.

- workspace_views/recording.py:210: Target platform 기본 UAM, 수동 endpoint, SYS/COMP(0=Any), Connect/Stop, phase/accepted count. 탐색·후보표·저장 버튼은 없다. 플랫폼 선택 신호는 표시를 갱신한다.
- target_drone_controller.py: Target runtime/configuration/기록 owner. Drone은 receive-only receiver/worker, UAM은 DEFERRED이며 worker가 없다. 동작 중 재설정은 거부한다.
- workstation_core/target_drone_stream.py: HEARTBEAT 신원 확인, ATTITUDE/GPS_RAW_INT/GPS2_RAW 수신, source별 필터, 수신·기록 카운터. 첫 heartbeat가 기대 ID와 다르면 거부하므로 혼합 수신 포트에서 선택한 후보 연결을 보장하려면 보완이 필요하다.
- drone_telemetry_controller.py: 별도 운용 Drone 링크. GPS 수신 및 명시적 RTCM 전달 기능. Target source와 자동 공유하지 않는다.
- workstation_core/drone_network_discovery.py: 기본 포트14550–14559, selectable이 SYS=3에 고정. Target의14570 입력 예시를 포함하지 않는다. raw frame header로 HEARTBEAT ID를 추출하며 CRC를 확인하지 않는다. SO_REUSEADDR 사용, 부분 bind 실패의 상세 반환과 협조적 취소도 Target 요구에 부족하다.
- workstation_core/pixhawk_attitude.py:888: 기존 USB probe는 GCS heartbeat_send를 호출한다. Target 수신 전용 탐색에는 그대로 재사용할 수 없다.
- Target snapshot에는 수신 카운터가 있지만 화면용 heartbeat/GPS/ATTITUDE age 계약은 추가 설계가 필요하다.

기존 테스트에서 UAM worker 미생성·Target 수신 전용·원래 실행 기록 보존을 확인하는 사례가 존재한다. 이번 계획 단계에는 시험을 실행하지 않았다. 이전의2243 통과는 이전 변경의 증거다.

실행 환경: 앞선 화면 확인은 system Python + 임시 데이터/설정 + 네트워크 격리로 실행한 확인용 창이다. 공식 실행기는 .venv/의존성 및 현재 데이터 루트의 모델 준비가 필요하다. 그 창의 관찰은 실장비 탐색 성공 증거가 아니다.

관련 소스 해시: baseline-inventory.json. 계획 작성 시 실제 source/status를 조사했으며 하드웨어 I/O는 수행하지 않았다.
