# Target Drone 연결부 수정 계획

Data / Capture의 Target reference에 네트워크·USB 탐색과 후보 선택을 추가한다. 운영자는 발견된 주소와 기체 ID를 확인한 뒤 Connect target으로 연결한다.

1. Find Network / Find USB / Cancel과 고정 높이 후보표를 추가한다. 주소·포트/baud·SYS/COMP·최근 heartbeat·점유 여부를 표시한다.
2. Use selected는 연결 입력만 채운다. Connect에서 기체를 다시 확인하고, 실제 heartbeat·GPS·ATTITUDE 수신 상태를 구분해 보여준다.
3. 수동 입력과 명시적 Save를 제공한다. 재시작 시 저장된 연결 정보만 복원하고 자동 연결하지 않는다.
4. Target 경로는 수신 전용으로 구성한다. UAM은 사후 RTK 대기 상태를 유지한다. 활성 P1/P2/Telemetry의 장치·포트는 점유 중으로 표시한다.
5. 탐색 취소·중복 클릭·끊김·잘못된 기체·오래된 결과·앱 종료를 함께 검증한다. 긴 오류와 후보 갱신이 입력/선택/스크롤을 움직이지 않도록 한다.

현재 Telemetry 탐색은 SYS=3 전용이고 기존 USB 탐색은 GCS heartbeat를 보낸다. 따라서 표시 구성과 장치 열거는 재사용하되 Target 전용 수신·신원·자원 처리를 추가한다. UDP 탐색은 PC로 도착하는 heartbeat만 찾으며, 기체 측 송신 설정이 없으면 후보가 보이지 않을 수 있다.

구현은 계약/점유 보호→탐색 worker→연결 신원·상태→화면→저장·종료→회귀 순서다. 세부 단계는 implementation-plan.md, 시험 기준은 verification.md에 있다.

사용자의 후속 “수정 시작해”로 구현 범위가 승인되었고 소프트웨어 구현·검증을 완료했다. 결과는 report.md에 있다. 실장비 탐색은 수행하지 않았다. 새 패키지 설치, 모델 provisioning, 비행 제어, Telemetry 통합/자동 공유는 포함하지 않는다.
