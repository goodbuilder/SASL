# Target reference 연결 탐색 개선 제안

사용자 요청: “수정 계획해.” 후속 “수정 시작해”로 구현이 승인되었다. 현재 결과는 report.md에 있다.
분류: backend-coupled. 위치: Data / Capture의 기존 Target reference 영역.

목표는 대상 드론의 주소를 모르는 운영자가 수신 가능한 MAVLink 후보를 찾고, 기체 ID를 확인한 다음 직접 연결하는 것이다. 네트워크 UDP와 USB/Telemetry Radio를 포함한다. 기존 수동 주소 입력도 지원한다. UAM은 외부 사후 RTK 대기(DEFERRED) 흐름을 유지한다.

제안 범위: 탐색·취소·후보 선택, 연결 초안·명시적 저장, 연결/해제, identity 재검증, heartbeat/GPS/ATTITUDE 수신 상태, 자원 충돌·종료 처리. 대상 비행 제어, 파라미터 변경, RTCM 송신, logger 제어, 자동 재연결, UAM 사후자료 importer, 서브넷 전체 스캔은 이번 계획의 범위 밖이다.

수용 기준:
- T1: UDP·USB 후보를 출처/장치/포트/baud/SYS/COMP/관찰시각과 함께 제시한다.
- T2: 선택은 초안만 바꾸며 연결·설정 저장은 각각 명시적 동작이다. 재시작은 미연결 상태다.
- T3: 실제 수신으로 연결 신원과 데이터 freshness를 확인한다. HEARTBEAT만으로 GPS/RTK 수신을 주장하지 않는다.
- T4: 탐색·연결 경로에서 MAVLink 송신이 0회다. UAM은 worker/socket을 만들지 않는다.
- T5: 활성 P1/P2/Telemetry/Base GNSS 자원을 중복으로 열지 않는다. 취소·재탐색·지연 결과·종료가 안전하게 처리된다.
- T6: 기존 기록/좌표 계약과 화면 조작 안정성을 검증한다.

[사용자용 계획](approval-summary.md) · [구현 순서](implementation-plan.md) · [검증 기준](verification.md)
