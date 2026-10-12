# 결정 기록

- 2026-10-10 / 사용자 요청: Target Drone 연결부에 카메라·Pixhawk 같은 탐색 UI 필요성을 제기한 뒤 수정 계획을 요청했다. 요청 범위는 계획이다.
- 기존 대화로 Drone live reference/UAM deferred, 독립 Telemetry 연결, 수신 전용 요구를 확인했다. 추가 인터뷰 없이 네트워크+USB와 수동 입력을 제안 범위로 삼았다.
- 여러 과거 미완료 패킷은 별도 기능이다. 새 Target reference 패킷을 생성했으며 과거 작업 승인을 이 변경에 전용하지 않는다.
- 코드 조사: Telemetry SYS=3 고정, 헤더 기반 ID 추출, 기본14550–14559, SO_REUSEADDR; USB probe heartbeat 송신. 직접 복제 대신 좁은 helper 재사용과 Target 전용 검증 adapter를 제안했다.
- 기체 ID만으로 Target·Telemetry·P1/P2를 자동 연결하거나 source를 합치지 않는다. 선택한 endpoint/peer/IDs를 연결 시 재확인한다.
- 신규 좌표 계산/CSV 변경은 계획에 없다. Target freshness·신원 표현과 설정만 추가한다.
- 기본시간/포트수/후보수는 backend-contract.md의 제안값이며 아직 실측하지 않았다. 사용자별 기체 주소를 추정해 저장하지 않는다.

- 사용자 “수정 시작해”: 이 패킷의 구현을 명시적으로 승인. 추가 승인 질문 없이 S0–S6 진행.

- 구현 조사 보정: sessionless Target reference의 실제 위치는 Data → Status(legacy Current)다. Capture라는 초기 문서 표현을 현재 UI와 맞추었고, 위치 이동 없이 기존 영역에 조립한다.
- discovery worker는 Qt를 요구하지 않는 bounded Python thread이며 controller가 소유한다. 별도 worker 파일 대신 동일 책임의 controller run 경계에 두었다.
- 실제 수신은 새 ReceiveOnlyConnection에서 독점 UDP/serial, CRC parser 및 peer pinning을 수행한다. USB 송신 없는 관찰은 heartbeat가 이미 흐르는 장치에 한정된다.
- TargetReferenceSettings는 nested immutable 설정으로 저장한다. 일반 Settings의 flat field 편집 경로와 구분한다.
- 원시 좌표/telemetry 필드 변환 없이 lifecycle record에 optional expected/verified peer metadata만 추가했다.
- 비동기 Stop의 측정 상태 알림은 queued QObject relay로 GUI thread에 복귀한다.
