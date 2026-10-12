# 화면 구성과 안정성

기존 QGroupBox/QGridLayout/QTableView, command button, statusStrip, compactContract와 docs/ui/04_COMPONENT_SYSTEM.md·08_DESIGN_SYSTEM.md를 따른다. Target reference를 별도 소형 view로 추출해 RecordingWorkspace에 조립한다.

배치 제안:
1. Target platform, Receive-only 표기.
2. 연결 방식(Network UDP / USB·Radio / 수동 고급 입력), 포트 또는 장치, baud, SYS/COMP.
3. Find Network · Find USB · Cancel.
4. 고정 높이 후보표: 상태 | 출처/장치 | 수신 포트/baud | SYS | COMP | heartbeat 나이. peer IP/송신 포트/NIC/오류 상세는 tooltip/Inspector.
5. Use selected · Connect target · Stop target · Save.
6. 현재 연결 endpoint·확인된 IDs, heartbeat/GPS1/GPS2/ATTITUDE age, GPS fix/RTK 상태, 수신/거부/기록 오류.

UAM에서는 탐색·접속 입력 영역을 비활성화하고 사후 RTK 대기 사유를 표시한다. 모드 전환으로 작업 화면이 크게 이동하지 않도록 공간을 안정적으로 유지한다.

후보표는 4~6행 높이와 내부 scroll, 고정된 column 정책을 사용한다. 일반 상태 갱신은 선택·입력·focus·outer scroll을 바꾸지 않는다. 새 탐색을 수락할 때 이전 후보와 선택만 명시적으로 초기화한다. 긴 endpoint/에러는 생략+tooltip, 상세는 Inspector. 누락값은 --, age는 s 단위이며 범위를 넘는 값은 고정 폭 표현을 사용한다. 상태 표시 갱신은 최대5Hz로 제한한다.

구현 전후1366×900과1600×980,100/150% 배율에서 빈 후보/다수 후보/긴 오류/age 증가/모드 전환을 비교한다. 현재 확인용 창의 실제 사용자 조작을 변경해 스크린샷을 수집하지 않는다. 본 패킷은 배치 제안이며 시각 검증 결과는 아직 없다.
