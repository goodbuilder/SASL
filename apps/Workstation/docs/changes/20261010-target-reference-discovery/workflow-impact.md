# 운영 흐름 제안

현재: Drone 선택 → 외부에서 주소·baud·SYS/COMP 확인 → 수동 입력 → Connect target → phase/count 확인.

제안: Drone 선택 → Find Network 또는 Find USB → 후보의 출처·기체 ID·관찰 나이 확인 → Use selected → 입력 초안 검토 → Connect target → 신원 확인 및 각 데이터 수신 상태 확인. Save는 연결과 독립적인 명시적 설정 저장이다. 수동 주소 입력 경로도 같은 연결 검증을 거친다.

Find Network는 선택한 PC 수신 포트에 이미 도착하는 MAVLink를 수동 관찰한다. 다른 서브넷의 드론을 자동으로 찾아 송신 대상을 바꾸는 기능은 아니다. 결과0건이면 포트·네트워크·기체 측 송신 대상을 확인하도록 안내한다. Find USB는 사용 가능한 직렬 장치에서 설정 baud로 수신하고 송신 없이 heartbeat를 확인한다. 스트림이 없는 장치는 신원 미확인으로 표시한다.

Use selected는 endpoint/baud/기대 SYS/COMP/관찰 peer로 초안을 채운다. 후보=연결 성공으로 표시하지 않는다. 탐색 결과는 저장하지 않으며, Save는 선호 연결 설정만 보존한다. 실시간 확인 상태는 재시작 때 초기화한다.

탐색 중 중복 실행은 거부하고 취소할 수 있다. 취소 완료 전 새 탐색/연결은 열지 않는다. 연결 중 후보/모드 변경은 잠그고 Stop 완료 후 수정한다. Drone→UAM도 Stop 완료 후 선택하고 Set UAM deferred로 명시적으로 적용한다. 기존 live 상태와 편집 초안을 혼동하지 않도록 양쪽을 표시한다.

연결 끊김/기체 ID 불일치/포트 점유/무응답은 서로 다른 원인과 다음 행동을 표시한다. 자동 재연결은 하지 않는다. 기록 중 탐색은 새 target 기록을 만들지 않는다. Target 연결 실패가 P1/P2·카메라·Measurement를 막지 않도록 기존 관찰 흐름을 유지한다.

교육 변화는 탐색→선택→연결 세 단계와 HEARTBEAT/유효 GPS의 구분이다. 저장 파일의 frame/axis와 기존 source_id는 바꾸지 않는다.
