# 화면 명령 계약

제안 명령: target.discover_network, target.discover_usb, target.cancel_discovery, target.use_candidate, target.connect, target.stop, target.save_settings. 기존 backend target_drone_connect/stop 호출 형식은 호환한다.

탐색 상태: IDLE / SEARCHING / FOUND / EMPTY / PARTIAL / CANCELLED / FAILED. 연결 상태는 별도 NOT_CONFIGURED / CONNECTING / STREAMING / IDENTITY_MISMATCH / STOPPING / STOPPED / FAILED / DEFERRED다. STREAMING과 유효 GPS/RTK 표시를 분리한다.

Find: UI 검증 후 owner가 수락했을 때만 이전 snapshot 초기화. Use selected: 현재 generation과 유효기간을 통과한 후보만 초안 적용. Connect: 초안의 endpoint/baud/IDs를 검증하고 actual postcondition까지 CONNECTING. Save: 검증된 선호 필드만 저장; 저장 성공이 연결 상태를 승격하지 않는다.

수신 없는 USB도 장치 존재 후보로 보이되 verified로 표시하지 않는다. 관찰된 component0을 Any로 자동 변환하지 않는다(현재 Target 연결 계약은1..255/None). 그 후보는 미지원 사유를 표시하고 정확한 ID 지원을 별도 계약 검토 없이 추정하지 않는다. 수동 Any 입력은 현재 기능을 보존하되 의미를 명시한다.

현재 접속 중인 endpoint/IDs와 아직 적용하지 않은 초안을 구분한다. busy 상태는 버튼 비활성뿐 아니라 backend에서도 거부한다. 재시작 때 저장값을 초안으로만 복원한다. Target/Telemetry와 같은 기체일 수 있어도 기체 ID만으로 source를 자동 합치지 않는다.
