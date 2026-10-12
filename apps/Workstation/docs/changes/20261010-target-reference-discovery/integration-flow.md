# 행위와 실제 결과 연결

| 행동 | owner 처리 | 화면 성공 근거 | 실패·복구 |
|---|---|---|---|
| Find Network/USB | 범위 검증→점유 확인→bounded worker | 현 generation의 관찰 후보 | EMPTY/PARTIAL/FAILED와 원인; 재탐색 |
| Cancel | cancel 요청→I/O 정리→finished | CANCELLED, 자원 반환 | 반환 전 새 작업 금지; 종료 상태 노출 |
| Use selected | generation/age/identity 검증 | endpoint/IDs 초안만 변경 | 만료·불명확 후보는 사유 표시 |
| Connect target | probe 종료 확인→독점 획득→peer/heartbeat 재검증 | 연결 신원 확인, 데이터 수신은 별도 | mismatch/busy/timeout 구분; Stop/재선택 |
| Stop target | 수신 종료→기존 drain/기록 처리 | worker 종료와 기존 기록 계약 완료 | 종료 실패 표시, 새 owner 생성 금지 |
| Save | 선호 설정 원자 저장 | 저장됨, 실제 연결 상태 유지 | 기존 파일 보존과 오류 |
| UAM 적용 | live/probe 없음 확인→DEFERRED | 사후 외부 RTK 대기 | 연결 중이면 먼저 Stop |
| 앱 종료 | 탐색 취소·join→기존 Target close | 활성 worker/socket/serial 목록 비움 | 기존 종료 정책에서 미완료 표시 |
