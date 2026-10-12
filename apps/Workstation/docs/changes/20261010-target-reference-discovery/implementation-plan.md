# 구현 계획 — 승인 및 소프트웨어 구현 완료

| 단계 | 작업 | 완료 기준/다음 단계 조건 |
|---|---|---|
| S0 | 승인 범위와 source 기준 재확인, 관련 UI baseline/자원·응답성 관찰 | 기존 미커밋 변경 분리 식별, 좌표/소유권 계약 기록 |
| S1 | Target 후보·상태·설정 계약, 독점 endpoint/장치 접근 adapter | 신원/점유/세대/저장 실패 관련 독립 검증 통과 |
| S2 | passive UDP·USB 탐색과 bounded 취소 worker | 송신0, socket/serial 정리, invalid packet·occupied·partial·cancel 검증 |
| S3 | Target 연결 handoff와 peer/IDs 재검증, 수신 freshness snapshot | 여러 source/동일 IDs의 peer 구분, late result 차단, UAM worker0 |
| S4 | TargetReferenceView·후보표·선택·연결·Save 연결 | 실제 버튼→backend→화면 흐름, 상태와 초안 분리, GUI 응답성 |
| S5 | restart/shutdown/기록 통합과 제품문서 | source/좌표/기록 보존, 소유권 inventory, 이식성 검사 |
| S6 | 관련 회귀·화면 비교·명시적 실장비 범위 검증 | verification.md 요구사항별 근거와 한계 기록 |

S1–S3를 먼저 구현해 검증한 뒤 S4 화면을 연결한다. 각 단계 PASS/FAIL/NOT_RUN을 verification.md에 즉시 추가한다. 앞 단계의 관련 실패가 남으면 의존 단계 완료를 주장하지 않는다.

고정되지 않은 장비 조건은 수신 포트·baud·네트워크 송신 설정이다. 계획은 조정 가능한 범위와 수동 입력을 제공하므로 계획 수립을 막는 질문은 없다. field tuning은 현장 증거로 결정한다.

기존 설정 필드는 기본값으로 유지하고 Target 전용 필드만 추가한다. 되돌리기는 새 view/owner 연결과 선택적 설정 필드를 대상으로 한다. 기존 기록을 다시 쓰거나 unrelated git 변경을 reset하지 않는다.

최종 결과와 한계: report.md 및 evidence/final-acceptance.json. S0–S6 소프트웨어 검증 완료; 물리장비 적합성은 별도이다.
