# Workflow Impact

## Affected operators and workflow IDs

- `DLOG-01`: 복수 장비 장애 원인 조사
- `DLOG-02`: 특정 장비/수준 로그 격리
- `DLOG-03`: 과거 행을 읽는 동안 새 로그 처리

## Current workflow

이상 상태 확인 → 현재 Workspace 유지 → Diagnostics/Log 열기 → 세 텍스트 묶음을 수동
검색 → 관련 장비 메시지를 눈으로 구분 → 수정/재시도 → 상태 회복 확인.

## Proposed workflow

이상 상태 확인 → 현재 Workspace 유지 → Diagnostics/Log 열기 → 시간순 `All` 확인 →
필요 시 Source/Device/Level/search 필터 → 과거 행 검토 중에는 표시 일시정지 또는 현재
스크롤 유지 → 재개/Follow latest → 수정/재시도 → 동일 타임라인에서 회복 확인.

## Preserved, added, removed, and reordered steps

- 보존: Diagnostics는 Workspace를 대체하지 않으며 Inspector 선택도 유지한다.
- 보존: 기존 raw message와 FTP 전용 탭, 기존 source text fallback.
- 추가: 시간순 event model, 필터, 검색, 표시 일시정지, 복사.
- 제거: 없음. 기존 장비 command나 recovery 절차는 변하지 않는다.
- 재정렬: Log 표시만 source-widget 순서에서 event 발생 순서로 바뀐다.

## Decisions, confirmations, handoffs, waits, and artifacts

필터는 backend event의 read-only projection이다. 어떤 장비 명령도 발생시키지 않으며
status/readiness를 재계산하지 않는다. 별도 새 저장 artifact는 만들지 않는다.

## Interruption, cancellation, retry, restart, and resume

- `Pause view`는 표시만 멈추고 backend 수집은 계속한다.
- Resume은 pending event를 순서대로 반영한다.
- 2,000-event bounded window를 넘긴 경우 누락 수/범위를 조용한 경고로 표시한다.
- 앱 restart 후에는 현재와 마찬가지로 in-memory Log가 초기화된다.

## Degraded, error, and recovery behavior

- structured facade가 없는 test/compatibility backend는 기존 `diagnostic_text()`를 단일
  `Application/Unassigned` source로 표시한다.
- 모호한 legacy message는 장비를 추측하지 않고 `Shared/Unassigned`로 남긴다.
- 필터 결과가 0건이면 factual empty state를 표시하고 정상 상태로 오인시키지 않는다.

## Completion and next actions

운영자는 장애와 회복 이벤트를 같은 타임라인에서 확인한 뒤 기존 Workspace에서 작업을
계속한다. Log 화면에 별도의 성공 판정이나 command 완료 상태를 만들지 않는다.

## Training, operating-procedure, and compatibility impact

새 교육은 `All`에서 먼저 상관관계를 본 뒤 필요할 때 필터링한다는 한 단계뿐이다.
기존 FTP 조사 절차와 장비 복구 절차는 유지된다.

## Workflow-to-screen, command, state, and regression traceability

`DiagnosticsContent.Log` → backend diagnostic event batch → panel-owned bounded store → existing
`_log` producer. Command/state/device owner 방향은 바뀌지 않는다.
