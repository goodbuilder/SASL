# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-08-26 — Superseded interpretation

Codex는 최초 답변을 장비별 탭 대신 `All` 시간순 타임라인과 필터를 중심으로 구현하는
의도로 해석했다. 사용자가 2026-08-26에 이 해석이 의도와 다르다고 바로잡았으므로 이
결정은 폐기한다.

## 2026-08-26 — Accepted requirement correction

Diagnostics의 `Log`는 장비별로 나뉜 고정 view를 제공해야 한다. 필터 조작은 장비 구분의
주 수단이 아니다. 정확한 장비 grouping과 전체/공용 view 구성은 open 상태다.

## 2026-08-26 — Previous proposal awaiting revision

- FTP 전용 탭은 유지하되 FTP event도 All 상관관계 타임라인에 포함한다.
- 표시 일시정지는 수집 일시정지가 아니다.
- 최근 2,000 event만 in-memory로 유지하고 overflow를 명시한다.
- 모호한 legacy message의 장비를 추측하지 않는다.
- 설정/persistence/export/clear와 전체 logging rewrite는 범위에서 제외한다.

위 항목 중 장비별 고정 view와 충돌하는 All/filter 우선순위는 폐기하며, 나머지 보존·안전
조건은 grouping 확정 후 다시 검토한다.
