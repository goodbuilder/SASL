# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-08-26 — Accepted direction

사용자는 장비별 로그 탐색 개선에 동의했다. 장비별 탭만 분리하지 않고 `All` 시간순
타임라인을 유지하면서 필터로 좁히는 방향을 사용한다.

## 2026-08-26 — Proposed for packet approval

- FTP 전용 탭은 유지하되 FTP event도 All 상관관계 타임라인에 포함한다.
- 표시 일시정지는 수집 일시정지가 아니다.
- 최근 2,000 event만 in-memory로 유지하고 overflow를 명시한다.
- 모호한 legacy message의 장비를 추측하지 않는다.
- 설정/persistence/export/clear와 전체 logging rewrite는 범위에서 제외한다.
