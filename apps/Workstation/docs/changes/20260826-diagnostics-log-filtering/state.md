---
schema: modify-gui-change-state-v1
change_id: 20260826-diagnostics-log-filtering
status: interviewing
mode: backend-coupled
updated: 2026-08-26
baseline_ref: c439799
last_completed: "User correction recorded: Log must be split into device-specific views, not primarily presented as one filtered stream"
next_question: "Log 안에 전체/WIRIS Left/WIRIS Right/P1/P2/Base GNSS·NTRIP/Drone·공용 하위 탭을 두는 구성이 맞습니까?"
read_next:
  - docs/changes/20260826-diagnostics-log-filtering/change-brief.md
  - docs/changes/20260826-diagnostics-log-filtering/decisions.md
  - docs/changes/20260826-diagnostics-log-filtering/frontend-contract.md
affected_product_docs:
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/02_INFORMATION_ARCHITECTURE.md
  - docs/ui/03_WORKSPACE_LAYOUT.md
  - docs/ui/04_COMPONENT_SYSTEM.md
  - docs/ui/06_STATUS_MODEL.md
  - docs/ui/08_DESIGN_SYSTEM.md
  - README.md
blocking_questions: 1
source_changed: false
implementation_allowed: false
approval:
  state: not-requested
  scope: null
  conditions: []
  recorded_at: null
---

# Resume Notes

이번 변경은 다른 `paused`/`verifying` 패킷과 분리한다. Workstation 작업 트리에 다수의
기존 수정이 있으므로 관련 파일의 현재 내용을 보존하고 겹치는 부분만 최소 패치한다.
이전 All+filter 중심 계획은 사용자 의도와 달라 superseded 상태다. 장비별 view grouping을
확정한 뒤 승인 문서와 구현 슬라이스를 다시 작성한다. 소스 수정은 승인되지 않았다.
