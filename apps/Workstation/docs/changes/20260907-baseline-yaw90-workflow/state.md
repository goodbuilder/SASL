---
schema: modify-gui-change-state-v1
change_id: 20260907-baseline-yaw90-workflow
status: completed
mode: backend-coupled
updated: 2026-09-07
baseline_ref: "2458d29 plus preserved user worktree"
last_completed: "Query failure/recovery/cache preservation verified: 92 focused passed; full 2033 passed/2 unrelated failures/2 skipped; 1366px error display inspected"
next_question: null
read_next:
  - docs/changes/20260907-baseline-yaw90-workflow/change-brief.md
  - docs/changes/20260907-baseline-yaw90-workflow/current-behavior.md
  - docs/changes/20260907-baseline-yaw90-workflow/verification.md
affected_product_docs:
  - README.md
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/03_WORKSPACE_LAYOUT.md
  - docs/ui/06_STATUS_MODEL.md
  - docs/ui/10_DESIGN_AUDIT.md
blocking_questions: 0
source_changed: true
implementation_allowed: true
approval:
  state: approved
  scope: "Baseline Yaw90 Method tab, live azimuth and angle/attitude tables, moved Center line/Reverse/Telemetry installation table"
  conditions: ["Preserve existing methods, recordings, Pixhawk displays and user edits", "No physical level target column"]
  recorded_at: "2026-09-07 user: 구현 시작해."
---

# Resume Notes

Implementation explicitly approved in conversation. No further permission is pending.
Review reproduced stale Inspector details on local-tab exit and fixed-height previews after
viewport-only resizing. User approved both fixes ('해 그럼.'); no geometry/settings scope change.
Both are now fixed and regression-tested; verification.md distinguishes inherited/concurrent
worktree failures from the scoped UI results.
Additional approved fix: UI snapshot failure must not blank confirmed values or leave Inspector
CURRENT. Last valid P1/P2 distance inputs remain owned by the existing Measurement caches.
The display fallback is now implemented and tested. Failed/malformed queries retain confirmed
values with increasing ages and consistent table/Inspector errors; successful reads recover.
