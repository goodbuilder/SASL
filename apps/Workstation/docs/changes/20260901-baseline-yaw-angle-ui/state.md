---
schema: modify-gui-change-state-v1
change_id: 20260901-baseline-yaw-angle-ui
status: verifying
mode: backend-coupled
updated: 2026-09-01
baseline_ref: c2daf81
last_completed: "Software implementation, focused/dependent/full-range regression, visual review, README, and runtime check completed"
next_question: "When P1/P2/GNSS hardware is available, verify CURRENT/PARTIAL/STALE/recovery and signed left/right readability on the real rig"
read_next:
  - docs/changes/20260901-baseline-yaw-angle-ui/verification.md
  - docs/changes/20260901-baseline-yaw-angle-ui/ui-quality-and-stability.md
  - docs/changes/20260901-baseline-yaw-angle-ui/state.md
affected_product_docs:
  - README.md
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/03_WORKSPACE_LAYOUT.md
  - docs/ui/06_STATUS_MODEL.md
blocking_questions: 0
source_changed: true
implementation_allowed: true
approval:
  state: approved
  scope: "Signed horizontal Baseline-P1 and Baseline-P2 live values plus fixed top view below Pixhawk baseline; no verdict/history/persistence"
  conditions: []
  recorded_at: 2026-09-01
---

# Resume Notes

The approved software slice is implemented. Baseline−P1 and Baseline−P2 are separate independent
fields, and no setting/command/worker/schema/measurement semantic was added. Focused 122, dependent
214, and split full-range 764 + 1041 tests passed; one unrelated NVIDIA recorder test fails in
isolation. `./run.sh --check` passed with optional nvh264dec warnings. Keep the packet verifying
until real P1/P2/GNSS visual/freshness behavior is checked or the user accepts that residual.
