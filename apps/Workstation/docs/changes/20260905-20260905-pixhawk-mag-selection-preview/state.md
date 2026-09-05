---
schema: modify-gui-change-state-v1
change_id: 20260905-20260905-pixhawk-mag-selection-preview
status: verifying
mode: backend-coupled
updated: 2026-09-05
baseline_ref: "Workstation current worktree; Stable PX4 1.17 hardware evidence from P1/P2 Radio run"
last_completed: "Implemented topology snapshot, slot/role candidate preview, fast priority apply read path, real Qt UI priority run after GPS2 removal, direct PX4 active-device readback, and ATTITUDE delta evidence"
next_question: "Perform a visual/manual Workstation review of the P1/P2 preview at target resolution, then close the packet or record a named residual"
read_next:
  - docs/changes/20260905-20260905-pixhawk-mag-selection-preview/verification.md
affected_product_docs:
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/03_WORKSPACE_LAYOUT.md
blocking_questions: 0
source_changed: true
implementation_allowed: true
approval:
  state: approved
  scope: "P1/P2 MAG topology and priority candidate preview; preserve editable priority and existing active verification; no blocking or automatic slot remap"
  conditions:
    - "No firmware or physical hardware mutation"
    - "No automatic CAL_MAG ID/priority write outside existing Apply path"
  recorded_at: 2026-09-05
---

# Resume Notes

Implementation is complete for the approved slice. Remaining packet gate is visual/manual review
of the live P1/P2 preview; the full-suite residual failures are recorded in `verification.md`.
