---
schema: modify-gui-change-state-v1
change_id: 20260905-20260905-pixhawk-mag-selection-preview
status: verifying
mode: backend-coupled
updated: 2026-10-12
baseline_ref: "Workstation current worktree; Stable PX4 1.17 hardware evidence from P1/P2 Radio run"
last_completed: "Implemented staged MAG priority edits, timeout-only PARAM_SET retry, partial-readback invalidation, and the PX4 v1.17 voter-gap guard; latest focused regression 80 passed"
next_question: "Live visual and disarmed P1/P2 hardware acceptance remain unverified; obtain a separately authorized hardware session if that evidence is required"
read_next:
  - docs/changes/20260905-20260905-pixhawk-mag-selection-preview/decisions.md
  - docs/changes/20260905-20260905-pixhawk-mag-selection-preview/implementation-plan.md
  - docs/changes/20260905-20260905-pixhawk-mag-selection-preview/verification.md
affected_product_docs:
  - README.md
  - Pixhawk.md
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/03_WORKSPACE_LAYOUT.md
  - docs/ui/07_COMMAND_MODEL.md
blocking_questions: 0
source_changed: true
implementation_allowed: true
approval:
  state: approved
  scope: "Original P1/P2 topology preview plus 2026-10-12 RCA remediation: staged edits, timeout-only PARAM_SET retry, failed-write reconciliation, and proven PX4 v1.17 voter-gap guard"
  conditions:
    - "No firmware or physical hardware mutation"
    - "No automatic param save or reboot"
    - "No silent priority filler rewrite or CAL_MAG ID remap"
  recorded_at: 2026-10-12
---

# Resume Notes

The 2026-10-12 approved RCA follow-up is implemented and its focused automated checks pass. Keep
the earlier full-suite residual failures and original live visual/hardware verification status in
`verification.md`; no Pixhawk was connected or changed in this code-only slice.
