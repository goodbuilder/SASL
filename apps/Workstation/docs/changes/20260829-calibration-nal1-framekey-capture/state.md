---
schema: modify-gui-change-state-v1
change_id: 20260829-calibration-nal1-framekey-capture
status: completed
mode: backend-coupled
updated: 2026-08-29
baseline_ref: 7242452
last_completed: "Implemented and software-verified exact NAL1/FrameKey Auto capture"
next_question: "None; real dual-WIRIS cadence remains optional hardware/manual evidence"
read_next:
  - docs/changes/20260829-calibration-nal1-framekey-capture/approval-summary.md
  - docs/changes/20260829-calibration-nal1-framekey-capture/implementation-plan.md
  - docs/changes/20260829-calibration-nal1-framekey-capture/backend-contract.md
affected_product_docs:
  - README.md
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/06_STATUS_MODEL.md
  - docs/ui/07_COMMAND_MODEL.md
  - docs/ui/09_IMPLEMENTATION_PLAN.md
  - WIRIS.md
blocking_questions: 0
source_changed: true
implementation_allowed: true
approval:
  state: approved
  scope: "Calibration Auto capture accepts only exact NAL1 non-delta native FrameKeys; no timestamp-near substitution"
  conditions: []
  recorded_at: "2026-08-29T17:33:30+09:00"
---

# Resume Notes

Implementation and software verification are complete for the exact approved scope. Read
`verification.md` for commands/results and the accepted real-hardware residual. Preserve the
separate app-receive timing packet and unrelated dirty work.
