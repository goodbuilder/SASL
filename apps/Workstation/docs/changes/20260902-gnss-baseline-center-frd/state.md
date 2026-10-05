---
schema: modify-gui-change-state-v1
change_id: 20260902-gnss-baseline-center-frd
status: completed
mode: backend-coupled
updated: 2026-09-02
baseline_ref: 3288aee5f5fb7af5ab30024f93e838f76724a0b0
last_completed: "Approved GNSS baseline-center vertical slice completed: 16 focused, 257 affected, and qualified 1877-pass full regression; runtime/visual/scale/docs checks complete"
next_question: "No open question; packet completed. Optional real stationary three-receiver comparison remains hardware/manual evidence."
read_next:
  - docs/changes/20260902-gnss-baseline-center-frd/change-brief.md
  - docs/changes/20260902-gnss-baseline-center-frd/current-behavior.md
  - docs/changes/20260902-gnss-baseline-center-frd/decisions.md
  - docs/changes/20260902-gnss-baseline-center-frd/workflow-impact.md
  - docs/changes/20260902-gnss-baseline-center-frd/ui-quality-and-stability.md
  - docs/changes/20260902-gnss-baseline-center-frd/impact-analysis.md
  - docs/changes/20260902-gnss-baseline-center-frd/frontend-contract.md
  - docs/changes/20260902-gnss-baseline-center-frd/backend-contract.md
  - docs/changes/20260902-gnss-baseline-center-frd/integration-flow.md
  - docs/changes/20260902-gnss-baseline-center-frd/implementation-plan.md
  - docs/changes/20260902-gnss-baseline-center-frd/approval-summary.md
  - docs/changes/20260902-gnss-baseline-center-frd/verification.md
affected_product_docs:
  - README.md
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/03_WORKSPACE_LAYOUT.md
  - docs/ui/06_STATUS_MODEL.md
  - docs/ui/09_IMPLEMENTATION_PLAN.md
  - docs/ui/10_DESIGN_AUDIT.md
  - docs/ui/11_DESIGN_AUDIT_RESULT.md
blocking_questions: 0
source_changed: true
implementation_allowed: false
approval:
  state: approved
  scope: "Read-only GNSS antenna-midpoint local-level F/R/D/Rg Telemetry display, shared ideal-disparity Rg/readiness, tests and product docs only"
  conditions:
    - "Keep GNSS-derived frame separate from camera baseline_center_frd"
    - "No commands, settings, persistence, recording schema, Measurement gate, device/protocol work, or new worker"
    - "Maximum source age 2.0 s; maximum three-source receive-timestamp spread 1000 ms"
  recorded_at: "2026-09-02"
---

# Resume Notes

The approved read-only GNSS-derived Telemetry display is implemented and software-verified.
No device was commanded. Real three-receiver field comparison remains optional manual evidence and
does not weaken the independently proven coordinate transform.
