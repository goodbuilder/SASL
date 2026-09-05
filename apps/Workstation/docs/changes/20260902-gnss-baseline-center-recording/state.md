---
schema: modify-gui-change-state-v1
change_id: 20260902-gnss-baseline-center-recording
status: completed
mode: backend-coupled
updated: 2026-09-02
baseline_ref: 3288aee5f5fb7af5ab30024f93e838f76724a0b0
last_completed: "Approved GNSS baseline-center persistence slice completed: 60 focused, 252 affected and qualified 1887-pass full regression; storage/XLSX/runtime/docs checks complete"
next_question: "No open question; packet completed. Optional real three-receiver field logging remains hardware/manual evidence."
read_next:
  - docs/changes/20260902-gnss-baseline-center-recording/change-brief.md
  - docs/changes/20260902-gnss-baseline-center-recording/current-behavior.md
  - docs/changes/20260902-gnss-baseline-center-recording/decisions.md
  - docs/changes/20260902-gnss-baseline-center-recording/workflow-impact.md
  - docs/changes/20260902-gnss-baseline-center-recording/ui-quality-and-stability.md
  - docs/changes/20260902-gnss-baseline-center-recording/impact-analysis.md
  - docs/changes/20260902-gnss-baseline-center-recording/frontend-contract.md
  - docs/changes/20260902-gnss-baseline-center-recording/backend-contract.md
  - docs/changes/20260902-gnss-baseline-center-recording/integration-flow.md
  - docs/changes/20260902-gnss-baseline-center-recording/implementation-plan.md
  - docs/changes/20260902-gnss-baseline-center-recording/approval-summary.md
  - docs/changes/20260902-gnss-baseline-center-recording/verification.md
affected_product_docs:
  - README.md
  - docs/RECORDING_SCHEMA.md
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/06_STATUS_MODEL.md
  - docs/ui/09_IMPLEMENTATION_PLAN.md
  - docs/ui/10_DESIGN_AUDIT.md
  - docs/ui/11_DESIGN_AUDIT_RESULT.md
blocking_questions: 0
source_changed: true
implementation_allowed: false
approval:
  state: approved
  scope: "Dedicated best-effort GNSS baseline-center JSONL stream and additive Latest Measurement XLSX sheet only"
  conditions:
    - "One attempted row per primary Drone GNSS callback only while recording"
    - "Use existing best-effort bulk writer; never gate Telemetry or Measurement"
    - "Keep raw JSONL, canonical Measurement fields and legacy Parquet unchanged"
    - "Additive Latest Measurement XLSX sheet is included"
  recorded_at: "2026-09-02"
---

# Resume Notes

The approved derived JSONL/XLSX persistence slice is implemented and software-verified. No device
was commanded. Real three-receiver field logging remains optional manual evidence and does not
weaken the independently verified recording/storage contract.
