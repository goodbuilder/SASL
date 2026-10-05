---
schema: modify-gui-change-state-v1
change_id: 20260903-jsonl-csv-mirror
status: verifying
mode: backend-coupled
updated: 2026-09-04
baseline_ref: "3288aee + preserved dirty worktree"
last_completed: "Storage, controller, Data Status, and dependent XLSX/lifecycle tests passed"
next_question: "Which README path should be updated? Proposed target: ./README.md"
read_next:
  - docs/changes/20260903-jsonl-csv-mirror/change-brief.md
  - docs/changes/20260903-jsonl-csv-mirror/current-behavior.md
  - docs/changes/20260903-jsonl-csv-mirror/field-inventory.md
  - docs/changes/20260903-jsonl-csv-mirror/approval-summary.md
  - docs/changes/20260903-jsonl-csv-mirror/workflow-impact.md
  - docs/changes/20260903-jsonl-csv-mirror/ui-quality-and-stability.md
  - docs/changes/20260903-jsonl-csv-mirror/impact-analysis.md
  - docs/changes/20260903-jsonl-csv-mirror/frontend-contract.md
  - docs/changes/20260903-jsonl-csv-mirror/backend-contract.md
  - docs/changes/20260903-jsonl-csv-mirror/integration-flow.md
  - docs/changes/20260903-jsonl-csv-mirror/implementation-plan.md
affected_product_docs:
  - README.md
  - docs/RECORDING_SCHEMA.md
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/03_WORKSPACE_LAYOUT.md
  - docs/ui/06_STATUS_MODEL.md
  - docs/ui/07_COMMAND_MODEL.md
  - docs/ui/README.md
blocking_questions: 0
source_changed: true
implementation_allowed: true
approval:
  state: approved
  scope: "Add same-stem CSV mirrors for the 16 sessionless JSONL streams, with independent bounded retry/state and Data Status visibility; preserve JSONL authority, XLSX, coordinates, clocks, and legacy Session outputs."
  conditions:
    - "No historical backfill or aggregate latest_measurement.csv"
    - "CSV failure must not reject Measurement or JSONL persistence"
    - "No coordinate, clock, or producer-field reinterpretation"
  recorded_at: 2026-09-04
---

# Resume Notes

The proposed slice is a backend-coupled Data Storage change with a small existing-table projection.
The user has confirmed that every current JSONL stream should have a CSV duplicate and explicitly
authorized implementation on 2026-09-04. Source and directly coupled Data Status changes are now
implemented and focused-verified. README target confirmation remains required before changing any
README.
