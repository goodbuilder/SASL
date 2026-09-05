---
schema: modify-gui-change-state-v1
change_id: 20260829-observation-first-measurement
status: verifying
mode: backend-coupled
updated: 2026-09-02
baseline_ref: a8f92b91691ca272af44b7a0910267b44afa304d
last_completed: "Corrected decoder-preflight rollback; 2 focused and 133 dependent tests plus 1889-test full suite passed"
next_question: "With live Preview and explicit NVIDIA unavailable, press Measurement once and confirm both Preview panes remain live; then Stop, select Auto, and confirm Measurement starts with decoded frames"
read_next:
  - docs/changes/20260829-observation-first-measurement/implementation-plan.md
  - docs/changes/20260829-observation-first-measurement/backend-contract.md
  - docs/changes/20260829-observation-first-measurement/current-behavior.md
  - docs/changes/20260829-observation-first-measurement/decisions.md
  - docs/changes/20260829-observation-first-measurement/verification.md
affected_product_docs:
  - README.md
  - docs/RECORDING_SCHEMA.md
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/06_STATUS_MODEL.md
  - docs/ui/07_COMMAND_MODEL.md
  - docs/ui/09_IMPLEMENTATION_PLAN.md
blocking_questions: 0
source_changed: true
implementation_allowed: true
approval:
  state: approved
  scope: "Observation-first Measurement admission, independent source rejoin, segmented recording, and non-gating best-effort persistence"
  conditions:
    - "Measurement starts immediately with zero available sources"
    - "Integrity and persistence never gate the current or next Measurement"
    - "Existing result accuracy, identity, coordinate and calibration gates remain authoritative"
    - "No Session/package/catalog/revalidation workflow"
  recorded_at: 2026-08-29
---

# Resume Notes

The user approved the complete plan and requested implementation. Preserve every unrelated dirty
change, especially the active tolerant decoded-recording and integrated-field-stability work.
