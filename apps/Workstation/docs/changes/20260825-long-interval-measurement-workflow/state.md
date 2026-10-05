---
schema: modify-gui-change-state-v1
change_id: 20260825-long-interval-measurement-workflow
status: paused
mode: backend-coupled
updated: 2026-08-26
baseline_ref: c4397995d864576c966702cfd3c398b6032f8990
last_completed: "Paused at user direction to prioritize the reproduced Ethernet MAVLink stability failure"
next_question: "Resume the recorded real-hardware Measurement field matrix after the Ethernet MAVLink stability packet is resolved"
read_next:
  - docs/changes/20260825-long-interval-measurement-workflow/approval-summary.md
  - docs/changes/20260825-long-interval-measurement-workflow/implementation-plan.md
  - docs/changes/20260825-long-interval-measurement-workflow/verification.md
  - docs/changes/20260825-long-interval-measurement-workflow/backend-contract.md
  - docs/changes/20260825-long-interval-measurement-workflow/frontend-contract.md
  - docs/changes/20260825-long-interval-measurement-workflow/integration-flow.md
affected_product_docs:
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/02_INFORMATION_ARCHITECTURE.md
  - docs/ui/03_WORKSPACE_LAYOUT.md
  - docs/ui/05_NAVIGATION.md
  - docs/ui/06_STATUS_MODEL.md
  - docs/ui/07_COMMAND_MODEL.md
  - workflow.md
  - docs/RECORDING_SCHEMA.md
  - README.md
blocking_questions: 0
source_changed: true
implementation_allowed: true
approval:
  state: approved
  scope: "Complete sessionless availability-first Measurement/Data vertical slice in approval-summary.md"
  conditions:
    - "Storage, old Session state, completeness, and save failures never gate Measurement Start"
    - "No existing evidence migration, rewrite, or deletion"
    - "No measurement algorithm, coordinate-frame, or device-protocol change"
  recorded_at: 2026-08-26T12:36:43+09:00
---

# Resume Notes

Implementation is complete in the working tree. Verification is paused while the separately owned
Ethernet MAVLink stability failure is resolved. Preserve all existing source and evidence.
