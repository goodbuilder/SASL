---
schema: modify-gui-change-state-v1
change_id: 20260826-stable-firmware-measurement-cycle
status: paused
mode: backend-coupled
updated: 2026-08-26
baseline_ref: 5bec92529e0954a96522d1de7c5e1e5f2ece38ca
last_completed: "Paused at user direction to prioritize the reproduced Ethernet MAVLink stability failure"
next_question: "Resume approval of the shared shutdown-only vertical slice after the Ethernet MAVLink stability packet is resolved"
read_next:
  - docs/changes/20260826-stable-firmware-measurement-cycle/approval-summary.md
  - docs/changes/20260826-stable-firmware-measurement-cycle/implementation-plan.md
  - docs/changes/20260826-stable-firmware-measurement-cycle/verification.md
  - docs/changes/20260826-stable-firmware-measurement-cycle/change-brief.md
  - docs/changes/20260826-stable-firmware-measurement-cycle/current-behavior.md
  - docs/changes/20260826-stable-firmware-measurement-cycle/workflow-impact.md
  - docs/changes/20260826-stable-firmware-measurement-cycle/impact-analysis.md
  - docs/changes/20260826-stable-firmware-measurement-cycle/decisions.md
  - docs/changes/20260826-stable-firmware-measurement-cycle/frontend-contract.md
  - docs/changes/20260826-stable-firmware-measurement-cycle/backend-contract.md
  - docs/changes/20260826-stable-firmware-measurement-cycle/integration-flow.md
affected_product_docs:
  - README.md
  - workflow.md
  - Pixhawk.md
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/05_NAVIGATION.md
  - docs/ui/06_STATUS_MODEL.md
  - docs/ui/07_COMMAND_MODEL.md
  - ../StereoWIRIS/docs/PROJECT.md
  - ../StereoWIRIS/docs/usage.md
  - ../StereoWIRIS/docs/COORDINATE_CONTRACT.md
  - ../StereoWIRIS/UPDATE.md
blocking_questions: 1
source_changed: false
implementation_allowed: false
approval:
  state: requested
  scope: null
  conditions: []
  recorded_at: null
---

# Resume Notes

No source modification is approved. This packet is paused behind the separately owned Ethernet
MAVLink stability change.
