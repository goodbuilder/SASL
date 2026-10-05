---
schema: modify-gui-change-state-v1
change_id: 20260901-baseline-yaw-angle-ui
status: verifying
mode: backend-coupled
updated: 2026-09-03
baseline_ref: c2daf81
last_completed: "GNSS-only Telemetry angle implemented; focused/dependent/full tests and launcher check completed"
next_question: "Optional hardware/manual check of CURRENT/STALE angle readability during camera installation"
read_next:
  - docs/changes/20260901-baseline-yaw-angle-ui/change-brief.md
  - docs/changes/20260901-baseline-yaw-angle-ui/current-behavior.md
  - docs/changes/20260901-baseline-yaw-angle-ui/decisions.md
  - docs/changes/20260901-baseline-yaw-angle-ui/frontend-contract.md
  - docs/changes/20260901-baseline-yaw-angle-ui/backend-contract.md
  - docs/changes/20260901-baseline-yaw-angle-ui/integration-flow.md
  - docs/changes/20260901-baseline-yaw-angle-ui/implementation-plan.md
  - docs/changes/20260901-baseline-yaw-angle-ui/verification.md
  - docs/changes/20260901-baseline-yaw-angle-ui/state.md
affected_product_docs:
  - README.md
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/03_WORKSPACE_LAYOUT.md
  - docs/ui/06_STATUS_MODEL.md
blocking_questions: 0
source_changed: true
implementation_allowed: false
approval:
  state: approved
  scope: "Add GNSS-only horizontal P1→P2 baseline versus P1/P2-midpoint-to-Drone angle and absolute 90° error to Devices > Telemetry; remove the unneeded Drone ATTITUDE amendment; preserve F/R/D/Rg and all existing device/measurement/recording contracts"
  conditions: []
  recorded_at: "2026-09-03"
---

# Resume Notes

The 2026-09-01 P1/P2 yaw diagnostic remains separate. On 2026-09-03 the operator clarified that
camera installation must use the GNSS-only angle between the P1→P2 baseline and the midpoint-to-Drone
GNSS vector. The approved additive Telemetry slice is implemented: the live mapping and fixed table
show the angle and absolute 90° error; no Drone ATTITUDE/yaw, device, worker, command, persistence,
measurement, or recording change was made. Software verification passed; real-rig readability remains
optional hardware/manual evidence.
