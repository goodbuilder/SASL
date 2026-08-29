---
schema: modify-gui-change-state-v1
change_id: 20260829-pixhawk-network-recovery
status: verifying
mode: backend-coupled
updated: 2026-08-29
baseline_ref: 37417a222cf8b6053094c33bb0bb666507041dc6
last_completed: "Assignment-centric Pixhawk UI complete; 221 focused and 1583 passed/2 skipped full suite; visual/static/launcher checks pass"
next_question: "After the planned reboot, restart the new Workstation and run real Find/Assign/Connect through expected-sysid 60-second LIVE"
read_next:
  - docs/changes/20260829-pixhawk-network-recovery/approval-summary.md
  - docs/changes/20260829-pixhawk-network-recovery/implementation-plan.md
  - docs/changes/20260829-pixhawk-network-recovery/verification.md
  - docs/changes/20260829-pixhawk-network-recovery/backend-contract.md
  - docs/changes/20260829-pixhawk-network-recovery/frontend-contract.md
affected_product_docs:
  - README.md
  - Pixhawk.md
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/03_WORKSPACE_LAYOUT.md
  - docs/ui/06_STATUS_MODEL.md
  - docs/ui/07_COMMAND_MODEL.md
  - docs/ui/09_IMPLEMENTATION_PLAN.md
blocking_questions: 0
source_changed: true
implementation_allowed: true
approval:
  state: approved
  scope: "Read-only Pixhawk NIC/address/route discovery, independent P1/P2 assignment persistence and summaries, route-preflight partial Connect, and compact UI workflow"
  conditions: []
  recorded_at: "2026-08-29T12:41:37+09:00"
assignment_ui_amendment:
  state: completed
  scope: "Candidate-only discovery rows plus independent State/NIC/Local IPv4/Endpoint/Route summaries for P1 and P2"
  preserved: "side-specific Assign and Connect preflight; no host/PX4 mutation; same/different NIC allowed"
  verification: "221 focused; 1583 passed, 2 skipped full; candidate-only table and independent P1/P2 card visual PASS"
  blocking_decision: null
  recorded_at: "2026-08-29T15:24:18+09:00"
---

# Resume Notes

Preserve the existing dirty worktree and all other packet scopes. Host IP/routes/NetworkManager and
PX4 firmware/config must remain read-only in this change. Same Camera/Pixhawk NIC selection is allowed
without UI warning by explicit user decision; do not claim long-duration field stability from software tests.
