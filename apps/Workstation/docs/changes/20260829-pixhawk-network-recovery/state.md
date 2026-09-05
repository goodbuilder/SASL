---
schema: modify-gui-change-state-v1
change_id: 20260829-pixhawk-network-recovery
status: interviewing
mode: backend-coupled
updated: 2026-08-30
baseline_ref: 37417a222cf8b6053094c33bb0bb666507041dc6
last_completed: "User decided pre-existing offline affected owners do not veto another selected role; required reconnect evidence uses selected role plus previously healthy owners"
next_question: "In the normal Assign-before-Connect case where the selected role itself was not connected before the click, should verified host routes persist until later Connect, or must Assign obtain target device evidence and restore on timeout?"
read_next:
  - docs/changes/20260829-pixhawk-network-recovery/approval-summary.md
  - docs/changes/20260829-pixhawk-network-recovery/decisions.md
  - docs/changes/20260829-pixhawk-network-recovery/workflow-impact.md
  - docs/changes/20260829-pixhawk-network-recovery/backend-contract.md
  - docs/changes/20260829-pixhawk-network-recovery/integration-flow.md
affected_product_docs:
  - README.md
  - Pixhawk.md
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/03_WORKSPACE_LAYOUT.md
  - docs/ui/06_STATUS_MODEL.md
  - docs/ui/07_COMMAND_MODEL.md
  - docs/ui/09_IMPLEMENTATION_PLAN.md
blocking_questions: 1
source_changed: true
implementation_allowed: false
approval:
  state: approved
  scope: "Explicit confirmed one-/many-NIC Camera/P1/P2 NetworkManager role composition and verified move with affected-owner automatic reconnect/rollback"
  conditions: []
  recorded_at: "2026-08-29T16:42:26+09:00"
assignment_ui_amendment:
  state: completed
  scope: "Candidate-only discovery rows plus independent State/NIC/Local IPv4/Endpoint/Route summaries for P1 and P2"
  preserved: "side-specific Assign and Connect preflight; no host/PX4 mutation; same/different NIC allowed"
  verification: "221 focused; 1583 passed, 2 skipped full; candidate-only table and independent P1/P2 card visual PASS"
  blocking_decision: null
  recorded_at: "2026-08-29T15:24:18+09:00"
network_role_application_amendment:
  state: verifying
  scope: "Explicit operator-confirmed Camera/Pixhawk NetworkManager role application on any selected physical NIC"
  supersedes: "Prior no-host-network-mutation decision only"
  composition: "A NIC may hold Camera, P1 and P2 simultaneously; mutations are role-scoped unions"
  topology: "One or many runtime-discovered NICs; no fixed count/name/model; roles distributed arbitrarily"
  move_policy: "Verify target first; remove only moved source role; commit last; rollback on failure"
  active_owner_policy: "Automatically stop/reconnect only affected source/target NIC owners; unrelated NICs continue; mapping locks preserved"
  blocking_decision: null
  recorded_at: "2026-08-29"
offline_assignment_amendment:
  state: interviewing
  scope: "Keep automatic previous-NIC restoration on required device-evidence failure while correcting which pre-operation owners are required to recover"
  field_evidence: "Split-NIC Assign reached valid host state but silent P2 caused device-evidence timeout and ROLLBACK PARTIAL; direct role transaction plus later PX4 boot produced USB-free P1/P2 LIVE"
  selected_failure_policy: "Automatically restore previous NIC; do not commit ASSIGNED DEVICE NOT SEEN"
  required_evidence_policy: "Selected role plus other affected owners healthy before mutation; already-offline owners remain offline and cannot veto"
  blocking_decision: "Normal Assign-before-Connect behavior when the selected role has no pre-operation live session"
  recorded_at: "2026-08-30"
---

# Resume Notes

Preserve the existing dirty worktree and all other packet scopes. The approved NetworkManager
implementation remains the baseline, but no further source edit is authorized until the offline
assignment completion/rollback decision is recorded and the amendment is re-approved. PX4
firmware/config remains read-only. Same Camera/Pixhawk NIC selection is allowed without UI warning;
do not claim long-duration field stability from the successful short split-NIC run.
