---
schema: modify-gui-change-state-v1
change_id: 20260831-pixhawk-radio-fallback
status: verifying
mode: backend-coupled
updated: 2026-09-04
baseline_ref: c2daf816f57dd733889b37858a14b9ebbded86e6
last_completed: "Recorded operator evidence that stopping RTCM and reconnecting P1/P2 did not restore IMU; read-only settings/log diagnosis shows those attempts used Ethernet preflight, not Telemetry Radio."
next_question: "After selecting the populated P1/P2 Telemetry Radio endpoints and explicitly connecting them, should Radio request SCALED_IMU/2/3 at 5 Hz immediately after qualification while retaining RTCM priority?"
read_next:
  - docs/changes/20260831-pixhawk-radio-fallback/change-brief.md
  - docs/changes/20260831-pixhawk-radio-fallback/decisions.md
  - docs/changes/20260831-pixhawk-radio-fallback/current-behavior.md
  - docs/changes/20260831-pixhawk-radio-fallback/workflow-impact.md
  - docs/changes/20260831-pixhawk-radio-fallback/impact-analysis.md
  - docs/changes/20260831-pixhawk-radio-fallback/frontend-contract.md
  - docs/changes/20260831-pixhawk-radio-fallback/backend-contract.md
  - docs/changes/20260831-pixhawk-radio-fallback/approval-summary.md
  - docs/changes/20260831-pixhawk-radio-fallback/verification.md
affected_product_docs:
  - README.md
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/05_NAVIGATION.md
  - docs/ui/06_STATUS_MODEL.md
  - docs/ui/07_COMMAND_MODEL.md
  - docs/ui/09_IMPLEMENTATION_PLAN.md
  - docs/ui/10_DESIGN_AUDIT.md
  - docs/ui/11_DESIGN_AUDIT_RESULT.md
blocking_questions: 0
source_changed: true
implementation_allowed: true
approval:
  state: approved
  scope: "Existing approved Radio workflow; the 2026-09-02 guided P1 Radio + P2 Ethernet + Drone Radio topology amendment was withdrawn on 2026-09-03"
  conditions:
    - "No automatic Connect or failover"
    - "No device write without explicit UI confirmation and current identity/isolation evidence"
    - "Physical cable, radio removal, and power-cycle steps remain operator-owned"
  recorded_at: "2026-09-03"
---

# Resume Notes

This packet is separate from `20260826-eth-mavlink-stability`, whose approved scope explicitly
excluded telemetry-radio integration. The 2026-09-02 amendment supersedes the original three-radio
field assumption for this preset: P1 remains D30/NETID25, P2 moves to Ethernet `10.0.0.3:14551`
with its air radio off, and Drone moves to DU/NETID26. Software verification is complete except for
one unrelated environment-owned CUDA/NVENC test failure. Real UI-driven forward, qualification,
restart/rollback, RTCM/RTK, and soak evidence remains `NOT_RUN`.

## 2026-09-02 hardware outcome

The rig gained a third ground radio, so the field topology is now three Telemetry Radio pairs on
NETID 25/26/27 rather than the approved P1 Radio + P2 Ethernet + Drone Radio preset. All three
links are qualified live by expected sysid. The approved preset and its implementation are
unchanged and remain the UI-owned path; this topology was provisioned from the terminal and has no
entry in `pixhawk_topology_operation.json`.

Workstation settings now select Telemetry Radio for all three boards: `D30JNMVH` for P1,
`DU0E49VU` for P2 and `DU0E4O0F` for the Drone, all at 57600.

Still open: an application-owned Connect/qualify has not been run against this topology, and
RTCM/RTK, soak, restart/Resume and Guided rollback remain `NOT_RUN`.

## 2026-09-03 removal outcome

The operator withdrew the guided topology feature. Its UI, commands, controller/worker/core
contract, journal path, lifecycle cleanup, cross-feature locks, and dedicated tests were removed.
Normal per-board connection and explicit P2 ETH MAV workflows remain in scope. The running GUI must
be restarted to load the removal; no device or saved Pixhawk setting was changed by the source edit.
