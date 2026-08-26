---
schema: modify-gui-change-state-v1
change_id: 20260826-eth-mavlink-stability
status: verifying
mode: backend-coupled
updated: 2026-08-26
baseline_ref: c4397995d864576c966702cfd3c398b6032f8990
last_completed: "Two hardware recovery attempts recorded: first restored only a 208-second dual-link capture; second produced ping-alive but zero Ethernet MAVLink before identity, including after a clean Workstation restart"
next_question: "Temporarily connect only the named P1 USB link for read-only heartbeat sysid, AUTOPILOT_VERSION, MAV_2_* and possible mavlink status inventory; do not write parameters/configuration"
read_next:
  - docs/changes/20260826-eth-mavlink-stability/PLAN.md
  - docs/changes/20260826-eth-mavlink-stability/approval-summary.md
  - docs/changes/20260826-eth-mavlink-stability/change-brief.md
  - docs/changes/20260826-eth-mavlink-stability/current-behavior.md
  - docs/changes/20260826-eth-mavlink-stability/workflow-impact.md
  - docs/changes/20260826-eth-mavlink-stability/impact-analysis.md
  - docs/changes/20260826-eth-mavlink-stability/decisions.md
  - docs/changes/20260826-eth-mavlink-stability/frontend-contract.md
  - docs/changes/20260826-eth-mavlink-stability/backend-contract.md
  - docs/changes/20260826-eth-mavlink-stability/integration-flow.md
  - docs/changes/20260826-eth-mavlink-stability/implementation-plan.md
  - docs/changes/20260826-eth-mavlink-stability/verification.md
  - docs/changes/20260826-eth-mavlink-stability/ui-quality-and-stability.md
affected_product_docs:
  - README.md
  - Pixhawk.md
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/06_STATUS_MODEL.md
  - docs/ui/07_COMMAND_MODEL.md
  - docs/ui/09_IMPLEMENTATION_PLAN.md
blocking_questions: 1
source_changed: true
implementation_allowed: true
approval:
  state: approved
  scope: "PLAN.md software slices 1-7 and recovery policy 1; hardware slices 8-9 remain evidence-gated"
  conditions:
    - "Update Pixhawk.md with cause and remedy only after dual-Pixhawk hardware acceptance succeeds"
    - "Describe the cause as a high-confidence inference and the Workstation change as a verified mitigation unless a PX4 stack trace or firmware fix proves more"
    - "Resolution documentation requires three independent cold-boot cycles of at least 10 minutes, a 60-minute dual-link full-load soak, one-side isolation, and verified recovery"
  recorded_at: "2026-08-26T16:32:20+09:00"
---

# Resume Notes

This packet is approved for implementation with recovery policy 1. The actual hardware firmware
could not be re-read because both boards were ping-alive but MAVLink-silent and no USB serial device
was present; do not infer v1.17.0 from the untracked bundle. Re-read firmware/sysid/params after safe
operator recovery and before hardware acceptance. Pixhawk.md remains unchanged until every hardware
resolution gate succeeds.

Handoff integrity and workspace notes:

- `PLAN.md` SHA-256: `b51477f7d526e9eb0fb959e4bd7391341adef53d11c913d04555405bcea2249d`.
- The entire change packet is currently untracked; it is available to another session in this
  workspace but will not survive a fresh clone until intentionally added/committed.
- ETH owner files already have unrelated/user modifications; preserve their diffs.
- `python3 tools/check_portable_paths.py` currently reports 12 pre-existing violations in
  StereoWIRIS calibration/change evidence, Workstation `intrinsics_zoom_08.json`, Simulation
  `goal.md`, and StereoWIRIS settings. It did not report this ETH plan/packet.
