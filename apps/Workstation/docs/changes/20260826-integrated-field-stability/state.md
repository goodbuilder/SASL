---
schema: modify-gui-change-state-v1
change_id: 20260826-integrated-field-stability
status: implementing
mode: backend-coupled
updated: 2026-08-26
baseline_ref: c4397995d864576c966702cfd3c398b6032f8990
last_completed: "MAVLink passive gate failed on both sides; Workstation opened for P1 USB and NTRIP session credential setup"
next_question: "Connect only P1 USB and Apply the SASL/NGII session password in Settings > GNSS, then confirm both actions"
read_next:
  - docs/changes/20260826-integrated-field-stability/approval-summary.md
  - docs/changes/20260826-integrated-field-stability/implementation-plan.md
  - docs/changes/20260826-integrated-field-stability/verification.md
  - docs/changes/20260826-integrated-field-stability/current-behavior.md
  - docs/changes/20260826-integrated-field-stability/backend-contract.md
  - docs/changes/20260826-integrated-field-stability/frontend-contract.md
affected_product_docs:
  - README.md
  - WIRIS.md
  - workflow.md
  - docs/RECORDING_SCHEMA.md
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/06_STATUS_MODEL.md
  - docs/ui/07_COMMAND_MODEL.md
  - docs/ui/09_IMPLEMENTATION_PLAN.md
  - ../StereoWIRIS/UPDATE.md
  - ../StereoWIRIS/docs/usage.md
  - ../../README.md
blocking_questions: 0
source_changed: false
implementation_allowed: true
approval:
  state: approved
  scope: "Integrated field-stability goal in approval-summary.md and implementation-plan.md"
  conditions: []
  recorded_at: "2026-08-26T21:23:57+09:00"
---

# Resume Notes

The user explicitly started the previously designed goal. Preserve all existing dirty changes and
prior packet decisions. Short independent probes precede any 60-minute soak. No device setting,
firmware, reboot/power-cycle or destructive recovery is approved. Six GPT-5.6-sol ultra read-only
audits are in progress; merge their findings into this packet before selecting the first source slice.
The integrated 60-minute soak is Workstation-only; StereoWIRIS Legacy has separate focused blank
restart/new-session/MP4 gates and never owns hardware concurrently with Workstation.
