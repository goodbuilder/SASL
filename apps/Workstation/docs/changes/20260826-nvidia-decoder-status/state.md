---
schema: modify-gui-change-state-v1
change_id: 20260826-nvidia-decoder-status
status: verifying
mode: backend-coupled
updated: 2026-08-26
baseline_ref: c4397995d864576c966702cfd3c398b6032f8990
last_completed: "Software slices 1-5 PASS: focused 101, full 1231 passed/1 skipped, doctor auto->nvh264dec, CUDA-memory synthetic decode, stable UI sequence, and live Workstation process/window"
next_question: "After closing the separately running StereoWIRIS app, start Workstation Preview and verify Left/Right both show GPU · nvh264dec with live frames and sustained resource evidence"
read_next:
  - docs/changes/20260826-nvidia-decoder-status/approval-summary.md
  - docs/changes/20260826-nvidia-decoder-status/current-behavior.md
  - docs/changes/20260826-nvidia-decoder-status/implementation-plan.md
  - docs/changes/20260826-nvidia-decoder-status/verification.md
  - docs/changes/20260826-nvidia-decoder-status/frontend-contract.md
  - docs/changes/20260826-nvidia-decoder-status/backend-contract.md
affected_product_docs:
  - README.md
  - docs/ui/01_USER_WORKFLOW.md
  - docs/ui/06_STATUS_MODEL.md
  - docs/ui/09_IMPLEMENTATION_PLAN.md
blocking_questions: 0
source_changed: true
implementation_allowed: true
approval:
  state: approved
  scope: "Auto GPU-first setting plus actual decoder/fallback projection in existing Camera status, CAM tooltip, and Diagnostics Performance"
  conditions:
    - "No new worker, decoder concurrency, rate, quality, GPU clock, or affinity policy"
    - "Auto CPU fallback remains available and is visibly distinguished from explicit Software"
    - "No new global badge; reuse Camera status and Diagnostics Performance"
    - "Real dual-WIRIS preview remains a hardware/manual acceptance condition"
  recorded_at: "2026-08-26T17:45:05+09:00"
---

# Resume Notes

Approved software implementation is complete and persisted mode is Auto. NVIDIA hardware,
driver/plugin, doctor selection, synthetic CUDA-memory decode, UI projection, and regressions pass.
Keep the packet verifying until real Left/Right WIRIS Preview is tested with StereoWIRIS stopped.
