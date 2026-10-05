---
schema: modify-gui-change-state-v1
change_id: 20260826-canonical-gstreamer-runtime
status: verifying
mode: backend-coupled
updated: 2026-08-27
baseline_ref: 5bec92529e0954a96522d1de7c5e1e5f2ece38ca
last_completed: "Current host and contaminated-environment launcher/doctor checks pass with closed Python/GI/GStreamer boundaries"
next_question: "After MAVLink load isolation, can sustained dual-camera software/NVIDIA recording and cleanup verification complete?"
read_next:
  - docs/changes/20260826-canonical-gstreamer-runtime/verification.md
  - README.md
affected_product_docs:
  - README.md
  - apps/StereoWIRIS/docs/usage.md
  - apps/Workstation/README.md
blocking_questions: 0
source_changed: true
implementation_allowed: true
approval:
  state: approved
  scope: canonical-gstreamer-runtime
  conditions: []
  recorded_at: 2026-08-26
---

# Resume Notes

Code, manifests, launchers, tests, and product documentation are implemented. The current host venv
has now been repaired and both launchers pass with `PYTHONNOUSERSITE=1`; NVIDIA driver 595.84,
`nvh264dec`, and short real dual-camera NVDEC startup also pass. The packet remains in verification
for a clean-host recreation proof, real software-decoder coverage, sustained dual-camera
recording/MP4/cleanup, and a separate clean-OS/VM proof. The repository portability audit now passes,
and a same-host fresh checkout/new venv passed both launchers under isolated HOME.
The launchers and doctor now also clear ambient Python, loader, GI and GStreamer plugin/system/
scanner variables before adding canonical paths. Deployment bootstrap uses `/usr/bin/python3 -S`
without importing third-party `packaging`; execution-only version checks load it after the venv gate.
