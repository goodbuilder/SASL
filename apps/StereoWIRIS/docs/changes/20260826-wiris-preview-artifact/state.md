---
schema: modify-gui-change-state-v1
change_id: 20260826-wiris-preview-artifact
status: approved
mode: backend-coupled
updated: 2026-08-26
baseline_ref: c4397995d864576c966702cfd3c398b6032f8990
last_completed: "Workstation GUI-decoupled frame/evidence/measurement architecture approved on 2026-08-26"
next_question: "Implement the approved exact-FrameKey shared-memory vertical slices and run bounded backpressure qualification"
read_next:
  - docs/changes/20260826-wiris-preview-artifact/approval-summary.md
  - docs/changes/20260826-wiris-preview-artifact/implementation-plan.md
  - docs/changes/20260826-wiris-preview-artifact/verification.md
affected_product_docs:
  - docs/README.md
  - docs/usage.md
  - docs/testing.md
  - UPDATE.md
  - ../Workstation/WIRIS.md
blocking_questions: 0
source_changed: true
implementation_allowed: true
approval:
  state: approved
  scope: "Keep the active decoder on RTP gap, suppress decoded output until a clean non-delta intra AU PTS, and retain software fallback only for decoder health failures"
  conditions: []
  recorded_at: "2026-08-26T14:03:19+09:00"
amendment:
  state: superseded_by_restored_diagnostic
  scope: "Temporarily change only VISIBLE QP from 1 to 5 under a single RTSP owner, measure motion freeze/bandwidth/quality, then restore QP 1 regardless of result"
  excludes: "permanent encoder change, bitrate/GOP/IDR change, reboot, Workstation modification"
workstation_amendment:
  state: approved
  scope: "Use one GStreamer rtspsrc per camera, tee parsed H264 to preview/YOLO and mux/filesink recording, and gate corrupt RTP/AU chains before the tee"
  preserves: "Workstation-only operation, manual generic reconnect policy, UI layout, camera encoder 0/1/90/0"
  backend_policy: "GStreamer raw H264 integrity-gated; OpenCV/FFmpeg and PyAV decoded re-encode allowed with unverified provenance"
  recovery_policy: "Wait clean non-delta intra in the same session, keep one file, fail closed after 30 seconds"
  recorded_at: "2026-08-26T18:39:12+09:00"
gui_decoupling_amendment:
  state: approved
  scope: "Remove GUI from native-frame transport, pairing, measurement compute, and evidence persistence by using exact FrameKey descriptors, a bounded shared-memory ring with pin/lease, independent control/data channels, and durable writer ACK/hash"
  rejects: "BlockingQueuedConnection as the final architecture, continuous multi-megabyte QImage queued signals, timestamp-nearest evidence substitution, and unlimited CUDA inference"
  preserves: "single-rtspsrc integrity gate, plain MP4 finalization, Workstation-only operation, exact camera settings, and latest-only preview"
  acceptance: "bounded queues under a 2-second GUI stall, independent Stop/Error, exact frame/hash provenance, 10-minute synthetic qualification, and 60-minute dual-camera soak"
  recorded_at: "2026-08-26T19:51:08+09:00"
---

# Resume Notes

The single-rtspsrc integrity gate and plain-MP4 path now pass focused tests and a real 404-second
record/decode run. Motion then exposed a second Workstation boundary: continuous native QImages
queued through Qt made parent RSS rise from roughly 0.5 GiB past 4 GiB and the GUI became
unresponsive. A temporary BlockingQueuedConnection bounded RSS but couples image delivery to GUI
and Stop, so the user rejected it as a final solution. The approved next architecture keeps native
rasters in a bounded child/shared-memory ring, pairs by exact `(camera, generation, frame_id)`, pins
only accepted pairs, persists them off-GUI, and reports READY only after durable ACK/hash. Control
events remain independent of all image queues. Defer `WIRIS.md` completion until the new synthetic
and 60-minute hardware acceptance pass.
