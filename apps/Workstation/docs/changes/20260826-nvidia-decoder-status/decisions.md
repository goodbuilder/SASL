# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-08-26 — Baseline diagnosis

- NVIDIA RTX 4050/driver 595.84 and GStreamer `nvh264dec` are currently healthy.
- Synthetic 1080p30 decode and CUDA-memory output passed.
- Persisted `rtsp_decoder=software` is the direct reason Workstation selects `avdec_h264`.

## Pending — Fallback policy and display placement

Recommended: Auto GPU-first with visible CPU fallback. Reuse Camera Decoder Left/Right as primary,
add actual decoder to CAM tooltips, and put driver/availability detail in Diagnostics Performance.
Do not add a new global badge.

## 2026-08-26 — Approved Auto GPU-first policy and minimal placement

- User approved Auto GPU-first with visible CPU fallback.
- Existing Camera Decoder Left/Right is the primary display; CAM tooltips are the compact secondary
  projection; Diagnostics Performance owns driver/GPU/NVDEC detail.
- No new global badge, worker, concurrency, rate, quality, affinity, clock, or power policy.
- Workspace documentation target is `./README.md`.

## 2026-08-26 — Software implementation complete; real dual RTSP remains open

- Auto is persisted and doctor resolves it to `nvh264dec` on the target RTX 4050.
- Existing Camera/CAM tooltip/Performance surfaces now project actual and fallback evidence.
- Synthetic NVDEC and full software regression pass. Real Left/Right WIRIS Preview was not started
  because a separately owned StereoWIRIS process is running; do not operate both apps concurrently.
