# Backend Contract

## Commands and validation

`current_decoder_selection(requested)` remains the sole availability policy. Auto requires software
decoder and prefers NVIDIA only when the `nvh264dec` factory, `nvidia-smi`, and the CUDA Driver API
`cuInit(0)` all pass.

## State, ownership, and concurrency

Camera worker owns actual pipeline. Panel stores per-side active decoder, process-local override and
bounded fallback reason/state projection. UI cannot infer success from requested mode or from
`set_state(PLAYING)`: only a decoded frame with the current pane generation activates a decoder.

## Worker resource budgets, scheduling, and backpressure

No new worker/thread/queue. One decoder per side. No catch-up or duplicate decoder. Existing preview
cap, frame queues, recording finalization, and fallback restart ordering remain.

## External boundaries and workers

Boundaries: `nvidia-smi` management probe, CUDA Driver API `cuInit(0)`, GStreamer `nvh264dec`
factory, actual decoder start/first-frame/stall, and current active pipeline status.

## Results, errors, timeout, retry, and cancellation

RTSP/source/parser, MP4 mux/filesink, decoder-health and generic-pipeline failures are distinct.
Only direct NVDEC health, or the 30-second decoded-output timeout while RTP input remains current,
may authorize Auto fallback. Preview-only fallback restarts the failed side inside its worker;
recording fallback finalizes/stops that side and records an override for the next side start. The
healthy side is never restarted. Explicit NVIDIA fails closed. Candidate, crash, terminal exit,
stop and close clear only the affected side's active evidence.

## Persistence, restart, and shutdown

Persist `rtsp_decoder=auto` only after approval. Runtime override is process-local. Shutdown stops
the active decoder and writer through existing lifecycle; no GPU state is persisted.
