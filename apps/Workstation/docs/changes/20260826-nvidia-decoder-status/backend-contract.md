# Backend Contract

## Commands and validation

`current_decoder_selection(requested)` remains the sole availability policy. Auto requires software
decoder and prefers NVIDIA only when driver and element probes both pass.

## State, ownership, and concurrency

Camera worker owns actual pipeline. Panel stores per-side active decoder and a bounded fallback
reason/state projection. UI cannot infer success from requested mode.

## Worker resource budgets, scheduling, and backpressure

No new worker/thread/queue. One decoder per side. No catch-up or duplicate decoder. Existing preview
cap, frame queues, recording finalization, and fallback restart ordering remain.

## External boundaries and workers

Boundaries: `nvidia-smi` driver probe, GStreamer `nvh264dec` factory, actual decoder start/first
frame/stall, and current active pipeline status.

## Results, errors, timeout, retry, and cancellation

Factory/start/5-second first-frame/3-second decoded-frame failures retain current rules. Auto may
fallback once; explicit NVIDIA fails. Stop/close clears active decoder evidence.

## Persistence, restart, and shutdown

Persist `rtsp_decoder=auto` only after approval. Runtime override is process-local. Shutdown stops
the active decoder and writer through existing lifecycle; no GPU state is persisted.
