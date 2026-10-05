# Backend Contract

## Commands and validation

Auto candidacy requires a current decoded frame whose parser-normalized AU contains NAL type 1,
excludes NAL5, has `DELTA_UNIT=false`, and whose classification PTS exactly equals decoded PTS. A
complete current-camera FrameKey must survive into checkerboard metadata. Missing/mismatched evidence
waits and never falls back.

## State, ownership, and concurrency

The parser probe owns classification; the RTP/timing tracker owns a bounded exact-PTS cache; the
native ring owns rasters/FrameKeys; CameraPane owns exact pin/read and calibration provenance checks;
the calibration controller owns Auto lifecycle/gates. Existing one-capture-worker exclusion remains.

## Worker resource budgets, scheduling, and backpressure

Map only non-delta AUs plus already-required recovery AUs. Bound classification at the existing RTP
cache scale and consume exact matches. A positive candidate bypasses the display-rate coalescer and
is protected in the existing bounded child/GUI preview slots until one GUI drain; a following
ordinary frame is dropped/coalesced instead. Never block a pad, decoder, GUI, YOLO, or recording for
Auto. Ring eviction is visible candidate loss.

## External boundaries and workers

The boundary is Annex-B `alignment=au` after `h264parse`, before decode. Exact decoded PTS carries the
classification into native descriptor/preview metadata. Checkerboard/capture workers consume copies
and never access GStreamer.

## Results, errors, timeout, retry, and cancellation

- Positive exact chain: continue to existing freshness/pose/timing/worker gates.
- Delta/NAL5/other/missing: wait.
- Missing/evicted/stale/wrong-camera/old-generation key: wait, no neighbor.
- Stop and late completion remain unchanged; next candidate is the only retry.

## Persistence, restart, and shutdown

Capture JSON records AU kind, NAL types, delta flag, AU/decoded PTS, and exact side keys. Restart and
generation changes clear bounded caches. Only the existing successful worker path persists files.
