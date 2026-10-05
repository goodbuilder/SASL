# Workflow Impact

## Affected operators and workflow IDs

- Calibration engineer; `CAL-CAPTURE-AUTO-NAL1` in `Calibration > Capture`.

## Current workflow

Start Auto, present new checkerboard poses, and let generic preview detections resolve native frames
by a 1 ms timestamp-near search. Existing pose, freshness, pair timing, worker validation, and
stereo-count rules then apply.

## Proposed workflow

The operator action is unchanged. Internally Auto waits for positive NAL1/non-delta frames, runs
checkerboard detection on those frames, resolves exactly the carried FrameKeys, and then applies all
existing downstream gates.

## Preserved, added, removed, and reordered steps

- Preserved: controls, interval, Stop/late-completion behavior, 2 s freshness, pose novelty, pair
  timing, K,D-only retention, paths, worker validation, and manual/FTP/OZ actions.
- Added: bounded AU classification, exact PTS handoff, NAL1 candidate state, exact key lookup, and
  persisted provenance.
- Removed from Auto only: timestamp-near native lookup and neighbor substitution.
- Reordered: compression provenance is established before checkerboard candidacy.

## Decisions, confirmations, handoffs, waits, and artifacts

Parser probe -> bounded PTS evidence -> decoded metadata -> native FrameKey -> checkerboard result ->
calibration controller -> capture worker. A missing handoff is visible waiting, never success.

## Interruption, cancellation, retry, restart, and resume

Stop and late completion preserve current behavior. Stream restart clears classification/FrameKey
state. Eviction or mismatch recovers only on the next positive candidate.

## Degraded, error, and recovery behavior

- Missing/non-GStreamer AU proof: Auto waits; manual capture remains available.
- Delta, NAL5, or other AU: ignored only by Auto.
- Stale, wrong-side, old-generation, or evicted key: wait without fallback.
- Pair outside the existing timing gate: current K,D-only behavior remains.
- Worker/checkerboard/save failure: report and wait for a later new candidate.

## Completion and next actions

Stop reports existing valid stereo count; the operator reviews the photo set and computes K,D/R,t.

## Training, operating-procedure, and compatibility impact

Auto may be slower and the status may wait for NAL1. No new control or data migration; manual capture
is the compatibility path when positive AU evidence is unavailable.

## Workflow-to-screen, command, state, and regression traceability

Unchanged Start command -> AU/PTS/FrameKey candidate state -> existing capture worker -> PNG/JSON ->
existing status/count. Regression covers positive, delta, missing, evicted, stale, generation,
duplicate, timing-only, Stop-with-inflight, and preserved manual behavior.
