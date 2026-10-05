# Integration Flow

## Action-to-postcondition traces

```text
Start Auto
  -> unchanged timer/setup
  -> h264parse non-delta AU: NAL types + exact PTS
  -> decoded appsink buffer with same PTS
  -> native ring publishes FrameKey
  -> preview/checkerboard metadata preserves AU evidence + FrameKey
  -> Auto validates positive NAL1 evidence
  -> CameraPane pins/reads exactly that FrameKey
  -> existing freshness, pose, pair timing, worker validation
  -> PNG + capture JSON + existing status/count
```

## Transition tables

| State | Input | Result |
|---|---|---|
| waiting-NAL1 | either positive candidate missing | wait; no save |
| waiting-exact | key missing/evicted/wrong generation | wait; no substitution |
| evaluating | exact two-sided frames | existing pose/pair timing gates |
| capturing | worker slot accepted | existing busy lifecycle |
| saved-stereo | both sides + timing eligible | count increments |
| saved-intrinsics | successful side(s), pair ineligible | retained; count unchanged |
| stopped | late completion | file may persist; stopped count unchanged |

## Failure and recovery flows

Malformed AU evidence, PTS miss, ring overwrite, restart, stale frame, duplicate pose, or worker
error fails closed for that candidate and recovers on a later positive candidate. No timestamp-near,
previous-generation, preview-pixel, or generic latest-frame fallback.

## Cross-feature contracts and dependency direction

`Stream worker -> additive frame metadata -> CameraPane exact-frame API -> Calibration`. No reverse
dependency and no effect on consumers that ignore the additive metadata.
