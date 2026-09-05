# Current Behavior

## Version and worktree context

- Baseline commit: `3288aee5f5fb7af5ab30024f93e838f76724a0b0`.
- The worktree contains extensive pre-existing user changes, including the affected
  measurement controller and tests. They must be preserved and inspected before edits.

## Reproduction

1. Receive valid P1/P2 attitude and GPS.
2. Start a later Measurement after P1 becomes unavailable.
3. Observe that the live method is not ready although the rig is physically static.

## Observed frontend behavior

The ready-method selector displays `준비된 측정 방식 없음` when every method has a
blocking readiness reason. There is currently no static/retained-input control or
visible retained-age state.

## Observed backend and persistence behavior

- `_measurement_live_readiness()` treats attitude older than 2 seconds as advisory,
  but attitude timestamped before `_measurement_started_monotonic_s` is a hard block.
- A missing/non-finite P1 or P2 GPS position is a hard block; the live NED baseline
  must be finite and positive.
- Runtime evidence from Measurement pair 199 recorded `left Pixhawk attitude missing`
  and `live GNSS baseline missing`; the current GNSS record reports
  `geometry_reason: GNSS position waiting: missing P1`.
- Measurement input evidence already records attitude/GPS payloads, timestamps,
  method instance identity, reasons, warnings, and geometry hashes.

## Documentation, code, test, and runtime conflicts

Product documentation requires fresh left/right attitude and a usable P1-to-P2 GNSS
baseline. The requested retained-static behavior intentionally changes that lasting
contract and therefore requires explicit approval and a product-document amendment.
