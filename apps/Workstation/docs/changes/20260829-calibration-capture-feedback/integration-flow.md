# Integration Flow

## Action-to-postcondition traces

Preview frame -> existing checkerboard worker -> newest found/corners/status -> overlay signal and
snapshot -> primary preview/board row.

Capture intent -> existing validation gates -> capture worker -> PNG/metadata result -> controller
run counters + cached inventory -> facade/evidence -> run/set/last rows.

## Transition tables

Waiting does not count. Worker start increments request/inflight. Stereo completion increments
stereo and decrements inflight. Partial completion increments K,D-only. Empty/failed completion
increments failure. Root change resets run state and reloads set inventory.

## Failure and recovery flows

Stale frame -> STALE until fresh result. Scan error -> UNAVAILABLE/`--` while last detail remains.
Worker failure -> failure counter and explicit last status; next existing capture retries.

## Cross-feature contracts and dependency direction

Camera/worker producers -> Calibration owner -> public read model -> Workspace. No reverse widget or
cross-feature private dependency is introduced.
