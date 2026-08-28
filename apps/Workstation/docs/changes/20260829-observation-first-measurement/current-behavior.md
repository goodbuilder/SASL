# Current Behavior

## Version and worktree context

Baseline `a8f92b9`; heavily dirty user-owned worktree with active tolerant recording and field
stability changes. Focused lifecycle baseline on 2026-08-29: 54 passed.

## Reproduction

`MeasurementRunController` enters STARTING and requires at least one camera side in
`confirm_measurement_running`. Camera readiness rollback calls `fail_measurement_start`. Final
artifact enqueue failure can keep the camera stop watchdog and run in STOPPING.

## Observed frontend behavior

Measurement Start stays enabled across storage state, but camera/Pixhawk recovery controls are
disabled whenever aggregate Recording is ACTIVE/FINALIZING.

## Observed backend and persistence behavior

Sessionless JSONL is best-effort. A failed stream drops later rows until retry. Camera finalization
evidence and run release are currently coupled. Reactivation reuses the current per-side MP4 path.

## Documentation, code, test, and runtime conflicts

The sessionless contract says persistence never gates Measurement, but artifact-enqueue retry can
delay `finish_measurement_run`. Existing tests encode camera-backed RUNNING and broad recording
locks; both must change under the approved observation-first contract.
