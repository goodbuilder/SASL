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

## 2026-09-02 real-device regression

The operator reported that the cameras disconnected immediately after pressing Measurement. The
saved runtime evidence reproduces a controller rollback defect rather than a physical camera or
network disconnect:

- `recordings/events.jsonl` records `measurement_start_requested` at 09:07:56, followed by a
  `camera/decoder start request was rejected` transition for both camera and recording sources.
- `recordings/application_log.jsonl` records the exact preflight rejection: explicit NVIDIA mode
  could not use `nvh264dec` and instructed the operator to choose Auto or Software.
- The same log then records both already-running Preview panes as `중지됨` and `종료 대기 중`.
- The later `run.log` sequence shows the operator-selected Auto policy choosing `avdec_h264` and
  successfully starting both Preview workers. This proves that the reported outage was caused by
  the application's rejection rollback, not loss of WIRIS reachability.

`_start_camera_streams()` rejects an unavailable explicit decoder before it starts a replacement
worker. `start_yolo()` nevertheless routes that no-mutation preflight rejection through
`_rollback_camera_start()`, which unconditionally stops both existing panes and clears the public
camera state. That behavior conflicts with the approved source-failure-isolation contract: the
Measurement may continue with its measurement camera sources unavailable, but an existing Preview
must remain live.
