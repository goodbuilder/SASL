# Current Behavior

## Version and worktree context

- Baseline commit: `5bec92529e0954a96522d1de7c5e1e5f2ece38ca`.
- Relevant pre-existing modifications are present in `docs/ui/03_WORKSPACE_LAYOUT.md`,
  `docs/ui/07_COMMAND_MODEL.md`, `workflow.md`, `workspaces.py`,
  `measurement_package_controller.py`, and `tests/test_measurement_session_controller.py`; they are
  user-owned and must be preserved.
- This packet is documentation-only and `implementation_allowed` remains false.

## Reproduction

Current documented flow:

```text
Create Session Folder
  -> optional Pre evidence creates one draft child
  -> Measurement start creates/reuses mNNNNNN and starts Detection/recording
  -> Measurement stop finalizes and seals that child
  -> repeat child or Close Session
```

Runtime example:

- Session `20260823_194605_395747300` remains `OPEN` on 2026-08-25.
- It records 122 application joins and active child `m000001`.
- `m000001` is `RECOVERABLE`, with five required artifacts still missing.
- Restart recovery adopts that child and refuses a new Measurement until the old child is sealed.

## Observed frontend behavior

- `Sessions > Current` presents `Create Session Folder` and `Close Session` as a top-level
  lifecycle.
- Session is described as a timestamped folder/current work run and as the daily-work owner in
  product documents.
- Measurement start requires an open parent. An old `RECOVERABLE` child is reflected as `REC RETRY`
  and makes retry/close the next lifecycle action.
- Navigation and review identify saved data by `(session_id, measurement_id)`.

## Observed backend and persistence behavior

- `MeasurementSession` owns one `OPEN/CLOSING/SEALED` parent and an ordered child index.
- Exactly one child may be mutable. `begin_measurement()` fails while
  `active_measurement_id` points to `COLLECTING`, `POST_CAPTURE`, `VERIFYING`, or `RECOVERABLE`.
- The parent already supports long duration and restart through `app_runs`; it does not own a live
  device worker itself.
- MeasurementPackage, not Session, is the atomic writer, retry, publication, and seal boundary.
- Session additionally owns shared Calibration/Alignment evidence, ordered child aggregation,
  `review.xlsx`, and `measurement_session_manifest.json`.
- Automatic startup recovery only proceeds when exactly one open Session is discoverable.

## Documentation, code, test, and runtime conflicts

- Product documents call Session a daily/current-run lifecycle, but runtime has kept one parent
  open across two dates and 122 app runs. The persisted model is closer to an experiment/campaign
  than a runtime session.
- Long intervals are technically supported by the parent, but the single mutable child makes an
  interrupted old Measurement a global gate on future Measurements.
- Removing Session outright would orphan existing shared Calibration/Alignment ownership and
  composite review identity unless those responsibilities move to another explicit grouping.
- After accepting multiple sequential Measurements in one app execution, the proposed normal
  capture flow converges with current behavior: current code already seals `m000001`, keeps device
  connections alive, and permits `m000002` under the same parent. The remaining operator-visible
  differences are manual Session create/close and the way an old `RECOVERABLE` child blocks future
  work.
