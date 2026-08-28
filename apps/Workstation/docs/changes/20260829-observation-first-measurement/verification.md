# Verification

## Requirement-to-evidence matrix

Slices: run admission; source joins; segment recording; persistence isolation; UI/commands;
integration/docs. Each must pass focused tests before dependent work.

| Slice | Evidence | Status |
|---|---|---|
| immediate run admission | 0-camera, invalid URL, recording failure, immediate sequential runs | PASS automated-fake |
| source isolation/rejoin | camera recovery and Pixhawk/GNSS/NTRIP/Target controllers | PASS automated-fake |
| segmented recording | no-overwrite allocation, presence state, late finalization | PASS automated-fake |
| persistence isolation | retry FIFO/cap, enqueue failure, RTCM rollback | PASS automated-real filesystem |
| UI/contracts | enablement, compact status, delegates, compilation | PASS offscreen |
| real devices | WIRIS/P1/P2/GNSS/NTRIP/Target recovery in one run | NOT_RUN hardware-manual |

## Focused test evidence

Baseline before source edits: 54 passed using the five lifecycle/storage/UI files recorded below.

- Run/camera focused: 20 passed.
- Recording/storage/UI focused: 95 passed, 1 skipped.
- Source-controller groups: 175 passed.
- Expanded changed-boundary group: 283 passed.

## Regression and application evidence

- Commit-index full suite: `1482 passed, 1 skipped` in 169.48 s.
- The broader dirty working tree, before final commit-only isolation, also passed
  `1544 passed, 2 skipped` in 156.52 s.
- Generated delegate check, Python compilation and `git diff --check -- .`: PASS.
- `./run.sh --check`: PASS, including GStreamer, NVDEC, FFmpeg and canonical venv.
- Portability: FAIL on one unrelated pre-existing dated-checkout string in repository-root
  `docs/changes/20260828-yolo-tolerant-input-trial/current-behavior.md`; this packet added no finding.

## Performance, load, and target-hardware evidence

Retry reserve saturation passes at the approved 256-record/8 MiB ceiling. No new media worker,
decoder, encoder or full-file validator was introduced. Target-hardware sustained evidence is NOT_RUN.

## Feature isolation and contract evidence

Source failure, artifact enqueue failure, late-generation no-op, manual-record lock and forbidden
mutation cases pass. Existing result/coordinate gates remain covered by the full suite.

## Visual authenticity and dynamic-layout stability evidence

Existing widgets only; offscreen UI contracts pass. No new page/table/control or resize path.

## Visual and hardware/manual evidence

Hardware/manual remains required; no camera/Pixhawk/GNSS field claim is made from fakes.

## Residual conditions

- Workstation `./README.md` was explicitly confirmed and updated.
- Real device join/rejoin matrix is NOT_RUN.
- Repository portability is blocked only by the unrelated pre-existing path above.

Baseline command:
`QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q tests/test_measurement_run_controller.py tests/test_camera_start_readiness.py tests/test_recording_controller_save_lifecycle.py tests/test_measurement_data_store.py tests/test_workstation_contracts.py`
