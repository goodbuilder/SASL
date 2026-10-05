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
| decoder preflight preserves Preview | 2026-09-02 explicit-NVIDIA rejection with live Preview | PASS automated-fake; corrected hardware rerun NOT_RUN |

## Focused test evidence

Baseline before source edits: 54 passed using the five lifecycle/storage/UI files recorded below.

- Run/camera focused: 20 passed.
- Recording/storage/UI focused: 95 passed, 1 skipped.
- Source-controller groups: 175 passed.
- Expanded changed-boundary group: 283 passed.
- Decoder-preflight correction: 2 passed.
- Camera/Measurement/recording dependent group after correction: 133 passed, 1 skipped.

## Regression and application evidence

- Commit-index full suite: `1482 passed, 1 skipped` in 169.48 s.
- The broader dirty working tree, before final commit-only isolation, also passed
  `1544 passed, 2 skipped` in 156.52 s.
- Generated delegate check, Python compilation and `git diff --check -- .`: PASS.
- `./run.sh --check`: PASS, including GStreamer, NVDEC, FFmpeg and canonical venv.
- Corrected dirty-worktree full suite: `1889 passed, 2 skipped` in 139.98 s.
- Focused Python compilation and affected-file `git diff --check`: PASS.
- Current-host `./run.sh --check`: PASS with expected warnings that `nvh264dec` and the NVIDIA
  driver are unavailable; Auto selected `avdec_h264`. This reproduces the capability boundary but
  is not a live-WIRIS Preview test.
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

## 2026-09-02 field evidence and corrective matrix

The field run reached Measurement admission, then rejected the requested explicit NVIDIA decoder
because `nvh264dec` was unavailable. Both live Preview panes were subsequently stopped by the
generic camera-start rollback. This is a required-slice FAIL, not a tolerated hardware condition.

Corrective verification must independently prove:

1. a no-mutation decoder preflight rejection keeps existing left/right pane ownership and public
   Preview state unchanged;
2. the temporary automatic recording owner is released and both Measurement camera/recording
   sources become unavailable without closing the observation interval;
3. failures after replacement has actually begun retain the existing teardown behavior;
4. Auto/Software selection can still start a later Measurement, while explicit NVIDIA remains
   fail-closed when its capability is absent.

Corrective automated results:

- Items 1 and 2: PASS. The regression retains both pane worker identities and public Preview state,
  observes no camera-stop call, releases temporary automatic recording, and publishes Measurement
  camera unavailability.
- Item 3: PASS in the 133-test dependent group; existing first-frame timeout, partial-start,
  replacement retry/timeout, recording stop and teardown cases remain green.
- Item 4: PASS for policy/runtime checks. Explicit rejection remains fail-closed, and the current
  host execution check selects `avdec_h264` only under Auto. A corrected live-WIRIS retry remains
  NOT_RUN hardware-manual.

Commands:

```text
QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q -p no:cacheprovider tests/test_workstation_camera.py::test_measurement_decoder_preflight_rejection_preserves_live_preview tests/test_workstation_camera.py::test_camera_stream_preflight_rejection_does_not_cancel_current_owner
QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q -p no:cacheprovider tests/test_workstation_camera.py tests/test_camera_stream_recovery.py tests/test_measurement_run_controller.py tests/test_recording_controller_save_lifecycle.py
QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q -p no:cacheprovider
../../.venv/bin/python -m py_compile camera_stream_controller.py tests/test_workstation_camera.py
./run.sh --check
```

Baseline command:
`QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q tests/test_measurement_run_controller.py tests/test_camera_start_readiness.py tests/test_recording_controller_save_lifecycle.py tests/test_measurement_data_store.py tests/test_workstation_contracts.py`
