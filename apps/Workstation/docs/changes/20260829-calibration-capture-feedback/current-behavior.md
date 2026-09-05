# Current Behavior

## Version and worktree context

- Baseline commit: `7242452`
- Worktree contains broad pre-existing user changes and two active Calibration packets:
  app-receive timing and NAL1/FrameKey Auto capture.
- Baseline screenshot: `/tmp/workstation-calibration-baseline-1000x700.png`.

## Reproduction

Open Calibration and keep the default `촬영` tab selected. The tab shows zoom controls,
four capture actions and `L: WAITING · R: WAITING · Pair: --`, but no camera image.

## Observed frontend behavior

- Checkerboard overlay exists only in collapsed `고급 캘리브레이션 > Live`.
- `simple_calibration_capture_status` mirrors only `capture_summary_label`, which is the latest
  per-side event and has no cumulative count.
- Session pair count and candidate file rows are visible only after opening Advanced and choosing
  a Photo set.

## Observed backend and persistence behavior

- `CheckerboardPreviewWorker` already produces per-side found/corners/status at 2 Hz.
- `CheckerboardCaptureWorker` persists checkerboard-valid PNGs and capture metadata, including
  stereo eligibility and K,D-only sides.
- Auto keeps `_auto_capture_saved_pairs`, but that count resets per Auto start and is not a
  complete manual/FTP/current-set inventory.

## Documentation, code, test, and runtime conflicts

Product documents describe a simple primary calibration workflow and require critical status
without navigation, while current code hides recognition and candidate evidence in Advanced.
Existing overlay/Auto focused tests pass; real WIRIS display behavior remains hardware-manual.
