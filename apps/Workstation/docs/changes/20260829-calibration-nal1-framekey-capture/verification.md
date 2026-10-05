# Verification

## Requirement-to-evidence matrix

| Requirement | Evidence | Status |
|---|---|---|
| NAL1 non-delta only | hand-built Annex-B + Gst flags | PASS (automated-fake) |
| exact AU PTS -> decoded metadata | fake parser/decoded boundary | PASS (automated-fake) |
| exact FrameKey, no 1 ms lookup | CameraPane/controller/ring tests | PASS (automated-real shared memory + fake controller) |
| existing freshness/pair/pose/save preserved | calibration regressions | PASS (automated) |
| candidate survives bounded preview coalescing | child queue + CameraPane pending-slot tests | PASS (automated) |
| unrelated consumers remain usable | camera/stream/full regressions | PASS (automated) |
| real dual-camera cadence | hardware/manual | BLOCKED until hardware run |

## Focused test evidence

Focused affected regression:

```bash
QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q \
  tests/test_camera_stream_recovery.py tests/test_workstation_yolo_stream.py \
  tests/test_camera_pipeline_backpressure.py tests/test_camera_preview_backpressure.py \
  tests/test_native_frame_ring.py tests/test_native_calibration_frame.py \
  tests/test_camera_preview_ui_load.py tests/test_calibration_quality_integration.py \
  tests/test_workstation_calibration.py
```

Result: **165 passed in 8.29 s**. This includes positive/negative NAL fixtures, bounded exact-PTS
handoff, one-shot cache consumption, exact shared-memory FrameKey lookup, eviction/no-neighbor,
wrong side/generation, NAL1-only preview admission, candidate priority through both bounded preview
coalescers, pose/timing/count preservation, persistence, and UI tooltip/status checks.

## Regression and application evidence

Full project suite from `apps/Workstation`:

```bash
QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q
```

Result: **1631 passed, 1 skipped in 138.80 s**.

Application/runtime boundary:

```bash
./run.sh --check
```

Result: **PASS** for the canonical interpreter, distributions, GStreamer introspection/elements,
NVDEC selection, NVIDIA driver, model path, and Workstation runtime contract.

Static evidence: changed Python modules compile successfully and `git diff --check --
apps/Workstation` reports no whitespace errors.

## Performance, load, and target-hardware evidence

No worker/process/GPU allocation was added. Only non-delta AUs are mapped; the PTS cache is bounded
at 240 entries and exact matches are removed. Existing bounded preview channels protect one positive
candidate until one GUI drain and continue to coalesce/drop ordinary frames. The existing two-second
GUI-stall backpressure/stop test passes. Runtime doctor proves the configured GStreamer/NVDEC
boundary is available; sustained real-camera load remains hardware-manual.

## Feature isolation and contract evidence

The full suite confirms additive metadata compatibility and preserved Preview/YOLO/Measurement/
recording/manual/FTP/calibration behavior. Calibration imports no GStreamer object and consumes only
the existing frame-metadata and CameraPane contracts.

## Visual authenticity and dynamic-layout stability evidence

No widget/layout was added. Existing wrapped status/tooltips were updated and offscreen Calibration
workspace tests pass, including the new exact-NAL1 tooltip and waiting messages.

## Visual and hardware/manual evidence

Real dual-camera NAL1 cadence, pair yield, and sustained responsiveness remain unverified. No field
success is claimed from software fakes.

## Residual conditions

Hardware cadence cannot be claimed from fakes. The user-approved field limitation already treats
this as residual manual evidence. Coordinate review found no transform change: native per-side
OpenCV pixels remain `u` right/`v` down; freshness and pair timing retain separate clocks.

The repository portability checker currently reports 11 violations exclusively in the separate
`20260829-calibration-app-receive-timing` packet and a pre-existing root trial packet. This packet,
its README changes, and affected product docs contain no dated-checkout or user-home absolute path;
unrelated documents were not modified.

Implementation identity: baseline commit `7242452` plus the uncommitted working-tree hunks recorded
by change packet `20260829-calibration-nal1-framekey-capture`.
