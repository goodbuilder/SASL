# Verification

## Requirement-to-evidence matrix

| Requirement | Evidence | Status |
| --- | --- | --- |
| Live uses app receive, not NTP | sentinel-clock pure test | PASS |
| signed delta and absolute gate | hand-calculated ns fixtures | PASS |
| missing/invalid receive fails closed | negative tests | PASS |
| metadata is auditable/non-precision | real worker JSON inspection test | PASS |
| Auto uses same rule | controller integration test | PASS |
| FTP/exposure/legacy preserved | focused regression tests | PASS |
| UI wording/layout stable | static/offscreen Qt checks | PASS |
| no wider regression | full pytest/application smoke | PASS |

## Focused test evidence

Baseline before source edits: `9 passed, 50 deselected` using the command recorded in
`current-behavior.md`.

Post-change focused/expanded commands:

```bash
/home/moontaehyung/바탕화면/SASL-20260821/.venv/bin/python -m pytest -q tests/test_workstation_calibration.py \
  tests/test_calibration_input_snapshot.py tests/test_calibration_quality_integration.py \
  tests/test_native_calibration_frame.py tests/test_wiris_ftp_capture_source.py \
  tests/test_calibration_photo_time_ui.py
# 121 passed

/home/moontaehyung/바탕화면/SASL-20260821/.venv/bin/python -m pytest -q tests/test_workstation_calibration.py \
  tests/test_calibration_input_snapshot.py tests/test_calibration_quality_integration.py \
  tests/test_native_calibration_frame.py tests/test_wiris_ftp_capture_source.py \
  tests/test_calibration_photo_time_ui.py tests/test_native_frame_ring.py
# 138 passed
```

Evidence type: automated-fake plus real Qt worker/file persistence; no camera hardware claim.

## Regression and application evidence

```bash
/home/moontaehyung/바탕화면/SASL-20260821/.venv/bin/python -m pytest -q
```

Final clean run after the user closed independent Workstation PID 92393:
`1630 passed, 1 skipped in 140.66s`.

The previously conflicting boundary was first verified independently:

```bash
/home/moontaehyung/바탕화면/SASL-20260821/.venv/bin/python -m pytest -q \
  tests/test_native_frame_ring.py::test_startup_reclaims_only_dead_workstation_native_frame_segments
# 1 passed
```

An earlier full attempt observed three failures because approved timing files and concurrent H.264
evidence changes landed while pytest was running; all three passed immediately in isolated rerun.
A later run had one live-shm conflict because an independent Workstation process was active. After
the shared worktree stabilized and that process closed, the expanded 138-test set and clean full
suite both passed.

## Performance, load, and target-hardware evidence

No resource topology change. `./run.sh --check` PASS: canonical venv, PyQt/GStreamer, required
elements, NVIDIA driver and `auto -> nvh264dec`. Real dual-WIRIS field capture remains NOT_RUN until
hardware/manual execution.

## Feature isolation and contract evidence

PASS: FTP dispatch, verified exposure, legacy RTCP metadata, Photo set input snapshot and Measurement-
independent contracts remain covered. `git diff --check` and scoped diff audit PASS.

## Visual authenticity and dynamic-layout stability evidence

PASS: existing Calibration workspace offscreen Qt test preserved preview splitter geometry and
validated the app-receive/non-exposure tooltip. No new component/style/control.

## Visual and hardware/manual evidence

Hardware/manual dual-WIRIS field capture: NOT_RUN and explicitly separate from software acceptance.
No screenshot required because no geometry changed; offscreen sequence PASS.

## Residual conditions

Decoded-frame receipt cannot prove camera exposure simultaneity; this limitation remains even after
software PASS. No unresolved software-verification blocker remains.
