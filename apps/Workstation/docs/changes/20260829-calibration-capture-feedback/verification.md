# Verification

## Requirement-to-evidence matrix

PASS — inventory truth, run lifecycle, facade compatibility, primary overlay/status, stable layout,
preserved commands/selection and OpenCV pixel/RDF coordinate contract are covered below.

## Focused test evidence

- PASS — automated-real filesystem inventory and existing session-count compatibility:
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q -o cache_dir=/tmp/workstation-pytest-cache tests/test_calibration_capture_feedback.py tests/test_calibration_input_snapshot.py -k 'capture_inventory or calibration_session_pair_count'`
  -> `5 passed, 7 deselected`.
- PASS — capture snapshot, controller lifecycle, public facade/evidence, primary UI, Calibration,
  FTP, delegate and contract regression:
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 XDG_CONFIG_HOME=/tmp/workstation-test-xdg /usr/bin/python3 -m pytest -q -o cache_dir=/tmp/workstation-pytest-cache tests/test_calibration_capture_feedback.py tests/test_calibration_quality_ui.py tests/test_workstation_calibration.py tests/test_calibration_quality_integration.py tests/test_calibration_input_snapshot.py tests/test_panel_delegate_bindings.py tests/test_workstation_contracts.py tests/test_wiris_ftp_capture_source.py`
  -> `153 passed in 7.06s`.
- PASS — K,D command/snapshot/solver regression:
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 XDG_CONFIG_HOME=/tmp/workstation-kd-xdg /usr/bin/python3 -m pytest -q -o cache_dir=/tmp/workstation-kd-pytest-cache tests/test_calibration_solver_quality.py tests/test_calibration_input_snapshot.py tests/test_calibration_quality_integration.py tests/test_workstation_calibration.py -k 'solver or intrinsic or calibration_and_validation_run_off_ui_thread or compute_requires_explicit_photo_session_selection or explicit_calibration_file_selection'`
  -> `19 passed, 63 deselected`.
- FAIL as configured — the exact persisted AutoCapture input snapshot resolves 37 Left and 32 Right
  K,D images, but Left contains 34x 1280x720 plus 3x 1920x1080 and Right contains 30x 1280x720 plus
  2x 1920x1080. The current 13x8 pattern also does not match the photographed 10x8 inner corners.
  The real solver therefore fail-closes on raster mismatch (and detects zero with 13x8).
- PASS execution / FAIL quality — with the same snapshot restricted to homogeneous 1280x720 and
  the photographed 10x8 pattern, the real `CheckerboardCalibrationWorker` completed with 72 progress
  events. Both sides retained 10 representative poses, emitted finite 3x3 K and five D values, and
  reported Left 0.212317 px / Right 0.222823 px RMS. Combined quality remained `FAIL` for insufficient
  coverage/perspective/effective poses and principal-point/subset instability; the result is not
  applicable.

## Regression and application evidence

- PASS — full Workstation suite from `apps/Workstation` using system PyQt5 because the local
  `apps/Workstation/.venv` lacks PyQt5:
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 XDG_CONFIG_HOME=/tmp/workstation-full-xdg /usr/bin/python3 -m pytest -q -o cache_dir=/tmp/workstation-full-pytest-cache`
  -> `1643 passed, 2 skipped, 1 pre-existing SciPy/NumPy warning in 132.06s`.
- PASS — final full suite after README approval, K,D field-data verification, final delegate stub,
  and all documentation changes:
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 XDG_CONFIG_HOME=/tmp/workstation-final-xdg /usr/bin/python3 -m pytest -q -o cache_dir=/tmp/workstation-final-pytest-cache`
  -> `1645 passed, 2 skipped, 1 pre-existing SciPy/NumPy warning in 131.98s`.
- PASS — canonical launcher dependency check with a temporary owned model copy:
  `SASL_WORKSTATION_DATA_ROOT=/tmp/workstation-smoke-data ./run.sh --check`
  -> runtime check OK; optional NVDEC/NVIDIA unavailable, Auto selected `avdec_h264`.
- PASS — canonical root venv, real `WorkstationMainWindow`, no-device offscreen start, Calibration
  navigation and bounded close -> `WORKSTATION_GUI_SMOKE_EXIT 0 visible False`.

## Performance, load, and target-hardware evidence

No worker allocation change. Existing overlay is projected into a second visible widget at the
existing 2 Hz cadence; snapshot polling is directory-stamp cached. Automated GUI remained responsive.
GPU/NVIDIA was unavailable in the smoke environment and real WIRIS remains hardware-manual.

## Feature isolation and contract evidence

PASS — no new worker/queue/setting/persisted schema; public additive facade is covered by
`missing_backend_facades`, delegate tests and legacy lightweight backend fallbacks.

## Visual authenticity and dynamic-layout stability evidence

PASS (offscreen) — `/tmp/workstation-calibration-feedback-full-1366x768-v2.png` shows the existing
compact Qt visual language. With Navigation, Inspector and Diagnostics open, board states, commands,
run/set counters and last status remain above the preview and inside the unscrolled viewport.
`/tmp/workstation-calibration-feedback-populated-1366x768.png` verifies representative FOUND 80 /
NOT_FOUND states, 14/11/2/1 run counts, 13/20 set counts and duplicate-pose recovery text.

## Visual and hardware/manual evidence

Real WIRIS live checkerboard display remains unverified until hardware/manual execution, as allowed
by the approved plan. Existing real saved WIRIS images were processed by the canonical K,D worker;
execution passed while applicability correctly remained quality `FAIL`.

## Residual conditions

- Repository-wide `tools/check_portable_paths.py` remains FAIL on 11 pre-existing violations in
  the active `calibration-app-receive-timing` packet and root-level `yolo-tolerant-input-trial`
  packet. A scoped scan over every file changed by this packet found no concrete POSIX user-home
  or dated-checkout path. Those unrelated packets were not rewritten.
