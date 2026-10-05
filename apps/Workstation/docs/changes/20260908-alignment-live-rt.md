# Alignment live R,t

User-authorized scope (2026-09-08): show automatically changing Attitude+GNSS and
GNSS Yaw90 transforms, and remove References/manual registration controls.

Source of truth: `pixhawk_canonical_transform`, `camera_translation_right_in_left_camera`,
`select_baseline_yaw`, and the user's request for read-only live values. Inputs are
latest selected P1/P2 GNSS WGS84/MSL positions and body FRD→NED attitude. GPS differences
are NED metres; output is optical RDF canonical `X_right = R X_left + t`, with t in metres.
Yaw90 uses both sides' own R/P and the selected common yaw; the optical capture reference's
separate P1-R/P alignment assumption is not substituted here.

Changes: read-only `alignment_rt_controller.py` query, `StandalonePanelBackend` facade,
two fixed live rows in OpticalPosePane and visible Alignment polling; References tab and
its registration buttons removed. Manual candidate selection, Apply, Combine, camera frames,
measurement caches, settings, files and device connections are not mutated by this query.

Slice checks:

- Logic: identity/nontrivial rotation checked against independently calculated point
  coordinates; T/C distinction, selected receivers, live Reverse, stale/future timestamps,
  missing GNSS/attitude, invalid quaternion and vertical-only baseline covered.
- Boundary/UI: no command on query, two rows before any registration, changing GNSS updates t,
  existing candidate selection and Combine checks preserved. Invalid query retains last values
  as ERROR, age advances, recovery returns STALE/LIVE, missing input clears numbers.
- Layout: References removal updates method indices. Baseline Yaw90's installation surface
  keeps its first-viewport tables; live values follow that page. Other methods show the live
  readout above candidate review. Existing theme, fixed row heights and in-place cells reused.
- Regression and visual checks: PASS as detailed below.

No new runtime process, hardware operation, external dependency or persistence schema.
Backend reference-registration APIs are retained for compatibility with earlier callers;
they are not used by the live display.

Verification (Workstation directory, `PYTHONPATH=..:../../common/python ../../.venv/bin/python -m pytest`):

- `-q tests/test_alignment_live_rt.py tests/test_alignment_live_rt_ui.py tests/test_optical_pose_workspace.py tests/test_baseline_yaw90_workspace.py tests/test_optical_yaw90_reference.py`: 69 passed.
- `-q tests/test_workstation_ui.py tests/test_workstation_contracts.py tests/test_workstation_calibration.py`: 113 passed.
- Full `-q`: 2110 passed, 2 skipped, one failure in the old References tab expectation.
  That expectation was updated for the authorized tab removal; `-q tests/test_workstation_lifecycle.py`
  then passed all 34 tests, and `-q --lf` confirmed the formerly failing navigation test passes.
- Added main-window poll and standalone-facade checks: `-q tests/test_alignment_live_rt_ui.py`: 5 passed.
  The visible Alignment page queries and renders on the existing status poll; hidden Alignment does not query.
- Offscreen rendering using the actual main-window stylesheet verifies complete three-line 3×3
  matrices, six-decimal metre components, state and age. Fixed column widths, disabled auto-wrap/elision,
  and full-value tooltips prevent the initially observed matrix clipping. Baseline setup still meets
  the existing 1366×768/1920×1080 first-viewport checks.
- `git diff --check` and changed-file portable-path scan: PASS.

This is automated/synthetic verification, not live hardware accuracy validation. The user app
was not restarted or connected to devices. Source updates take effect after app restart.
Coordinate contract checked: GNSS NED / attitude body FRD→NED / canonical optical RDF R,t;
right optical center C and translation t retain distinct signs/directions. No stored measurement changed.
