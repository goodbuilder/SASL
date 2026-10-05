# Measurement comparison refresh with nominal K,D

Scope: repair the existing analysis preview when current operating intrinsics
are nominal; preserve the measurement workers, result persistence and layout.

The preview previously returned no rows without `active_intrinsics`, even though
Measurement supports nominal operating K,D. With an old calibration loaded it
used that calibration instead of the current nominal profile. Both cases were
reproduced by new tests before the controller change (two failures).

The preview now reads the geometry-approved operating profile, converts nominal
K,D through the existing conversion, and labels its rows `NOMINAL K,D
(uncalibrated)`. An unavailable operating profile cannot reuse a loaded
calibration. Existing candidate K,D identity checks remain in force.

## Sequential verification

- Contract PASS: K,D is in native optical pixel units. Relative poses remain
  left-to-right OpenCV RDF; triangulation converts to optical-baseline-center
  FRD in metres. No coordinate formula was changed.
- Calculation PASS: parallel cameras and a 1 m baseline use the independent
  oracle depth = focal / disparity. The test accounts for the nominal principal
  point and the half-baseline shift in Right.
- Running handoff PASS: two detection positions through `_refresh_distance`
  update comparison rows while still submitting both pairs to method evaluation.
- Boundary PASS: unavailable operating profile returns no rows rather than
  reusing loaded K,D; mismatched candidate identity remains blocked.
- Regression PASS: 119 tests passed with
  `../../.venv/bin/python -m pytest tests/test_candidate_comparison_isolation.py tests/test_method_evaluation.py tests/test_measurement_geometry_policy.py tests/test_workstation_ui.py -q`.
  After extending the regression through the RUNNING handoff, the comparison
  test file was rerun separately.
- Hardware UNVERIFIED: no live camera run or running application restart.

README addition prepared, pending confirmation of the target README path:

> The Extrinsics comparison preview uses current geometry-approved operating
> K,D, including nominal K,D without a loaded calibration. Nominal rows show
> `NOMINAL K,D (uncalibrated)`. Comparison rows remain analysis-only previews.
