# EXP-20260623-463 b7x15 common0.05 raw_forward forward_add degree1 ridge3 scout

## Purpose

Change the burst aggregation shape from `b15x7` to `b7x15` while keeping the
same total observation count (`105`) and raw-forward forward-add correction.
This tests the opposite aggregation direction from `b21x5`: fewer bursts with
more observations per burst.

## Method

- Method name: `b7x15_common0p05_rawforward_forwardadd_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: corrected stereo-center OpenCV optical RDF raw forward scalar.
- Calibration-only inputs: train split truth residuals used to fit the pooled
  degree-1 ridge model.
- Output contract: apply the fitted residual as a stereo-center OpenCV optical
  RDF forward additive correction before exporting public
  `FRD+range(forward,right,down,range)`.
- Pass/fail criteria: all 120 local scout rows over seeds `298180..298199` must
  pass the existing runtime deployment gate.

## Result

`FAIL`, with `100/120` passing rows.

- Failure distribution: all 20 failures are `intrinsic_scale_stress`.
- Target max absolute error: observation `4.462819 m`, residual `4.442819 m`.
- Range RMSE mean: observation `0.921180 m`, residual `0.914699 m`.
- Position RMSE mean: observation `2.491063 m`, residual `2.488488 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.604826 m` to `3.624826 m`.
- Worst residual row: `disparity_noise_stress` seed `298183`,
  `4.442819 m`.

## Interpretation

`b7x15` is worse than both `b15x7` and `b21x5` on aggregate target max and RMSE
while still failing every intrinsic-scale row. Fewer, larger bursts are not a
useful standalone direction for this correction family.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: corrected stereo-center RDF raw forward, not public FRD
  `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF forward additive correction.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
