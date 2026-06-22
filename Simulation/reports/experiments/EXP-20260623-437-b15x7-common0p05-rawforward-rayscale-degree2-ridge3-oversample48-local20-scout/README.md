# EXP-20260623-437 b15x7 common0.05 raw_forward ray_scale degree2 ridge3 scout

## Purpose

Test whether raw stereo-center RDF forward is a better ray-scale feature than
log disparity in the common0.05 degree-2 branch.

## Method

- Method name: `b15x7_common0p05_rawforward_rayscale_degree2_ridge3_oversample48_local20_scout`
- Runtime inputs: corrected stereo-center RDF raw forward after the base runtime
  profile.
- Calibration-only inputs: train split truth residuals used to fit the pooled
  degree-2 ridge model.
- Output contract: apply the fitted residual as a stereo-center OpenCV optical
  RDF ray scale before exporting public `FRD+range(forward,right,down,range)`.
- Pass/fail criteria: all 120 local scout rows over seeds `298180..298199` must
  pass the existing runtime deployment gate.

## Result

`FAIL`, with `100/120` passing rows. All 20 failures were
`intrinsic_scale_stress`.

- Target max absolute error: observation `4.131533 m`, residual `4.037141 m`.
- Range RMSE mean: observation `0.912856 m`, residual `0.914628 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.502513 m`.
- Tracked intrinsic seed `298188`: target max absolute error regressed from
  `3.623977 m` to `3.864106 m`, while range RMSE improved from `2.409173 m` to
  `2.364045 m`.

## Interpretation

Raw-forward ray-scale correction is weaker than log-disparity ray-scale in this
branch: it reduces worst target error less and regresses aggregate
range/position RMSE more. Do not promote as a standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: stereo-center RDF raw forward scalar, not a public FRD field.
- Correction axis: stereo-center RDF ray scale over right/down/forward.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
