# EXP-20260623-436 b15x7 common0.05 log_disparity ray_scale degree2 ridge300 scout

## Purpose

Increase ridge regularization on the common0.05 log-disparity ray-scale branch
from `3` to `300` to see whether stronger regularization reduces aggregate
range/position regression.

## Method

- Method name: `b15x7_common0p05_logdisparity_rayscale_degree2_ridge300_oversample48_local20_scout`
- Runtime inputs: log-scaled fused measured disparity after the base runtime
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

- Target max absolute error: observation `4.131533 m`, residual `3.990097 m`.
- Range RMSE mean: observation `0.912856 m`, residual `0.914296 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.500392 m`.
- Tracked intrinsic seed `298188`: target max absolute error regressed from
  `3.623977 m` to `3.811794 m`, while range RMSE improved from `2.409173 m` to
  `2.357702 m`.

## Interpretation

Ridge `300` is not a fix for the ray-scale tradeoff. It slightly lowers the
worst target max compared with ridge `3`, but aggregate range/position RMSE and
intrinsic pass count remain poor. Do not promote as a standalone deployment
candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled fused measured disparity scalar, not an `x/y/z`
  alias.
- Correction axis: stereo-center RDF ray scale over right/down/forward.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
