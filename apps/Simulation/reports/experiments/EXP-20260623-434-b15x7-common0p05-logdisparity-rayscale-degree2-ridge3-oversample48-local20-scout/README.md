# EXP-20260623-434 b15x7 common0.05 log_disparity ray_scale degree2 ridge3 scout

## Purpose

Switch the common0.05 log-disparity degree-2 residual branch from forward
additive correction to ray-scale correction to test whether scaling the full
stereo-center RDF ray changes intrinsic-scale tail behavior.

## Method

- Method name: `b15x7_common0p05_logdisparity_rayscale_degree2_ridge3_oversample48_local20_scout`
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

- Target max absolute error: observation `4.131533 m`, residual `3.993196 m`.
- Range RMSE mean: observation `0.912856 m`, residual `0.914251 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.500469 m`.
- Tracked intrinsic seed `298188`: target max absolute error regressed from
  `3.623977 m` to `3.814988 m`, while range RMSE improved from `2.409173 m` to
  `2.358796 m`.

## Interpretation

Ray-scale correction strongly reduces the worst target max error but regresses
aggregate range/position RMSE and worsens the tracked intrinsic target tail.
This is not a standalone deployment candidate, but it is a useful tradeoff
anchor for later blending or gate-conditioned methods.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled fused measured disparity scalar, not an `x/y/z`
  alias.
- Correction axis: stereo-center RDF ray scale over right/down/forward.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
