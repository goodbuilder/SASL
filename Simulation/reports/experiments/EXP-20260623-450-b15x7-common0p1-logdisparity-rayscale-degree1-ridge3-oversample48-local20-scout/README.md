# EXP-20260623-450 b15x7 common0.1 log_disparity ray_scale degree1 ridge3 scout

## Purpose

Increase the aggregate-stable log-disparity ray-scale branch common fraction
from `0.05` to `0.1` to test whether stricter common sampling reduces
intrinsic-scale failures without returning to the high-regression interaction
feature.

## Method

- Method name: `b15x7_common0p1_logdisparity_rayscale_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: log-scaled fused measured disparity.
- Calibration-only inputs: train split truth residuals used to fit the pooled
  degree-1 ridge model.
- Output contract: apply the fitted residual as a stereo-center OpenCV optical
  RDF ray scale before exporting public `FRD+range(forward,right,down,range)`.
- Pass/fail criteria: all 120 local scout rows over seeds `298180..298199` must
  pass the existing runtime deployment gate.

## Result

`FAIL`, with `100/120` passing rows.

- Failure distribution: all 20 failures are `intrinsic_scale_stress`.
- Target max absolute error: observation `4.348758 m`, residual `4.317681 m`.
- Range RMSE mean: observation `0.916652 m`, residual `0.924274 m`.
- Position RMSE mean: observation `2.488401 m`, residual `2.500485 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.679468 m` to `3.703490 m`.
- Worst residual row: `disparity_noise_stress` seed `298188`,
  `4.317681 m`.

## Interpretation

Raising the common fraction to `0.1` does not recover intrinsic-scale failures
and is slightly worse than the common0.05 log-disparity ray-scale branch on
aggregate target max, range RMSE, and position RMSE. This is not a standalone
deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled fused measured disparity scalar, not an `x/y/z`
  alias.
- Correction axis: stereo-center RDF ray scale over right/down/forward.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
