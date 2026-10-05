# EXP-20260623-424 b15x7 common0.1 log_disparity forward_add ridge30 scout

## Purpose

Tune the promising log-disparity additive branch by lowering ridge regularization
from `300` to `30`.

## Method

- Method name: `b15x7_common0p1_logdisparity_forwardadd_ridge30_oversample48_local20_scout`
- Runtime inputs: log-scaled fused measured disparity after the base runtime
  profile.
- Calibration-only inputs: train split truth residuals used to fit the pooled
  degree-1 ridge model.
- Output contract: add the fitted residual to stereo-center OpenCV optical RDF
  `forward` before exporting public `FRD+range(forward,right,down,range)`.
- Pass/fail criteria: all 120 local scout rows over seeds `298180..298199` must
  pass the existing runtime deployment gate.

## Result

`FAIL`, with `100/120` passing rows. All 20 failures were
`intrinsic_scale_stress`.

- Target max absolute error: observation `4.348758 m`, residual `4.336394 m`.
- Range RMSE mean: observation `0.916652 m`, residual `0.911417 m`.
- Position RMSE mean: observation `2.488401 m`, residual `2.486428 m`.
- Tracked intrinsic seed `298188`: target max absolute error regressed from
  `3.679468 m` to `3.689610 m`.

## Interpretation

Lowering ridge to `30` slightly improves the aggregate metrics over ridge `300`,
but it does not change the pass count and slightly worsens the tracked
intrinsic-scale tail. Continue tuning around log-disparity additive, but do not
promote this as a standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled fused measured disparity scalar, not an `x/y/z`
  alias.
- Correction axis: stereo-center RDF `forward` only.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
