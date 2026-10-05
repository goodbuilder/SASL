# EXP-20260623-426 b15x7 common0.1 log_disparity forward_add ridge0.3 scout

## Purpose

Check whether the log-disparity additive branch continues improving when ridge
is lowered from `3` to `0.3`.

## Method

- Method name: `b15x7_common0p1_logdisparity_forwardadd_ridge0p3_oversample48_local20_scout`
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

- Target max absolute error: observation `4.348758 m`, residual `4.336353 m`.
- Range RMSE mean: observation `0.916652 m`, residual `0.911413 m`.
- Position RMSE mean: observation `2.488401 m`, residual `2.486427 m`.
- Tracked intrinsic seed `298188`: target max absolute error regressed from
  `3.679468 m` to `3.689651 m`.

## Interpretation

Ridge `0.3` is effectively saturated against ridge `3`: aggregate gains change
only in the fourth to sixth decimal places while the intrinsic-scale failures
remain unchanged. Move future search to clip/common-fraction/blend axes rather
than lower ridge in this branch.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled fused measured disparity scalar, not an `x/y/z`
  alias.
- Correction axis: stereo-center RDF `forward` only.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
