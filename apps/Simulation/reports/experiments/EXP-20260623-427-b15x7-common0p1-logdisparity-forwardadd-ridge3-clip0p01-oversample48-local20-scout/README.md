# EXP-20260623-427 b15x7 common0.1 log_disparity forward_add ridge3 clip0.01 scout

## Purpose

Reduce the correction clip on the log-disparity additive branch from `0.02` to
`0.01` to see whether intrinsic-scale tail regressions can be softened.

## Method

- Method name: `b15x7_common0p1_logdisparity_forwardadd_ridge3_clip0p01_oversample48_local20_scout`
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

- Target max absolute error: observation `4.348758 m`, residual `4.338758 m`.
- Range RMSE mean: observation `0.916652 m`, residual `0.913748 m`.
- Position RMSE mean: observation `2.488401 m`, residual `2.487273 m`.
- Tracked intrinsic seed `298188`: target max absolute error regressed from
  `3.679468 m` to `3.689468 m`.

## Interpretation

Clip `0.01` softens the intrinsic-tail regression slightly compared with
clip `0.02`, but it also gives back much of the aggregate gain and does not
change the pass count. It is not a standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled fused measured disparity scalar, not an `x/y/z`
  alias.
- Correction axis: stereo-center RDF `forward` only.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
