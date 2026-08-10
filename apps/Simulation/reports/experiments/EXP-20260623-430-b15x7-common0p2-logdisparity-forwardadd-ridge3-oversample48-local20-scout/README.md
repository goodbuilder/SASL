# EXP-20260623-430 b15x7 common0.2 log_disparity forward_add ridge3 scout

## Purpose

Test the opposite common-fraction direction from common0.05 by widening the
log-disparity additive residual subset to `common_fraction=0.2`.

## Method

- Method name: `b15x7_common0p2_logdisparity_forwardadd_ridge3_oversample48_local20_scout`
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

- Target max absolute error: observation `4.285147 m`, residual `4.270090 m`.
- Range RMSE mean: observation `0.924142 m`, residual `0.918877 m`.
- Position RMSE mean: observation `2.492225 m`, residual `2.490188 m`.
- Tracked intrinsic seed `298188`: target max absolute error regressed from
  `3.671248 m` to `3.685612 m`.

## Interpretation

Common fraction `0.2` still improves residual metrics within its own filtered
population, but the filtered observation baseline is worse than common0.05 and
the intrinsic-scale pass count remains unchanged. This direction is weaker than
the common0.05 anchor.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled fused measured disparity scalar, not an `x/y/z`
  alias.
- Correction axis: stereo-center RDF `forward` only.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
