# EXP-20260623-428 b15x7 common0.05 log_disparity forward_add ridge3 scout

## Purpose

Move the log-disparity additive branch from `common_fraction=0.1` to `0.05` to
test whether a stricter common residual subset improves aggregate and tail
behavior.

## Method

- Method name: `b15x7_common0p05_logdisparity_forwardadd_ridge3_oversample48_local20_scout`
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

- Target max absolute error: observation `4.131533 m`, residual `4.121506 m`.
- Range RMSE mean: observation `0.912856 m`, residual `0.907490 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.484591 m`.
- Tracked intrinsic seed `298188`: target max absolute error regressed from
  `3.623977 m` to `3.637634 m`.

## Interpretation

Common fraction `0.05` improves residual metrics within its own filtered
contract and lowers the observed/residual worst target compared with the
common0.1 scout population. It still fails every intrinsic-scale row and
regresses the tracked intrinsic seed, so it remains an exploration anchor rather
than a deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled fused measured disparity scalar, not an `x/y/z`
  alias.
- Correction axis: stereo-center RDF `forward` only.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
