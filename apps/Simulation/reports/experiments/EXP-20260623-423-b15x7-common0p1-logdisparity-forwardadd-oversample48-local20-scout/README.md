# EXP-20260623-423 b15x7 common0.1 log_disparity forward_add scout

## Purpose

Test whether log-scaling the fused disparity scalar improves the additive
forward-residual branch beyond the raw-forward and plain-disparity anchors.

## Method

- Method name: `b15x7_common0p1_logdisparity_forwardadd_oversample48_local20_scout`
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

- Target max absolute error: observation `4.348758 m`, residual `4.336767 m`.
- Range RMSE mean: observation `0.916652 m`, residual `0.911453 m`.
- Position RMSE mean: observation `2.488401 m`, residual `2.486445 m`.
- Tracked intrinsic seed `298188`: target max absolute error regressed from
  `3.679468 m` to `3.689243 m`.

## Interpretation

Log-disparity additive correction is a useful aggregate anchor: it improves
overall target max error and aggregate range/position RMSE. It still fails all
intrinsic-scale rows and regresses the tracked intrinsic seed, so it is not a
deployment candidate by itself. Continue searching around this branch with
different regularization, clips, or blends.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled fused measured disparity scalar, not an `x/y/z`
  alias.
- Correction axis: stereo-center RDF `forward` only.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
