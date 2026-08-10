# EXP-20260623-429 b15x7 common0.05 log_disparity forward_add ridge3 clip0.01 scout

## Purpose

Combine the common0.05 log-disparity additive anchor with a lower correction
clip to see whether intrinsic-scale tail failures improve without losing the
filtered aggregate gain.

## Method

- Method name: `b15x7_common0p05_logdisparity_forwardadd_ridge3_clip0p01_oversample48_local20_scout`
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

- Target max absolute error: observation `4.131533 m`, residual `4.121533 m`.
- Range RMSE mean: observation `0.912856 m`, residual `0.909881 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.485453 m`.
- Tracked intrinsic seed `298188`: target max absolute error regressed from
  `3.623977 m` to `3.633976 m`.

## Interpretation

Clip `0.01` improves the tracked intrinsic seed versus common0.05 clip `0.02`,
but gives back a large share of the aggregate improvement and leaves the pass
count unchanged. This is not a standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled fused measured disparity scalar, not an `x/y/z`
  alias.
- Correction axis: stereo-center RDF `forward` only.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
