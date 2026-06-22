# EXP-20260623-438 b15x7 common0.05 log_disparity_forward_product forward_add degree2 ridge3 scout

## Purpose

Test whether a log-scaled disparity-forward interaction improves the common0.05
forward-additive branch compared with log-disparity alone.

## Method

- Method name: `b15x7_common0p05_logdisparityforwardproduct_forwardadd_degree2_ridge3_oversample48_local20_scout`
- Runtime inputs: log-scaled scalar interaction of fused measured disparity and
  corrected stereo-center RDF raw forward.
- Calibration-only inputs: train split truth residuals used to fit the pooled
  degree-2 ridge model.
- Output contract: add the fitted residual to stereo-center OpenCV optical RDF
  `forward` before exporting public `FRD+range(forward,right,down,range)`.
- Pass/fail criteria: all 120 local scout rows over seeds `298180..298199` must
  pass the existing runtime deployment gate.

## Result

`FAIL`, with `100/120` passing rows. All 20 failures were
`intrinsic_scale_stress`.

- Target max absolute error: observation `4.131533 m`, residual `4.151533 m`.
- Range RMSE mean: observation `0.912856 m`, residual `0.916590 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.487374 m`.
- Tracked intrinsic seed `298188`: target max absolute error improved from
  `3.623977 m` to `3.603977 m`.

## Interpretation

The interaction feature improves the tracked intrinsic seed and worst failing
seed, but worsens overall target max and aggregate range/position RMSE. It is
not a standalone deployment candidate; it may be useful only for a later
gate-conditioned blend.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled scalar interaction of fused disparity and
  stereo-center RDF raw forward, not an `x/y/z` alias.
- Correction axis: stereo-center RDF `forward` only.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
