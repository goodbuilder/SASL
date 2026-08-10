# EXP-20260623-444 b15x7 common0.1 log_disparity_forward_product ray_scale degree1 ridge3 scout

## Purpose

Raise the degree-1 log-disparity-forward-product ray-scale branch from
`common_fraction=0.05` to `0.1` to test whether a wider residual population
reduces aggregate regression while preserving strict pass behavior.

## Method

- Method name: `b15x7_common0p1_logdisparityforwardproduct_rayscale_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: log-scaled scalar interaction of fused measured disparity and
  corrected stereo-center RDF raw forward.
- Calibration-only inputs: train split truth residuals used to fit the pooled
  degree-1 ridge model.
- Output contract: apply the fitted residual as a stereo-center OpenCV optical
  RDF ray scale before exporting public `FRD+range(forward,right,down,range)`.
- Pass/fail criteria: all 120 local scout rows over seeds `298180..298199` must
  pass the existing runtime deployment gate.

## Result

`STRICT_PASS`, with `120/120` passing rows.

- Target max absolute error: observation `4.348758 m`, residual `5.094019 m`.
- Range RMSE mean: observation `0.916652 m`, residual `1.108084 m`.
- Position RMSE mean: observation `2.488401 m`, residual `2.563997 m`.
- Intrinsic seed `298188`: target max absolute error improved from
  `3.679468 m` to `2.510924 m`.
- Worst residual row: `disparity_noise_stress` seed `298188`,
  `5.094019 m`.

## Interpretation

Common fraction `0.1` preserves strict pass behavior but worsens worst residual
target error compared with common0.05 and keeps large aggregate regression. This
is not a terminal success and not a standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled scalar interaction of fused disparity and
  stereo-center RDF raw forward, not an `x/y/z` alias.
- Correction axis: stereo-center RDF ray scale over right/down/forward.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
