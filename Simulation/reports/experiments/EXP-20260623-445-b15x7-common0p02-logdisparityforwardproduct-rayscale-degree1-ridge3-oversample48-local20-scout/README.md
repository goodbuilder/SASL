# EXP-20260623-445 b15x7 common0.02 log_disparity_forward_product ray_scale degree1 ridge3 scout

## Purpose

Narrow the strict-pass log-disparity-forward-product ray-scale degree-1 branch
from `common_fraction=0.05` to `0.02` to test whether aggregate regression can
be reduced while preserving `120/120` pass behavior.

## Method

- Method name: `b15x7_common0p02_logdisparityforwardproduct_rayscale_degree1_ridge3_oversample48_local20_scout`
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

- Target max absolute error: observation `4.255268 m`, residual `5.000456 m`.
- Range RMSE mean: observation `0.912369 m`, residual `1.106347 m`.
- Position RMSE mean: observation `2.486413 m`, residual `2.563112 m`.
- Intrinsic seed `298188`: target max absolute error improved from
  `3.638059 m` to `2.441333 m`.
- Worst residual row: `disparity_noise_stress` seed `298185`,
  `5.000456 m`.

## Interpretation

Common fraction `0.02` keeps strict pass behavior but does not reduce aggregate
regression. Worst target error is worse than common0.05. This is not a terminal
success and not a standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled scalar interaction of fused disparity and
  stereo-center RDF raw forward, not an `x/y/z` alias.
- Correction axis: stereo-center RDF ray scale over right/down/forward.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
