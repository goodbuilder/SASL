# EXP-20260623-443 b15x7 common0.05 log_disparity_forward_product ray_scale degree1 ridge3 scout

## Purpose

Lower the high-pass-count log-disparity-forward-product ray-scale branch from
degree 2 to degree 1 to test whether pass count can reach 120/120 and whether
aggregate regression is reduced.

## Method

- Method name: `b15x7_common0p05_logdisparityforwardproduct_rayscale_degree1_ridge3_oversample48_local20_scout`
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

- Target max absolute error: observation `4.131533 m`, residual `4.861383 m`.
- Range RMSE mean: observation `0.912856 m`, residual `1.105966 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.562818 m`.
- Intrinsic seed `298188`: target max absolute error improved from
  `3.623977 m` to `2.445411 m`.
- Worst residual row: `disparity_noise_stress` seed `298196`,
  `4.861383 m`.

## Interpretation

Degree 1 reaches `120/120`, but still heavily regresses aggregate range/position
RMSE and overall target max. This is not a terminal success and not a standalone
deployment candidate. Keep it as a high-pass-count, high-regression anchor for
future blending or conditional selection.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled scalar interaction of fused disparity and
  stereo-center RDF raw forward, not an `x/y/z` alias.
- Correction axis: stereo-center RDF ray scale over right/down/forward.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
