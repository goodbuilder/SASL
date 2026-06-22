# EXP-20260623-446 b15x7 common0.05 log_disparity_forward_product ray_scale degree1 ridge0.3 scout

## Purpose

Lower ridge on the strict-pass common0.05 log-disparity-forward-product ray-scale
degree-1 branch from `3` to `0.3` to test whether the branch is regularization
sensitive.

## Method

- Method name: `b15x7_common0p05_logdisparityforwardproduct_rayscale_degree1_ridge0p3_oversample48_local20_scout`
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

- Target max absolute error: observation `4.131533 m`, residual `4.861392 m`.
- Range RMSE mean: observation `0.912856 m`, residual `1.105969 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.562819 m`.
- Intrinsic seed `298188`: target max absolute error improved from
  `3.623977 m` to `2.445390 m`.
- Worst residual row: `disparity_noise_stress` seed `298196`,
  `4.861392 m`.

## Interpretation

Ridge `0.3` is effectively identical to ridge `3` at scout precision. The
branch remains a strict-pass but high-regression anchor, not a standalone
deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled scalar interaction of fused disparity and
  stereo-center RDF raw forward, not an `x/y/z` alias.
- Correction axis: stereo-center RDF ray scale over right/down/forward.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
