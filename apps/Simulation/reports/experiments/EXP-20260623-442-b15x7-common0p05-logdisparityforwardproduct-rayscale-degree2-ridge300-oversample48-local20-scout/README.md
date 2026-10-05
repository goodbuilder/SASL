# EXP-20260623-442 b15x7 common0.05 log_disparity_forward_product ray_scale degree2 ridge300 scout

## Purpose

Increase ridge regularization on the high-pass-count log-disparity-forward-product
ray-scale branch to test whether aggregate regression can be reduced without
losing the `119/120` pass behavior.

## Method

- Method name: `b15x7_common0p05_logdisparityforwardproduct_rayscale_degree2_ridge300_oversample48_local20_scout`
- Runtime inputs: log-scaled scalar interaction of fused measured disparity and
  corrected stereo-center RDF raw forward.
- Calibration-only inputs: train split truth residuals used to fit the pooled
  degree-2 ridge model.
- Output contract: apply the fitted residual as a stereo-center OpenCV optical
  RDF ray scale before exporting public `FRD+range(forward,right,down,range)`.
- Pass/fail criteria: all 120 local scout rows over seeds `298180..298199` must
  pass the existing runtime deployment gate.

## Result

`FAIL`, with `118/120` passing rows. Both failures were
`intrinsic_scale_stress` seeds `298188` and `298198`.

- Target max absolute error: observation `4.131533 m`, residual `4.906402 m`.
- Range RMSE mean: observation `0.912856 m`, residual `1.115710 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.584595 m`.
- Failing intrinsic seed `298188`: target max absolute error improved from
  `3.623977 m` to `2.670056 m`, but still failed the gate.

## Interpretation

Ridge `300` does not repair the high-pass-count ray-scale tradeoff. It leaves
aggregate RMSE severely regressed and adds a second intrinsic-scale failure
relative to ridge `3`. Do not promote as a standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled scalar interaction of fused disparity and
  stereo-center RDF raw forward, not an `x/y/z` alias.
- Correction axis: stereo-center RDF ray scale over right/down/forward.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
