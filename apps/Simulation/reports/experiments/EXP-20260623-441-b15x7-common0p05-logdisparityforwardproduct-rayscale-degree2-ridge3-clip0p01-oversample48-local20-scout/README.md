# EXP-20260623-441 b15x7 common0.05 log_disparity_forward_product ray_scale degree2 ridge3 clip0.01 scout

## Purpose

Reduce the correction clip on the high-pass-count log-disparity-forward-product
ray-scale branch to test whether aggregate regression can be reduced while
keeping the `119/120` pass count.

## Method

- Method name: `b15x7_common0p05_logdisparityforwardproduct_rayscale_degree2_ridge3_clip0p01_oversample48_local20_scout`
- Runtime inputs: log-scaled scalar interaction of fused measured disparity and
  corrected stereo-center RDF raw forward.
- Calibration-only inputs: train split truth residuals used to fit the pooled
  degree-2 ridge model.
- Output contract: apply the fitted residual as a stereo-center OpenCV optical
  RDF ray scale before exporting public `FRD+range(forward,right,down,range)`.
- Pass/fail criteria: all 120 local scout rows over seeds `298180..298199` must
  pass the existing runtime deployment gate.

## Result

`FAIL`, with `119/120` passing rows. The only failure was
`intrinsic_scale_stress` seed `298188`.

- Target max absolute error: observation `4.131533 m`, residual `4.907761 m`.
- Range RMSE mean: observation `0.912856 m`, residual `1.116085 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.584802 m`.
- Failing intrinsic seed `298188`: target max absolute error improved from
  `3.623977 m` to `2.668908 m`, but still failed the gate.

## Interpretation

Clip `0.01` matches clip `0.02` at scout precision. This branch remains a
high-pass-count but high-aggregate-regression tradeoff anchor, not a standalone
deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled scalar interaction of fused disparity and
  stereo-center RDF raw forward, not an `x/y/z` alias.
- Correction axis: stereo-center RDF ray scale over right/down/forward.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
