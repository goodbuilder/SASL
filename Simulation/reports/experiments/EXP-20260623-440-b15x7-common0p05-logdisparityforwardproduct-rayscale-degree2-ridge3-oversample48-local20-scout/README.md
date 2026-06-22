# EXP-20260623-440 b15x7 common0.05 log_disparity_forward_product ray_scale degree2 ridge3 scout

## Purpose

Test whether the log-disparity-forward-product interaction that helped selected
intrinsic tails can improve pass count when applied as a full RDF ray-scale
correction.

## Method

- Method name: `b15x7_common0p05_logdisparityforwardproduct_rayscale_degree2_ridge3_oversample48_local20_scout`
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

This branch is the first recent scout to raise pass count to `119/120`, but it
does so while heavily worsening aggregate RMSE and overall residual target max.
It is not a standalone deployment candidate. Keep it as a high-pass-count
tradeoff anchor for later blending or gate-specific selection.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled scalar interaction of fused disparity and
  stereo-center RDF raw forward, not an `x/y/z` alias.
- Correction axis: stereo-center RDF ray scale over right/down/forward.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
