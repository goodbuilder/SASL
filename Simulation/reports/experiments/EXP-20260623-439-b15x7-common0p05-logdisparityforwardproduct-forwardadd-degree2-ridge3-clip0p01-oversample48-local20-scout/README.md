# EXP-20260623-439 b15x7 common0.05 log_disparity_forward_product forward_add degree2 ridge3 clip0.01 scout

## Purpose

Reduce the clip on the log-disparity-forward-product additive branch to see
whether selected intrinsic-tail improvements can be kept while reducing
aggregate regression.

## Method

- Method name: `b15x7_common0p05_logdisparityforwardproduct_forwardadd_degree2_ridge3_clip0p01_oversample48_local20_scout`
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

- Target max absolute error: observation `4.131533 m`, residual `4.141533 m`.
- Range RMSE mean: observation `0.912856 m`, residual `0.914583 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.486908 m`.
- Tracked intrinsic seed `298188`: target max absolute error improved from
  `3.623977 m` to `3.613977 m`.

## Interpretation

Clip `0.01` keeps part of the selected intrinsic-tail improvement and reduces
aggregate regression compared with clip `0.02`, but aggregate metrics still
worsen versus observation and pass count is unchanged. Do not promote this as a
standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled scalar interaction of fused disparity and
  stereo-center RDF raw forward, not an `x/y/z` alias.
- Correction axis: stereo-center RDF `forward` only.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
