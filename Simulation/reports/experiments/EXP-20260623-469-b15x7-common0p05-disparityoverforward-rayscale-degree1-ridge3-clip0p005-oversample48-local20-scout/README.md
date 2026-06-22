# EXP-20260623-469 b15x7 common0.05 disparity_over_forward ray_scale degree1 ridge3 clip0.005 scout

## Purpose

Lower the clip on the disparity-over-forward ray-scale branch from `0.02` to
`0.005`. The goal is to test whether smaller ray-scale correction bounds reduce
the aggregate blow-up observed in EXP-20260623-468 while preserving selected
intrinsic-tail relief.

## Method

- Method name: `b15x7_common0p05_disparityoverforward_rayscale_degree1_ridge3_clip0p005_oversample48_local20_scout`
- Runtime inputs: fused measured disparity divided by corrected stereo-center
  OpenCV optical RDF raw forward scalar.
- Calibration-only inputs: train split truth residuals used to fit the pooled
  degree-1 ridge model.
- Output contract: apply the fitted residual as a stereo-center OpenCV optical
  RDF ray scale before exporting public `FRD+range(forward,right,down,range)`.
- Pass/fail criteria: all 120 local scout rows over seeds `298180..298199` must
  pass the existing runtime deployment gate.

## Result

`FAIL`, with `100/120` passing rows.

- Failure distribution: all 20 failures are `intrinsic_scale_stress`.
- Target max absolute error: observation `4.131533 m`, residual `4.373617 m`.
- Range RMSE mean: observation `0.912856 m`, residual `0.975815 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.523561 m`.
- Intrinsic seed `298188`: target max absolute error improved from
  `3.623977 m` to `3.378348 m`.
- Worst residual row: `disparity_noise_stress` seed `298189`,
  `4.373617 m`.

## Interpretation

Clip `0.005` duplicates clip `0.02` at scout precision. The branch keeps the
same selected intrinsic relief and the same aggregate blow-up, so it is not a
standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: fused disparity divided by corrected stereo-center RDF raw
  forward, not public FRD `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF ray scale over right/down/forward.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
