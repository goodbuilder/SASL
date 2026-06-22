# EXP-20260623-470 b15x7 common0.05 disparity_over_forward ray_scale degree1 ridge300 scout

## Purpose

Increase ridge regularization from `3` to `300` on the disparity-over-forward
ray-scale branch. The goal is to test whether stronger shrinkage reduces the
aggregate blow-up observed in EXP-20260623-468 while preserving selected
intrinsic-tail relief.

## Method

- Method name: `b15x7_common0p05_disparityoverforward_rayscale_degree1_ridge300_oversample48_local20_scout`
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
- Target max absolute error: observation `4.131533 m`, residual `4.374069 m`.
- Range RMSE mean: observation `0.912856 m`, residual `0.975941 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.523628 m`.
- Intrinsic seed `298188`: target max absolute error improved from
  `3.623977 m` to `3.377886 m`.
- Worst residual row: `disparity_noise_stress` seed `298189`,
  `4.374069 m`.

## Interpretation

Ridge `300` does not shrink the aggregate blow-up; it is slightly worse than
ridge `3` at scout precision while keeping the same intrinsic-scale failure
pattern. This is not a standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: fused disparity divided by corrected stereo-center RDF raw
  forward, not public FRD `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF ray scale over right/down/forward.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
