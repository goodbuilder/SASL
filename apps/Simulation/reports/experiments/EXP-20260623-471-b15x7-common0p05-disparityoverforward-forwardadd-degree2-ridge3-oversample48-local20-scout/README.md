# EXP-20260623-471 b15x7 common0.05 disparity_over_forward forward_add degree2 ridge3 scout

## Purpose

Raise the disparity-over-forward forward-add branch from degree 1 to degree 2.
The goal is to test whether a small nonlinear term increases selected
intrinsic-tail relief without using the aggregate-unstable ray-scale correction.

## Method

- Method name: `b15x7_common0p05_disparityoverforward_forwardadd_degree2_ridge3_oversample48_local20_scout`
- Runtime inputs: fused measured disparity divided by corrected stereo-center
  OpenCV optical RDF raw forward scalar.
- Calibration-only inputs: train split truth residuals used to fit the pooled
  degree-2 ridge model.
- Output contract: apply the fitted residual as a stereo-center OpenCV optical
  RDF forward additive correction before exporting public
  `FRD+range(forward,right,down,range)`.
- Pass/fail criteria: all 120 local scout rows over seeds `298180..298199` must
  pass the existing runtime deployment gate.

## Result

`FAIL`, with `100/120` passing rows.

- Failure distribution: all 20 failures are `intrinsic_scale_stress`.
- Target max absolute error: observation `4.131533 m`, residual `4.151533 m`.
- Range RMSE mean: observation `0.912856 m`, residual `0.914358 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.487698 m`.
- Intrinsic seed `298188`: target max absolute error improved from
  `3.623977 m` to `3.603977 m`.
- Worst residual row: `disparity_noise_stress` seed `298189`,
  `4.151533 m`.

## Interpretation

Degree 2 effectively duplicates the degree-1 disparity-over-forward forward-add
branch on target max and pass count, with slightly worse RMSE. It is not a
standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: fused disparity divided by corrected stereo-center RDF raw
  forward, not public FRD `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF forward additive correction.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
