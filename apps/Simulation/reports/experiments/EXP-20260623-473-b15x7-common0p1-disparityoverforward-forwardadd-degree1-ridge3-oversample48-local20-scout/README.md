# EXP-20260623-473 b15x7 common0.1 disparity_over_forward forward_add degree1 ridge3 scout

## Purpose

Raise the common fraction from `0.05` to `0.1` for the
disparity-over-forward forward-add branch. The goal is to complete the common
fraction sweep and test whether stricter common sampling improves the selected
intrinsic-tail relief.

## Method

- Method name: `b15x7_common0p1_disparityoverforward_forwardadd_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: fused measured disparity divided by corrected stereo-center
  OpenCV optical RDF raw forward scalar.
- Calibration-only inputs: train split truth residuals used to fit the pooled
  degree-1 ridge model.
- Output contract: apply the fitted residual as a stereo-center OpenCV optical
  RDF forward additive correction before exporting public
  `FRD+range(forward,right,down,range)`.
- Pass/fail criteria: all 120 local scout rows over seeds `298180..298199` must
  pass the existing runtime deployment gate.

## Result

`FAIL`, with `100/120` passing rows.

- Failure distribution: all 20 failures are `intrinsic_scale_stress`.
- Target max absolute error: observation `4.348758 m`, residual `4.368757 m`.
- Range RMSE mean: observation `0.916652 m`, residual `0.918012 m`.
- Position RMSE mean: observation `2.488401 m`, residual `2.489456 m`.
- Intrinsic seed `298188`: target max absolute error improved from
  `3.679468 m` to `3.659469 m`.
- Worst residual row: `disparity_noise_stress` seed `298188`,
  `4.368757 m`.

## Interpretation

Common `0.1` is worse than common `0.05` and common `0.02` on aggregate target
max and RMSE while still failing every intrinsic-scale row. This is not a
standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: fused disparity divided by corrected stereo-center RDF raw
  forward, not public FRD `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF forward additive correction.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
