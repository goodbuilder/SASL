# EXP-20260623-459 b15x7 common0.05 raw_forward forward_add degree1 ridge3 clip0.01 scout

## Purpose

Test a midpoint correction clip between the raw-forward forward-add `0.02`
branch and the softened `0.005` branch. The goal is to see whether clip `0.01`
keeps more RMSE improvement than `0.005` while reducing intrinsic-scale
regression versus `0.02`.

## Method

- Method name: `b15x7_common0p05_rawforward_forwardadd_degree1_ridge3_clip0p01_oversample48_local20_scout`
- Runtime inputs: corrected stereo-center OpenCV optical RDF raw forward scalar.
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
- Target max absolute error: observation `4.131533 m`, residual `4.121533 m`.
- Range RMSE mean: observation `0.912856 m`, residual `0.909327 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.485194 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.623977 m` to `3.633976 m`.
- Worst residual row: `disparity_noise_stress` seed `298189`,
  `4.121533 m`.

## Interpretation

Clip `0.01` lands between clip `0.005` and clip `0.02`: it keeps a moderate
RMSE improvement and a moderate intrinsic tail regression, but it still fails
every intrinsic-scale row. This is not a standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: corrected stereo-center RDF raw forward, not public FRD
  `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF forward additive correction.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
