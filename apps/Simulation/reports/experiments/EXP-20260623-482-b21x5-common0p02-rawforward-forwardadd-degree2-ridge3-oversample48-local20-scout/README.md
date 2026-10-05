# EXP-20260623-482 b21x5 common0.02 raw_forward forward_add degree2 ridge3 scout

## Purpose

Increase the polynomial degree from 1 to 2 on the b21x5 common `0.02`
raw-forward additive branch. The goal is to test whether a curved residual
model improves the intrinsic-scale tail without losing the low-RMSE behavior of
EXP479.

## Method

- Method name: `b21x5_common0p02_rawforward_forwardadd_degree2_ridge3_oversample48_local20_scout`
- Runtime inputs: corrected stereo-center OpenCV optical RDF raw forward
  scalar.
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
- Target max absolute error: observation `4.076885 m`, residual `4.056885 m`.
- Range RMSE mean: observation `0.908840 m`, residual `0.902118 m`.
- Position RMSE mean: observation `2.484420 m`, residual `2.481834 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.587690 m` to `3.607690 m`.
- Worst residual row: `disparity_noise_stress` seed `298191`,
  `4.056885 m`.

## Interpretation

Degree 2 duplicates EXP479 target behavior and is slightly worse on average
RMSE. It does not solve the intrinsic-scale gate, so keep degree 1 as the
cleaner b21x5 common `0.02` baseline.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: corrected stereo-center RDF raw forward, not public FRD
  `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF forward additive correction.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
