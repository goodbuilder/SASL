# EXP-20260623-461 b15x7 common0.05 raw_forward forward_add degree1 ridge3 oversample12 scout

## Purpose

Reduce target oversampling further from `24` to `12` for the raw-forward
forward-add branch. The goal is to test whether less target duplication
continues the RMSE improvement trend or changes the intrinsic-scale failure
pattern.

## Method

- Method name: `b15x7_common0p05_rawforward_forwardadd_degree1_ridge3_oversample12_local20_scout`
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
- Target max absolute error: observation `4.184992 m`, residual `4.164993 m`.
- Range RMSE mean: observation `0.912015 m`, residual `0.905362 m`.
- Position RMSE mean: observation `2.484885 m`, residual `2.482285 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.623439 m` to `3.643439 m`.
- Worst residual row: `disparity_noise_stress` seed `298187`,
  `4.164993 m`.

## Interpretation

Oversample `12` gives the lowest RMSE mean in this oversample sweep but worsens
target max versus oversample `24` and still fails every intrinsic-scale row.
This is not a standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: corrected stereo-center RDF raw forward, not public FRD
  `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF forward additive correction.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
