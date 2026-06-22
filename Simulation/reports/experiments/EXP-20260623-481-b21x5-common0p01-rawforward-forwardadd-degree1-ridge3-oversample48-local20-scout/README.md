# EXP-20260623-481 b21x5 common0.01 raw_forward forward_add degree1 ridge3 scout

## Purpose

Lower the common fraction for the b21x5 raw-forward additive branch from
`0.02` to `0.01`. The goal is to test whether a more selective common sample
keeps improving RMSE or starts trading target-tail stability away.

## Method

- Method name: `b21x5_common0p01_rawforward_forwardadd_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: corrected stereo-center OpenCV optical RDF raw forward
  scalar.
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
- Target max absolute error: observation `4.082800 m`, residual `4.062800 m`.
- Range RMSE mean: observation `0.908683 m`, residual `0.901896 m`.
- Position RMSE mean: observation `2.484307 m`, residual `2.481682 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.604101 m` to `3.624101 m`.
- Worst residual row: `disparity_noise_stress` seed `298180`,
  `4.062800 m`.

## Interpretation

Common `0.01` slightly improves average RMSE relative to EXP479 but gives back
target-tail stability and still fails every intrinsic-scale row. Keep EXP479 as
the better target-tail baseline and this run as the lower-common RMSE contrast.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: corrected stereo-center RDF raw forward, not public FRD
  `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF forward additive correction.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
