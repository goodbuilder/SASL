# EXP-20260623-457 b15x7 common0.02 raw_forward forward_add degree1 ridge3 scout

## Purpose

Lower the low-regression raw-forward forward-add branch common fraction from
`0.05` to `0.02` to test whether looser common sampling improves aggregate
RMSE while preserving the same correction contract.

## Method

- Method name: `b15x7_common0p02_rawforward_forwardadd_degree1_ridge3_oversample48_local20_scout`
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
- Target max absolute error: observation `4.255268 m`, residual `4.235268 m`.
- Range RMSE mean: observation `0.912369 m`, residual `0.905603 m`.
- Position RMSE mean: observation `2.486413 m`, residual `2.483771 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.638059 m` to `3.658059 m`.
- Worst residual row: `disparity_noise_stress` seed `298185`,
  `4.235268 m`.

## Interpretation

Common `0.02` gives the best RMSE mean in the raw-forward forward-add sweep,
but target max is worse than common `0.05` and every intrinsic-scale row still
fails. This is not a standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: corrected stereo-center RDF raw forward, not public FRD
  `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF forward additive correction.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
