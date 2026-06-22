# EXP-20260623-462 b21x5 common0.05 raw_forward forward_add degree1 ridge3 scout

## Purpose

Change the burst aggregation shape from `b15x7` to `b21x5` while keeping the
same total observation count (`105`) and raw-forward forward-add correction.
The goal is to test whether more bursts with fewer observations each changes
intrinsic-scale tails or aggregate RMSE.

## Method

- Method name: `b21x5_common0p05_rawforward_forwardadd_degree1_ridge3_oversample48_local20_scout`
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
- Target max absolute error: observation `4.219042 m`, residual `4.199042 m`.
- Range RMSE mean: observation `0.911104 m`, residual `0.904418 m`.
- Position RMSE mean: observation `2.485454 m`, residual `2.482848 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.603210 m` to `3.623209 m`.
- Worst residual row: `disparity_noise_stress` seed `298188`,
  `4.199042 m`.

## Interpretation

`b21x5` improves RMSE mean versus the `b15x7` raw-forward forward-add baseline
but worsens target max and still fails every intrinsic-scale row. This is not a
standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: corrected stereo-center RDF raw forward, not public FRD
  `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF forward additive correction.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
