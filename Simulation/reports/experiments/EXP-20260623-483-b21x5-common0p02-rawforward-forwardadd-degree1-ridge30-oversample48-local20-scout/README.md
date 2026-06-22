# EXP-20260623-483 b21x5 common0.02 raw_forward forward_add degree1 ridge30 scout

## Purpose

Increase ridge regularization from `3` to `30` on the b21x5 common `0.02`
raw-forward additive branch. The goal is to test whether stronger coefficient
shrinkage reduces intrinsic target regressions without sacrificing the EXP479
aggregate gains.

## Method

- Method name: `b21x5_common0p02_rawforward_forwardadd_degree1_ridge30_oversample48_local20_scout`
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
- Target max absolute error: observation `4.076885 m`, residual `4.056885 m`.
- Range RMSE mean: observation `0.908840 m`, residual `0.902026 m`.
- Position RMSE mean: observation `2.484420 m`, residual `2.481774 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.587690 m` to `3.607690 m`.
- Worst residual row: `disparity_noise_stress` seed `298191`,
  `4.056885 m`.

## Interpretation

Ridge `30` is effectively identical to EXP479 ridge `3`; it does not reduce the
intrinsic-scale failures or change the target-tail behavior. Keep EXP479 as the
simpler baseline.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: corrected stereo-center RDF raw forward, not public FRD
  `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF forward additive correction.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
