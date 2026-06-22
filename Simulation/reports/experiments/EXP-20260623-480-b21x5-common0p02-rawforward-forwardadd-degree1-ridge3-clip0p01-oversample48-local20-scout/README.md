# EXP-20260623-480 b21x5 common0.02 raw_forward forward_add degree1 ridge3 clip0.01 scout

## Purpose

Reduce the correction clip on the b21x5 common `0.02` raw-forward additive
branch from `0.02` to `0.01`. The goal is to test whether a smaller forward
additive correction reduces intrinsic target regressions while preserving most
of the aggregate RMSE gain.

## Method

- Method name: `b21x5_common0p02_rawforward_forwardadd_degree1_ridge3_clip0p01_oversample48_local20_scout`
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
- Target max absolute error: observation `4.076885 m`, residual `4.066885 m`.
- Range RMSE mean: observation `0.908840 m`, residual `0.905240 m`.
- Position RMSE mean: observation `2.484420 m`, residual `2.482994 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.587690 m` to `3.597690 m`.
- Worst residual row: `disparity_noise_stress` seed `298191`,
  `4.066885 m`.

## Interpretation

Clip `0.01` halves the selected intrinsic target regression relative to
EXP479, but it also gives back much of the aggregate improvement and still
fails every intrinsic-scale row. Keep EXP479 as the stronger baseline and this
run as the clip tradeoff record.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: corrected stereo-center RDF raw forward, not public FRD
  `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF forward additive correction.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
