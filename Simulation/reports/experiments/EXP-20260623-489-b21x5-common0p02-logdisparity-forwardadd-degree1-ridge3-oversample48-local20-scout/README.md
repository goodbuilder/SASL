# EXP-20260623-489 b21x5 common0.02 log_disparity forward_add degree1 ridge3 scout

## Purpose

Test log disparity as the feature for the b21x5 common `0.02` additive branch.
The goal is to see whether a disparity-domain feature can retain the low-RMSE
behavior of raw forward while changing the intrinsic-scale failure pattern.

## Method

- Method name: `b21x5_common0p02_logdisparity_forwardadd_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: log of fused measured disparity.
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
- Target max absolute error: observation `4.076885 m`, residual `4.065071 m`.
- Range RMSE mean: observation `0.908840 m`, residual `0.903244 m`.
- Position RMSE mean: observation `2.484420 m`, residual `2.482324 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.587690 m` to `3.598194 m`.
- Worst residual row: `disparity_noise_stress` seed `298191`,
  `4.065071 m`.

## Interpretation

Log disparity improves aggregate target max and average RMSE, but it is weaker
than the raw-forward EXP479 baseline and still fails every intrinsic-scale row.
Keep it as a disparity-domain low-regression contrast only.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log of fused measured disparity, not public FRD `forward` and
  not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF forward additive correction.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
