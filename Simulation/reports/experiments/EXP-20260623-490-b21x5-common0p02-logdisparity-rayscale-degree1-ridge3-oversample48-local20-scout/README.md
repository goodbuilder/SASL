# EXP-20260623-490 b21x5 common0.02 log_disparity ray_scale degree1 ridge3 scout

## Purpose

Move the log-disparity feature from forward additive correction to whole-ray
scaling. The goal is to test whether disparity-domain ray scaling improves the
target tail without the raw-forward ray-scale RMSE penalty.

## Method

- Method name: `b21x5_common0p02_logdisparity_rayscale_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: log of fused measured disparity.
- Calibration-only inputs: train split truth residuals used to fit the pooled
  degree-1 ridge model.
- Output contract: apply the fitted residual as a stereo-center OpenCV optical
  RDF ray-scale correction before exporting public
  `FRD+range(forward,right,down,range)`.
- Pass/fail criteria: all 120 local scout rows over seeds `298180..298199` must
  pass the existing runtime deployment gate.

## Result

`FAIL`, with `100/120` passing rows.

- Failure distribution: all 20 failures are `intrinsic_scale_stress`.
- Target max absolute error: observation `4.076885 m`, residual `4.052020 m`.
- Range RMSE mean: observation `0.908840 m`, residual `0.917946 m`.
- Position RMSE mean: observation `2.484420 m`, residual `2.497215 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.587690 m` to `3.608497 m`.
- Worst residual row: `disparity_noise_stress` seed `298191`,
  `4.052020 m`.

## Interpretation

Log-disparity ray scaling reduces aggregate target max more than additive log
disparity, but it regresses range and position RMSE and still fails every
intrinsic-scale row. Reject it as a standalone deployment path.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log of fused measured disparity, not public FRD `forward` and
  not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF ray scale over
  `(right, down, forward)`.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
