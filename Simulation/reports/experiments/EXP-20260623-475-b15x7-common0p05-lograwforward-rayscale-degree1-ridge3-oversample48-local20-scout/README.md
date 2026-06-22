# EXP-20260623-475 b15x7 common0.05 log_raw_forward ray_scale degree1 ridge3 scout

## Purpose

Move the log-raw-forward feature from the additive forward branch to a
whole-ray scale branch. The goal is to test whether a compressed forward-depth
feature can reduce target-tail errors without relying on a direct forward-axis
offset.

## Method

- Method name: `b15x7_common0p05_lograwforward_rayscale_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: log of corrected stereo-center OpenCV optical RDF raw
  forward scalar.
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
- Target max absolute error: observation `4.131533 m`, residual `4.111345 m`.
- Range RMSE mean: observation `0.912856 m`, residual `0.921721 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.499329 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.623977 m` to `3.657109 m`.
- Worst residual row: `disparity_noise_stress` seed `298189`,
  `4.111345 m`.

## Interpretation

The ray-scale log-forward branch slightly lowers aggregate target max but
worsens average range and position RMSE, and it still fails every intrinsic
row. Keep this only as a contrast for compressed forward-depth features.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log of corrected stereo-center RDF raw forward, not public FRD
  `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF ray scale over
  `(right, down, forward)`.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
