# EXP-20260623-449 b15x7 common0.05 log_disparity ray_scale degree1 ridge3 scout

## Purpose

Replace the high-pass-count log-disparity-forward-product interaction feature
with log-disparity alone while keeping ray-scale correction. The goal is to
test whether a lower-coupling feature can reduce aggregate regression, even if
it loses intrinsic-scale pass count.

## Method

- Method name: `b15x7_common0p05_logdisparity_rayscale_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: log-scaled fused measured disparity.
- Calibration-only inputs: train split truth residuals used to fit the pooled
  degree-1 ridge model.
- Output contract: apply the fitted residual as a stereo-center OpenCV optical
  RDF ray scale before exporting public `FRD+range(forward,right,down,range)`.
- Pass/fail criteria: all 120 local scout rows over seeds `298180..298199` must
  pass the existing runtime deployment gate.

## Result

`FAIL`, with `100/120` passing rows.

- Failure distribution: all 20 failures are `intrinsic_scale_stress`.
- Target max absolute error: observation `4.131533 m`, residual `4.109082 m`.
- Range RMSE mean: observation `0.912856 m`, residual `0.921242 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.499025 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.623977 m` to `3.658092 m`.
- Worst residual row: `disparity_noise_stress` seed `298189`,
  `4.109082 m`.

## Interpretation

Log-disparity ray-scale degree-1 is much more aggregate-stable than the
log-disparity-forward-product ray-scale strict-pass branch, but it loses every
intrinsic-scale row. This is not a standalone deployment candidate; keep it as
evidence that the interaction term is the source of most intrinsic-scale pass
recovery and most aggregate blow-up.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled fused measured disparity scalar, not an `x/y/z`
  alias.
- Correction axis: stereo-center RDF ray scale over right/down/forward.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
