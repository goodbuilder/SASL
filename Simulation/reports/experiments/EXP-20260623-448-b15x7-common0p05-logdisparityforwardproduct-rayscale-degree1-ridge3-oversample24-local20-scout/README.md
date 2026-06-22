# EXP-20260623-448 b15x7 common0.05 log_disparity_forward_product ray_scale degree1 ridge3 oversample24 scout

## Purpose

Reduce the high-pass-count ray-scale branch target oversample count from `48`
to `24` to test whether less calibration target duplication reduces aggregate
regression while preserving the strict-pass gate behavior.

## Method

- Method name: `b15x7_common0p05_logdisparityforwardproduct_rayscale_degree1_ridge3_oversample24_local20_scout`
- Runtime inputs: log-scaled scalar interaction of fused measured disparity and
  corrected stereo-center RDF raw forward.
- Calibration-only inputs: train split truth residuals used to fit the pooled
  degree-1 ridge model.
- Output contract: apply the fitted residual as a stereo-center OpenCV optical
  RDF ray scale before exporting public `FRD+range(forward,right,down,range)`.
- Pass/fail criteria: all 120 local scout rows over seeds `298180..298199` must
  pass the existing runtime deployment gate.

## Result

`STRICT_PASS`, with `120/120` passing rows.

- Target max absolute error: observation `4.151822 m`, residual `4.888552 m`.
- Range RMSE mean: observation `0.912811 m`, residual `1.105861 m`.
- Position RMSE mean: observation `2.486087 m`, residual `2.562346 m`.
- Intrinsic seed `298188`: target max absolute error improved from
  `3.998646 m` to `2.791564 m`.
- Worst residual row: `disparity_noise_stress` seed `298187`,
  `4.888552 m`.

## Interpretation

Lowering the target oversample count keeps the strict-pass result but does not
solve the aggregate regression. Range and position RMSE remain close to the
oversample48 high-pass-count branch, and target max absolute error still
regresses substantially. This is not a standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled scalar interaction of fused disparity and
  stereo-center RDF raw forward, not an `x/y/z` alias.
- Correction axis: stereo-center RDF ray scale over right/down/forward.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
