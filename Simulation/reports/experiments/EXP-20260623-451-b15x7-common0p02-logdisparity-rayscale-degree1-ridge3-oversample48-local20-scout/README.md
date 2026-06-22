# EXP-20260623-451 b15x7 common0.02 log_disparity ray_scale degree1 ridge3 scout

## Purpose

Lower the aggregate-stable log-disparity ray-scale branch common fraction from
`0.05` to `0.02` to test whether a looser common-sample filter improves
aggregate accuracy or intrinsic-scale behavior.

## Method

- Method name: `b15x7_common0p02_logdisparity_rayscale_degree1_ridge3_oversample48_local20_scout`
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
- Target max absolute error: observation `4.255268 m`, residual `4.220372 m`.
- Range RMSE mean: observation `0.912369 m`, residual `0.920500 m`.
- Position RMSE mean: observation `2.486413 m`, residual `2.498693 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.638059 m` to `3.673890 m`.
- Worst residual row: `disparity_noise_stress` seed `298185`,
  `4.220372 m`.

## Interpretation

Common `0.02` slightly improves range and position RMSE versus common `0.05`
but worsens target max and still fails every intrinsic-scale row. This is not a
standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled fused measured disparity scalar, not an `x/y/z`
  alias.
- Correction axis: stereo-center RDF ray scale over right/down/forward.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
