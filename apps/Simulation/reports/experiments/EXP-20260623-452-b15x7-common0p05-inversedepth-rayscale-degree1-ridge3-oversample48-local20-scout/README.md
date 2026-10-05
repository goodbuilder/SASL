# EXP-20260623-452 b15x7 common0.05 inverse_depth ray_scale degree1 ridge3 scout

## Purpose

Test inverse-depth as a ray-scale residual feature. Previous inverse-depth
additive correction behaved like the disparity additive branch; this scout
checks whether applying the same distance scalar as a ray scale gives a better
intrinsic-scale and aggregate tradeoff.

## Method

- Method name: `b15x7_common0p05_inversedepth_rayscale_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: inverse-depth scalar derived from the fused stereo estimate.
- Calibration-only inputs: train split truth residuals used to fit the pooled
  degree-1 ridge model.
- Output contract: apply the fitted residual as a stereo-center OpenCV optical
  RDF ray scale before exporting public `FRD+range(forward,right,down,range)`.
- Pass/fail criteria: all 120 local scout rows over seeds `298180..298199` must
  pass the existing runtime deployment gate.

## Result

`FAIL`, with `100/120` passing rows.

- Failure distribution: all 20 failures are `intrinsic_scale_stress`.
- Target max absolute error: observation `4.131533 m`, residual `4.260457 m`.
- Range RMSE mean: observation `0.912856 m`, residual `0.947511 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.509684 m`.
- Intrinsic seed `298188`: target max absolute error improved from
  `3.623977 m` to `3.496363 m`.
- Worst residual row: `disparity_noise_stress` seed `298189`,
  `4.260457 m`.

## Interpretation

Inverse-depth ray scale improves selected intrinsic rows but still fails all
intrinsic-scale rows and is aggregate-worse than log-disparity ray scale. This
is not a standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: inverse-depth scalar, not an `x/y/z` alias.
- Correction axis: stereo-center RDF ray scale over right/down/forward.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
