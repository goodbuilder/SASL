# EXP-20260623-474 b15x7 common0.05 forward_over_disparity ray_scale degree1 ridge3 scout

## Purpose

Move the forward-over-disparity feature from the additive forward branch to a
whole-ray scale branch. The goal is to test whether the ratio feature can lower
the intrinsic tail without only shifting the stereo-center RDF forward axis.

## Method

- Method name: `b15x7_common0p05_forwardoverdisparity_rayscale_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: corrected stereo-center OpenCV optical RDF raw forward scalar
  divided by fused measured disparity.
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
- Target max absolute error: observation `4.131533 m`, residual `4.054890 m`.
- Range RMSE mean: observation `0.912856 m`, residual `0.916160 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.504185 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.623977 m` to `3.883755 m`.
- Worst residual row: `intrinsic_scale_stress` seed `298184`,
  `4.054890 m`.

## Interpretation

The ray-scale branch lowers the aggregate target max but worsens average range
and position RMSE, and it still fails every intrinsic-scale row. This is useful
as a contrast to the additive forward-over-disparity duplicate, but it is not a
standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: corrected stereo-center RDF raw forward divided by fused
  disparity, not public FRD `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF ray scale over
  `(right, down, forward)`.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
