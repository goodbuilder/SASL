# EXP-20260623-478 b21x5 common0.05 forward_over_disparity ray_scale degree1 ridge3 scout

## Purpose

Apply the forward-over-disparity ray-scale branch to the `b21x5` burst
geometry. The goal is to test whether b21x5 can keep the target-tail relief
seen in b15x7 ray scaling while reducing the RMSE penalty.

## Method

- Method name: `b21x5_common0p05_forwardoverdisparity_rayscale_degree1_ridge3_oversample48_local20_scout`
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
- Target max absolute error: observation `4.219042 m`, residual `4.056638 m`.
- Range RMSE mean: observation `0.911104 m`, residual `0.915198 m`.
- Position RMSE mean: observation `2.485454 m`, residual `2.503430 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.603210 m` to `3.862302 m`.
- Worst residual row: `intrinsic_scale_stress` seed `298187`,
  `4.056638 m`.

## Interpretation

The b21x5 ray-scale branch keeps strong target-tail relief but worsens average
range and position RMSE, and the intrinsic gate remains fully failed. Keep this
as a target-tail contrast only.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: corrected stereo-center RDF raw forward divided by fused
  disparity, not public FRD `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF ray scale over
  `(right, down, forward)`.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
