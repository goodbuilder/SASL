# EXP-20260623-453 b15x7 common0.05 raw_forward ray_scale degree1 ridge3 scout

## Purpose

Test corrected stereo-center RDF raw forward as a degree-1 ray-scale residual
feature. The goal is to see whether a geometric distance proxy can improve
aggregate target error without the high aggregate blow-up seen in the
log-disparity-forward-product interaction branch.

## Method

- Method name: `b15x7_common0p05_rawforward_rayscale_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: corrected stereo-center OpenCV optical RDF raw forward scalar.
- Calibration-only inputs: train split truth residuals used to fit the pooled
  degree-1 ridge model.
- Output contract: apply the fitted residual as a stereo-center OpenCV optical
  RDF ray scale before exporting public `FRD+range(forward,right,down,range)`.
- Pass/fail criteria: all 120 local scout rows over seeds `298180..298199` must
  pass the existing runtime deployment gate.

## Result

`FAIL`, with `100/120` passing rows.

- Failure distribution: all 20 failures are `intrinsic_scale_stress`.
- Target max absolute error: observation `4.131533 m`, residual `3.985144 m`.
- Range RMSE mean: observation `0.912856 m`, residual `0.913125 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.499152 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.623977 m` to `3.799632 m`.
- Worst residual row: `disparity_noise_stress` seed `298189`,
  `3.985144 m`.

## Interpretation

Raw-forward ray scale is the strongest aggregate-stable contrast branch in this
group: it lowers target max and barely moves range RMSE. It still fails every
intrinsic-scale row and worsens intrinsic seed `298188`, so it is not a
standalone deployment candidate. Keep it as a low-regression contrast branch
for future gated or blended ideas.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: corrected stereo-center RDF raw forward, not public FRD
  `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF ray scale over right/down/forward.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
