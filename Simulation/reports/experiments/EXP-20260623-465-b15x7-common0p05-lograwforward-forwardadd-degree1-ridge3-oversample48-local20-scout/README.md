# EXP-20260623-465 b15x7 common0.05 log_raw_forward forward_add degree1 ridge3 scout

## Purpose

Replace the linear raw-forward feature with log-scaled raw forward while
keeping the forward-add correction. The goal is to test whether compressing
forward-distance variation keeps the low-regression behavior while reducing
intrinsic-tail sensitivity.

## Method

- Method name: `b15x7_common0p05_lograwforward_forwardadd_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: log-scaled corrected stereo-center OpenCV optical RDF raw
  forward scalar.
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
- Target max absolute error: observation `4.131533 m`, residual `4.122734 m`.
- Range RMSE mean: observation `0.912856 m`, residual `0.907625 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.484664 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.623977 m` to `3.636818 m`.
- Worst residual row: `disparity_noise_stress` seed `298189`,
  `4.122734 m`.

## Interpretation

Log raw forward behaves like a softened raw-forward additive correction. It is
less aggressive than linear raw forward and does not recover intrinsic-scale
rows. This is not a standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log of corrected stereo-center RDF raw forward, not public FRD
  `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF forward additive correction.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
