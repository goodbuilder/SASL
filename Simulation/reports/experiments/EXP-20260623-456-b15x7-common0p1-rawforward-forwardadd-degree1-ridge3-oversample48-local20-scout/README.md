# EXP-20260623-456 b15x7 common0.1 raw_forward forward_add degree1 ridge3 scout

## Purpose

Increase the low-regression raw-forward forward-add branch common fraction from
`0.05` to `0.1` to test whether stricter common sampling improves intrinsic
behavior or aggregate RMSE.

## Method

- Method name: `b15x7_common0p1_rawforward_forwardadd_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: corrected stereo-center OpenCV optical RDF raw forward scalar.
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
- Target max absolute error: observation `4.348758 m`, residual `4.328759 m`.
- Range RMSE mean: observation `0.916652 m`, residual `0.910094 m`.
- Position RMSE mean: observation `2.488401 m`, residual `2.485815 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.679468 m` to `3.699468 m`.
- Worst residual row: `disparity_noise_stress` seed `298188`,
  `4.328759 m`.

## Interpretation

Common `0.1` preserves the RMSE-improving character of raw-forward forward-add
but worsens target max versus common `0.05` and still fails every intrinsic
row. This is not a standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: corrected stereo-center RDF raw forward, not public FRD
  `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF forward additive correction.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
