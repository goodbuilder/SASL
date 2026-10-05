# EXP-20260623-466 b15x7 common0.05 forward_over_disparity forward_add degree1 ridge3 scout

## Purpose

Test a ratio feature, corrected stereo-center RDF raw forward divided by fused
measured disparity, with the forward-add correction. The goal is to see whether
a disparity-normalized forward proxy changes the intrinsic-scale failure mode
relative to raw-forward alone.

## Method

- Method name: `b15x7_common0p05_forwardoverdisparity_forwardadd_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: corrected stereo-center OpenCV optical RDF raw forward scalar
  divided by fused measured disparity.
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
- Target max absolute error: observation `4.131533 m`, residual `4.111533 m`.
- Range RMSE mean: observation `0.912856 m`, residual `0.906159 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.483979 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.623977 m` to `3.643976 m`.
- Worst residual row: `disparity_noise_stress` seed `298189`,
  `4.111533 m`.

## Interpretation

Forward-over-disparity effectively duplicates the linear raw-forward
forward-add branch at scout precision. It improves RMSE but fails every
intrinsic-scale row. This is not a standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: corrected stereo-center RDF raw forward divided by fused
  measured disparity, not public FRD `forward` and not an ambiguous `x/y/z`
  alias.
- Correction axis: stereo-center RDF forward additive correction.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
