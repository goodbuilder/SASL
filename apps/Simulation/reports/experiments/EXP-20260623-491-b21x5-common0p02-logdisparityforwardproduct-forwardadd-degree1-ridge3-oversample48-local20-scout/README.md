# EXP-20260623-491 b21x5 common0.02 log_disparity_forward_product forward_add degree1 ridge3 scout

## Purpose

Test the log-disparity-forward product feature with b21x5 common `0.02`
forward-add correction. The goal is to see whether combining disparity and raw
forward information gives the selected intrinsic relief seen in inverse-depth
features without the same aggregate regression.

## Method

- Method name: `b21x5_common0p02_logdisparityforwardproduct_forwardadd_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: product of log fused measured disparity and corrected
  stereo-center OpenCV optical RDF raw forward.
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
- Target max absolute error: observation `4.076885 m`, residual `4.096885 m`.
- Range RMSE mean: observation `0.908840 m`, residual `0.909727 m`.
- Position RMSE mean: observation `2.484420 m`, residual `2.484721 m`.
- Intrinsic seed `298188`: target max absolute error improved from
  `3.587690 m` to `3.567690 m`.
- Worst residual row: `disparity_noise_stress` seed `298191`,
  `4.096885 m`.

## Interpretation

The product feature gives selected intrinsic relief but regresses aggregate
target max and average RMSE. It is not better than the raw-forward baseline and
is not a deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: product of log fused disparity and corrected stereo-center RDF
  raw forward, not public FRD `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF forward additive correction.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
