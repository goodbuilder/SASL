# EXP-20260623-492 b21x5 common0.02 log_disparity_forward_product ray_scale degree1 ridge3 scout

## Purpose

Test the log-disparity-forward product feature with whole-ray scaling under the
b21x5 common `0.02` geometry. The goal is to check whether the product feature
can reproduce the intrinsic relief seen in earlier ray-scale branches without
hiding aggregate regressions behind gate status.

## Method

- Method name: `b21x5_common0p02_logdisparityforwardproduct_rayscale_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: product of log fused measured disparity and corrected
  stereo-center OpenCV optical RDF raw forward.
- Calibration-only inputs: train split truth residuals used to fit the pooled
  degree-1 ridge model.
- Output contract: apply the fitted residual as a stereo-center OpenCV optical
  RDF ray-scale correction before exporting public
  `FRD+range(forward,right,down,range)`.
- Pass/fail criteria: all 120 local scout rows over seeds `298180..298199` must
  pass the existing runtime deployment gate.

## Result

`STRICT_PASS`, with `120/120` passing rows.

- Failure distribution: none under the current runtime deployment gate.
- Target max absolute error: observation `4.076885 m`, residual `4.793879 m`.
- Range RMSE mean: observation `0.908840 m`, residual `1.103529 m`.
- Position RMSE mean: observation `2.484420 m`, residual `2.561510 m`.
- Intrinsic seed `298188`: target max absolute error improved from
  `3.587690 m` to `2.442553 m`.
- Worst residual row: `disparity_noise_stress` seed `298191`,
  `4.793879 m`.

## Interpretation

This is a gate pass but not a deployment candidate. It greatly improves a
selected intrinsic row while worsening aggregate target max and average RMSE.
Treat the result as evidence that the current gate can miss aggregate blow-up,
not as a stopping point or final correction method.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: product of log fused disparity and corrected stereo-center RDF
  raw forward, not public FRD `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF ray scale over
  `(right, down, forward)`.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
