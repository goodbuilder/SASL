# EXP-20260623-421 b15x7 common0.1 disparity-forward-product forward_add scout

## Purpose

Test whether the product interaction that reached strict-pass in ray-scale form
can provide a better aggregate tradeoff when applied as a forward additive
residual.

## Method

- Method name: `b15x7_common0p1_disparityforwardproduct_forwardadd_oversample48_local20_scout`
- Runtime inputs: scalar product of fused measured disparity and corrected
  stereo-center RDF raw forward.
- Calibration-only inputs: train split truth residuals used to fit the pooled
  degree-1 ridge model.
- Output contract: add the fitted residual to stereo-center OpenCV optical RDF
  `forward` before exporting public `FRD+range(forward,right,down,range)`.
- Pass/fail criteria: all 120 local scout rows over seeds `298180..298199` must
  pass the existing runtime deployment gate.

## Result

`FAIL`, with `100/120` passing rows. All 20 failures were
`intrinsic_scale_stress`.

- Target max absolute error: observation `4.348758 m`, residual `4.368757 m`.
- Range RMSE mean: observation `0.916652 m`, residual `0.917345 m`.
- Position RMSE mean: observation `2.488401 m`, residual `2.488641 m`.
- Tracked intrinsic seed `298188`: target max absolute error improved from
  `3.679468 m` to `3.659469 m`, but the scenario still failed the gate.

## Interpretation

The product additive branch regresses aggregate RMSE less than the plain scalar
additive branches, but it still worsens aggregate RMSE and does not fix the
intrinsic-scale failures. Do not promote it as a standalone method.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: scalar product of fused measured disparity and stereo-center
  RDF raw forward, not an `x/y/z` alias.
- Correction axis: stereo-center RDF `forward` only.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
