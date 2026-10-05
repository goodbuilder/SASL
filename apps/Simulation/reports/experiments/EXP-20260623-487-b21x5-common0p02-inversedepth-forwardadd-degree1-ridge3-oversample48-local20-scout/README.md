# EXP-20260623-487 b21x5 common0.02 inverse_depth forward_add degree1 ridge3 scout

## Purpose

Switch the b21x5 common `0.02` additive branch from raw forward to inverse
depth. The goal is to test whether a normalized depth feature can relieve
intrinsic-scale rows that raw-forward additive keeps failing.

## Method

- Method name: `b21x5_common0p02_inversedepth_forwardadd_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: inverse depth derived from the corrected stereo-center
  OpenCV optical RDF forward scalar.
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
- Range RMSE mean: observation `0.908840 m`, residual `0.910405 m`.
- Position RMSE mean: observation `2.484420 m`, residual `2.485551 m`.
- Intrinsic seed `298188`: target max absolute error improved from
  `3.587690 m` to `3.567690 m`.
- Worst residual row: `disparity_noise_stress` seed `298191`,
  `4.096885 m`.

## Interpretation

Inverse depth gives selected intrinsic relief but worsens aggregate target max
and average RMSE, and the intrinsic gate still fails. Reject it as a standalone
deployment path.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: inverse depth from corrected stereo-center RDF raw forward, not
  public FRD `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF forward additive correction.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
