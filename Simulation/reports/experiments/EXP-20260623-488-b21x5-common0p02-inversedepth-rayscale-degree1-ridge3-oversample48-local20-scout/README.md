# EXP-20260623-488 b21x5 common0.02 inverse_depth ray_scale degree1 ridge3 scout

## Purpose

Move the inverse-depth feature from forward additive correction to whole-ray
scaling. The goal is to test whether the selected intrinsic relief seen in
EXP487 can be amplified without unacceptable aggregate regression.

## Method

- Method name: `b21x5_common0p02_inversedepth_rayscale_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: inverse depth derived from the corrected stereo-center
  OpenCV optical RDF forward scalar.
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
- Target max absolute error: observation `4.076885 m`, residual `4.204353 m`.
- Range RMSE mean: observation `0.908840 m`, residual `0.944352 m`.
- Position RMSE mean: observation `2.484420 m`, residual `2.507927 m`.
- Intrinsic seed `298188`: target max absolute error improved from
  `3.587690 m` to `3.459413 m`.
- Worst residual row: `disparity_noise_stress` seed `298191`,
  `4.204353 m`.

## Interpretation

Inverse-depth ray scaling gives stronger selected intrinsic relief than
forward-add inverse depth, but it causes a large aggregate target and RMSE
regression. Reject it as a standalone deployment path and keep it only as an
intrinsic-relief contrast.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: inverse depth from corrected stereo-center RDF raw forward, not
  public FRD `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF ray scale over
  `(right, down, forward)`.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
