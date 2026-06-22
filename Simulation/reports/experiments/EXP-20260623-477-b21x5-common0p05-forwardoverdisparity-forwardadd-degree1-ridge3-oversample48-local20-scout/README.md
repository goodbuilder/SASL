# EXP-20260623-477 b21x5 common0.05 forward_over_disparity forward_add degree1 ridge3 scout

## Purpose

Apply the forward-over-disparity additive branch to the `b21x5` burst geometry.
The goal is to test whether the ratio feature remains a raw-forward duplicate
under a burst layout that improves average RMSE.

## Method

- Method name: `b21x5_common0p05_forwardoverdisparity_forwardadd_degree1_ridge3_oversample48_local20_scout`
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
- Target max absolute error: observation `4.219042 m`, residual `4.199042 m`.
- Range RMSE mean: observation `0.911104 m`, residual `0.904404 m`.
- Position RMSE mean: observation `2.485454 m`, residual `2.482839 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.603210 m` to `3.623209 m`.
- Worst residual row: `disparity_noise_stress` seed `298188`,
  `4.199042 m`.

## Interpretation

The b21x5 forward-over-disparity additive branch improves aggregate target max
and average RMSE, but it still fails every intrinsic-scale row and regresses
the selected intrinsic target max. Keep it as a low-regression branch, not a
deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: corrected stereo-center RDF raw forward divided by fused
  disparity, not public FRD `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF forward additive correction.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
