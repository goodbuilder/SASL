# EXP-20260623-476 b21x5 common0.05 log_raw_forward forward_add degree1 ridge3 scout

## Purpose

Apply the log-raw-forward additive branch to the `b21x5` burst geometry. The
goal is to test whether the geometry that helped raw-forward additive RMSE can
also keep a compressed forward-depth feature from increasing the target tail.

## Method

- Method name: `b21x5_common0p05_lograwforward_forwardadd_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: log of corrected stereo-center OpenCV optical RDF raw
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
- Target max absolute error: observation `4.219042 m`, residual `4.207238 m`.
- Range RMSE mean: observation `0.911104 m`, residual `0.905883 m`.
- Position RMSE mean: observation `2.485454 m`, residual `2.483518 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.603210 m` to `3.615854 m`.
- Worst residual row: `disparity_noise_stress` seed `298188`,
  `4.207238 m`.

## Interpretation

The b21x5 log-forward additive branch improves aggregate target max and average
RMSE, but the intrinsic-scale gate still fails and a selected intrinsic row
regresses. Keep this as a promising low-regression branch, not a deployment
candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log of corrected stereo-center RDF raw forward, not public FRD
  `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF forward additive correction.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
