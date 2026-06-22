# EXP-20260623-484 b21x5 common0.02 raw_forward ray_scale degree1 ridge3 scout

## Purpose

Switch the b21x5 common `0.02` raw-forward branch from forward additive
correction to whole-ray scaling. The goal is to test whether the strong EXP479
baseline can lower target-tail errors further by scaling the full stereo-center
RDF ray instead of only adding to forward.

## Method

- Method name: `b21x5_common0p02_rawforward_rayscale_degree1_ridge3_oversample48_local20_scout`
- Runtime inputs: corrected stereo-center OpenCV optical RDF raw forward
  scalar.
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
- Target max absolute error: observation `4.076885 m`, residual `3.927041 m`.
- Range RMSE mean: observation `0.908840 m`, residual `0.910064 m`.
- Position RMSE mean: observation `2.484420 m`, residual `2.497484 m`.
- Intrinsic seed `298188`: target max absolute error regressed from
  `3.587690 m` to `3.735512 m`.
- Worst residual row: `disparity_noise_stress` seed `298191`,
  `3.927041 m`.

## Interpretation

Ray scaling gives the strongest target-tail reduction in this b21x5 common
`0.02` branch, but it worsens average range and position RMSE and still fails
every intrinsic-scale row. Keep this as a target-tail contrast, not a deployment
candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: corrected stereo-center RDF raw forward, not public FRD
  `forward` and not an ambiguous `x/y/z` alias.
- Correction axis: stereo-center RDF ray scale over
  `(right, down, forward)`.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
