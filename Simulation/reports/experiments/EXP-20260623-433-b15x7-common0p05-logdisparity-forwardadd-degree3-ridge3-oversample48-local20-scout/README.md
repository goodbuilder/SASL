# EXP-20260623-433 b15x7 common0.05 log_disparity forward_add degree3 ridge3 scout

## Purpose

Check whether increasing the common0.05 log-disparity additive polynomial from
degree 2 to degree 3 yields meaningful additional improvement.

## Method

- Method name: `b15x7_common0p05_logdisparity_forwardadd_degree3_ridge3_oversample48_local20_scout`
- Runtime inputs: log-scaled fused measured disparity after the base runtime
  profile.
- Calibration-only inputs: train split truth residuals used to fit the pooled
  degree-3 ridge model.
- Output contract: add the fitted residual to stereo-center OpenCV optical RDF
  `forward` before exporting public `FRD+range(forward,right,down,range)`.
- Pass/fail criteria: all 120 local scout rows over seeds `298180..298199` must
  pass the existing runtime deployment gate.

## Result

`FAIL`, with `100/120` passing rows. All 20 failures were
`intrinsic_scale_stress`.

- Target max absolute error: observation `4.131533 m`, residual `4.111533 m`.
- Range RMSE mean: observation `0.912856 m`, residual `0.906195 m`.
- Position RMSE mean: observation `2.486601 m`, residual `2.484004 m`.
- Tracked intrinsic seed `298188`: target max absolute error regressed from
  `3.623977 m` to `3.643976 m`.

## Interpretation

Degree 3 is nearly saturated against degree 2: target max, pass count, and
tracked intrinsic-tail behavior are unchanged, while aggregate RMSE improves
only slightly. Do not promote this as a standalone deployment candidate.

## Coordinate Contract Review

- Internal frame: stereo-center OpenCV optical RDF
  `(right, down, forward)`.
- Feature axis: log-scaled fused measured disparity scalar, not an `x/y/z`
  alias.
- Correction axis: stereo-center RDF `forward` only.
- Public output: `FRD+range(forward,right,down,range)`.
- Truth usage: calibration/evaluation only; no runtime ground-truth dependency.
