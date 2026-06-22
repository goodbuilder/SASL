# EXP-20260622-299 Ratio Feature Residual Fail

## Purpose

Try new runtime-safe scalar residual features that combine stereo-center RDF
forward with measured disparity as ratios:

- `forward_over_disparity = raw_forward_m / measured_disparity_px`
- `disparity_over_forward = measured_disparity_px / raw_forward_m`

This keeps the existing one-dimensional runtime residual profile schema while
testing a different physical feature family from the promoted product residual.

## Method

- Base profile:
  `Simulation/reports/experiments/EXP-20260617-198-bbox-outlier-causal-image-prefilter/base_profile/recommended_runtime_profile.json`
- Fusion shell: b7x3 trimmed-mean / trimmed-mean, trim `0.05`,
  common fraction `0.5`
- Seeds: `290700..290729`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Trials: `30` per scenario, `180` total

## Result

Status: `FAIL`

Best method:

`multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5_residual_forward_add_forward_over_disparity_deg1_ridge10_clip0p1_pooled`

The best method passed `150/180`. Mean range RMSE improved from
`1.017804 m` observation to `1.001663 m` residual, and target max-abs improved
from `5.384175 m` to `5.306922 m`, but the strict per-seed gate failed.

## Coordinate Contract

- Feature inputs are `measured_disparity_px` and `raw_forward_m`.
- `raw_forward_m` is stereo-center OpenCV optical RDF forward.
- Runtime correction output remains stereo-center RDF.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is calibration/evaluation-only and is not a runtime input.
