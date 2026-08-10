# EXP-20260622-305 Log-Disparity Aggregate Improve Boundary

## Purpose

Follow up on the repeated pattern where product-feature residuals pass the
strict raw-baseline gate but regress against the fused observation aggregate
metrics. This scout prioritizes log-disparity residuals that can improve target
max-abs and range RMSE relative to the observation fusion.

## Method

- Base profile:
  `Simulation/reports/experiments/EXP-20260617-198-bbox-outlier-causal-image-prefilter/base_profile/recommended_runtime_profile.json`
- Scout:
  `EXP-SEARCH-20260622-aggregate-improve-disparity-residual-fresh20`
- Fusion shells: b7x3 and b9x5 trimmed-mean / trimmed-mean, trim `0.05`,
  common fraction `0.5`
- Residual features: `disparity`, `log_disparity`
- Correction: `ray_scale`, degrees `1` and `2`
- Seeds: `292900..292919`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Trials: `20` per scenario, `120` total

## Result

Status: `FAIL` under the strict raw-baseline gate, but useful aggregate
boundary evidence.

Best method by the existing script ordering:

`multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b9x5_trim0p05_common0p5_residual_ray_scale_log_disparity_deg2_ridge0_clip0p25_pooled`

It passed `100/120`, not `120/120`. Unlike the product variants, it improved
two observation-relative aggregate metrics:

- Target max-abs: `5.278599 m` observation to `5.094577 m` residual
- Range RMSE mean: `0.987246 m` observation to `0.977568 m` residual
- Position RMSE mean regressed slightly: `2.527364 m` to `2.533482 m`

No method in this scout improved target max-abs, range mean, and position mean
simultaneously. `top_target_range_candidates.csv` records the top 40 candidates
that improved target and range together.

## Coordinate Contract

- Feature inputs are `measured_disparity_px` or `log(measured_disparity_px)`.
- Runtime correction output remains stereo-center RDF ray scale.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is calibration/evaluation-only and is not a runtime input.
