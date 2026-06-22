# EXP-20260622-302 Median B9x5 Product Residual Fresh100

## Purpose

Test a median inner aggregation shell for the runtime-safe product residual.
This changes the observation fusion method from trimmed-mean inner aggregation
to median inner aggregation while keeping b9x5 and the same product-feature
ray-scale residual family.

## Method

- Base profile:
  `Simulation/reports/experiments/EXP-20260617-198-bbox-outlier-causal-image-prefilter/base_profile/recommended_runtime_profile.json`
- Fresh30 discovery:
  `EXP-SEARCH-20260622-burst-median-product-ratio-fresh30`
- Fresh100 replay:
  `EXP-SEARCH-20260622-median-b9x5-product-residual-fresh100`
- Fusion shell: b9x5 median / trimmed-mean, trim `0.05`,
  common fraction `0.5`
- Residual feature: `measured_disparity_px * raw_forward_m`
- Correction: `ray_scale`, degree `1`, ridge `10`, clip `0.05`
- Fresh100 seeds: `291900..291999`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Trials: `100` per scenario, `600` total

## Result

Status: `STRICT_PASS`

Best method:

`multiburst_observation_inner_median_outer_trimmed_mean_b9x5_trim0p05_common0p5_residual_ray_scale_disparity_forward_product_deg1_ridge10_clip0p05_pooled`

The fresh30 search passed `180/180`, and the single-candidate fresh100 replay
passed `600/600`.

This is another candidate-family variant, not a stopping condition. Aggregate
metrics regressed again: target max-abs error rose from `5.613278 m`
observation to `6.324428 m` residual, range RMSE mean rose from `0.984837 m`
to `1.138284 m`, and position RMSE mean rose from `2.528447 m` to
`2.583810 m`.

## Coordinate Contract

- Feature inputs are `measured_disparity_px` and `raw_forward_m`.
- `raw_forward_m` is stereo-center OpenCV optical RDF forward.
- Runtime correction output remains stereo-center RDF ray scale.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is calibration/evaluation-only and is not a runtime input.
