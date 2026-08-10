# EXP-20260622-303 Common0p8 B9x5 Product Residual Fresh100

## Purpose

Test whether changing the common-fraction gate in the pooled residual profile
changes the strict-pass and aggregate-error behavior. The scout searched
common fractions `0.35`, `0.65`, and `0.8`; the best candidate used `0.8`.

## Method

- Base profile:
  `Simulation/reports/experiments/EXP-20260617-198-bbox-outlier-causal-image-prefilter/base_profile/recommended_runtime_profile.json`
- Fresh20 discovery:
  `EXP-SEARCH-20260622-common-fraction-product-residual-fresh20`
- Fresh100 replay:
  `EXP-SEARCH-20260622-common0p8-b9x5-product-residual-fresh100`
- Fusion shell: b9x5 trimmed-mean / trimmed-mean, trim `0.05`,
  common fraction `0.8`
- Residual feature: `measured_disparity_px * raw_forward_m`
- Correction: `ray_scale`, degree `1`, ridge `10`, clip `0.05`
- Fresh100 seeds: `292300..292399`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Trials: `100` per scenario, `600` total

## Result

Status: `STRICT_PASS`

Best method:

`multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b9x5_trim0p05_common0p8_residual_ray_scale_disparity_forward_product_deg1_ridge10_clip0p05_pooled`

The fresh20 scout passed `120/120`, and the single-candidate fresh100 replay
passed `600/600`.

Changing common fraction to `0.8` did not solve the aggregate regression:
target max-abs error rose from `5.436126 m` observation to `6.179476 m`
residual, range RMSE mean rose from `1.012408 m` to `1.158251 m`, and position
RMSE mean rose from `2.546519 m` to `2.597030 m`.

## Coordinate Contract

- Feature inputs are `measured_disparity_px` and `raw_forward_m`.
- `raw_forward_m` is stereo-center OpenCV optical RDF forward.
- Runtime correction output remains stereo-center RDF ray scale.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is calibration/evaluation-only and is not a runtime input.
