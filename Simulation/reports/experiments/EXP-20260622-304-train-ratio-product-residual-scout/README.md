# EXP-20260622-304 Train Ratio Product Residual Scout

## Purpose

Probe calibration-time train-ratio sensitivity for the product residual family.
This changes how much calibration/evaluation truth is used for fitting versus
validation, while preserving the runtime input contract.

## Method

- Base profile:
  `Simulation/reports/experiments/EXP-20260617-198-bbox-outlier-causal-image-prefilter/base_profile/recommended_runtime_profile.json`
- Scouts:
  - `EXP-SEARCH-20260622-train-ratio-product-residual-fresh20-r035`
  - `EXP-SEARCH-20260622-train-ratio-product-residual-fresh20-r065`
- Fusion shell searched: b7x3 and b9x5 trimmed-mean / trimmed-mean,
  trim `0.05`, common fraction `0.5`
- Residual feature: `measured_disparity_px * raw_forward_m`
- Train ratios: `0.35` and `0.65`
- Seeds:
  - train ratio `0.35`: `292500..292519`
  - train ratio `0.65`: `292700..292719`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Trials: `20` per scenario per train-ratio scout

## Result

Status: `STRICT_PASS` scout only

Both train-ratio scouts selected:

`multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b9x5_trim0p05_common0p5_residual_ray_scale_disparity_forward_product_deg1_ridge100_clip0p02_pooled`

The `0.35` scout passed `120/120`; the `0.65` scout also passed `120/120`.

Aggregate regressions persisted:

- Train ratio `0.35`: target max-abs `5.088561 m` to `5.802600 m`;
  range RMSE mean `0.986631 m` to `1.141651 m`; position RMSE mean
  `2.531148 m` to `2.586573 m`.
- Train ratio `0.65`: target max-abs `4.898873 m` to `5.628896 m`;
  range RMSE mean `0.982524 m` to `1.137092 m`; position RMSE mean
  `2.524531 m` to `2.580028 m`.

This is not enough evidence for promotion. It records that changing the
calibration split alone did not remove the aggregate-error problem.

## Coordinate Contract

- Feature inputs are `measured_disparity_px` and `raw_forward_m`.
- `raw_forward_m` is stereo-center OpenCV optical RDF forward.
- Runtime correction output remains stereo-center RDF ray scale.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is calibration/evaluation-only and is not a runtime input.
