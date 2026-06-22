# EXP-20260622-300 B9x5 Product Residual Fresh100

## Purpose

Test whether the promoted product-feature residual remains strict-pass when the
observation fusion shell is changed from b7x3 to b9x5. This is a new
aggregation-structure experiment, not just another seed tail for the existing
b7x3 method.

## Method

- Base profile:
  `Simulation/reports/experiments/EXP-20260617-198-bbox-outlier-causal-image-prefilter/base_profile/recommended_runtime_profile.json`
- Fresh30 discovery:
  `EXP-SEARCH-20260622-burst-structure-product-residual-fresh30`
- Fresh100 replay:
  `EXP-SEARCH-20260622-b9x5-product-residual-fresh100`
- Fusion shell: b9x5 trimmed-mean / trimmed-mean, trim `0.05`,
  common fraction `0.5`
- Residual feature: `measured_disparity_px * raw_forward_m`
- Correction: `ray_scale`, degree `1`, ridge `10`, clip `0.05`
- Seeds: `291100..291199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Trials: `100` per scenario, `600` total

## Result

Status: `STRICT_PASS`

Best method:

`multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b9x5_trim0p05_common0p5_residual_ray_scale_disparity_forward_product_deg1_ridge10_clip0p05_pooled`

The method passed `600/600` fresh100 trials. This result broadens the product
residual family from b7x3 to b9x5, but it is not a terminal success criterion:
new correction methods should continue to be generated and tested until the
operator explicitly stops the search.

Important caveat: strict pass did not mean every aggregate metric improved.
Target max-abs error rose from `5.179764 m` observation to `5.891166 m`
residual, range RMSE mean rose from `0.985318 m` to `1.139599 m`, and position
RMSE mean rose from `2.529238 m` to `2.584590 m`. Treat this as evidence that
the strict gate and mean-error tradeoffs need to remain visible in later
selection work.

## Coordinate Contract

- Feature inputs are `measured_disparity_px` and `raw_forward_m`.
- `raw_forward_m` is stereo-center OpenCV optical RDF forward.
- Runtime correction output remains stereo-center RDF ray scale.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is calibration/evaluation-only and is not a runtime input.
