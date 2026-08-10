# EXP-20260622-301 B11x5 Product Residual Fresh100

## Purpose

Test a larger b11x5 observation aggregation shell for the runtime-safe product
residual. This explores whether increasing the number of bursts and per-burst
observations changes the strict-pass boundary or the aggregate RMSE tradeoff.

## Method

- Base profile:
  `Simulation/reports/experiments/EXP-20260617-198-bbox-outlier-causal-image-prefilter/base_profile/recommended_runtime_profile.json`
- Fresh30 discovery:
  `EXP-SEARCH-20260622-b11-product-residual-fresh30`
- Fresh100 replay:
  `EXP-SEARCH-20260622-b11x5-product-residual-fresh100`
- Fusion shell: b11x5 trimmed-mean / trimmed-mean, trim `0.1`,
  common fraction `0.5`
- Residual feature: `measured_disparity_px * raw_forward_m`
- Correction: `ray_scale`, degree `1`, ridge `100`, clip `0.02`
- Fresh100 seeds: `291700..291799`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Trials: `100` per scenario, `600` total

## Result

Status: `STRICT_PASS`

Best method:

`multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b11x5_trim0p1_common0p5_residual_ray_scale_disparity_forward_product_deg1_ridge100_clip0p02_pooled`

The fresh30 search passed `180/180`, and the single-candidate fresh100 replay
passed `600/600`.

This result should not be treated as a terminal success. The larger aggregation
shell again regressed aggregate metrics: target max-abs error rose from
`5.539617 m` observation to `6.281258 m` residual, range RMSE mean rose from
`0.962747 m` to `1.132220 m`, and position RMSE mean rose from `2.514229 m` to
`2.580054 m`. Later selection work should explicitly balance strict gate
behavior against these aggregate regressions.

## Coordinate Contract

- Feature inputs are `measured_disparity_px` and `raw_forward_m`.
- `raw_forward_m` is stereo-center OpenCV optical RDF forward.
- Runtime correction output remains stereo-center RDF ray scale.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is calibration/evaluation-only and is not a runtime input.
