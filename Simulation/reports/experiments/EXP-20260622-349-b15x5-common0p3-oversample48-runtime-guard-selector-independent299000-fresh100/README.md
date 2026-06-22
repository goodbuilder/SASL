# EXP-20260622-349 B15x5 Common0.3 Oversample48 Guard Selector Independent299000 Fresh100

## Purpose

Validate the EXP348 oversample48 runtime guard selector on an independent
fresh100 seed block.

## Method

- Seeds: `299000..299099`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Guard: use oversample48 product fallback when
  `raw_forward_mean_m > 1771.0 m`
- Non-guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b15x5_trim0p1_common0p3_residual_forward_add_log_disparity_deg3_ridge100_clip0p1_pooled`
- Guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b15x5_trim0p1_common0p3_residual_ray_scale_disparity_forward_product_deg1_ridge300_clip0p02_pooled`
- Guard branch target oversample count: `48`

## Result

Status: `STRICT_PASS` on independent fresh100.

The selector passed `600/600`:

- Target max-abs: `5.158900 m` observation to `5.058902 m` selected residual
- Range RMSE mean: `0.932607 m` observation to `0.815013 m` selected residual
- Position RMSE mean: `2.493837 m` observation to `2.389088 m` selected residual
- Branch counts: guard `100`, non-guard `500`

This independently reproduces the EXP348 selector behavior. It remains a
validation milestone, not a terminal search success.

## Coordinate Contract

- Guard feature `raw_forward_mean_m` is stereo-center OpenCV optical RDF
  forward after base profile correction and b15x5 common0.3 observation fusion.
- The guard feature is computed over the trailing 520-sample runtime
  measurement window; scenario labels and truth are not runtime selector
  inputs.
- Non-guard branch uses `log(measured_disparity_px)` and outputs stereo-center
  RDF forward-add correction.
- Guard branch uses `measured_disparity_px * raw_forward_m` and outputs
  stereo-center RDF ray-scale correction.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
