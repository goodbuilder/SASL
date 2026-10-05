# EXP-20260622-323 B11x5 Runtime Guard Selector Fresh50

## Purpose

Try a larger b11x5 runtime-guard selector. This keeps the runtime
`raw_forward_mean_m` intrinsic guard and replaces the selector branches with
b11x5 log-disparity forward-add and b11x5 product ray-scale fallback.

## Method

- Seeds: `295700..295749`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Guard: use product fallback when `raw_forward_mean_m > 1771.0 m`
- Non-guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b11x5_trim0p1_common0p5_residual_forward_add_log_disparity_deg3_ridge100_clip0p1_pooled`
- Guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b11x5_trim0p1_common0p5_residual_ray_scale_disparity_forward_product_deg1_ridge100_clip0p02_pooled`

## Result

Status: `STRICT_PASS` on fresh50.

The b11x5 runtime-guard selector passed `300/300` and improved all tracked
observation-relative aggregate metrics:

- Target max-abs: `5.267452 m` observation to `5.167452 m` selected residual
- Range RMSE mean: `0.958913 m` observation to `0.842607 m` selected residual
- Position RMSE mean: `2.510560 m` observation to `2.406343 m` selected residual

The b11x5 forward-add branch alone passed `250/300` but improved aggregate
metrics. The product fallback branch alone passed `300/300` but regressed
aggregate metrics. The guard selector combines strict pass behavior with
aggregate improvement.

This is a new selector variant, not a terminal success. It needs fresh100
validation and packaging/tests before promotion.

## Coordinate Contract

- Guard feature `raw_forward_mean_m` is stereo-center OpenCV optical RDF
  forward averaged over the default runtime measurement window.
- Forward-add branch uses `log(measured_disparity_px)` and outputs
  stereo-center RDF forward-add correction.
- Product fallback branch uses `measured_disparity_px * raw_forward_m` and
  outputs stereo-center RDF ray-scale correction.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is calibration/evaluation-only and is not a runtime input.
