# EXP-20260622-321 B9x5 Runtime Guard Selector Fresh100

## Purpose

Expand EXP320's b9x5 runtime-guard selector from fresh50 to fresh100. The
selector uses the runtime `raw_forward_mean_m` guard to switch between a b9x5
log-disparity forward-add branch and a b9x5 product ray-scale fallback branch.

## Method

- Seeds: `295500..295599`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Guard: use product fallback when `raw_forward_mean_m > 1771.0 m`
- Non-guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b9x5_trim0p05_common0p5_residual_forward_add_log_disparity_deg3_ridge100_clip0p1_pooled`
- Guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b9x5_trim0p05_common0p5_residual_ray_scale_disparity_forward_product_deg1_ridge10_clip0p05_pooled`

## Result

Status: `STRICT_PASS` on fresh100.

The b9x5 runtime-guard selector passed `600/600` and improved all tracked
observation-relative aggregate metrics:

- Target max-abs: `5.489495 m` observation to `5.389497 m` selected residual
- Range RMSE mean: `0.987098 m` observation to `0.870405 m` selected residual
- Position RMSE mean: `2.533194 m` observation to `2.428333 m` selected residual

The b9x5 forward-add branch alone passed `500/600` but improved aggregate
metrics. The product fallback branch alone passed `600/600` but regressed
aggregate metrics. The guard selector again combines strict pass behavior with
aggregate improvement.

This is strong simulation evidence, not a stopping condition. Continue with
additional stress variants and packaging/tests.

## Coordinate Contract

- Guard feature `raw_forward_mean_m` is stereo-center OpenCV optical RDF
  forward averaged over the default runtime measurement window.
- Forward-add branch uses `log(measured_disparity_px)` and outputs
  stereo-center RDF forward-add correction.
- Product fallback branch uses `measured_disparity_px * raw_forward_m` and
  outputs stereo-center RDF ray-scale correction.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is calibration/evaluation-only and is not a runtime input.
