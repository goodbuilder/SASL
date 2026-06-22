# EXP-20260622-360 B15x5 Common0.2 Oversample48 Runtime Guard Selector Fresh100

## Purpose

Replace the EXP348 guard branch with the common0.2 oversample48 product fallback
from EXP359 and check whether the selector improves aggregate metrics while
retaining strict pass.

## Method

- Seeds: `298100..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Guard: use common0.2 oversample48 product fallback when
  `raw_forward_mean_m > 1771.0 m`
- Non-guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b15x5_trim0p1_common0p3_residual_forward_add_log_disparity_deg3_ridge100_clip0p1_pooled`
- Guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b15x5_trim0p1_common0p2_residual_ray_scale_disparity_forward_product_deg1_ridge300_clip0p02_pooled`
- Guard branch target oversample count: `48`

## Result

Status: `STRICT_PASS` on fresh100.

The selector passed `600/600`:

- Target max-abs: `4.732347 m` observation to `4.632350 m` selected residual
- Range RMSE mean: `0.932889 m` observation to `0.815233 m` selected residual
- Position RMSE mean: `2.496001 m` observation to `2.391193 m` selected residual
- Branch counts: guard `100`, non-guard `500`

This slightly improves range and position aggregate means versus the common0.3
oversample48 guard branch in EXP348, while preserving strict pass.

## Coordinate Contract

- Guard feature `raw_forward_mean_m` is stereo-center OpenCV optical RDF
  forward after base profile correction and b15x5 common0.3 observation fusion.
- The guard feature is computed over the trailing 520-sample runtime
  measurement window; scenario labels and truth are not runtime selector
  inputs.
- Non-guard branch uses `log(measured_disparity_px)` and outputs stereo-center
  RDF forward-add correction.
- Guard branch uses `measured_disparity_px * raw_forward_m` after b15x5
  common0.2 fusion and outputs stereo-center RDF ray-scale correction.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
