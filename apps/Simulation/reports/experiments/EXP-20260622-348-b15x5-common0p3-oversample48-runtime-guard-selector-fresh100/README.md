# EXP-20260622-348 B15x5 Common0.3 Oversample48 Runtime Guard Selector Fresh100

## Purpose

Combine the aggregate-improving b15x5 common0.3 forward-add branch with the
EXP347 strict-pass oversample48 product fallback only for high-forward guard
rows.

## Method

- Seeds: `298100..298199`
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

The runtime guard feature is the trailing `RuntimeFeatureWindow(max_samples=520)`
mean of stereo-center RDF forward after base profile correction and b15x5
common0.3 observation fusion.

## Result

Status: `STRICT_PASS` on fresh100.

The selector passed `600/600` while preserving the common0.3 aggregate
improvement:

- Target max-abs: `4.732347 m` observation to `4.632350 m` selected residual
- Range RMSE mean: `0.933151 m` observation to `0.815560 m` selected residual
- Position RMSE mean: `2.496262 m` observation to `2.391517 m` selected residual
- Branch counts: guard `100`, non-guard `500`

This is stronger than EXP336 because the oversample48 ridge300 product fallback
recovers the isolated seed `298188` intrinsic failure, while the selector avoids
using the globally-regressing product fallback outside guard rows.

This is still not a terminal search success. Continue exploring new correction
ideas and independent validations.

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
