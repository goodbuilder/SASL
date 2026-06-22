# EXP-20260622-336 B15x5 Common0.3 Runtime Guard Selector Fresh100

## Purpose

Validate the b15x5 common0.3 runtime guard selector on an independent fresh100
seed block after its fresh50 aggregate improvement.

## Method

- Seeds: `298100..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Guard: use product fallback when `raw_forward_mean_m > 1771.0 m`
- Non-guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b15x5_trim0p1_common0p3_residual_forward_add_log_disparity_deg3_ridge100_clip0p1_pooled`
- Guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b15x5_trim0p1_common0p3_residual_ray_scale_disparity_forward_product_deg1_ridge100_clip0p02_pooled`

The runtime guard feature was regenerated from the b15x5 common0.3 fused
observation stream after base profile correction, using the trailing
`RuntimeFeatureWindow(max_samples=520)`.

## Result

Status: `FAIL` on fresh100.

The common0.3 selector passed `599/600`. It retained strong aggregate
improvement but failed strict pass:

- Target max-abs: `4.732347 m` observation to `4.632350 m` selected residual
- Range RMSE mean: `0.933270 m` observation to `0.815703 m` selected residual
- Position RMSE mean: `2.496382 m` observation to `2.391662 m` selected residual

Branch results:

- Forward-add branch: `500/600 FAIL`, aggregate-improving
- Product fallback branch: `599/600 FAIL`, aggregate-regressing
- Selector: `599/600 FAIL`, guard branch `100`, non-guard branch `500`

This is useful but not promotable under the current strict-pass criterion.
Treat common0.3 as a high-aggregate, strict-fail variant and continue searching.

## Coordinate Contract

- Guard feature `raw_forward_mean_m` is stereo-center OpenCV optical RDF
  forward after base profile correction and b15x5 common0.3 observation fusion.
- The guard feature is computed over the trailing 520-sample runtime
  measurement window; scenario labels and truth are not runtime selector
  inputs.
- Forward-add branch uses `log(measured_disparity_px)` and outputs
  stereo-center RDF forward-add correction.
- Product fallback branch uses `measured_disparity_px * raw_forward_m` and
  outputs stereo-center RDF ray-scale correction.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
