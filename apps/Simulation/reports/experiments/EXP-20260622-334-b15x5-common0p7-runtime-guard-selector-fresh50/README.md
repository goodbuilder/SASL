# EXP-20260622-334 B15x5 Common0.7 Runtime Guard Selector Fresh50

## Purpose

Try a higher common-error fraction inside the b15x5 runtime guard selector.
This keeps the b15x5 shell, trim0.1, and guard structure, but changes
`common_fraction` from `0.5` to `0.7`.

## Method

- Seeds: `297700..297749`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Guard: use product fallback when `raw_forward_mean_m > 1771.0 m`
- Non-guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b15x5_trim0p1_common0p7_residual_forward_add_log_disparity_deg3_ridge100_clip0p1_pooled`
- Guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b15x5_trim0p1_common0p7_residual_ray_scale_disparity_forward_product_deg1_ridge100_clip0p02_pooled`

The runtime guard feature was regenerated from the b15x5 common0.7 fused
observation stream after base profile correction, using the trailing
`RuntimeFeatureWindow(max_samples=520)`.

## Result

Status: `STRICT_PASS` on fresh50.

The common0.7 selector passed `300/300`. The guard selected product ray-scale
fallback for `50` intrinsic-like rows and log-disparity forward-add for `250`
non-guard rows.

Observation-relative aggregate metrics all improved:

- Target max-abs: `4.888533 m` observation to `4.788535 m` selected residual
- Range RMSE mean: `0.955876 m` observation to `0.838520 m` selected residual
- Position RMSE mean: `2.507251 m` observation to `2.402174 m` selected residual

Compared with b15x5 common0.5 fresh50, common0.7 is weaker on range and
position mean. Keep common0.5 as the stronger candidate and treat common0.7 as
a rejected or low-priority variant.

This is not a terminal search success. Continue exploring correction variants
until explicitly stopped.

## Coordinate Contract

- Guard feature `raw_forward_mean_m` is stereo-center OpenCV optical RDF
  forward after base profile correction and b15x5 common0.7 observation fusion.
- The guard feature is computed over the trailing 520-sample runtime
  measurement window; scenario labels and truth are not runtime selector
  inputs.
- Forward-add branch uses `log(measured_disparity_px)` and outputs
  stereo-center RDF forward-add correction.
- Product fallback branch uses `measured_disparity_px * raw_forward_m` and
  outputs stereo-center RDF ray-scale correction.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
