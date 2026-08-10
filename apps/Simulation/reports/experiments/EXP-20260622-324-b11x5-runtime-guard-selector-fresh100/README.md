# EXP-20260622-324 B11x5 Runtime Guard Selector Fresh100

## Purpose

Validate the b11x5 runtime raw-forward guard selector on an independent
fresh100 seed block. This continues the correction-method search after the
fresh50 pass and does not treat a pass as terminal success.

## Method

- Seeds: `295900..295999`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Guard: use product fallback when `raw_forward_mean_m > 1771.0 m`
- Non-guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b11x5_trim0p1_common0p5_residual_forward_add_log_disparity_deg3_ridge100_clip0p1_pooled`
- Guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b11x5_trim0p1_common0p5_residual_ray_scale_disparity_forward_product_deg1_ridge100_clip0p02_pooled`

The runtime guard feature was regenerated from the same b11x5 fused
observation stream used by the branch metrics. The feature window is the
default trailing `RuntimeFeatureWindow(max_samples=520)`.

## Result

Status: `STRICT_PASS` on fresh100.

The b11x5 runtime-guard selector passed `600/600`. The guard selected product
ray-scale fallback for `100` intrinsic-like rows and log-disparity forward-add
for `500` non-guard rows.

Observation-relative aggregate metrics all improved:

- Target max-abs: `5.538053 m` observation to `5.438055 m` selected residual
- Range RMSE mean: `0.956992 m` observation to `0.841180 m` selected residual
- Position RMSE mean: `2.505756 m` observation to `2.401922 m` selected residual

The forward-add branch alone passed `500/600` and improved aggregate metrics.
The product fallback branch alone passed `600/600` but regressed aggregate
metrics. The selector preserves strict pass behavior while keeping the
aggregate improvement from the non-guard branch.

This is not a terminal search success. Continue exploring new correction
methods, guard variants, and packaging/tests until explicitly stopped.

## Coordinate Contract

- Guard feature `raw_forward_mean_m` is stereo-center OpenCV optical RDF
  forward after base profile correction and b11x5 observation fusion.
- The guard feature is computed over the trailing 520-sample runtime
  measurement window; scenario labels and truth are not runtime selector
  inputs.
- Forward-add branch uses `log(measured_disparity_px)` and outputs
  stereo-center RDF forward-add correction.
- Product fallback branch uses `measured_disparity_px * raw_forward_m` and
  outputs stereo-center RDF ray-scale correction.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
