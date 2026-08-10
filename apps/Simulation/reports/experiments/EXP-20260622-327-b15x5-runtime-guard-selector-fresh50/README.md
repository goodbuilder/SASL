# EXP-20260622-327 B15x5 Runtime Guard Selector Fresh50

## Purpose

Try a larger b15x5 observation-fusion shell inside the runtime raw-forward
guard selector. This extends the b11x5 and b13x5 selector search with a new
correction-method variant.

## Method

- Seeds: `296500..296549`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Guard: use product fallback when `raw_forward_mean_m > 1771.0 m`
- Non-guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b15x5_trim0p1_common0p5_residual_forward_add_log_disparity_deg3_ridge100_clip0p1_pooled`
- Guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b15x5_trim0p1_common0p5_residual_ray_scale_disparity_forward_product_deg1_ridge100_clip0p02_pooled`

The runtime guard feature was regenerated from the b15x5 fused observation
stream after base profile correction, using the trailing
`RuntimeFeatureWindow(max_samples=520)`.

## Result

Status: `STRICT_PASS` on fresh50.

The b15x5 runtime-guard selector passed `300/300`. The guard selected product
ray-scale fallback for `50` intrinsic-like rows and log-disparity forward-add
for `250` non-guard rows.

Observation-relative aggregate metrics all improved:

- Target max-abs: `5.113483 m` observation to `5.013486 m` selected residual
- Range RMSE mean: `0.945477 m` observation to `0.827734 m` selected residual
- Position RMSE mean: `2.502511 m` observation to `2.397323 m` selected residual

The forward-add branch alone passed `250/300` and improved aggregate metrics.
The product fallback branch alone passed `300/300` but regressed aggregate
metrics. The selector combines strict pass behavior with aggregate improvement.

This is not a terminal search success. Run fresh100 validation and continue
trying new correction ideas until explicitly stopped.

## Coordinate Contract

- Guard feature `raw_forward_mean_m` is stereo-center OpenCV optical RDF
  forward after base profile correction and b15x5 observation fusion.
- The guard feature is computed over the trailing 520-sample runtime
  measurement window; scenario labels and truth are not runtime selector
  inputs.
- Forward-add branch uses `log(measured_disparity_px)` and outputs
  stereo-center RDF forward-add correction.
- Product fallback branch uses `measured_disparity_px * raw_forward_m` and
  outputs stereo-center RDF ray-scale correction.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
