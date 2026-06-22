# EXP-20260622-332 B15x5 Trim0.2 Runtime Guard Selector Fresh50

## Purpose

Try stronger trimmed-mean outlier rejection inside the b15x5 runtime guard
selector. This keeps the b15x5 shell and guard structure, but changes
`trim_fraction` from `0.1` to `0.2`.

## Method

- Seeds: `297300..297349`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Guard: use product fallback when `raw_forward_mean_m > 1771.0 m`
- Non-guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b15x5_trim0p2_common0p5_residual_forward_add_log_disparity_deg3_ridge100_clip0p1_pooled`
- Guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b15x5_trim0p2_common0p5_residual_ray_scale_disparity_forward_product_deg1_ridge100_clip0p02_pooled`

The runtime guard feature was regenerated from the b15x5 trim0.2 fused
observation stream after base profile correction, using the trailing
`RuntimeFeatureWindow(max_samples=520)`.

## Result

Status: `STRICT_PASS` on fresh50.

The trim0.2 selector passed `300/300`. The guard selected product ray-scale
fallback for `50` intrinsic-like rows and log-disparity forward-add for `250`
non-guard rows.

Observation-relative aggregate metrics all improved:

- Target max-abs: `5.237591 m` observation to `5.137592 m` selected residual
- Range RMSE mean: `0.945077 m` observation to `0.828129 m` selected residual
- Position RMSE mean: `2.499793 m` observation to `2.395200 m` selected residual

Compared with b15x5 trim0.1 fresh50, trim0.2 is mixed: range mean is slightly
worse, while position mean is slightly better. Keep this as an exploratory
variant, not a replacement, until independent fresh100 evidence exists.

This is not a terminal search success. Continue exploring correction variants
until explicitly stopped.

## Coordinate Contract

- Guard feature `raw_forward_mean_m` is stereo-center OpenCV optical RDF
  forward after base profile correction and b15x5 trim0.2 observation fusion.
- The guard feature is computed over the trailing 520-sample runtime
  measurement window; scenario labels and truth are not runtime selector
  inputs.
- Forward-add branch uses `log(measured_disparity_px)` and outputs
  stereo-center RDF forward-add correction.
- Product fallback branch uses `measured_disparity_px * raw_forward_m` and
  outputs stereo-center RDF ray-scale correction.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
