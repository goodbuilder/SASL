# EXP-20260622-330 B17x5 Runtime Guard Selector Fresh100

## Purpose

Validate the b17x5 runtime raw-forward guard selector on an independent
fresh100 seed block. This checks whether the larger-shell improvement observed
on fresh50 persists beyond b15x5.

## Method

- Seeds: `297100..297199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Guard: use product fallback when `raw_forward_mean_m > 1771.0 m`
- Non-guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b17x5_trim0p1_common0p5_residual_forward_add_log_disparity_deg3_ridge100_clip0p1_pooled`
- Guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b17x5_trim0p1_common0p5_residual_ray_scale_disparity_forward_product_deg1_ridge100_clip0p02_pooled`

The runtime guard feature was regenerated from the b17x5 fused observation
stream after base profile correction, using the trailing
`RuntimeFeatureWindow(max_samples=520)`.

## Result

Status: `STRICT_PASS` on fresh100.

The b17x5 runtime-guard selector passed `600/600`. The guard selected product
ray-scale fallback for `100` intrinsic-like rows and log-disparity forward-add
for `500` non-guard rows.

Observation-relative aggregate metrics all improved:

- Target max-abs: `5.027737 m` observation to `4.927738 m` selected residual
- Range RMSE mean: `0.940480 m` observation to `0.823055 m` selected residual
- Position RMSE mean: `2.499276 m` observation to `2.394601 m` selected residual

The b17x5 fresh100 selector remains strict-pass and aggregate-improving, but
its range mean is slightly worse than the b15x5 fresh100 selector. Treat this
as evidence that the larger-shell sweep may be approaching a plateau.

This is not a terminal search success. Continue exploring guard sensitivity,
nearby shell sizes, and packaging/tests until explicitly stopped.

## Coordinate Contract

- Guard feature `raw_forward_mean_m` is stereo-center OpenCV optical RDF
  forward after base profile correction and b17x5 observation fusion.
- The guard feature is computed over the trailing 520-sample runtime
  measurement window; scenario labels and truth are not runtime selector
  inputs.
- Forward-add branch uses `log(measured_disparity_px)` and outputs
  stereo-center RDF forward-add correction.
- Product fallback branch uses `measured_disparity_px * raw_forward_m` and
  outputs stereo-center RDF ray-scale correction.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
