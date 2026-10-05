# EXP-20260622-341 B15x5 Common0.3/Common0.4 Cross Guard Selector Fresh100

## Purpose

Test whether the common0.3 aggregate-improving non-guard branch can keep its
benefit while borrowing the common0.4 product fallback for high-forward guard
rows on the same fresh100 seed block.

## Method

- Seeds: `298100..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Guard: use common0.4 product fallback when `raw_forward_mean_m > 1771.0 m`
- Non-guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b15x5_trim0p1_common0p3_residual_forward_add_log_disparity_deg3_ridge100_clip0p1_pooled`
- Guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b15x5_trim0p1_common0p4_residual_ray_scale_disparity_forward_product_deg1_ridge100_clip0p02_pooled`

The runtime guard feature was reused from the b15x5 common0.3 fused
observation stream after base profile correction, using the trailing
`RuntimeFeatureWindow(max_samples=520)`.

## Result

Status: `FAIL` on fresh100.

The cross-common selector passed `599/600`. It preserved the common0.3
aggregate improvement level but did not recover the strict failure:

- Target max-abs: `4.732347 m` observation to `4.632350 m` selected residual
- Range RMSE mean: `0.933193 m` observation to `0.815708 m` selected residual
- Position RMSE mean: `2.496306 m` observation to `2.391667 m` selected residual

The failed row remained seed `298188`, `intrinsic_scale_stress`, selected by
the common0.4 product guard branch. This indicates that simply swapping the
product fallback common fraction is not enough for the common0.3 failure case.

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
