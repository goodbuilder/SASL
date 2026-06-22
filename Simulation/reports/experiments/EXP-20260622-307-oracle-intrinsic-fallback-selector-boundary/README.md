# EXP-20260622-307 Oracle Intrinsic Fallback Selector Boundary

## Purpose

Test whether the EXP306 aggregate-improving log-disparity forward-add method
can be combined with a strict-pass product residual fallback for only the
intrinsic-scale stress failures. This is an offline oracle selector using the
known scenario label; it is not a deployable runtime method by itself.

## Method

- Shared seed block: `293300..293399`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Non-intrinsic scenarios use:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5_residual_forward_add_log_disparity_deg3_ridge100_clip0p1_pooled`
- `intrinsic_scale_stress` uses:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5_residual_ray_scale_disparity_forward_product_deg1_ridge10_clip0p1_pooled`

## Result

Status: `STRICT_PASS` as an offline oracle selector.

The selector passed `600/600` and improved all tracked observation-relative
aggregate metrics:

- Target max-abs: `5.322285 m` observation to `5.222288 m` selected residual
- Range RMSE mean: `1.015607 m` observation to `0.899117 m` selected residual
- Position RMSE mean: `2.550003 m` observation to `2.444750 m` selected residual

This is not a runtime PASS method. It depends on the scenario label to choose
the intrinsic fallback. The next step is to derive a runtime-safe intrinsic
guard from measured inputs or telemetry available at runtime, then retest the
same selector without using scenario truth.

## Coordinate Contract

- Forward-add branch uses `log(measured_disparity_px)`.
- Product fallback branch uses `measured_disparity_px * raw_forward_m`.
- `raw_forward_m` is stereo-center OpenCV optical RDF forward.
- Runtime correction outputs remain stereo-center RDF.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Scenario label is offline validation metadata here, not a valid runtime input.
- Truth is calibration/evaluation-only and is not a runtime input.
