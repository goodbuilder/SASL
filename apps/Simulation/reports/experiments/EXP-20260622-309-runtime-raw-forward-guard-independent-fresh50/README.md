# EXP-20260622-309 Runtime Raw-Forward Guard Independent Fresh50

## Purpose

Retest the EXP308 runtime-derived raw-forward guard on an independent seed
block. This checks whether the `raw_forward_mean_m > 1771.0 m` intrinsic
fallback threshold generalizes beyond the seed block used to discover it.

## Method

- Independent seeds: `293500..293549`
- Guard feature: `raw_forward_mean_m`
- Guard threshold: use product fallback when `raw_forward_mean_m > 1771.0 m`
- Non-guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5_residual_forward_add_log_disparity_deg3_ridge100_clip0p1_pooled`
- Guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5_residual_ray_scale_disparity_forward_product_deg1_ridge10_clip0p1_pooled`

`raw_forward_mean_m` again perfectly separated `intrinsic_scale_stress` from
the other five scenarios:

- Intrinsic: `1773.035354..1773.898241 m`
- Non-intrinsic: `1766.551191..1770.486843 m`

## Result

Status: `STRICT_PASS` on independent fresh50.

The runtime-guard selector passed `300/300` and improved all tracked
observation-relative aggregate metrics:

- Target max-abs: `6.063932 m` observation to `5.963932 m` selected residual
- Range RMSE mean: `1.015685 m` observation to `0.900106 m` selected residual
- Position RMSE mean: `2.549468 m` observation to `2.445042 m` selected residual

This strengthens the guard-selector direction, but it is still not a final
search stop. More seeds, stress variants, and implementation tests are needed.

## Coordinate Contract

- Guard feature `raw_forward_mean_m` is stereo-center OpenCV optical RDF
  forward averaged over the measurement window.
- Forward-add branch uses `log(measured_disparity_px)`.
- Product fallback branch uses `measured_disparity_px * raw_forward_m`.
- Runtime correction outputs remain stereo-center RDF.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is calibration/evaluation-only and is not a runtime input.
