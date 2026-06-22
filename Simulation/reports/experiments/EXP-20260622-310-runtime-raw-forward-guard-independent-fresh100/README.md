# EXP-20260622-310 Runtime Raw-Forward Guard Independent Fresh100

## Purpose

Extend EXP309 from independent fresh50 to an independent fresh100 seed block.
This tests whether the runtime-derived `raw_forward_mean_m > 1771.0 m` guard
continues to separate intrinsic-scale cases and preserve the selector's
aggregate improvements.

## Method

- Independent seeds: `293700..293799`
- Guard feature: `raw_forward_mean_m`
- Guard threshold: use product fallback when `raw_forward_mean_m > 1771.0 m`
- Non-guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5_residual_forward_add_log_disparity_deg3_ridge100_clip0p1_pooled`
- Guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5_residual_ray_scale_disparity_forward_product_deg1_ridge10_clip0p1_pooled`

`raw_forward_mean_m` again perfectly separated `intrinsic_scale_stress` from
the other five scenarios:

- Intrinsic: `1773.214608..1774.047560 m`
- Non-intrinsic: `1766.728702..1770.635875 m`

## Result

Status: `STRICT_PASS` on independent fresh100.

The runtime-guard selector passed `600/600` and improved all tracked
observation-relative aggregate metrics:

- Target max-abs: `6.146146 m` observation to `6.046149 m` selected residual
- Range RMSE mean: `1.013402 m` observation to `0.898223 m` selected residual
- Position RMSE mean: `2.545801 m` observation to `2.441838 m` selected residual

This is the strongest guard-selector evidence so far, but it remains an
ongoing search result rather than a terminal success. Next work should package
the selector manifest and add runtime/unit tests for the guard behavior.

## Coordinate Contract

- Guard feature `raw_forward_mean_m` is stereo-center OpenCV optical RDF
  forward averaged over the measurement window.
- Forward-add branch uses `log(measured_disparity_px)`.
- Product fallback branch uses `measured_disparity_px * raw_forward_m`.
- Runtime correction outputs remain stereo-center RDF.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is calibration/evaluation-only and is not a runtime input.
