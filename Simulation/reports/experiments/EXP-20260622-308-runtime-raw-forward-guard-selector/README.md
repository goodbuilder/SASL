# EXP-20260622-308 Runtime Raw-Forward Guard Selector

## Purpose

Replace the EXP307 offline scenario-oracle selector with a runtime-derived
guard. The guard uses only the measurement-window feature
`raw_forward_mean_m`, then applies the product fallback for intrinsic-like
windows and the aggregate-improving log-disparity forward-add method otherwise.

## Method

- Shared seed block: `293300..293399`
- Guard feature: `raw_forward_mean_m`
- Guard threshold: use product fallback when `raw_forward_mean_m > 1771.0 m`
- Non-guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5_residual_forward_add_log_disparity_deg3_ridge100_clip0p1_pooled`
- Guard branch:
  `multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5_residual_ray_scale_disparity_forward_product_deg1_ridge10_clip0p1_pooled`

`raw_forward_mean_m` perfectly separated `intrinsic_scale_stress` from the
other five scenarios on this seed block:

- Intrinsic: `1773.006249..1774.096836 m`
- Non-intrinsic: `1766.495820..1770.685056 m`

## Result

Status: `STRICT_PASS` on this fresh100 seed block.

The runtime-guard selector passed `600/600` and improved all tracked
observation-relative aggregate metrics:

- Target max-abs: `5.322285 m` observation to `5.222288 m` selected residual
- Range RMSE mean: `1.015607 m` observation to `0.899117 m` selected residual
- Position RMSE mean: `2.550003 m` observation to `2.444750 m` selected residual

This is stronger than EXP307 because it does not consume scenario labels or
truth at runtime. It is still not a final method: the guard threshold has only
same-block evidence and must be retested on independent seeds and stress
variants.

## Coordinate Contract

- Guard feature `raw_forward_mean_m` is stereo-center OpenCV optical RDF
  forward averaged over the measurement window.
- Forward-add branch uses `log(measured_disparity_px)`.
- Product fallback branch uses `measured_disparity_px * raw_forward_m`.
- Runtime correction outputs remain stereo-center RDF.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is calibration/evaluation-only and is not a runtime input.
