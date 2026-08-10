# EXP-20260622-298 Log Product Residual Boundary

## Purpose

Try a new runtime-safe residual feature family after the promoted
`disparity_forward_product` method: `log(measured_disparity_px * raw_forward_m)`.

This experiment is not a final PASS candidate. It records a useful boundary:
the log-product ray-scale residual passes a small fresh30 screen but does not
clear the six-scenario fresh100 gate.

## Method

- Base profile:
  `Simulation/reports/experiments/EXP-20260617-198-bbox-outlier-causal-image-prefilter/base_profile/recommended_runtime_profile.json`
- Fusion shell: b7x3 trimmed-mean / trimmed-mean, trim `0.05`,
  common fraction `0.5`
- Primary feature: `log_disparity_forward_product`
- Feature contract: `log(measured_disparity_px * raw_forward_m)`
- `raw_forward_m`: stereo-center OpenCV optical RDF forward axis
- Runtime truth: `false`

## Results

- Fresh30 screen, seeds `290300..290329`: best log-product ray-scale method
  passed `180/180 STRICT_PASS`.
- Fresh100 single replay, seeds `290500..290599`: best method failed
  `599/600`.
- Fresh100 tuned grid over degree/ridge/clip on the same seed block still
  failed `599/600`.

Best tuned method:

`multiburst_observation_inner_trimmed_mean_outer_trimmed_mean_b7x3_trim0p05_common0p5_residual_ray_scale_log_disparity_forward_product_deg1_ridge100_clip0p02_pooled`

The remaining failure was in `intrinsic_scale_stress`; the residual improved
target-window error but regressed range RMSE versus observation for the failing
seed.

## Next Search Direction

Do not promote this method. Continue with different feature families or richer
models rather than only extending this log-product scalar residual.

## Coordinate Contract

- Image feature inputs: `measured_disparity_px`, `raw_forward_m`.
- `raw_forward_m` is stereo-center OpenCV optical RDF forward.
- Runtime correction output remains stereo-center RDF.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
- Truth is calibration/evaluation-only and is not a runtime input.
