# EXP-20260622-355 B15x5 Log-Product Oversample48 Local20 Scout

## Purpose

Scout whether replacing the product fallback feature with
`log_disparity_forward_product` improves the high-forward intrinsic row or
reduces aggregate product fallback regression.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x5, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.3`
- Target oversample count: `48`
- Feature space: `log_disparity_forward_product`
- Correction space: `ray_scale`
- Degrees: `1`, `2`
- Ridge lambdas: `100`, `300`
- Clip: `0.02`

## Result

Status: mixed.

Degree1 variants passed `120/120`, but they were slightly worse than the
standard product feature. Degree2 variants failed `117/120`.

Best passing method:

- Degree `1`, ridge `300`
- Target max-abs: `4.720330 m` observation to `5.453579 m` residual
- Range RMSE mean: `0.934776 m` observation to `1.116132 m` residual
- Position RMSE mean: `2.497594 m` observation to `2.569056 m` residual

For seed `298188`, `intrinsic_scale_stress`, degree1 ridge `100` had the lowest
residual target max-abs at `2.537581 m`, but the overall method did not improve
on EXP346/EXP347 enough to prioritize expansion.

## Coordinate Contract

- `raw_forward_m` in the log-product feature is stereo-center OpenCV optical RDF
  forward after base profile correction and b15x5 common0.3 observation fusion.
- Product feature family uses measured disparity and `raw_forward_m`.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
