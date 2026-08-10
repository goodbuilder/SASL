# EXP-20260622-352 B15x5 Product Oversample32 Local20 Scout

## Purpose

Scout whether reducing product fallback target oversampling from `48` to `32`
can keep the known high-forward intrinsic row passing while reducing the global
product fallback regression.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x5, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.3`
- Target oversample count: `32`
- Feature space: `disparity_forward_product`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambdas: `100`, `300`, `1000`
- Clip: `0.02`

## Result

Status: `STRICT_PASS` on the local20 scout.

All ridge variants passed `120/120`. The script-selected method was ridge
`1000`:

- Target max-abs: `4.730302 m` observation to `5.462658 m` residual
- Range RMSE mean: `0.934030 m` observation to `1.114280 m` residual
- Position RMSE mean: `2.496769 m` observation to `2.567785 m` residual

For seed `298188`, `intrinsic_scale_stress`, ridge `100` has the lowest
residual target max-abs at `2.583701 m`. This is worse than the oversample48
scout, so oversample32 is not prioritized for fresh100 expansion yet.

## Coordinate Contract

- `raw_forward_m` in the product feature is stereo-center OpenCV optical RDF
  forward after base profile correction and b15x5 common0.3 observation fusion.
- Product feature uses `measured_disparity_px * raw_forward_m`.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
