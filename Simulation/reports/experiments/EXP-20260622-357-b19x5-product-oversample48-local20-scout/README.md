# EXP-20260622-357 B19x5 Product Oversample48 Local20 Scout

## Purpose

Scout whether a larger b19x5 common0.3 fusion shell improves the oversample48
product guard fallback compared with b15x5.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b19x5, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.3`
- Target oversample count: `48`
- Feature space: `disparity_forward_product`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `STRICT_PASS` on the local20 scout.

The method passed `120/120`:

- Target max-abs: `4.325483 m` observation to `5.069510 m` residual
- Range RMSE mean: `0.925022 m` observation to `1.108965 m` residual
- Position RMSE mean: `2.492557 m` observation to `2.564876 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.746343 m`, worse than the b15x5 oversample48 product fallback. A larger
fusion shell improves raw observation aggregate but does not improve this guard
fallback.

## Coordinate Contract

- `raw_forward_m` in the product feature is stereo-center OpenCV optical RDF
  forward after base profile correction and b19x5 common0.3 observation fusion.
- Product feature uses `measured_disparity_px * raw_forward_m`.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
