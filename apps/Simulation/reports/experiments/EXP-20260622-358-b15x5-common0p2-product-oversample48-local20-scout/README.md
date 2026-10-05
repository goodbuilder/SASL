# EXP-20260622-358 B15x5 Common0.2 Product Oversample48 Local20 Scout

## Purpose

Scout whether lowering the product guard branch common fraction from `0.3` to
`0.2` improves the oversample48 high-forward intrinsic fallback.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x5, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.2`
- Target oversample count: `48`
- Feature space: `disparity_forward_product`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `STRICT_PASS` on the local20 scout.

The method passed `120/120`:

- Target max-abs: `4.559166 m` observation to `5.274375 m` residual
- Range RMSE mean: `0.927983 m` observation to `1.112743 m` residual
- Position RMSE mean: `2.494131 m` observation to `2.566748 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.525492 m`, slightly better than b15x5 common0.3 oversample48. This is a
useful candidate for fresh100 expansion as a guard branch variant.

## Coordinate Contract

- `raw_forward_m` in the product feature is stereo-center OpenCV optical RDF
  forward after base profile correction and b15x5 common0.2 observation fusion.
- Product feature uses `measured_disparity_px * raw_forward_m`.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
