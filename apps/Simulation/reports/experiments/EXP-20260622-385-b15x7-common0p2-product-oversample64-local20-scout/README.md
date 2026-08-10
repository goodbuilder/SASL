# EXP-20260622-385 B15x7 Common0.2 Product Oversample64 Local20 Scout

## Purpose

Increase target oversampling from `48` to `64` for the b15x7 common0.2 product
ray-scale branch. EXP384 showed oversample32 gave up target-row improvement; this
scout checks whether stronger target emphasis improves the tracked row or simply
keeps aggregate regression. A strict pass is not a terminal success.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.2`
- Target oversample count: `64`
- Feature space: `disparity_forward_product`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `STRICT_PASS` on the local20 scout.

The method passed `120/120`:

- Target max-abs: `4.409085 m` observation to `5.103839 m` residual
- Range RMSE mean: `0.924076 m` observation to `1.110134 m` residual
- Position RMSE mean: `2.491994 m` observation to `2.565336 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.491665 m`, effectively matching b15x7 oversample48. Higher target
oversampling does not materially improve the tracked row and leaves aggregate
regression in place.

## Coordinate Contract

- Fusion remains stereo-center OpenCV optical RDF; target oversampling changes
  training sample weighting only, not runtime inputs or axis order.
- `disparity_forward_product` uses fused measured disparity multiplied by
  stereo-center RDF `raw_forward_m` after base profile correction and b15x7
  common0.2 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
