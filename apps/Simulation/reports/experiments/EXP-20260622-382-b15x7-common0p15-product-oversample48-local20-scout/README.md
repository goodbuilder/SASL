# EXP-20260622-382 B15x7 Common0.15 Product Oversample48 Local20 Scout

## Purpose

Lower the common fraction from b15x7 common0.2 to common0.15 for the product
ray-scale branch. EXP378 showed b15x7 common0.2 was the best tracked target row
in the local fusion neighborhood; this scout checks whether a lower common
fraction improves it further. A strict pass is not a terminal success.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.15`
- Target oversample count: `48`
- Feature space: `disparity_forward_product`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `STRICT_PASS` on the local20 scout.

The method passed `120/120`:

- Target max-abs: `4.535557 m` observation to `5.247943 m` residual
- Range RMSE mean: `0.920433 m` observation to `1.107525 m` residual
- Position RMSE mean: `2.490293 m` observation to `2.564072 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.621128 m`, worse than b15x7 common0.2. Lowering common fraction does not
improve the b15x7 branch.

## Coordinate Contract

- Fusion remains stereo-center OpenCV optical RDF; b15x7 changes observation
  fusion only, not axis order.
- `disparity_forward_product` uses fused measured disparity multiplied by
  stereo-center RDF `raw_forward_m` after base profile correction and b15x7
  common0.15 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
