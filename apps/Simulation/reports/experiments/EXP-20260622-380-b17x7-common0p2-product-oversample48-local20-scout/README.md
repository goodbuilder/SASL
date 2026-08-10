# EXP-20260622-380 B17x7 Common0.2 Product Oversample48 Local20 Scout

## Purpose

Increase burst count from b15x7 to b17x7 for the common0.2 product ray-scale
branch. EXP378 suggested b15x7 was better than b15x5 on the tracked target row;
this scout checks whether increasing burst count keeps improving that branch.
A strict pass is not a terminal success.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b17x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
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

- Target max-abs: `4.429384 m` observation to `5.172883 m` residual
- Range RMSE mean: `0.919064 m` observation to `1.106531 m` residual
- Position RMSE mean: `2.489480 m` observation to `2.563334 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.556537 m`, worse than b15x7. Increasing burst count improves observation
aggregate but does not improve the tracked target row.

## Coordinate Contract

- Fusion remains stereo-center OpenCV optical RDF; b17x7 changes only
  observation count, not axis order.
- `disparity_forward_product` uses fused measured disparity multiplied by
  stereo-center RDF `raw_forward_m` after base profile correction and b17x7
  common0.2 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
