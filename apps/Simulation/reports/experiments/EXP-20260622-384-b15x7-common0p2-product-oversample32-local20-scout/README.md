# EXP-20260622-384 B15x7 Common0.2 Product Oversample32 Local20 Scout

## Purpose

Reduce target oversampling from `48` to `32` for the b15x7 common0.2 product
ray-scale branch. EXP378 used oversample48 and improved the tracked target row;
this scout checks whether less target emphasis reduces aggregate regression
without losing target protection. A strict pass is not a terminal success.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.2`
- Target oversample count: `32`
- Feature space: `disparity_forward_product`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `STRICT_PASS` on the local20 scout.

The method passed `120/120`:

- Target max-abs: `4.369534 m` observation to `5.097352 m` residual
- Range RMSE mean: `0.923776 m` observation to `1.109908 m` residual
- Position RMSE mean: `2.491515 m` observation to `2.564902 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.522041 m`, worse than b15x7 oversample48. Lower target oversampling does not
remove aggregate regression and gives up target-row improvement.

## Coordinate Contract

- Fusion remains stereo-center OpenCV optical RDF; target oversampling changes
  training sample weighting only, not runtime inputs or axis order.
- `disparity_forward_product` uses fused measured disparity multiplied by
  stereo-center RDF `raw_forward_m` after base profile correction and b15x7
  common0.2 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
