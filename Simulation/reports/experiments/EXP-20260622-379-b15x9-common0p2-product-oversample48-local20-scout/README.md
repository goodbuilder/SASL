# EXP-20260622-379 B15x9 Common0.2 Product Oversample48 Local20 Scout

## Purpose

Increase observations per burst from b15x7 to b15x9 for the common0.2 product
ray-scale branch. EXP378 improved the tracked target row versus b15x5; this
scout checks whether a still denser burst continues that trend. A strict pass is
not a terminal success.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x9, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
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

- Target max-abs: `4.390745 m` observation to `5.099745 m` residual
- Range RMSE mean: `0.920086 m` observation to `1.106775 m` residual
- Position RMSE mean: `2.489877 m` observation to `2.563327 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.654313 m`, worse than b15x7 and b15x5 common0.2. Longer bursts improve the
observation aggregate but do not monotonically improve the target row.

## Coordinate Contract

- Fusion remains stereo-center OpenCV optical RDF; b15x9 changes only
  observation count, not axis order.
- `disparity_forward_product` uses fused measured disparity multiplied by
  stereo-center RDF `raw_forward_m` after base profile correction and b15x9
  common0.2 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
