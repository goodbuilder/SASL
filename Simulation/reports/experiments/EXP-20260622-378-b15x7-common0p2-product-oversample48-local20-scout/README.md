# EXP-20260622-378 B15x7 Common0.2 Product Oversample48 Local20 Scout

## Purpose

Increase observations per burst from b15x5 to b15x7 for the common0.2 product
ray-scale branch. This tests whether a denser burst improves intrinsic-scale
target stability without changing the runtime feature or correction contract.
A strict pass is not a terminal success.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
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

- Target max-abs: `4.285147 m` observation to `5.026495 m` residual
- Range RMSE mean: `0.924142 m` observation to `1.110057 m` residual
- Position RMSE mean: `2.492225 m` observation to `2.565429 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.490142 m`, slightly better than b15x5 common0.2. Aggregate RMSE still
regresses, so this is a scout branch rather than a final method.

## Coordinate Contract

- Fusion remains stereo-center OpenCV optical RDF; b15x7 changes only
  observation count, not axis order.
- `disparity_forward_product` uses fused measured disparity multiplied by
  stereo-center RDF `raw_forward_m` after base profile correction and b15x7
  common0.2 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
