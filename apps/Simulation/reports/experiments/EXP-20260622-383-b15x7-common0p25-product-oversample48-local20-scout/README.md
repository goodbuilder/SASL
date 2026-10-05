# EXP-20260622-383 B15x7 Common0.25 Product Oversample48 Local20 Scout

## Purpose

Raise the common fraction from b15x7 common0.2 to common0.25 for the product
ray-scale branch. Together with EXP382 common0.15, this brackets the b15x7
common-fraction neighborhood. A strict pass is not a terminal success.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.25`
- Target oversample count: `48`
- Feature space: `disparity_forward_product`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `STRICT_PASS` on the local20 scout.

The method passed `120/120`:

- Target max-abs: `4.395511 m` observation to `5.141102 m` residual
- Range RMSE mean: `0.925898 m` observation to `1.109996 m` residual
- Position RMSE mean: `2.492848 m` observation to `2.565106 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.565573 m`, worse than b15x7 common0.2. The b15x7 branch remains best at
common0.2 within the sampled common-fraction neighborhood.

## Coordinate Contract

- Fusion remains stereo-center OpenCV optical RDF; b15x7 changes observation
  fusion only, not axis order.
- `disparity_forward_product` uses fused measured disparity multiplied by
  stereo-center RDF `raw_forward_m` after base profile correction and b15x7
  common0.25 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
