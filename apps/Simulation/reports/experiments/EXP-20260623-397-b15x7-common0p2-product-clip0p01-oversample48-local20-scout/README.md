# EXP-20260623-397 B15x7 Common0.2 Product Clip0.01 Oversample48 Local20 Scout

## Purpose

Test whether a more conservative ray-scale correction clip preserves the b15x7
common0.2 product branch while avoiding the aggregate RMSE regression seen in
nearby ray-scale variants.

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
- Clip: `0.01`

## Result

Status: `STRICT_PASS` on the local20 scout. This is not a terminal success; the
search continues.

The method passed `120/120`:

- Target max-abs: `4.285147 m` observation to `5.026495 m` residual
- Range RMSE mean: `0.924142 m` observation to `1.110057 m` residual
- Position RMSE mean: `2.492225 m` observation to `2.565429 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.490142 m`, matching the earlier b15x7 common0.2 product branch. The clip
change does not appear to be the active constraint for this branch; aggregate
RMSE still regresses.

## Coordinate Contract

- `disparity_forward_product` uses fused measured disparity multiplied by
  stereo-center OpenCV optical RDF `raw_forward_m` after base profile correction
  and b15x7 common0.2 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
