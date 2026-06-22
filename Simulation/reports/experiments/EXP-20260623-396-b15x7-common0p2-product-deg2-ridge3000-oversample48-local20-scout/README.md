# EXP-20260623-396 B15x7 Common0.2 Product Degree2 Ridge3000 Oversample48 Local20 Scout

## Purpose

Test whether stronger ridge regularization stabilizes the b15x7 quadratic
product ray-scale branch from EXP395. EXP395 reduced the failure count to two
intrinsic-scale seeds, but regressed the tracked target row and aggregate RMSE;
this scout checks whether ridge `3000` reduces that overfit pattern.

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
- Degree: `2`
- Ridge lambda: `3000`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `113/120`; all seven failures were `intrinsic_scale_stress`
seeds `298181`, `298185`, `298188`, `298190`, `298193`, `298198`, and
`298199`:

- Target max-abs: `4.285147 m` observation to `5.061488 m` residual
- Range RMSE mean: `0.924142 m` observation to `1.116914 m` residual
- Position RMSE mean: `2.492225 m` observation to `2.585454 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.735516 m`. Ridge `3000` slightly reduces the worst residual versus ridge
`300`, but increases intrinsic-scale failures from two to seven and keeps the
same aggregate RMSE regression. Reject this stronger regularization setting.

## Coordinate Contract

- `disparity_forward_product` uses fused measured disparity multiplied by
  stereo-center OpenCV optical RDF `raw_forward_m` after base profile correction
  and b15x7 common0.2 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
