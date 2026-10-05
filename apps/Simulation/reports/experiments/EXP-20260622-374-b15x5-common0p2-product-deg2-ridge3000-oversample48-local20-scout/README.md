# EXP-20260622-374 B15x5 Common0.2 Product Degree2 Ridge3000 Oversample48 Local20 Scout

## Purpose

Test stronger regularization for the quadratic product-feature model. EXP373
with ridge300 failed three intrinsic-scale rows; this scout raises ridge to
`3000` to see whether suppressing curvature improves gate stability.

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
- Degree: `2`
- Ridge lambda: `3000`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `115/120`; all `5` failures were `intrinsic_scale_stress`
seeds `298181`, `298188`, `298193`, `298198`, and `298199`:

- Target max-abs: `4.559166 m` observation to `5.315705 m` residual
- Range RMSE mean: `0.927983 m` observation to `1.119583 m` residual
- Position RMSE mean: `2.494131 m` observation to `2.586666 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.757494 m`. Stronger degree2 regularization does not improve the quadratic
branch; it increases the failing row count relative to ridge300.

## Coordinate Contract

- `disparity_forward_product` uses fused measured disparity multiplied by
  stereo-center OpenCV optical RDF `raw_forward_m` after base profile correction
  and b15x5 common0.2 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
