# EXP-20260623-395 B15x7 Common0.2 Product Degree2 Oversample48 Local20 Scout

## Purpose

Test a quadratic product ray-scale model under b15x7 common0.2 fusion. EXP373
showed b15x5 degree2 failed three intrinsic-scale rows and regressed aggregate
RMSE; this scout checks whether b15x7 fusion reduces the target failure count.

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
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `118/120`; both failures were `intrinsic_scale_stress` seeds
`298188` and `298198`:

- Target max-abs: `4.285147 m` observation to `5.073388 m` residual
- Range RMSE mean: `0.924142 m` observation to `1.120158 m` residual
- Position RMSE mean: `2.492225 m` observation to `2.587237 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.725512 m`. B15x7 degree2 reduces the failure count versus b15x5 degree2, but
the tracked target row and aggregate RMSE are worse than degree1 product.

## Coordinate Contract

- `disparity_forward_product` uses fused measured disparity multiplied by
  stereo-center OpenCV optical RDF `raw_forward_m` after base profile correction
  and b15x7 common0.2 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
