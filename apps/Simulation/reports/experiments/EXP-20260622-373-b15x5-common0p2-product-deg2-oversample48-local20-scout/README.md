# EXP-20260622-373 B15x5 Common0.2 Product Degree2 Oversample48 Local20 Scout

## Purpose

Test whether a quadratic product-feature model improves the b15x5 common0.2
oversample48 guard fallback. Degree1 passes locally but regresses aggregate
RMSE; this scout checks whether degree2 can improve the intrinsic-scale target
rows.

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
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `117/120`; all `3` failures were `intrinsic_scale_stress`
seeds `298188`, `298193`, and `298198`:

- Target max-abs: `4.559166 m` observation to `5.327278 m` residual
- Range RMSE mean: `0.927983 m` observation to `1.122826 m` residual
- Position RMSE mean: `2.494131 m` observation to `2.588446 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.747281 m`, worse than degree1 common0.2. Degree2 reduces the number of
failing rows versus the single-feature non-product scouts, but the three target
failures and aggregate regression reject this setting.

## Coordinate Contract

- `disparity_forward_product` uses fused measured disparity multiplied by
  stereo-center OpenCV optical RDF `raw_forward_m` after base profile correction
  and b15x5 common0.2 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
