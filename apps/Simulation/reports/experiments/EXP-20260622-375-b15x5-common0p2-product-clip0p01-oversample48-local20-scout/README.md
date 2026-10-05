# EXP-20260622-375 B15x5 Common0.2 Product Clip0.01 Oversample48 Local20 Scout

## Purpose

Test whether reducing the degree1 product branch correction clip from `0.02` to
`0.01` reduces aggregate regression for the b15x5 common0.2 oversample48 guard
fallback. A strict pass is not a terminal success.

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
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.01`

## Result

Status: `STRICT_PASS` on the local20 scout.

The method passed `120/120`:

- Target max-abs: `4.559166 m` observation to `5.274375 m` residual
- Range RMSE mean: `0.927983 m` observation to `1.112743 m` residual
- Position RMSE mean: `2.494131 m` observation to `2.566748 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.525492 m`, matching the clip0.02 branch. The branch is not clip-limited at
`0.01` for this local20 scout, so the aggregate regression remains.

## Coordinate Contract

- `disparity_forward_product` uses fused measured disparity multiplied by
  stereo-center OpenCV optical RDF `raw_forward_m` after base profile correction
  and b15x5 common0.2 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
