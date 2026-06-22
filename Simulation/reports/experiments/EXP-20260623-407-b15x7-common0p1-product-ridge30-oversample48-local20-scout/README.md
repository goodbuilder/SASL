# EXP-20260623-407 B15x7 Common0.1 Product Ridge30 Oversample48 Local20 Scout

## Purpose

Test the lower common-mode fraction side after EXP406 showed common0.3 worsened
the b15x7 product ridge30 branch. This checks whether a looser common filter
improves observation baseline or residual aggregate behavior.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.1`
- Target oversample count: `48`
- Feature space: `disparity_forward_product`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `30`
- Clip: `0.02`

## Result

Status: `STRICT_PASS` on the local20 scout. This is not a terminal success; the
search continues.

The method passed `120/120`:

- Target max-abs: `4.348758 m` observation to `5.093682 m` residual
- Range RMSE mean: `0.916652 m` observation to `1.108002 m` residual
- Position RMSE mean: `2.488401 m` observation to `2.563913 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.510555 m`. Common0.1 improves observation aggregate versus common0.2 but
worsens the tracked target row and leaves a large residual aggregate regression.
Keep only as a nonterminal candidate; lower common threshold is not enough.

## Coordinate Contract

- `disparity_forward_product` uses fused measured disparity multiplied by
  stereo-center OpenCV optical RDF `raw_forward_m` after base profile correction
  and b15x7 common0.1 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
