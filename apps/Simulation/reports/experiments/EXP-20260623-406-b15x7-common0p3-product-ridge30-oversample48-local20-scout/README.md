# EXP-20260623-406 B15x7 Common0.3 Product Ridge30 Oversample48 Local20 Scout

## Purpose

Test whether increasing the b15x7 common-mode fraction to `0.3` improves the
product ray-scale branch when paired with the lower ridge `30` setting from
EXP403.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.3`
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

- Target max-abs: `4.481928 m` observation to `5.201588 m` residual
- Range RMSE mean: `0.929492 m` observation to `1.112801 m` residual
- Position RMSE mean: `2.494618 m` observation to `2.566535 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.662742 m`. Compared with common0.2 ridge30, common0.3 worsens the observation
baseline, worst residual, and aggregate RMSE. Keep only as a nonterminal
candidate; common-threshold increase is not a useful improvement axis here.

## Coordinate Contract

- `disparity_forward_product` uses fused measured disparity multiplied by
  stereo-center OpenCV optical RDF `raw_forward_m` after base profile correction
  and b15x7 common0.3 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
