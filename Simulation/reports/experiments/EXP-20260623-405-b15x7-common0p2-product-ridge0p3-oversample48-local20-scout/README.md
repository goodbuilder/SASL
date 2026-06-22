# EXP-20260623-405 B15x7 Common0.2 Product Ridge0.3 Oversample48 Local20 Scout

## Purpose

Test whether the lower-ridge target-row movement from EXP403 and EXP404
continues below ridge `3`, or whether the b15x7 common0.2 product ray-scale
branch has reached a practical regularization plateau.

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
- Ridge lambda: `0.3`
- Clip: `0.02`

## Result

Status: `STRICT_PASS` on the local20 scout. This is not a terminal success; the
search continues.

The method passed `120/120`:

- Target max-abs: `4.285147 m` observation to `5.027511 m` residual
- Range RMSE mean: `0.924142 m` observation to `1.110352 m` residual
- Position RMSE mean: `2.492225 m` observation to `2.565507 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`2.487816 m`. This is indistinguishable from ridge `3` at scout precision:
lower ridge is saturated and does not address aggregate RMSE regression.

## Coordinate Contract

- `disparity_forward_product` uses fused measured disparity multiplied by
  stereo-center OpenCV optical RDF `raw_forward_m` after base profile correction
  and b15x7 common0.2 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
