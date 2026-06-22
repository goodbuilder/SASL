# EXP-20260622-372 B15x5 Common0.2 DisparityOverForward Oversample48 Local20 Scout

## Purpose

Test `disparity_over_forward` as the opposite ratio to
`forward_over_disparity` for the b15x5 common0.2 oversample48 guard fallback.
The goal is to see whether the inverse ratio improves the intrinsic-scale
target row without excessive aggregate regression.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x5, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.2`
- Target oversample count: `48`
- Feature space: `disparity_over_forward`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.559166 m` observation to `4.802444 m` residual
- Range RMSE mean: `0.927983 m` observation to `0.986408 m` residual
- Position RMSE mean: `2.494131 m` observation to `2.529173 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.446209 m`, better than the other single-feature ratio scouts but still
failing the strict gate. The aggregate and max-target regressions make this
unsuitable as a standalone guard fallback.

## Coordinate Contract

- `disparity_over_forward` uses fused measured disparity divided by
  stereo-center OpenCV optical RDF `raw_forward_m` after base profile correction
  and b15x5 common0.2 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
