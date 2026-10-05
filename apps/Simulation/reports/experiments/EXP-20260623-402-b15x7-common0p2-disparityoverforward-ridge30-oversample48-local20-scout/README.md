# EXP-20260623-402 B15x7 Common0.2 DisparityOverForward Ridge30 Oversample48 Local20 Scout

## Purpose

Test the lower-regularization side of the b15x7 `disparity_over_forward`
ray-scale branch after EXP401 showed ridge `3000` worsened worst target and
aggregate RMSE.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.2`
- Target oversample count: `48`
- Feature space: `disparity_over_forward`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `30`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.285147 m` observation to `4.528040 m` residual
- Range RMSE mean: `0.924142 m` observation to `0.982878 m` residual
- Position RMSE mean: `2.492225 m` observation to `2.527314 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.428529 m`. Ridge `30` behaves close to ridge `3000`: it keeps the tracked
target row near EXP391 but does not reduce intrinsic-scale failures and worsens
aggregate RMSE. Reject this lower-ridge ratio ray-scale branch.

## Coordinate Contract

- `disparity_over_forward` divides fused measured disparity by corrected
  stereo-center OpenCV optical RDF `raw_forward_m` after base profile correction
  and b15x7 common0.2 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
