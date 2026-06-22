# EXP-20260623-416 B15x7 Common0.1 DisparityOverForward RayScale Oversample48 Local20 Scout

## Purpose

Test the `disparity_over_forward` ratio feature under common0.1 fusion. The
common0.2 version had a relatively good tracked intrinsic-scale target row, so
this scout checks whether common0.1's lower observation aggregate baseline helps
that ratio branch.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.1`
- Target oversample count: `48`
- Feature space: `disparity_over_forward`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.348758 m` observation to `4.593801 m` residual
- Range RMSE mean: `0.916652 m` observation to `0.978649 m` residual
- Position RMSE mean: `2.488401 m` observation to `2.525023 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.435931 m`. The tracked seed improves versus several raw-forward variants, but
the worst target and aggregate RMSE regress heavily. Reject as a standalone
method.

## Coordinate Contract

- `disparity_over_forward` divides fused measured disparity by corrected
  stereo-center OpenCV optical RDF `raw_forward_m` after base profile correction
  and b15x7 common0.1 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
