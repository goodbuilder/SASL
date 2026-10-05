# EXP-20260623-392 B15x7 Common0.2 Disparity Oversample48 Local20 Scout

## Purpose

Test plain fused disparity as the ray-scale regression feature under b15x7
common0.2 fusion. This compares the simplest disparity scalar against the
product and ratio feature family.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.2`
- Target oversample count: `48`
- Feature space: `disparity`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.285147 m` observation to `4.411758 m` residual
- Range RMSE mean: `0.924142 m` observation to `0.955472 m` residual
- Position RMSE mean: `2.492225 m` observation to `2.513818 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.544095 m`, matching the b15x7 inverse-depth scout. Plain disparity is not a
standalone guard fallback for intrinsic-scale rows.

## Coordinate Contract

- `disparity` is the fused measured disparity scalar and does not alias any RDF
  or FRD position axis.
- `ray_scale` outputs a stereo-center OpenCV optical RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
