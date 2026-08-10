# EXP-20260623-390 B15x7 Common0.2 ForwardOverDisparity Oversample48 Local20 Scout

## Purpose

Retest `forward_over_disparity` under b15x7 common0.2 fusion. EXP371 showed
this ratio failed under b15x5 while keeping aggregate RMSE near observation;
this scout checks whether b15x7 changes that intrinsic-scale failure pattern.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.2`
- Target oversample count: `48`
- Feature space: `forward_over_disparity`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.285147 m` observation to `4.261286 m` residual
- Range RMSE mean: `0.924142 m` observation to `0.924598 m` residual
- Position RMSE mean: `2.492225 m` observation to `2.508058 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.914895 m`, failing the strict gate. The feature keeps aggregate RMSE near
observation but remains too weak for intrinsic-scale target protection.

## Coordinate Contract

- `forward_over_disparity` uses stereo-center OpenCV optical RDF `raw_forward_m`
  after base profile correction and b15x7 common0.2 fusion, divided by fused
  measured disparity.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
