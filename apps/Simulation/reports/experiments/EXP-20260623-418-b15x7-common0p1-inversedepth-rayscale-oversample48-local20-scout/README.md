# EXP-20260623-418 B15x7 Common0.1 InverseDepth RayScale Oversample48 Local20 Scout

## Purpose

Test the `inverse_depth` feature under common0.1 fusion. Previous common0.2
scouts showed inverse-depth and plain disparity matching; this scout checks
whether that equivalence also holds with the common0.1 fusion baseline.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.1`
- Target oversample count: `48`
- Feature space: `inverse_depth`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.348758 m` observation to `4.477432 m` residual
- Range RMSE mean: `0.916652 m` observation to `0.950578 m` residual
- Position RMSE mean: `2.488401 m` observation to `2.511222 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.549610 m`. The result matches EXP417 plain disparity at scout precision, so
inverse-depth is not a distinct improvement axis here. Reject as a standalone
method.

## Coordinate Contract

- `inverse_depth` uses the fused measured-disparity scalar path after base
  profile correction and b15x7 common0.1 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
