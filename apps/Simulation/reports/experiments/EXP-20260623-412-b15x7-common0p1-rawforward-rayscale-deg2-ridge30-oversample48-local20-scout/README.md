# EXP-20260623-412 B15x7 Common0.1 RawForward RayScale Degree2 Ridge30 Oversample48 Local20 Scout

## Purpose

Test whether quadratic capacity improves the common0.1 raw-forward ray-scale
branch after ridge and clip tuning failed to reduce intrinsic-scale failures.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.1`
- Target oversample count: `48`
- Feature space: `raw_forward`
- Correction space: `ray_scale`
- Degree: `2`
- Ridge lambda: `30`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.348758 m` observation to `4.123928 m` residual
- Range RMSE mean: `0.916652 m` observation to `0.917463 m` residual
- Position RMSE mean: `2.488401 m` observation to `2.503725 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.899702 m`. Degree `2` lowers the worst target residual but does not reduce
the failure count, and it worsens both the tracked target row and aggregate RMSE
relative to degree `1`. Reject as a standalone method.

## Coordinate Contract

- `raw_forward` is the corrected stereo-center OpenCV optical RDF forward
  component after base profile correction and b15x7 common0.1 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
