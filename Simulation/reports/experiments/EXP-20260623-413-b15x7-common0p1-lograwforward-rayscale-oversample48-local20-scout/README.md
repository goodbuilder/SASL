# EXP-20260623-413 B15x7 Common0.1 LogRawForward RayScale Oversample48 Local20 Scout

## Purpose

Test whether log-scaling the common0.1 raw-forward feature improves the
ray-scale branch after raw-forward degree, ridge, and clip changes did not reduce
intrinsic-scale failures.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.1`
- Target oversample count: `48`
- Feature space: `log_raw_forward`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.348758 m` observation to `4.321244 m` residual
- Range RMSE mean: `0.916652 m` observation to `0.924913 m` residual
- Position RMSE mean: `2.488401 m` observation to `2.500849 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.701293 m`. Log scaling improves the tracked seed versus raw-forward ray-scale
but worsens worst target and range aggregate, and it still fails every
intrinsic-scale row. Reject as a standalone method.

## Coordinate Contract

- `log_raw_forward` is the log transform of corrected stereo-center OpenCV
  optical RDF `raw_forward_m` after base profile correction and b15x7 common0.1
  fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
