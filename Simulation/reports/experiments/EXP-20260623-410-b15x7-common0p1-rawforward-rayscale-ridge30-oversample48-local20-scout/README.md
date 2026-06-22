# EXP-20260623-410 B15x7 Common0.1 RawForward RayScale Ridge30 Oversample48 Local20 Scout

## Purpose

Test whether lower ridge regularization improves the common0.1 raw-forward
ray-scale branch from EXP409. EXP409 nearly preserved range RMSE but failed all
intrinsic-scale target rows.

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
- Degree: `1`
- Ridge lambda: `30`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.348758 m` observation to `4.183116 m` residual
- Range RMSE mean: `0.916652 m` observation to `0.916100 m` residual
- Position RMSE mean: `2.488401 m` observation to `2.500517 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.838256 m`. Ridge `30` is effectively tied with ridge `300`: it does not
reduce intrinsic-scale failures and still worsens position RMSE. Reject as a
standalone method.

## Coordinate Contract

- `raw_forward` is the corrected stereo-center OpenCV optical RDF forward
  component after base profile correction and b15x7 common0.1 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
