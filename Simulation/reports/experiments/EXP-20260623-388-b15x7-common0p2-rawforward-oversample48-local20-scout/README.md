# EXP-20260623-388 B15x7 Common0.2 RawForward Oversample48 Local20 Scout

## Purpose

Retest the `raw_forward` ray-scale feature under b15x7 common0.2 fusion.
EXP369 showed raw-forward failed under b15x5 while preserving aggregate RMSE;
this scout checks whether b15x7 fusion changes that failure pattern.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.2`
- Target oversample count: `48`
- Feature space: `raw_forward`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.285147 m` observation to `4.190198 m` residual
- Range RMSE mean: `0.924142 m` observation to `0.921861 m` residual
- Position RMSE mean: `2.492225 m` observation to `2.503317 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.836855 m`, failing the strict gate. B15x7 fusion preserves the aggregate
benefit clue but does not fix target-row protection.

## Coordinate Contract

- `raw_forward` is stereo-center OpenCV optical RDF forward after base profile
  correction and b15x7 common0.2 observation fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
