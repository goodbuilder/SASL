# EXP-20260622-370 B15x5 Common0.2 LogRawForward Oversample48 Local20 Scout

## Purpose

Test `log_raw_forward` as a scaled variant of the raw-forward feature for the
b15x5 common0.2 oversample48 guard fallback. This checks whether compressing
the stereo-center RDF forward feature improves intrinsic-scale behavior.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x5, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.2`
- Target oversample count: `48`
- Feature space: `log_raw_forward`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.559166 m` observation to `4.541692 m` residual
- Range RMSE mean: `0.927983 m` observation to `0.934412 m` residual
- Position RMSE mean: `2.494131 m` observation to `2.505724 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.714327 m`, failing the strict gate. The log transform is slightly better than
raw-forward on the target row but worse on aggregate range RMSE.

## Coordinate Contract

- `log_raw_forward` is computed from stereo-center OpenCV optical RDF
  `raw_forward_m` after base profile correction and b15x5 common0.2 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
