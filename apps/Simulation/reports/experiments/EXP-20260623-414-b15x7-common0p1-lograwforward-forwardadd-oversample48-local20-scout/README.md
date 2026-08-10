# EXP-20260623-414 B15x7 Common0.1 LogRawForward ForwardAdd Oversample48 Local20 Scout

## Purpose

Test whether log-scaling the common0.1 raw-forward feature improves the additive
aggregate-anchor branch from EXP408.

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
- Correction space: `forward_add`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.348758 m` observation to `4.338005 m` residual
- Range RMSE mean: `0.916652 m` observation to `0.911593 m` residual
- Position RMSE mean: `2.488401 m` observation to `2.486522 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.688441 m`. The method is close to the raw-forward additive anchor but does
not improve aggregate RMSE and still fails every intrinsic-scale row. Reject as
a standalone method.

## Coordinate Contract

- `log_raw_forward` is the log transform of corrected stereo-center OpenCV
  optical RDF `raw_forward_m` after base profile correction and b15x7 common0.1
  fusion.
- `forward_add` adds a residual to stereo-center RDF forward before public
  topic export.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
