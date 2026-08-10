# EXP-20260623-399 B15x7 Common0.2 RawForward ForwardAdd Degree2 Oversample48 Local20 Scout

## Purpose

Test whether adding quadratic capacity to the aggregate-preserving raw-forward
`forward_add` branch from EXP398 reduces intrinsic-scale target failures without
giving up the aggregate RMSE improvement.

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
- Correction space: `forward_add`
- Degree: `2`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.285147 m` observation to `4.265148 m` residual
- Range RMSE mean: `0.924142 m` observation to `0.917960 m` residual
- Position RMSE mean: `2.492225 m` observation to `2.489773 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.691247 m`. Degree `2` does not reduce the failure count or tracked target row
versus EXP398, and aggregate RMSE is slightly worse. Reject this quadratic
raw-forward additive branch as a standalone method.

## Coordinate Contract

- `raw_forward` is the corrected stereo-center OpenCV optical RDF forward
  component after base profile correction and b15x7 common0.2 fusion.
- `forward_add` adds a residual to stereo-center RDF forward before public
  topic export.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
