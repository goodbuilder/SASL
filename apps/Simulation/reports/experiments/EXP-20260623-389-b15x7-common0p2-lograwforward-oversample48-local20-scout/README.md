# EXP-20260623-389 B15x7 Common0.2 LogRawForward Oversample48 Local20 Scout

## Purpose

Test whether log scaling the `raw_forward` feature improves the b15x7
common0.2 ray-scale branch. EXP388 raw-forward preserved aggregate RMSE but
failed all intrinsic-scale target rows.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
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

- Target max-abs: `4.285147 m` observation to `4.255010 m` residual
- Range RMSE mean: `0.924142 m` observation to `0.930427 m` residual
- Position RMSE mean: `2.492225 m` observation to `2.503667 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.700561 m`, failing the strict gate. Log scaling improves the tracked target
row versus raw-forward but still fails all intrinsic-scale rows and gives up the
raw-forward aggregate improvement.

## Coordinate Contract

- `log_raw_forward` is computed from stereo-center OpenCV optical RDF
  `raw_forward_m` after base profile correction and b15x7 common0.2 fusion.
- `ray_scale` outputs a stereo-center RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
