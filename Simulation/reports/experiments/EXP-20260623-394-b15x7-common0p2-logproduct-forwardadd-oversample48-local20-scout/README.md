# EXP-20260623-394 B15x7 Common0.2 LogProduct ForwardAdd Oversample48 Local20 Scout

## Purpose

Test log-product `forward_add` under b15x7 common0.2 fusion. EXP393 showed
non-log product forward-add preserved aggregate RMSE but under-corrected
intrinsic-scale target rows; this scout checks whether log scaling changes that
additive branch.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.2`
- Target oversample count: `48`
- Feature space: `log_disparity_forward_product`
- Correction space: `forward_add`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.285147 m` observation to `4.305146 m` residual
- Range RMSE mean: `0.924142 m` observation to `0.924613 m` residual
- Position RMSE mean: `2.492225 m` observation to `2.492380 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.651248 m`, matching non-log product forward-add. Log scaling does not change
this additive branch materially.

## Coordinate Contract

- `log_disparity_forward_product` uses the log of fused measured disparity
  multiplied by stereo-center OpenCV optical RDF `raw_forward_m` after base
  profile correction and b15x7 common0.2 fusion.
- `forward_add` adds a residual to stereo-center RDF forward and preserves the
  public `FRD+range` output contract after RDF-to-FRD ordering.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
