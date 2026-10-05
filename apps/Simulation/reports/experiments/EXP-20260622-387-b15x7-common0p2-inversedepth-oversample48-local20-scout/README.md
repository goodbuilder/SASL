# EXP-20260622-387 B15x7 Common0.2 InverseDepth Oversample48 Local20 Scout

## Purpose

Retest the `inverse_depth` ray-scale feature under b15x7 common0.2 fusion.
EXP368 showed inverse-depth failed under b15x5; this scout checks whether the
b15x7 fusion point changes the intrinsic-scale failure pattern.

## Method

- Seeds: `298180..298199`
- Scenarios:
  `nominal`, `yaw_pitch_stress`, `bbox_outlier_stress`,
  `combined_sensor_stress`, `disparity_noise_stress`,
  `intrinsic_scale_stress`
- Fusion: b15x7, inner/outer `trimmed_mean`, `trim_fraction=0.1`,
  `common_fraction=0.2`
- Target oversample count: `48`
- Feature space: `inverse_depth`
- Correction space: `ray_scale`
- Degree: `1`
- Ridge lambda: `300`
- Clip: `0.02`

## Result

Status: `FAIL` on the local20 scout.

The method passed `100/120`; all `20` failures were `intrinsic_scale_stress`:

- Target max-abs: `4.285147 m` observation to `4.411758 m` residual
- Range RMSE mean: `0.924142 m` observation to `0.955472 m` residual
- Position RMSE mean: `2.492225 m` observation to `2.513818 m` residual

For seed `298188`, `intrinsic_scale_stress`, residual target max-abs was
`3.544095 m`, failing the strict gate. B15x7 fusion preserves the small
aggregate regression clue but does not fix inverse-depth target protection.

## Coordinate Contract

- `inverse_depth` is derived from the fused stereo measurement, not from
  calibration/evaluation truth.
- `ray_scale` outputs a stereo-center OpenCV optical RDF ray-scale correction.
- Truth is calibration/evaluation only and is not a runtime input.
- Public StereoDistance output remains `FRD+range(forward,right,down,range)`.
